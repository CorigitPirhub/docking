from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from .collision import CollisionEngine
from .config import Config
from .controllers import PathTrackingController
from .grid_planner import GridAStarPlanner, GridPlannerConfig
from .kinematics import VehicleGeometry
from .lc_corridor import CorridorReciprocityPlanner
from .math_utils import angle_diff
from .planner import LocalPlanner
from .sensors import line_blocked_by_obstacles
from .types import ControlCommand, Obstacle, VehicleState


@dataclass(frozen=True)
class StagingPlan:
    leader_goal: VehicleState
    follower_goal: VehicleState
    leader_path_xy: np.ndarray
    follower_path_xy: np.ndarray
    score: float
    reason: str
    control_mode: str = "generic"
    leader_primitive_states: np.ndarray | None = None
    leader_primitive_cmd: np.ndarray | None = None
    leader_primitive_steps: int = 0
    metadata: dict[str, Any] | None = None


def _path_length(path_xy: np.ndarray | None) -> float:
    if path_xy is None or len(path_xy) < 2:
        return math.inf
    return float(np.sum(np.linalg.norm(np.diff(path_xy, axis=0), axis=1)))


def _pose_xy(state: VehicleState) -> np.ndarray:
    return np.array([float(state.x), float(state.y)], dtype=float)


class CooperativeStagingPlanner:
    def __init__(self, cfg: Config, *, seed: int) -> None:
        self.cfg = cfg
        self.rng = np.random.default_rng(int(seed))
        self.geom = VehicleGeometry(cfg.vehicle)
        self.collision = CollisionEngine(cfg.vehicle, cfg.safety)
        self.corridor_planner = CorridorReciprocityPlanner(cfg, seed=int(seed) + 707)

    def _candidate_poses(self, leader: VehicleState, follower: VehicleState, *, semantic_hints: dict[str, Any] | None = None) -> list[VehicleState]:
        _ = follower
        hints = dict(semantic_hints or {})
        out: list[VehicleState] = [leader.copy()]
        anchor_pose = hints.get("leader_stage_anchor_pose")
        anchor_xy = hints.get("leader_stage_anchor_xy")
        if isinstance(anchor_pose, dict):
            out.append(VehicleState(
                vehicle_id=int(leader.vehicle_id),
                x=float(anchor_pose.get("x", leader.x)),
                y=float(anchor_pose.get("y", leader.y)),
                yaw=float(anchor_pose.get("yaw", leader.yaw)),
                v=0.0,
                delta=0.0,
                mode=leader.mode,
            ))
        elif isinstance(anchor_xy, (list, tuple)) and len(anchor_xy) >= 2:
            yaw_hint = float(hints.get("leader_stage_anchor_yaw", 0.0))
            out.append(VehicleState(
                vehicle_id=int(leader.vehicle_id),
                x=float(anchor_xy[0]),
                y=float(anchor_xy[1]),
                yaw=float(yaw_hint),
                v=0.0,
                delta=0.0,
                mode=leader.mode,
            ))
        yaw0 = float(leader.yaw)
        for _i in range(70):
            r = float(self.rng.uniform(0.0, 6.0))
            th = float(self.rng.uniform(-math.pi, math.pi))
            x = float(leader.x + r * math.cos(th))
            y = float(leader.y + r * math.sin(th))
            yaw_travel = float(math.atan2(y - float(leader.y), x - float(leader.x))) if r > 0.35 else yaw0
            out.append(VehicleState(vehicle_id=int(leader.vehicle_id), x=x, y=y, yaw=yaw0, v=0.0, delta=0.0, mode=leader.mode))
            out.append(VehicleState(vehicle_id=int(leader.vehicle_id), x=x, y=y, yaw=yaw_travel, v=0.0, delta=0.0, mode=leader.mode))
        for _i in range(20):
            x = float(self.rng.uniform(-10.0, 10.0))
            y = float(self.rng.uniform(-4.5, 4.5))
            yaw_travel = float(math.atan2(y - float(leader.y), x - float(leader.x)))
            out.append(VehicleState(vehicle_id=int(leader.vehicle_id), x=x, y=y, yaw=yaw0, v=0.0, delta=0.0, mode=leader.mode))
            out.append(VehicleState(vehicle_id=int(leader.vehicle_id), x=x, y=y, yaw=yaw_travel, v=0.0, delta=0.0, mode=leader.mode))
        return out

    def _in_bounds(self, state: VehicleState, margin: float) -> bool:
        half_w = 0.5 * float(self.cfg.environment.width) - float(margin)
        half_h = 0.5 * float(self.cfg.environment.height) - float(margin)
        return (-half_w <= float(state.x) <= half_w) and (-half_h <= float(state.y) <= half_h)

    def _predock_pose(self, leader_goal: VehicleState, standoff: float) -> VehicleState:
        heading = np.array([math.cos(float(leader_goal.yaw)), math.sin(float(leader_goal.yaw))], dtype=float)
        rear = self.geom.rear_hitch(leader_goal)
        center = rear - heading * float(self.geom.front_hitch_x + float(standoff))
        return VehicleState(
            vehicle_id=-1,
            x=float(center[0]),
            y=float(center[1]),
            yaw=float(leader_goal.yaw),
            v=0.0,
            delta=0.0,
        )

    def _sample_path_states(
        self,
        path_xy: np.ndarray,
        *,
        proto: VehicleState,
        goal_yaw: float,
        sample_step: float,
    ) -> list[VehicleState]:
        if path_xy is None or len(path_xy) == 0:
            return [proto.copy()]
        if len(path_xy) == 1:
            only = proto.copy()
            only.x = float(path_xy[0, 0])
            only.y = float(path_xy[0, 1])
            only.yaw = float(goal_yaw)
            return [only]
        samples: list[VehicleState] = []
        prev_yaw = float(proto.yaw)
        first = True
        for idx in range(len(path_xy) - 1):
            p0 = np.asarray(path_xy[idx], dtype=float)
            p1 = np.asarray(path_xy[idx + 1], dtype=float)
            seg = p1 - p0
            seg_len = float(np.linalg.norm(seg))
            if seg_len <= 1e-9:
                continue
            yaw = float(math.atan2(float(seg[1]), float(seg[0])))
            n = max(1, int(math.ceil(seg_len / max(sample_step, 1e-3))))
            start_j = 0 if first else 1
            for j in range(start_j, n + 1):
                alpha = float(j / n)
                xy = (1.0 - alpha) * p0 + alpha * p1
                state = proto.copy()
                state.x = float(xy[0])
                state.y = float(xy[1])
                state.yaw = yaw if idx < len(path_xy) - 2 or alpha < 0.999 else float(goal_yaw)
                if abs(float(state.yaw) - prev_yaw) > math.pi:
                    state.yaw = float(goal_yaw)
                prev_yaw = float(state.yaw)
                samples.append(state)
            first = False
        if not samples:
            samples.append(proto.copy())
        samples[-1] = proto.copy()
        samples[-1].x = float(path_xy[-1, 0])
        samples[-1].y = float(path_xy[-1, 1])
        samples[-1].yaw = float(goal_yaw)
        return samples

    def _path_min_clearance(
        self,
        path_xy: np.ndarray,
        *,
        proto: VehicleState,
        goal_yaw: float,
        obstacles: list[Obstacle],
    ) -> float:
        min_clear = float("inf")
        sample_step = float(max(self.cfg.safety.swept_sample_distance, 0.18))
        for state in self._sample_path_states(path_xy, proto=proto, goal_yaw=goal_yaw, sample_step=sample_step):
            if not self._in_bounds(state, margin=0.65):
                return -1.0
            if self.collision.in_collision(state, obstacles, []):
                return -1.0
            min_clear = min(min_clear, float(self.collision.min_clearance_vehicle_obstacles(state, obstacles)))
        if math.isinf(min_clear):
            return 1e6
        return float(min_clear)

    def plan(
        self,
        *,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
        prefer_static_leader: bool = False,
        semantic_hints: dict[str, Any] | None = None,
        use_lc_hybrid_search: bool = True,
    ) -> StagingPlan:
        grid = GridAStarPlanner(
            width=float(self.cfg.environment.width),
            height=float(self.cfg.environment.height),
            obstacles=obstacles,
            cfg=GridPlannerConfig(resolution=0.18, inflation_radius=0.55, boundary_block=1, max_expansions=260_000),
        )

        hints = dict(semantic_hints or {})
        lane_enabled = bool(hints.get("enabled", False) and str(hints.get("category", "")).upper() == "LC")
        if lane_enabled and not prefer_static_leader:
            corridor_plan = self.corridor_planner.plan(
                obstacles=obstacles,
                leader=leader,
                follower=follower,
                lane=hints,
                use_hybrid_search=bool(use_lc_hybrid_search),
            )
            if corridor_plan is not None:
                return StagingPlan(
                    leader_goal=corridor_plan.leader_goal.copy(),
                    follower_goal=corridor_plan.follower_goal.copy(),
                    leader_path_xy=np.asarray(corridor_plan.leader_path_xy, dtype=float),
                    follower_path_xy=np.asarray(corridor_plan.follower_path_xy, dtype=float),
                    score=float(corridor_plan.score),
                    reason=str(corridor_plan.reason),
                    control_mode="corridor_reciprocal",
                    leader_primitive_states=np.asarray(corridor_plan.leader_primitive_states, dtype=float),
                    leader_primitive_cmd=np.asarray(corridor_plan.leader_primitive_cmd, dtype=float),
                    leader_primitive_steps=int(corridor_plan.leader_primitive_steps),
                    metadata=dict(corridor_plan.metadata),
                )
        anchor_xy = None
        if isinstance(hints.get("leader_stage_anchor_xy"), (list, tuple)) and len(hints.get("leader_stage_anchor_xy")) >= 2:
            anchor_xy = np.asarray(hints.get("leader_stage_anchor_xy"), dtype=float)
        anchor_yaw = float(hints.get("leader_stage_anchor_yaw", 0.0))

        best: StagingPlan | None = None
        predock_standoff = float(max(0.70 if lane_enabled else 0.85, float(self.cfg.docking.stage_switch_distance) - (0.55 if lane_enabled else 0.45)))
        clearance_req = float(0.22 if lane_enabled else max(self.cfg.scenario.dock_clearance_min, 0.55))
        path_clear_req = float(max(float(self.cfg.safety.min_clearance) + (0.08 if lane_enabled else 0.22), 0.18 if lane_enabled else 0.32))

        if lane_enabled and anchor_xy is not None and isinstance(hints.get("leader_stage_path_xy"), list) and isinstance(hints.get("follower_stage_path_xy"), list):
            semantic_cand = VehicleState(
                vehicle_id=int(leader.vehicle_id),
                x=float(anchor_xy[0]),
                y=float(anchor_xy[1]),
                yaw=float(anchor_yaw),
                v=0.0,
                delta=0.0,
                mode=leader.mode,
            )
            if self._in_bounds(semantic_cand, margin=0.7) and not self.collision.in_collision(semantic_cand, obstacles, []) and self.collision.min_clearance_vehicle_obstacles(semantic_cand, obstacles) >= clearance_req:
                semantic_predock = self._predock_pose(semantic_cand, standoff=predock_standoff)
                semantic_predock = VehicleState(
                    vehicle_id=int(follower.vehicle_id),
                    x=float(semantic_predock.x),
                    y=float(semantic_predock.y),
                    yaw=float(semantic_predock.yaw),
                    v=0.0,
                    delta=0.0,
                    mode=follower.mode,
                )
                if self._in_bounds(semantic_predock, margin=0.7) and not self.collision.in_collision(semantic_predock, obstacles, [semantic_cand]) and self.collision.min_clearance_vehicle_obstacles(semantic_predock, obstacles) >= clearance_req:
                    cam = self.geom.front_hitch(semantic_predock)
                    rear = self.geom.rear_hitch(semantic_cand)
                    if not line_blocked_by_obstacles(cam, rear, obstacles):
                        semantic_leader_path = np.asarray(hints.get("leader_stage_path_xy"), dtype=float)
                        semantic_follower_path = np.asarray(hints.get("follower_stage_path_xy"), dtype=float)
                        return StagingPlan(
                            leader_goal=semantic_cand,
                            follower_goal=semantic_predock,
                            leader_path_xy=semantic_leader_path.astype(float),
                            follower_path_xy=semantic_follower_path.astype(float),
                            score=float(_path_length(semantic_leader_path) + 1.15 * _path_length(semantic_follower_path)),
                            reason="semantic_anchor",
                        )

        for cand in self._candidate_poses(leader, follower, semantic_hints=hints):
            if prefer_static_leader and (abs(float(cand.x - leader.x)) + abs(float(cand.y - leader.y)) > 1e-6):
                continue
            if not self._in_bounds(cand, margin=0.7):
                continue
            if self.collision.in_collision(cand, obstacles, []):
                continue
            if self.collision.min_clearance_vehicle_obstacles(cand, obstacles) < clearance_req:
                continue

            predock = self._predock_pose(cand, standoff=predock_standoff)
            predock = VehicleState(
                vehicle_id=int(follower.vehicle_id),
                x=float(predock.x),
                y=float(predock.y),
                yaw=float(predock.yaw),
                v=0.0,
                delta=0.0,
                mode=follower.mode,
            )
            if not self._in_bounds(predock, margin=0.7):
                continue
            if self.collision.in_collision(predock, obstacles, [cand]):
                continue
            if self.collision.min_clearance_vehicle_obstacles(predock, obstacles) < clearance_req:
                continue

            cam = self.geom.front_hitch(predock)
            rear = self.geom.rear_hitch(cand)
            if line_blocked_by_obstacles(cam, rear, obstacles):
                continue

            use_semantic_anchor = bool(
                lane_enabled
                and anchor_xy is not None
                and float(np.linalg.norm(_pose_xy(cand) - anchor_xy)) <= 0.35
                and abs(angle_diff(float(cand.yaw), float(anchor_yaw))) <= math.radians(20.0)
                and isinstance(hints.get("leader_stage_path_xy"), list)
                and len(hints.get("leader_stage_path_xy")) >= 2
            )
            leader_path = np.asarray(hints.get("leader_stage_path_xy"), dtype=float) if use_semantic_anchor else grid.plan(start_xy=_pose_xy(leader), goal_xy=_pose_xy(cand))
            follower_path = np.asarray(hints.get("follower_stage_path_xy"), dtype=float) if use_semantic_anchor and isinstance(hints.get("follower_stage_path_xy"), list) and len(hints.get("follower_stage_path_xy")) >= 2 else grid.plan(start_xy=_pose_xy(follower), goal_xy=_pose_xy(predock))
            if leader_path is None or follower_path is None:
                continue

            def attach_approach(path_xy: np.ndarray, goal: VehicleState, base_dist: float) -> np.ndarray:
                if path_xy is None or len(path_xy) < 2:
                    return path_xy
                heading = np.array([math.cos(float(goal.yaw)), math.sin(float(goal.yaw))], dtype=float)
                goal_xy = _pose_xy(goal)
                for d in (float(base_dist), float(0.9 * base_dist), 0.75, 0.55):
                    pre_xy = goal_xy - heading * float(d)
                    pre = VehicleState(
                        vehicle_id=int(goal.vehicle_id),
                        x=float(pre_xy[0]),
                        y=float(pre_xy[1]),
                        yaw=float(goal.yaw),
                        v=0.0,
                        delta=0.0,
                        mode=goal.mode,
                    )
                    if not self._in_bounds(pre, margin=0.9):
                        continue
                    if self.collision.in_collision(pre, obstacles, []):
                        continue
                    if np.linalg.norm(path_xy[-1] - pre_xy) < 0.25:
                        return path_xy.astype(float)
                    return np.vstack([path_xy[:-1], pre_xy[None, :], goal_xy[None, :]]).astype(float)
                return path_xy.astype(float)

            leader_path = np.asarray(leader_path, dtype=float) if use_semantic_anchor else attach_approach(np.asarray(leader_path, dtype=float), cand, base_dist=1.1)
            follower_path = np.asarray(follower_path, dtype=float) if use_semantic_anchor else attach_approach(np.asarray(follower_path, dtype=float), predock, base_dist=0.95)
            leader_len = _path_length(leader_path)
            follower_len = _path_length(follower_path)
            if not (math.isfinite(leader_len) and math.isfinite(follower_len)):
                continue

            leader_path_clear = self._path_min_clearance(leader_path, proto=leader, goal_yaw=float(cand.yaw), obstacles=obstacles)
            follower_path_clear = self._path_min_clearance(follower_path, proto=follower, goal_yaw=float(predock.yaw), obstacles=obstacles)
            if use_semantic_anchor and leader_path_clear < 0.0:
                leader_path_clear = float(max(path_clear_req, self.collision.min_clearance_vehicle_obstacles(cand, obstacles)))
            corridor_clear = float(min(leader_path_clear, follower_path_clear))
            if corridor_clear < path_clear_req:
                continue

            score = 1.0 * leader_len + 1.15 * follower_len + 0.2 * abs(angle_diff(float(cand.yaw), float(leader.yaw)))
            yaw_travel = float(math.atan2(float(cand.y) - float(leader.y), float(cand.x) - float(leader.x)))
            score += 2.2 * abs(angle_diff(float(cand.yaw), float(yaw_travel)))
            site_clear = self.collision.min_clearance_vehicle_obstacles(cand, obstacles)
            score += 0.6 / max(float(site_clear), 0.2)
            score += 0.85 / max(float(corridor_clear), 0.18)
            if lane_enabled and anchor_xy is not None:
                score += 3.2 * float(np.linalg.norm(_pose_xy(cand) - anchor_xy))
                score += 0.8 * abs(angle_diff(float(cand.yaw), float(anchor_yaw)))

            if best is None or float(score) < float(best.score):
                best = StagingPlan(
                    leader_goal=cand.copy(),
                    follower_goal=predock.copy(),
                    leader_path_xy=leader_path.astype(float),
                    follower_path_xy=follower_path.astype(float),
                    score=float(score),
                    reason="ok",
                )

        if best is None:
            cand = leader.copy()
            predock = self._predock_pose(cand, standoff=predock_standoff)
            predock = VehicleState(
                vehicle_id=int(follower.vehicle_id),
                x=float(predock.x),
                y=float(predock.y),
                yaw=float(predock.yaw),
                v=0.0,
                delta=0.0,
                mode=follower.mode,
            )
            leader_path = np.stack([_pose_xy(leader), _pose_xy(cand)], axis=0)
            follower_path = np.stack([_pose_xy(follower), _pose_xy(predock)], axis=0)
            return StagingPlan(
                leader_goal=cand,
                follower_goal=predock,
                leader_path_xy=leader_path,
                follower_path_xy=follower_path,
                score=float("inf"),
                reason="fallback_no_candidate",
            )

        return best


class StagingTracker:
    def __init__(self, cfg: Config, *, steering_mode: str = "stanley") -> None:
        self.cfg = cfg
        self.tracker = PathTrackingController(cfg.vehicle, cfg.control, steering_mode=steering_mode)
        self.collision = CollisionEngine(cfg.vehicle, cfg.safety)
        self.local_planner = LocalPlanner(cfg.vehicle, cfg.planner, self.collision, cfg.control.dt)

    def _near_goal(self, state: VehicleState, goal: VehicleState) -> bool:
        d = float(np.linalg.norm(state.xy() - goal.xy()))
        if d > 0.40:
            return False
        return abs(angle_diff(float(goal.yaw), float(state.yaw))) <= math.radians(55.0)

    def _progress_index(self, state: VehicleState, path_xy: np.ndarray) -> int:
        d = np.linalg.norm(path_xy - state.xy(), axis=1)
        return int(np.argmin(d))

    def _lookahead_target(self, state: VehicleState, path_xy: np.ndarray) -> np.ndarray:
        idx = self._progress_index(state, path_xy)
        lookahead = float(0.75 + 0.45 * min(max(abs(float(state.v)), 0.0), 1.2))
        acc = 0.0
        tgt = np.asarray(path_xy[idx], dtype=float)
        for j in range(idx, len(path_xy) - 1):
            p0 = np.asarray(path_xy[j], dtype=float)
            p1 = np.asarray(path_xy[j + 1], dtype=float)
            seg = p1 - p0
            seg_len = float(np.linalg.norm(seg))
            if seg_len <= 1e-9:
                continue
            if acc + seg_len >= lookahead:
                ratio = float((lookahead - acc) / seg_len)
                tgt = (1.0 - ratio) * p0 + ratio * p1
                return tgt.astype(float)
            acc += seg_len
            tgt = p1
        return tgt.astype(float)

    def _speed_cap(self, state: VehicleState, *, target_speed: float, obstacles: list[Obstacle], other: VehicleState | None) -> float:
        clr_obs = float(self.collision.min_clearance_vehicle_obstacles(state, obstacles)) if obstacles else 1e6
        clr_veh = float(self.collision.min_clearance_vehicle_vehicle(state, other)) if other is not None else 1e6
        clr = float(min(clr_obs, clr_veh))
        cap = float(target_speed)
        if clr < 0.70:
            cap = min(cap, 0.48)
        if clr < 0.50:
            cap = min(cap, 0.30)
        if clr < 0.36:
            cap = min(cap, 0.18)
        return cap

    def _project_safe(
        self,
        *,
        state: VehicleState,
        cmd: ControlCommand,
        obstacles: list[Obstacle],
        other: VehicleState | None,
    ) -> ControlCommand:
        max_rate = math.radians(float(self.cfg.vehicle.max_steer_rate_deg_s))
        candidates = [
            cmd,
            ControlCommand(accel=min(0.0, float(cmd.accel)), steer_rate=float(cmd.steer_rate)),
            ControlCommand(accel=-float(self.cfg.vehicle.max_decel), steer_rate=0.0),
            ControlCommand(accel=-0.8, steer_rate=float(max_rate)),
            ControlCommand(accel=-0.8, steer_rate=float(-max_rate)),
        ]
        best: ControlCommand | None = None
        best_dev = float("inf")
        best_clear = -1e9
        for cand in candidates:
            nxt = self.local_planner.model.step(state, cand, float(self.cfg.control.dt))
            collide = self.collision.in_collision(nxt, obstacles, [other] if other is not None else [])
            if collide:
                continue
            clr_obs = float(self.collision.min_clearance_vehicle_obstacles(nxt, obstacles)) if obstacles else 1e6
            clr_veh = float(self.collision.min_clearance_vehicle_vehicle(nxt, other)) if other is not None else 1e6
            clr = float(min(clr_obs, clr_veh))
            dev = float((float(cand.accel) - float(cmd.accel)) ** 2 + (float(cand.steer_rate) - float(cmd.steer_rate)) ** 2)
            if dev < best_dev - 1e-12 or (abs(dev - best_dev) <= 1e-12 and clr > best_clear):
                best = cand
                best_dev = dev
                best_clear = clr
        if best is None:
            return ControlCommand(accel=-float(self.cfg.vehicle.max_decel), steer_rate=0.0)
        return best

    def command(
        self,
        *,
        state: VehicleState,
        goal: VehicleState,
        path_xy: np.ndarray,
        target_speed: float,
        obstacles: list[Obstacle] | None = None,
        other: VehicleState | None = None,
    ) -> tuple[ControlCommand, bool]:
        if self._near_goal(state, goal):
            if abs(float(state.v)) > 0.05:
                accel = float(-math.copysign(float(self.cfg.vehicle.max_decel), float(state.v)))
            else:
                accel = 0.0
            return ControlCommand(accel=accel, steer_rate=float(-state.delta / max(self.cfg.control.dt, 1e-3))), True

        obs = list(obstacles) if obstacles is not None else []
        clr_obs = float(self.collision.min_clearance_vehicle_obstacles(state, obs)) if obs else 1e6
        clr_veh = float(self.collision.min_clearance_vehicle_vehicle(state, other)) if other is not None else 1e6
        guard_clear = float(min(clr_obs, clr_veh))
        need_guard = bool((obs or other is not None) and guard_clear < 0.90)
        v_ref = self._speed_cap(state, target_speed=float(target_speed), obstacles=obs, other=other) if need_guard else float(target_speed)
        use_path = path_xy is not None and len(path_xy) >= 2
        goal_xy = self._lookahead_target(state, path_xy) if use_path else goal.xy()
        dist_to_goal = float(np.linalg.norm(state.xy() - goal.xy()))
        goal_yaw = float(goal.yaw) if dist_to_goal <= 1.0 else float(math.atan2(float(goal_xy[1] - state.y), float(goal_xy[0] - state.x)))

        if not use_path or len(path_xy) < 3:
            cmd = self.tracker.track_point(
                state,
                float(goal_xy[0]),
                float(goal_xy[1]),
                float(goal_yaw),
                float(v_ref),
            )
        else:
            cmd = self.tracker.track_path(state, path_xy, target_speed=float(v_ref))

        if need_guard and float(v_ref) < float(max(abs(float(state.v)), 0.0)) and float(cmd.accel) > 0.0:
            cmd = ControlCommand(accel=0.0, steer_rate=float(cmd.steer_rate))
        if need_guard:
            nxt = self.local_planner.model.step(state, cmd, float(self.cfg.control.dt))
            if self.collision.in_collision(nxt, obs, [other] if other is not None else []):
                cmd = self._project_safe(state=state, cmd=cmd, obstacles=obs, other=other)
        return cmd, False
