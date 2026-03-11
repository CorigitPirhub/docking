from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .collision import CollisionEngine
from .config import Config
from .controllers import PathTrackingController
from .coop_docking import StagingTracker
from .grid_planner import GridAStarPlanner, GridPlannerConfig
from .kinematics import AckermannModel, VehicleGeometry
from .math_utils import angle_diff, clamp
from .planner import LocalPlanner
from .sensors import line_blocked_by_obstacles
from .types import ControlCommand, Obstacle, VehicleState


def _pose_xy(state: VehicleState) -> np.ndarray:
    return np.array([float(state.x), float(state.y)], dtype=float)


def _path_length(path_xy: np.ndarray | None) -> float:
    if path_xy is None or len(path_xy) < 2:
        return math.inf
    return float(np.sum(np.linalg.norm(np.diff(path_xy, axis=0), axis=1)))


@dataclass(frozen=True)
class TerminalTubePlan:
    leader_goal: VehicleState
    follower_goal: VehicleState
    leader_path_xy: np.ndarray
    follower_path_xy: np.ndarray
    corridor_clearance_m: float
    tube_radius_m: float
    leader_shift_m: float
    score: float
    reason: str


@dataclass(frozen=True)
class VisibilityRestagePlan:
    leader_goal: VehicleState
    follower_goal: VehicleState
    leader_path_xy: np.ndarray
    leader_primitive_states: np.ndarray
    leader_primitive_cmd: np.ndarray
    leader_primitive_steps: int
    follower_path_xy: np.ndarray
    follower_view_goal: VehicleState
    follower_view_path_xy: np.ndarray
    corridor_clearance_m: float
    visibility_mean: float
    visibility_tail_min: float
    reacq_distance_m: float
    leader_shift_m: float
    score: float
    reason: str


class Stage25PathSupport:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.geom = VehicleGeometry(cfg.vehicle)
        self.collision = CollisionEngine(cfg.vehicle, cfg.safety)

    def in_bounds(self, state: VehicleState, margin: float = 0.65) -> bool:
        half_w = 0.5 * float(self.cfg.environment.width) - float(margin)
        half_h = 0.5 * float(self.cfg.environment.height) - float(margin)
        return (-half_w <= float(state.x) <= half_w) and (-half_h <= float(state.y) <= half_h)

    def predock_pose(self, leader_goal: VehicleState, *, standoff: float, follower_id: int, follower_mode) -> VehicleState:
        heading = np.array([math.cos(float(leader_goal.yaw)), math.sin(float(leader_goal.yaw))], dtype=float)
        rear = self.geom.rear_hitch(leader_goal)
        center = rear - heading * float(self.geom.front_hitch_x + float(standoff))
        return VehicleState(
            vehicle_id=int(follower_id),
            x=float(center[0]),
            y=float(center[1]),
            yaw=float(leader_goal.yaw),
            v=0.0,
            delta=0.0,
            mode=follower_mode,
        )

    def attach_approach(self, path_xy: np.ndarray, goal: VehicleState, *, base_dist: float, obstacles: list[Obstacle]) -> np.ndarray:
        if path_xy is None or len(path_xy) < 2:
            return np.asarray(path_xy, dtype=float)
        heading = np.array([math.cos(float(goal.yaw)), math.sin(float(goal.yaw))], dtype=float)
        goal_xy = _pose_xy(goal)
        for dist in (float(base_dist), 0.92 * float(base_dist), 0.78 * float(base_dist), 0.58):
            pre_xy = goal_xy - heading * float(dist)
            pre = VehicleState(
                vehicle_id=int(goal.vehicle_id),
                x=float(pre_xy[0]),
                y=float(pre_xy[1]),
                yaw=float(goal.yaw),
                v=0.0,
                delta=0.0,
                mode=goal.mode,
            )
            if not self.in_bounds(pre, margin=0.85):
                continue
            if self.collision.in_collision(pre, obstacles, []):
                continue
            if np.linalg.norm(np.asarray(path_xy[-1], dtype=float) - pre_xy) < 0.22:
                return np.asarray(path_xy, dtype=float)
            return np.vstack([np.asarray(path_xy[:-1], dtype=float), pre_xy[None, :], goal_xy[None, :]]).astype(float)
        return np.asarray(path_xy, dtype=float)

    def densify_path(self, path_xy: np.ndarray, *, step: float = 0.16) -> np.ndarray:
        if path_xy is None or len(path_xy) < 2:
            return np.asarray(path_xy, dtype=float)
        out: list[np.ndarray] = [np.asarray(path_xy[0], dtype=float)]
        for idx in range(len(path_xy) - 1):
            p0 = np.asarray(path_xy[idx], dtype=float)
            p1 = np.asarray(path_xy[idx + 1], dtype=float)
            seg = p1 - p0
            seg_len = float(np.linalg.norm(seg))
            if seg_len <= 1e-9:
                continue
            n = max(1, int(math.ceil(seg_len / max(float(step), 1e-3))))
            for j in range(1, n + 1):
                alpha = float(j / n)
                out.append(((1.0 - alpha) * p0 + alpha * p1).astype(float))
        return np.stack(out, axis=0).astype(float)

    def sample_path_states(
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

    def path_min_clearance(
        self,
        path_xy: np.ndarray,
        *,
        proto: VehicleState,
        goal_yaw: float,
        obstacles: list[Obstacle],
    ) -> float:
        min_clear = float("inf")
        sample_step = float(max(self.cfg.safety.swept_sample_distance, 0.14))
        for state in self.sample_path_states(path_xy, proto=proto, goal_yaw=goal_yaw, sample_step=sample_step):
            if not self.in_bounds(state, margin=0.6):
                return -1.0
            if self.collision.in_collision(state, obstacles, []):
                return -1.0
            min_clear = min(min_clear, float(self.collision.min_clearance_vehicle_obstacles(state, obstacles)))
        if math.isinf(min_clear):
            return 1e6
        return float(min_clear)

    def trim_remaining_path(self, state: VehicleState, path_xy: np.ndarray | None) -> np.ndarray | None:
        if path_xy is None or len(path_xy) < 2:
            return path_xy
        path = np.asarray(path_xy, dtype=float)
        idx = int(np.argmin(np.linalg.norm(path - state.xy(), axis=1)))
        trimmed = path[idx:].astype(float)
        if len(trimmed) == 1:
            return np.vstack([state.xy()[None, :], trimmed]).astype(float)
        trimmed[0] = state.xy().astype(float)
        return trimmed

    def visibility_score(self, follower_state: VehicleState, leader_state: VehicleState, obstacles: list[Obstacle]) -> float:
        cam = self.geom.front_hitch(follower_state)
        rear = self.geom.rear_hitch(leader_state)
        rel = rear - cam
        dist = float(np.linalg.norm(rel))
        if dist > float(self.cfg.sensors.vision.max_distance):
            return 0.0
        fov_half = 0.5 * math.radians(float(self.cfg.sensors.vision.fov_deg))
        bearing = abs(float(angle_diff(float(math.atan2(float(rel[1]), float(rel[0]))), float(follower_state.yaw))))
        if bearing > fov_half:
            return 0.0
        if line_blocked_by_obstacles(cam, rear, obstacles):
            return 0.0
        dist_term = 1.0 - dist / max(float(self.cfg.sensors.vision.max_distance), 1e-6)
        fov_term = 1.0 - bearing / max(float(fov_half), 1e-6)
        return float(clamp(0.45 * dist_term + 0.55 * fov_term, 0.0, 1.0))

    def visibility_profile(
        self,
        *,
        follower_start: VehicleState,
        follower_path_xy: np.ndarray,
        follower_goal: VehicleState,
        leader_goal: VehicleState,
        obstacles: list[Obstacle],
    ) -> tuple[float, float, float]:
        states = self.sample_path_states(
            follower_path_xy,
            proto=follower_start,
            goal_yaw=float(follower_goal.yaw),
            sample_step=0.18,
        )
        if not states:
            return 0.0, 0.0, math.inf
        scores: list[float] = []
        reacq = math.inf
        travel = 0.0
        prev = states[0].xy()
        for state in states:
            travel += float(np.linalg.norm(state.xy() - prev))
            prev = state.xy()
            score = self.visibility_score(state, leader_goal, obstacles)
            scores.append(float(score))
            if score >= 0.35 and not math.isfinite(reacq):
                reacq = float(travel)
        arr = np.asarray(scores, dtype=float)
        tail = arr[-max(4, int(math.ceil(0.35 * len(arr)))) :]
        mean = float(np.mean(arr)) if len(arr) else 0.0
        tail_min = float(np.min(tail)) if len(tail) else 0.0
        return mean, tail_min, float(reacq)


class TubeTracker:
    def __init__(self, cfg: Config, *, steering_mode: str = "stanley") -> None:
        self.cfg = cfg
        self.tracker = PathTrackingController(cfg.vehicle, cfg.control, steering_mode=steering_mode)
        self.collision = CollisionEngine(cfg.vehicle, cfg.safety)
        self.local_planner = LocalPlanner(cfg.vehicle, cfg.planner, self.collision, cfg.control.dt)
        self.model = AckermannModel(cfg.vehicle)

    def _near_goal(self, state: VehicleState, goal: VehicleState) -> bool:
        d = float(np.linalg.norm(state.xy() - goal.xy()))
        if d > 0.35:
            return False
        return abs(angle_diff(float(goal.yaw), float(state.yaw))) <= math.radians(45.0)

    def _tube_radius_cap(self, *, target_speed: float, tube_radius: float, state: VehicleState, obstacles: list[Obstacle], other: VehicleState | None) -> float:
        clr_obs = float(self.collision.min_clearance_vehicle_obstacles(state, obstacles)) if obstacles else 1e6
        clr_veh = float(self.collision.min_clearance_vehicle_vehicle(state, other)) if other is not None else 1e6
        clr = float(min(clr_obs, clr_veh))
        cap = float(target_speed)
        if tube_radius < 0.10 or clr < 0.34:
            cap = min(cap, 0.22)
        elif tube_radius < 0.16 or clr < 0.42:
            cap = min(cap, 0.34)
        elif tube_radius < 0.24 or clr < 0.56:
            cap = min(cap, 0.48)
        else:
            cap = min(cap, 0.68)
        return float(cap)

    def _path_deviation(self, xy: np.ndarray, path_xy: np.ndarray) -> float:
        return float(np.min(np.linalg.norm(np.asarray(path_xy, dtype=float) - np.asarray(xy, dtype=float)[None, :], axis=1)))

    def _rollout_ok(
        self,
        *,
        state: VehicleState,
        cmd: ControlCommand,
        path_xy: np.ndarray,
        tube_radius: float,
        obstacles: list[Obstacle],
        other: VehicleState | None,
        horizon_steps: int = 4,
    ) -> tuple[bool, float, float]:
        cur = state.copy()
        min_clear = float("inf")
        max_dev = 0.0
        for _ in range(int(max(1, horizon_steps))):
            cur = self.model.step(cur, cmd, float(self.cfg.control.dt))
            if self.collision.in_collision(cur, obstacles, [other] if other is not None else []):
                return False, -1.0, float("inf")
            dev = self._path_deviation(cur.xy(), path_xy)
            if dev > float(tube_radius):
                return False, -1.0, float(dev)
            clr_obs = float(self.collision.min_clearance_vehicle_obstacles(cur, obstacles)) if obstacles else 1e6
            clr_veh = float(self.collision.min_clearance_vehicle_vehicle(cur, other)) if other is not None else 1e6
            min_clear = min(min_clear, float(min(clr_obs, clr_veh)))
            max_dev = max(max_dev, float(dev))
        if math.isinf(min_clear):
            min_clear = 1e6
        return True, float(min_clear), float(max_dev)

    def command(
        self,
        *,
        state: VehicleState,
        goal: VehicleState,
        path_xy: np.ndarray,
        target_speed: float,
        tube_radius: float,
        obstacles: list[Obstacle],
        other: VehicleState | None,
    ) -> tuple[ControlCommand, bool]:
        if self._near_goal(state, goal):
            if abs(float(state.v)) > 0.05:
                accel = float(-math.copysign(float(self.cfg.vehicle.max_decel), float(state.v)))
            else:
                accel = 0.0
            return ControlCommand(accel=accel, steer_rate=float(-state.delta / max(float(self.cfg.control.dt), 1e-3))), True

        use_path = path_xy is not None and len(path_xy) >= 2
        v_ref = self._tube_radius_cap(target_speed=float(target_speed), tube_radius=float(tube_radius), state=state, obstacles=obstacles, other=other)
        if not use_path or len(path_xy) < 3:
            nominal = self.tracker.track_point(state, float(goal.x), float(goal.y), float(goal.yaw), float(v_ref))
        else:
            nominal = self.tracker.track_path(state, np.asarray(path_xy, dtype=float), target_speed=float(v_ref))
        max_rate = math.radians(float(self.cfg.vehicle.max_steer_rate_deg_s))
        candidates = [
            nominal,
            ControlCommand(accel=min(0.0, float(nominal.accel)), steer_rate=float(nominal.steer_rate)),
            ControlCommand(accel=-float(self.cfg.vehicle.max_decel), steer_rate=0.0),
            ControlCommand(accel=-0.8, steer_rate=float(0.55 * max_rate)),
            ControlCommand(accel=-0.8, steer_rate=float(-0.55 * max_rate)),
            ControlCommand(accel=float(max(-0.4, nominal.accel)), steer_rate=float(clamp(float(nominal.steer_rate) + 0.22 * max_rate, -max_rate, max_rate))),
            ControlCommand(accel=float(max(-0.4, nominal.accel)), steer_rate=float(clamp(float(nominal.steer_rate) - 0.22 * max_rate, -max_rate, max_rate))),
        ]
        best: ControlCommand | None = None
        best_score = float("inf")
        best_clear = -1e9
        for cand in candidates:
            ok, min_clear, max_dev = self._rollout_ok(
                state=state,
                cmd=cand,
                path_xy=np.asarray(path_xy, dtype=float),
                tube_radius=float(tube_radius),
                obstacles=obstacles,
                other=other,
            )
            if not ok:
                continue
            dev = (float(cand.accel) - float(nominal.accel)) ** 2 + 0.18 * (float(cand.steer_rate) - float(nominal.steer_rate)) ** 2
            score = float(dev + 0.8 * max_dev - 0.35 * min_clear)
            if score < best_score - 1e-12 or (abs(score - best_score) <= 1e-12 and min_clear > best_clear):
                best = cand
                best_score = score
                best_clear = float(min_clear)
        if best is None:
            return ControlCommand(accel=-float(self.cfg.vehicle.max_decel), steer_rate=0.0), False
        return best, False


class TerminalViabilityTubeSkill:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.support = Stage25PathSupport(cfg)
        self.tracker = TubeTracker(cfg)

    def _planner(self, obstacles: list[Obstacle]) -> GridAStarPlanner:
        return GridAStarPlanner(
            width=float(self.cfg.environment.width),
            height=float(self.cfg.environment.height),
            obstacles=obstacles,
            cfg=GridPlannerConfig(resolution=0.16, inflation_radius=0.58, boundary_block=1, max_expansions=220_000),
        )

    def propose(
        self,
        *,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
        nominal_leader_goal: VehicleState,
        nominal_follower_goal: VehicleState,
        nominal_leader_path_xy: np.ndarray,
        nominal_follower_path_xy: np.ndarray,
        dock_zone_clearance_m: float,
    ) -> TerminalTubePlan | None:
        nominal_leader_shift = float(np.linalg.norm(nominal_leader_goal.xy() - leader.xy()))
        rem_leader = self.support.trim_remaining_path(leader, nominal_leader_path_xy)
        rem_follower = self.support.trim_remaining_path(follower, nominal_follower_path_xy)
        nominal_clear = min(
            float(self.support.path_min_clearance(np.asarray(rem_leader, dtype=float), proto=leader, goal_yaw=float(nominal_leader_goal.yaw), obstacles=obstacles)) if rem_leader is not None else 1e6,
            float(self.support.path_min_clearance(np.asarray(rem_follower, dtype=float), proto=follower, goal_yaw=float(nominal_follower_goal.yaw), obstacles=obstacles)) if rem_follower is not None else 1e6,
        )
        risk_signature = bool(
            (nominal_clear < max(float(self.cfg.safety.min_clearance) + 0.28, 0.55) and nominal_leader_shift > 1.45)
            or (nominal_leader_shift > 2.30 and float(dock_zone_clearance_m) < 0.82)
        )
        if not risk_signature:
            return None

        planner = self._planner(obstacles)
        heading = np.array([math.cos(float(leader.yaw)), math.sin(float(leader.yaw))], dtype=float)
        lateral = np.array([-heading[1], heading[0]], dtype=float)
        nominal_heading = float(math.atan2(float(nominal_leader_goal.y - leader.y), float(nominal_leader_goal.x - leader.x)))
        yaw_base = [float(leader.yaw), float(nominal_leader_goal.yaw), float(nominal_heading)]
        yaw_offsets = [0.0, math.radians(10.0), -math.radians(10.0), math.radians(18.0), -math.radians(18.0)]
        long_offsets = [0.0, 0.25, 0.55, 0.85, 1.15]
        lat_offsets = [0.0, -0.28, 0.28, -0.52, 0.52]
        clearance_req = float(max(self.cfg.scenario.dock_clearance_min - 0.02, 0.52))
        path_clear_req = float(max(float(self.cfg.safety.min_clearance) + 0.16, 0.28))
        predock_standoff = float(max(0.82, float(self.cfg.docking.stage_switch_distance) - 0.38))

        best: TerminalTubePlan | None = None
        seen: set[tuple[int, int, int]] = set()
        for long_d in long_offsets:
            for lat_d in lat_offsets:
                base_xy = leader.xy() + heading * float(long_d) + lateral * float(lat_d)
                for yaw_ref in yaw_base:
                    for yaw_off in yaw_offsets:
                        yaw = float(yaw_ref + yaw_off)
                        key = (int(round(float(base_xy[0]) * 10.0)), int(round(float(base_xy[1]) * 10.0)), int(round(math.degrees(yaw))))
                        if key in seen:
                            continue
                        seen.add(key)
                        cand = VehicleState(
                            vehicle_id=int(leader.vehicle_id),
                            x=float(base_xy[0]),
                            y=float(base_xy[1]),
                            yaw=yaw,
                            v=0.0,
                            delta=0.0,
                            mode=leader.mode,
                        )
                        if not self.support.in_bounds(cand, margin=0.75):
                            continue
                        if self.support.collision.in_collision(cand, obstacles, []):
                            continue
                        site_clear = float(self.support.collision.min_clearance_vehicle_obstacles(cand, obstacles))
                        if site_clear < clearance_req:
                            continue
                        predock = self.support.predock_pose(cand, standoff=predock_standoff, follower_id=int(follower.vehicle_id), follower_mode=follower.mode)
                        if not self.support.in_bounds(predock, margin=0.75):
                            continue
                        if self.support.collision.in_collision(predock, obstacles, [cand]):
                            continue
                        predock_clear = float(self.support.collision.min_clearance_vehicle_obstacles(predock, obstacles))
                        if predock_clear < clearance_req:
                            continue
                        if line_blocked_by_obstacles(self.support.geom.front_hitch(predock), self.support.geom.rear_hitch(cand), obstacles):
                            continue
                        leader_path = planner.plan(start_xy=_pose_xy(leader), goal_xy=_pose_xy(cand))
                        follower_path = planner.plan(start_xy=_pose_xy(follower), goal_xy=_pose_xy(predock))
                        if leader_path is None or follower_path is None:
                            continue
                        leader_path = self.support.attach_approach(np.asarray(leader_path, dtype=float), cand, base_dist=0.85, obstacles=obstacles)
                        follower_path = self.support.attach_approach(np.asarray(follower_path, dtype=float), predock, base_dist=0.90, obstacles=obstacles)
                        leader_path = self.support.densify_path(leader_path, step=0.14)
                        follower_path = self.support.densify_path(follower_path, step=0.14)
                        leader_len = _path_length(leader_path)
                        follower_len = _path_length(follower_path)
                        if not (math.isfinite(leader_len) and math.isfinite(follower_len)):
                            continue
                        leader_clear = float(self.support.path_min_clearance(leader_path, proto=leader, goal_yaw=float(cand.yaw), obstacles=obstacles))
                        follower_clear = float(self.support.path_min_clearance(follower_path, proto=follower, goal_yaw=float(predock.yaw), obstacles=obstacles))
                        corridor_clear = float(min(leader_clear, follower_clear))
                        if corridor_clear < path_clear_req:
                            continue
                        leader_shift = float(np.linalg.norm(cand.xy() - leader.xy()))
                        tube_radius = float(min(site_clear, predock_clear, corridor_clear) - float(self.cfg.safety.min_clearance))
                        if tube_radius < 0.08:
                            continue
                        score = 1.55 * leader_shift + 0.22 * leader_len + 1.0 * follower_len
                        score += 0.65 / max(tube_radius, 0.05)
                        score += 0.35 / max(corridor_clear, 0.18)
                        score += 0.15 * abs(angle_diff(float(cand.yaw), float(leader.yaw)))
                        score += 0.10 * np.linalg.norm(cand.xy() - nominal_leader_goal.xy())
                        if best is None or float(score) < float(best.score):
                            best = TerminalTubePlan(
                                leader_goal=cand.copy(),
                                follower_goal=predock.copy(),
                                leader_path_xy=leader_path.astype(float),
                                follower_path_xy=follower_path.astype(float),
                                corridor_clearance_m=float(corridor_clear),
                                tube_radius_m=float(tube_radius),
                                leader_shift_m=float(leader_shift),
                                score=float(score),
                                reason="tvt_local_certificate",
                            )
        if best is None:
            return None
        if not (
            float(best.corridor_clearance_m) > nominal_clear + 0.05
            or float(best.leader_shift_m) < nominal_leader_shift - 0.45
        ):
            return None
        return best

    def command(
        self,
        *,
        leader: VehicleState,
        follower: VehicleState,
        plan: TerminalTubePlan,
        obstacles: list[Obstacle],
    ) -> tuple[ControlCommand, ControlCommand, bool, str]:
        leader_rem = _path_length(self.support.trim_remaining_path(leader, plan.leader_path_xy))
        follower_rem = _path_length(self.support.trim_remaining_path(follower, plan.follower_path_xy))
        sync_ratio = float(leader_rem / max(follower_rem, 1e-3))
        leader_speed = float(clamp(0.24 + 0.12 * sync_ratio, 0.16, 0.42))
        follower_speed = float(clamp(0.34 + 0.22 * sync_ratio, 0.22, 0.64))
        leader_cmd, leader_done = self.tracker.command(
            state=leader,
            goal=plan.leader_goal,
            path_xy=plan.leader_path_xy,
            target_speed=float(leader_speed),
            tube_radius=float(max(0.10, 0.75 * plan.tube_radius_m)),
            obstacles=obstacles,
            other=follower,
        )
        if leader_done:
            if abs(float(leader.v)) > 0.03:
                leader_cmd = ControlCommand(accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(leader.v))), steer_rate=0.0)
            else:
                leader_cmd = ControlCommand(accel=0.0, steer_rate=float(-leader.delta / max(float(self.cfg.control.dt), 1e-3)))
        follower_cmd, follower_done = self.tracker.command(
            state=follower,
            goal=plan.follower_goal,
            path_xy=plan.follower_path_xy,
            target_speed=float(follower_speed),
            tube_radius=float(max(0.10, plan.tube_radius_m)),
            obstacles=obstacles,
            other=leader,
        )
        follower_goal_dist = float(np.linalg.norm(follower.xy() - plan.follower_goal.xy()))
        done = bool(leader_done and (follower_done or follower_goal_dist <= 0.72))
        substage = "TUBE_HOLD" if leader_done else "TUBE_TRACK"
        return leader_cmd, follower_cmd, done, substage


class VisibilityPersistentRestagingSkill:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.support = Stage25PathSupport(cfg)
        self.tracker = PathTrackingController(cfg.vehicle, cfg.control, steering_mode="stanley")
        self.branch_tracker = StagingTracker(cfg)
        self.local_planner = LocalPlanner(cfg.vehicle, cfg.planner, self.support.collision, cfg.control.dt)
        self.model = AckermannModel(cfg.vehicle)
        self._visible_hold_s = 0.0
        self._phase = "PAIR"
        self._leader_replay_step = 0

    def reset(self) -> None:
        self._visible_hold_s = 0.0
        self._phase = "PAIR"
        self._leader_replay_step = 0

    def _planner(self, obstacles: list[Obstacle]) -> GridAStarPlanner:
        return GridAStarPlanner(
            width=float(self.cfg.environment.width),
            height=float(self.cfg.environment.height),
            obstacles=obstacles,
            cfg=GridPlannerConfig(resolution=0.16, inflation_radius=0.54, boundary_block=1, max_expansions=260_000),
        )

    def _path_target(self, state: VehicleState, path_xy: np.ndarray, *, lookahead: float) -> np.ndarray:
        path = np.asarray(path_xy, dtype=float)
        idx = int(np.argmin(np.linalg.norm(path - state.xy(), axis=1)))
        acc = 0.0
        target = np.asarray(path[idx], dtype=float)
        for j in range(idx, len(path) - 1):
            p0 = np.asarray(path[j], dtype=float)
            p1 = np.asarray(path[j + 1], dtype=float)
            seg = p1 - p0
            seg_len = float(np.linalg.norm(seg))
            if seg_len <= 1e-9:
                continue
            if acc + seg_len >= lookahead:
                ratio = float((lookahead - acc) / seg_len)
                return ((1.0 - ratio) * p0 + ratio * p1).astype(float)
            acc += seg_len
            target = p1
        return np.asarray(target, dtype=float)

    def _leader_primitive_command(
        self,
        *,
        leader: VehicleState,
        primitive_states: np.ndarray,
        primitive_cmd: np.ndarray,
        primitive_steps: int,
        obstacles: list[Obstacle],
        follower: VehicleState,
    ) -> tuple[ControlCommand, bool]:
        primitive = np.asarray(primitive_states, dtype=float)
        if len(primitive) == 0:
            return ControlCommand(accel=0.0, steer_rate=0.0), True
        step_idx = int(min(max(self._leader_replay_step, 0), max(int(primitive_steps) - 1, 0)))
        target_idx = min(len(primitive) - 1, step_idx + 1)
        target = primitive[target_idx]
        cmd = ControlCommand(accel=float(primitive_cmd[0]), steer_rate=float(primitive_cmd[1]))
        nxt = self.model.step(leader, cmd, float(self.cfg.control.dt))
        if self.support.collision.in_collision(nxt, obstacles, [follower]):
            goal_state = VehicleState(
                vehicle_id=int(leader.vehicle_id),
                x=float(target[0]),
                y=float(target[1]),
                yaw=float(target[2]),
                v=0.0,
                delta=0.0,
                mode=leader.mode,
            )
            cmd, _ = self._track_site(
                state=leader,
                goal=goal_state,
                target_speed=0.18,
                obstacles=obstacles,
                other=follower,
            )
        else:
            self._leader_replay_step += 1
        done = bool(
            self._leader_replay_step >= int(primitive_steps)
            and float(np.linalg.norm(leader.xy() - primitive[-1, :2])) <= 0.35
            and abs(angle_diff(float(primitive[-1, 2]), float(leader.yaw))) <= math.radians(10.0)
        )
        return cmd, done

    def _leader_exposure_command(
        self,
        *,
        leader: VehicleState,
        goal: VehicleState,
        path_xy: np.ndarray,
        obstacles: list[Obstacle],
        follower: VehicleState,
    ) -> tuple[ControlCommand, bool]:
        dist = float(np.linalg.norm(leader.xy() - goal.xy()))
        if dist <= 0.45 and abs(angle_diff(float(goal.yaw), float(leader.yaw))) <= math.radians(35.0):
            if abs(float(leader.v)) > 0.05:
                accel = float(-math.copysign(float(self.cfg.vehicle.max_decel), float(leader.v)))
            else:
                accel = 0.0
            return ControlCommand(accel=accel, steer_rate=float(-leader.delta / max(float(self.cfg.control.dt), 1e-3))), True
        target_xy = self._path_target(leader, path_xy, lookahead=0.60)
        result = self.local_planner.plan_step(
            ego=leader,
            goal_xy=np.asarray(target_xy, dtype=float),
            goal_yaw=float(goal.yaw),
            obstacles=obstacles,
            dynamic_others=[follower],
            force_goal_yaw=bool(dist <= 0.90),
        )
        if result.feasible:
            return result.command, False
        cmd, _ = self.branch_tracker.command(
            state=leader,
            goal=goal,
            path_xy=path_xy,
            target_speed=0.22,
            obstacles=obstacles,
            other=follower,
        )
        return cmd, False

    def _relative_pose_errors(self, *, leader: VehicleState, follower: VehicleState) -> tuple[float, float, float, float]:
        rear = self.support.geom.rear_hitch(leader)
        front = self.support.geom.front_hitch(follower)
        rel = rear - front
        dist = float(np.linalg.norm(rel))
        lc = math.cos(float(leader.yaw))
        ls = math.sin(float(leader.yaw))
        x_l = float(lc * float(rel[0]) + ls * float(rel[1]))
        y_l = float(-ls * float(rel[0]) + lc * float(rel[1]))
        yaw_diff = float(angle_diff(float(leader.yaw), float(follower.yaw)))
        return float(x_l), float(y_l), float(yaw_diff), float(dist)

    def _terminal_pose_closure_command(
        self,
        *,
        leader: VehicleState,
        follower: VehicleState,
        obstacles: list[Obstacle],
    ) -> ControlCommand:
        max_rate = math.radians(float(self.cfg.vehicle.max_steer_rate_deg_s))
        primitives = [
            ControlCommand(accel=-0.8, steer_rate=-max_rate),
            ControlCommand(accel=-0.8, steer_rate=0.0),
            ControlCommand(accel=-0.8, steer_rate=max_rate),
            ControlCommand(accel=-0.4, steer_rate=-0.5 * max_rate),
            ControlCommand(accel=-0.4, steer_rate=0.5 * max_rate),
            ControlCommand(accel=0.0, steer_rate=-0.5 * max_rate),
            ControlCommand(accel=0.0, steer_rate=0.0),
            ControlCommand(accel=0.0, steer_rate=0.5 * max_rate),
            ControlCommand(accel=0.35, steer_rate=-0.5 * max_rate),
            ControlCommand(accel=0.35, steer_rate=0.0),
            ControlCommand(accel=0.35, steer_rate=0.5 * max_rate),
        ]
        best_cmd: ControlCommand | None = None
        best_cost = float('inf')
        x0, y0, yaw0, dist0 = self._relative_pose_errors(leader=leader, follower=follower)
        for cmd1 in primitives:
            for cmd2 in primitives:
                state = follower.copy()
                min_vis = 1e6
                path_len = 0.0
                feasible = True
                for step in range(8):
                    cmd = cmd1 if step < 4 else cmd2
                    nxt = self.model.step(state, cmd, float(self.cfg.control.dt))
                    if self.support.collision.in_collision(nxt, obstacles, [leader]):
                        feasible = False
                        break
                    vis = float(self.support.visibility_score(nxt, leader, obstacles))
                    min_vis = min(min_vis, vis)
                    path_len += float(np.linalg.norm(nxt.xy() - state.xy()))
                    state = nxt
                if not feasible:
                    continue
                x_l, y_l, yaw_diff, dist = self._relative_pose_errors(leader=leader, follower=state)
                yaw_abs = abs(float(yaw_diff))
                if min_vis < 0.05:
                    continue
                cost = 2.0 * abs(float(x_l)) + 5.5 * abs(float(y_l)) + 3.5 * yaw_abs + 0.5 * abs(float(state.v))
                cost += 0.30 * float(path_len) - 0.50 * float(min_vis)
                if yaw_abs > abs(float(yaw0)) + math.radians(8.0):
                    cost += 1.5
                if abs(float(y_l)) > abs(float(y0)) + 0.06:
                    cost += 1.0
                if float(dist) > float(dist0) + 0.18:
                    cost += 1.2
                if cost < best_cost:
                    best_cost = float(cost)
                    best_cmd = cmd1
        if best_cmd is None:
            return ControlCommand(accel=-float(self.cfg.vehicle.max_decel), steer_rate=0.0)
        return best_cmd

    def _leader_response_candidates(
        self,
        *,
        leader: VehicleState,
        follower: VehicleState,
        obstacles: list[Obstacle],
    ) -> list[tuple[VehicleState, np.ndarray, np.ndarray, np.ndarray, int, float]]:
        max_rate = math.radians(float(self.cfg.vehicle.max_steer_rate_deg_s))
        out: list[tuple[VehicleState, np.ndarray, np.ndarray, np.ndarray, int, float]] = []
        for accel in (-0.3, 0.0, 0.3, 0.6):
            for steer_rate in (-max_rate, -0.5 * max_rate, 0.0, 0.5 * max_rate, max_rate):
                steps = 24
                state = leader.copy()
                path_xy = [state.xy().copy()]
                states = [np.array([float(state.x), float(state.y), float(state.yaw)], dtype=float)]
                feasible = True
                cmd_arr = np.array([float(accel), float(steer_rate)], dtype=float)
                for _ in range(steps):
                    state = self.model.step(state, ControlCommand(accel=float(accel), steer_rate=float(steer_rate)), float(self.cfg.control.dt))
                    path_xy.append(state.xy().copy())
                    states.append(np.array([float(state.x), float(state.y), float(state.yaw)], dtype=float))
                    if self.support.collision.in_collision(state, obstacles, [follower]):
                        feasible = False
                        break
                if not feasible:
                    continue
                path = np.stack(path_xy, axis=0).astype(float)
                primitive = np.stack(states, axis=0).astype(float)
                response_cost = 0.55 * abs(float(accel)) + 0.20 * abs(float(steer_rate) / max(max_rate, 1e-6)) + 0.08 * _path_length(path)
                out.append((state.copy(), path, primitive, cmd_arr, int(steps), float(response_cost)))
        return out

    def propose(
        self,
        *,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
    ) -> VisibilityRestagePlan | None:
        planner = self._planner(obstacles)
        best: VisibilityRestagePlan | None = None
        for cand, leader_path, leader_primitive, leader_cmd, leader_steps, response_cost in self._leader_response_candidates(leader=leader, follower=follower, obstacles=obstacles):
            leader_clear = float(self.support.path_min_clearance(leader_path, proto=leader, goal_yaw=float(cand.yaw), obstacles=obstacles))
            if leader_clear < max(float(self.cfg.safety.min_clearance) + 0.06, 0.16):
                continue
            for view_standoff in (1.0, 1.2):
                view_goal = self.support.predock_pose(cand, standoff=float(view_standoff), follower_id=int(follower.vehicle_id), follower_mode=follower.mode)
                if not self.support.in_bounds(view_goal, margin=0.75):
                    continue
                if self.support.collision.in_collision(view_goal, obstacles, [cand]):
                    continue
                view_path = planner.plan(start_xy=_pose_xy(follower), goal_xy=_pose_xy(view_goal))
                if view_path is None:
                    continue
                view_path = self.support.attach_approach(np.asarray(view_path, dtype=float), view_goal, base_dist=0.92, obstacles=obstacles)
                view_path = self.support.densify_path(view_path, step=0.16)
                view_clear = float(self.support.path_min_clearance(view_path, proto=follower, goal_yaw=float(view_goal.yaw), obstacles=obstacles))
                if view_clear < max(float(self.cfg.safety.min_clearance) + 0.02, 0.10):
                    continue
                view_mean, view_tail, reacq = self.support.visibility_profile(
                    follower_start=follower,
                    follower_path_xy=view_path,
                    follower_goal=view_goal,
                    leader_goal=cand,
                    obstacles=obstacles,
                )
                if view_mean < 0.16 or view_tail < 0.45 or reacq > 1.60:
                    continue
                for capture_standoff in (0.65, 0.55, 0.45):
                    capture_goal = self.support.predock_pose(cand, standoff=float(capture_standoff), follower_id=int(follower.vehicle_id), follower_mode=follower.mode)
                    if not self.support.in_bounds(capture_goal, margin=0.75):
                        continue
                    if self.support.collision.in_collision(capture_goal, obstacles, [cand]):
                        continue
                    capture_path = planner.plan(start_xy=_pose_xy(view_goal), goal_xy=_pose_xy(capture_goal))
                    if capture_path is None:
                        continue
                    capture_path = self.support.attach_approach(np.asarray(capture_path, dtype=float), capture_goal, base_dist=0.70, obstacles=obstacles)
                    capture_path = self.support.densify_path(capture_path, step=0.12)
                    capture_clear = float(self.support.path_min_clearance(capture_path, proto=view_goal, goal_yaw=float(capture_goal.yaw), obstacles=obstacles))
                    if capture_clear < max(float(self.cfg.safety.min_clearance) + 0.04, 0.14):
                        continue
                    capture_mean, capture_tail, _ = self.support.visibility_profile(
                        follower_start=view_goal,
                        follower_path_xy=capture_path,
                        follower_goal=capture_goal,
                        leader_goal=cand,
                        obstacles=obstacles,
                    )
                    if capture_mean < 0.35 or capture_tail < 0.55:
                        continue
                    vis_now = float(self.support.visibility_score(follower, cand, obstacles))
                    leader_shift = float(np.linalg.norm(cand.xy() - leader.xy()))
                    corridor_clear = float(min(leader_clear, view_clear, capture_clear))
                    score = float(response_cost + 0.22 * leader_shift + 0.12 * _path_length(view_path) + 0.10 * _path_length(capture_path))
                    score += 0.80 * (1.0 - view_tail) + 0.45 * (1.0 - capture_tail) + 0.30 * float(reacq)
                    score += 0.15 / max(corridor_clear, 0.18) - 0.18 * vis_now
                    if best is None or score < float(best.score):
                        best = VisibilityRestagePlan(
                            leader_goal=cand.copy(),
                            follower_goal=capture_goal.copy(),
                            leader_path_xy=np.asarray(leader_path, dtype=float),
                            leader_primitive_states=np.asarray(leader_primitive, dtype=float),
                            leader_primitive_cmd=np.asarray(leader_cmd, dtype=float),
                            leader_primitive_steps=int(leader_steps),
                            follower_path_xy=np.asarray(capture_path, dtype=float),
                            follower_view_goal=view_goal.copy(),
                            follower_view_path_xy=np.asarray(view_path, dtype=float),
                            corridor_clearance_m=float(corridor_clear),
                            visibility_mean=float(min(view_mean, capture_mean)),
                            visibility_tail_min=float(min(view_tail, capture_tail)),
                            reacq_distance_m=float(reacq),
                            leader_shift_m=float(leader_shift),
                            score=float(score),
                            reason='vpcr_response_pair_capture',
                        )
        return best

    def command(
        self,
        *,
        leader: VehicleState,
        follower: VehicleState,
        plan: VisibilityRestagePlan,
        obstacles: list[Obstacle],
        visual_valid: bool,
    ) -> tuple[ControlCommand, ControlCommand, bool, str]:
        leader_cmd, leader_done = self._leader_primitive_command(
            leader=leader,
            primitive_states=plan.leader_primitive_states,
            primitive_cmd=plan.leader_primitive_cmd,
            primitive_steps=int(plan.leader_primitive_steps),
            obstacles=obstacles,
            follower=follower,
        )
        if leader_done:
            if abs(float(leader.v)) > 0.03:
                leader_cmd = ControlCommand(accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(leader.v))), steer_rate=0.0)
            else:
                leader_cmd = ControlCommand(accel=0.0, steer_rate=float(-leader.delta / max(float(self.cfg.control.dt), 1e-3)))

        vis_here = float(self.support.visibility_score(follower, leader, obstacles))
        if self._phase == 'PAIR':
            follower_cmd, follower_done = self.branch_tracker.command(
                state=follower,
                goal=plan.follower_view_goal,
                path_xy=plan.follower_view_path_xy,
                target_speed=float(0.18 if vis_here < 0.12 else 0.30),
                obstacles=obstacles,
                other=leader,
            )
            pair_ready = bool((leader_done or float(np.linalg.norm(leader.xy() - plan.leader_goal.xy())) <= 0.75) and (follower_done or float(np.linalg.norm(follower.xy() - plan.follower_view_goal.xy())) <= 0.85))
            if bool(visual_valid) and vis_here >= 0.20:
                self._visible_hold_s += float(self.cfg.control.dt)
            else:
                self._visible_hold_s = 0.0
            if pair_ready and self._visible_hold_s >= 0.30:
                self._phase = 'CAPTURE'
                self._visible_hold_s = 0.0
            return leader_cmd, follower_cmd, False, ('PAIR_VISIBLE_HOLD' if self._visible_hold_s > 0.0 else 'PAIR_APPROACH')

        if self._phase == 'CAPTURE':
            follower_cmd, follower_done = self.branch_tracker.command(
                state=follower,
                goal=plan.follower_goal,
                path_xy=plan.follower_path_xy,
                target_speed=float(0.18 if vis_here < 0.18 else 0.28),
                obstacles=obstacles,
                other=leader,
            )
            if bool(visual_valid) and vis_here >= 0.20:
                self._visible_hold_s += float(self.cfg.control.dt)
            else:
                self._visible_hold_s = 0.0
            x_l, y_l, yaw_diff, dist = self._relative_pose_errors(leader=leader, follower=follower)
            basin_ready = bool(
                float(dist) <= 1.60
                and abs(float(y_l)) <= 0.22
                and abs(float(yaw_diff)) <= math.radians(12.0)
                and self._visible_hold_s >= 0.20
            )
            if basin_ready:
                return leader_cmd, follower_cmd, True, 'CAPTURE_BASIN_READY'
            return leader_cmd, follower_cmd, False, ('CAPTURE_VISIBLE_HOLD' if self._visible_hold_s > 0.0 else 'CAPTURE_APPROACH')

        x_l, y_l, yaw_diff, dist = self._relative_pose_errors(leader=leader, follower=follower)
        yaw_err = abs(float(yaw_diff))
        if dist <= 0.20 and abs(float(y_l)) <= 0.03 and yaw_err <= math.radians(5.0):
            follower_cmd = ControlCommand(accel=0.0, steer_rate=float(-follower.delta / max(float(self.cfg.control.dt), 1e-3)))
            return leader_cmd, follower_cmd, True, 'CONTACT_READY'

        primitive_cmd = self._terminal_pose_closure_command(
            leader=leader,
            follower=follower,
            obstacles=obstacles,
        )
        follower_cmd = primitive_cmd
        return leader_cmd, follower_cmd, False, 'CONTACT_SERVO'
