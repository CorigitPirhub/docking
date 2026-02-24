from __future__ import annotations

import itertools
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from controller.path_tracker import DockingController, PurePursuitTracker, TrackerConfig
from core.articulation import ArticulatedTrain
from core.vehicle import AckermannVehicle, VehicleSpec, VehicleState
from env.scene_factory import SceneFactory
from env.world import KeepoutZone, WorldMap
from planner.cost_map import CostMap, DockingHub
from planner.game_theory_planner import PotentialGameDockingPlanner
from planner.hybrid_astar import HybridAStarPlanner
from planner.topology_mcts import DockingCostModel, DockingDecision, TopologyMCTSPlanner


@dataclass
class Frame:
    t: float
    phase: str
    poses: Dict[str, Tuple[float, float, float]]
    chain_ids: List[str]


@dataclass
class SimulationResult:
    success: bool
    message: str
    failure_code: Optional[str]
    decision: Optional[DockingDecision]
    frames: List[Frame]
    world: WorldMap
    vehicles: Dict[str, AckermannVehicle]
    metrics: Dict[str, float | str]


@dataclass
class SimulationConfig:
    mcts_iterations_per_hub: int = 300
    hub_samples: int = 140
    hub_top_k: int = 7
    min_hub_openness: float = 0.35
    obstacle_count: int = 9
    scene_mode: str = "simple"
    difficulty_level: int = 1
    top_planner_type: str = "mcts"  # "mcts" | "game"
    game_max_rounds: int = 18
    game_lambda_cross: float = 5.0
    game_lambda_conflict: float = 2.5
    game_lambda_order_reg: float = 0.55


class DockingSimulation:
    def __init__(self, seed: int = 5, config: Optional[SimulationConfig] = None):
        self.seed = seed
        self.config = config if config is not None else SimulationConfig()
        self.rng = np.random.default_rng(seed)
        self.world = WorldMap(size_m=100.0, resolution=0.5)
        self._scene_build = None

        self.vehicles: Dict[str, AckermannVehicle] = {}
        self.frames: List[Frame] = []
        self.time = 0.0

        self.path_tracker = PurePursuitTracker(TrackerConfig(base_speed=2.6, lookahead=2.8))
        self.docking_tracker = DockingController()

        self.dt = 0.1
        self._planner_cache: Dict[str, HybridAStarPlanner] = {}

    def _build_vehicle_specs(self) -> Dict[str, VehicleSpec]:
        specs: Dict[str, VehicleSpec] = {
            "R1": VehicleSpec(
                vehicle_id="R1",
                length=4.8,
                width=2.05,
                wheelbase=2.8,
                max_steer=0.52,
                steer_mode="front",
                front_port_offset=2.15,
                rear_port_offset=2.15,
            )
        }
        for j in range(2, 6):
            vid = f"R{j}"
            specs[vid] = VehicleSpec(
                vehicle_id=vid,
                length=4.6,
                width=2.0,
                wheelbase=2.7,
                max_steer=0.58,
                steer_mode="rear",
                front_port_offset=2.05,
                rear_port_offset=2.05,
            )
        return specs

    def _build_initial_poses(self) -> Dict[str, VehicleState]:
        rng = np.random.default_rng(self.seed + 911)
        anchors = {
            "R1": (14.0, 14.0),
            "R2": (86.0, 16.0),
            "R3": (84.0, 84.0),
            "R4": (16.0, 82.0),
            "R5": (86.0, 52.0),
        }

        poses: Dict[str, VehicleState] = {}
        accepted_xy: List[np.ndarray] = []
        min_dist = 13.5

        for vid in ["R1", "R2", "R3", "R4", "R5"]:
            ax, ay = anchors[vid]
            chosen = None
            for _ in range(100):
                x = float(np.clip(ax + rng.uniform(-6.0, 6.0), 8.0, 92.0))
                y = float(np.clip(ay + rng.uniform(-6.0, 6.0), 8.0, 92.0))
                p = np.array([x, y], dtype=float)
                if all(float(np.linalg.norm(p - q)) > min_dist for q in accepted_xy):
                    chosen = p
                    break
            if chosen is None:
                chosen = np.array([ax, ay], dtype=float)

            if vid == "R1":
                theta = math.atan2(50.0 - chosen[1], 50.0 - chosen[0]) + float(rng.uniform(-0.5, 0.5))
            else:
                theta = float(rng.uniform(-math.pi, math.pi))

            poses[vid] = VehicleState(x=float(chosen[0]), y=float(chosen[1]), theta=float(theta))
            accepted_xy.append(chosen)

        return poses

    def _init_world(self) -> None:
        poses = self._build_initial_poses()
        diff = int(np.clip(self.config.difficulty_level, 1, 3))

        docking_keepout_r = {1: 18.0, 2: 15.0, 3: 12.0}[diff]
        side_keepout_r = {1: 10.0, 2: 8.0, 3: 6.5}[diff]

        keepouts = [KeepoutZone(x=s.x, y=s.y, r=8.0) for s in poses.values()]
        # Keep central docking theater open.
        keepouts.extend(
            [
                KeepoutZone(x=50.0, y=50.0, r=docking_keepout_r),
                KeepoutZone(x=60.0, y=50.0, r=side_keepout_r),
                KeepoutZone(x=40.0, y=50.0, r=side_keepout_r),
            ]
        )
        scene_factory = SceneFactory(self.world, self.rng)
        build = scene_factory.generate(
            mode=self.config.scene_mode,
            difficulty_level=diff,
            keepouts=keepouts,
            obstacle_count_hint=self.config.obstacle_count,
        )
        self._scene_build = build

    def _init_vehicles(self) -> None:
        specs = self._build_vehicle_specs()
        poses = self._build_initial_poses()
        self.vehicles = {vid: AckermannVehicle(specs[vid], poses[vid]) for vid in specs.keys()}

    def _record(self, phase: str, train: Optional[ArticulatedTrain] = None) -> None:
        poses = {vid: v.pose() for vid, v in self.vehicles.items()}
        chain_ids = train.vehicle_ids() if train is not None else []
        self.frames.append(Frame(t=self.time, phase=phase, poses=poses, chain_ids=chain_ids))

    def _get_planner(self, vid: str) -> HybridAStarPlanner:
        if vid not in self._planner_cache:
            self._planner_cache[vid] = HybridAStarPlanner(
                world=self.world,
                spec=self.vehicles[vid].spec,
                xy_resolution=1.4,
                theta_bins=56,
                step_len=1.9,
                primitive_substeps=4,
                max_expansions=18000,
            )
        return self._planner_cache[vid]

    def _check_world_collision(
        self,
        veh: AckermannVehicle,
        static_vehicles: Optional[Sequence[AckermannVehicle]] = None,
    ) -> bool:
        return self.world.collision_for_pose(
            x=veh.state.x,
            y=veh.state.y,
            theta=veh.state.theta,
            spec=veh.spec,
            static_vehicles=static_vehicles,
        )

    def _scene_metrics(self) -> Dict[str, float | str]:
        out: Dict[str, float | str] = {
            "scene_mode": self.config.scene_mode,
            "difficulty_level": float(self.config.difficulty_level),
            "obstacle_count": float(len(self.world.obstacles)),
        }
        if self._scene_build is not None:
            out["inflation_radius"] = float(self._scene_build.inflation_radius)
        return out

    @staticmethod
    def _segments_intersect(p1: np.ndarray, p2: np.ndarray, q1: np.ndarray, q2: np.ndarray) -> bool:
        def orient(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
            return float((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))

        o1 = orient(p1, p2, q1)
        o2 = orient(p1, p2, q2)
        o3 = orient(q1, q2, p1)
        o4 = orient(q1, q2, p2)
        return (o1 * o2 < 0.0) and (o3 * o4 < 0.0)

    @staticmethod
    def _segment_to_segment_min_dist(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray) -> float:
        m = 16
        best = 1e18
        for i in range(m + 1):
            t = i / m
            pa = (1.0 - t) * a0 + t * a1
            for j in range(m + 1):
                s = j / m
                pb = (1.0 - s) * b0 + s * b1
                d = float(np.linalg.norm(pa - pb))
                if d < best:
                    best = d
        return best

    def _sequence_potential(
        self,
        hub: DockingHub,
        sequence: Sequence[str],
        vehicles: Dict[str, AckermannVehicle],
        cost_model: DockingCostModel,
    ) -> float:
        base = 0.0
        for step, vid in enumerate(sequence):
            base += cost_model.action_cost(vehicles[vid], hub, step=step, chain_len=1 + step)

        segs = []
        for step, vid in enumerate(sequence):
            src = np.array([vehicles[vid].state.x, vehicles[vid].state.y], dtype=float)
            target = cost_model.target_socket(hub, step)
            segs.append((src, target, vid, step))

        pair_term = 0.0
        for i in range(len(segs)):
            p0, p1, _, _ = segs[i]
            for j in range(i + 1, len(segs)):
                q0, q1, _, _ = segs[j]
                if self._segments_intersect(p0, p1, q0, q1):
                    pair_term += self.config.game_lambda_cross
                min_dist = self._segment_to_segment_min_dist(p0, p1, q0, q1)
                if min_dist < 4.0:
                    pair_term += self.config.game_lambda_conflict * (4.0 - min_dist) / 4.0

        return float(base + pair_term)

    def _plan_and_track(
        self,
        active: AckermannVehicle,
        goal: VehicleState,
        static_vehicles: Sequence[AckermannVehicle],
        phase: str,
        max_steps: int = 1400,
        mode: str = "pure_pursuit",
    ) -> Tuple[bool, str]:
        path = self._plan_path_with_fallback(
            active=active,
            goal=goal,
            static_vehicles=static_vehicles,
            aggressive=phase.startswith("dock_"),
        )
        if path is None or len(path) < 2:
            return False, "PathPlanningFail"

        if mode == "path_replay":
            replay_speed = 2.0
            prev = path[0]
            for st in path[1:]:
                seg = math.hypot(st.x - prev.x, st.y - prev.y)
                self.time += seg / max(replay_speed, 1e-3)
                active.set_pose(st.x, st.y, st.theta)
                if self._check_world_collision(active, static_vehicles=static_vehicles):
                    return False, "Collision"
                self._record(phase)
                prev = st
            active.set_pose(goal.x, goal.y, goal.theta)
            self._record(phase)
            return True, "OK"

        reached = False
        for _ in range(max_steps):
            v, steer, done = self.path_tracker.control(active, path)
            active.step(v=v, steer=steer, dt=self.dt)
            self.time += self.dt

            if self._check_world_collision(active, static_vehicles=static_vehicles):
                return False, "Collision"

            self._record(phase)
            if done:
                reached = True
                break

        if not reached:
            d_goal = math.hypot(active.state.x - goal.x, active.state.y - goal.y)
            if d_goal > 1.5:
                return False, "Timeout"

        # Snap to planned final state to remove tiny residual tracking errors.
        active.set_pose(goal.x, goal.y, goal.theta)
        self._record(phase)
        return True, "OK"

    def _plan_path_with_fallback(
        self,
        active: AckermannVehicle,
        goal: VehicleState,
        static_vehicles: Sequence[AckermannVehicle],
        aggressive: bool = False,
    ) -> Optional[List[VehicleState]]:
        planner = self._get_planner(active.spec.vehicle_id)
        path = planner.plan(start=active.state, goal=goal, static_vehicles=static_vehicles)
        if path is not None and len(path) >= 2:
            return path

        if not aggressive:
            return path

        fallback_settings = [
            dict(xy_resolution=1.2, theta_bins=72, step_len=1.4, primitive_substeps=5, max_expansions=120000),
            dict(xy_resolution=1.0, theta_bins=96, step_len=1.2, primitive_substeps=6, max_expansions=240000),
        ]
        for kw in fallback_settings:
            fallback_planner = HybridAStarPlanner(self.world, active.spec, **kw)
            path = fallback_planner.plan(start=active.state, goal=goal, static_vehicles=static_vehicles)
            if path is not None and len(path) >= 2:
                return path

        return None

    def _dock_vehicle(
        self,
        active: AckermannVehicle,
        train: ArticulatedTrain,
        other_static: Sequence[AckermannVehicle],
        phase: str,
    ) -> Tuple[bool, str]:
        goal_state = self._pre_dock_goal_state(active=active, train=train)

        static_for_path = list(other_static) + list(train.vehicles)
        static_for_path = [
            v
            for v in static_for_path
            if v.spec.vehicle_id not in {active.spec.vehicle_id, train.tail.spec.vehicle_id}
        ]

        ok, reason = self._plan_and_track(
            active,
            goal_state,
            static_for_path,
            phase=f"{phase}_approach",
            mode="path_replay",
        )
        if not ok:
            return False, reason

        # Final geometric closed-loop docking.
        max_align_steps = 260
        aligned = False
        for _ in range(max_align_steps):
            socket = train.tail.rear_port()
            socket_heading = train.tail.state.theta
            v, steer, done = self.docking_tracker.control(active, socket, socket_heading)
            active.step(v=v, steer=steer, dt=self.dt)
            self.time += self.dt

            # During insertion we allow contact with the tail vehicle only.
            static_align = [v for v in other_static if v.spec.vehicle_id != active.spec.vehicle_id]
            if self._check_world_collision(active, static_vehicles=static_align):
                return False, "Collision"

            self._record(f"{phase}_align", train=train)
            if done:
                aligned = True
                break

        if not aligned:
            return False, "Timeout"

        socket = train.tail.rear_port()
        socket_heading = train.tail.state.theta
        center = socket - active.spec.front_port_offset * np.array(
            [math.cos(socket_heading), math.sin(socket_heading)], dtype=float
        )
        active.set_pose(float(center[0]), float(center[1]), socket_heading)
        self._record(f"{phase}_locked", train=train)
        return True, "OK"

    def _pre_dock_goal_state(self, active: AckermannVehicle, train: ArticulatedTrain) -> VehicleState:
        tail = train.tail
        socket = tail.rear_port()
        socket_heading = tail.state.theta

        h = np.array([math.cos(socket_heading), math.sin(socket_heading)], dtype=float)
        pre_front = socket - 1.6 * h
        goal_center = pre_front - active.spec.front_port_offset * h
        return VehicleState(x=float(goal_center[0]), y=float(goal_center[1]), theta=socket_heading)

    def _select_next_trailer(
        self,
        train: ArticulatedTrain,
        free_trailers: Dict[str, AckermannVehicle],
        preferred_order: Sequence[str],
    ) -> Optional[str]:
        candidates = [vid for vid in preferred_order if vid in free_trailers]
        if not candidates:
            return None

        best_vid: Optional[str] = None
        best_score = 1e18

        for rank, vid in enumerate(candidates):
            active = free_trailers[vid]
            goal_state = self._pre_dock_goal_state(active=active, train=train)
            p0 = np.array([active.state.x, active.state.y], dtype=float)
            p1 = np.array([goal_state.x, goal_state.y], dtype=float)
            dist = float(np.linalg.norm(p1 - p0))

            # Lightweight corridor risk estimate (no full planner call).
            blocked = 0
            samples = 22
            for i in range(samples + 1):
                t = i / max(samples, 1)
                p = (1.0 - t) * p0 + t * p1
                if not self.world.is_free_point(float(p[0]), float(p[1]), use_inflated=True):
                    blocked += 1
            line_risk = blocked / max(samples + 1, 1)

            heading_pen = abs(((goal_state.theta - active.state.theta + math.pi) % (2.0 * math.pi)) - math.pi)
            score = dist + 48.0 * line_risk + 1.8 * heading_pen + 0.7 * rank
            if score < best_score:
                best_score = score
                best_vid = vid

        return best_vid

    def _run_train_motion_demo(self, train: ArticulatedTrain, duration: float = 18.0) -> Tuple[bool, str]:
        moved = 0.0
        max_steps = int(duration / self.dt)
        reverse_speed = -1.2

        for _ in range(max_steps):
            prev_poses = [v.pose() for v in train.vehicles]
            head_prev = np.array([train.head.state.x, train.head.state.y], dtype=float)

            train.step_head(v=reverse_speed, steer=0.0, dt=self.dt)
            self.time += self.dt

            collision = False
            for i, veh in enumerate(train.vehicles):
                others = [ov for j, ov in enumerate(train.vehicles) if j != i and abs(j - i) > 1]
                if self._check_world_collision(veh, static_vehicles=others):
                    collision = True
                    break

            if collision:
                for veh, pose in zip(train.vehicles, prev_poses):
                    veh.set_pose(*pose)
                if moved < 3.0:
                    return False, "Collision"
                break

            head_now = np.array([train.head.state.x, train.head.state.y], dtype=float)
            moved += float(np.linalg.norm(head_now - head_prev))
            self._record("train_motion", train=train)

        if moved < 3.0:
            return False, "Timeout"
        return True, "OK"

    def run(self) -> SimulationResult:
        self._init_world()
        self._init_vehicles()

        self._record("initial")

        head = self.vehicles["R1"]
        trailer_ids = ["R2", "R3", "R4", "R5"]
        trailers = [self.vehicles[vid] for vid in trailer_ids]

        cost_map = CostMap(self.world)
        planner_type = self.config.top_planner_type.strip().lower()
        if planner_type == "game":
            top_planner = PotentialGameDockingPlanner(
                cost_map=cost_map,
                max_rounds=self.config.game_max_rounds,
                seed=self.seed,
                min_hub_openness=self.config.min_hub_openness,
                lambda_cross=self.config.game_lambda_cross,
                lambda_conflict=self.config.game_lambda_conflict,
                lambda_order_reg=self.config.game_lambda_order_reg,
            )
        else:
            planner_type = "mcts"
            top_planner = TopologyMCTSPlanner(
                cost_map=cost_map,
                mcts_iterations_per_hub=self.config.mcts_iterations_per_hub,
                seed=self.seed,
                min_hub_openness=self.config.min_hub_openness,
            )

        t0 = time.perf_counter()
        decision = top_planner.plan(
            head=head,
            trailers=trailers,
            n_hub_samples=self.config.hub_samples,
            top_k_hubs=self.config.hub_top_k,
        )
        top_decision_time_s = float(time.perf_counter() - t0)

        vehicles_by_id = {v.spec.vehicle_id: v for v in trailers}
        cost_model = DockingCostModel(cost_map)
        seq_potential = self._sequence_potential(decision.hub, decision.sequence, vehicles_by_id, cost_model)
        opt_potential = float("inf")
        for perm in itertools.permutations(trailer_ids):
            p = self._sequence_potential(decision.hub, perm, vehicles_by_id, cost_model)
            if p < opt_potential:
                opt_potential = p
        opt_gap_ratio = (seq_potential - opt_potential) / max(abs(opt_potential), 1e-6)

        top_metrics: Dict[str, float | str] = {
            "top_planner_type": planner_type,
            "top_decision_time_s": top_decision_time_s,
            "top_decision_potential": float(seq_potential),
            "top_decision_optimal_potential": float(opt_potential),
            "top_decision_opt_gap_ratio": float(opt_gap_ratio),
        }
        if planner_type == "game" and getattr(top_planner, "last_debug", None) is not None:
            dbg = top_planner.last_debug
            top_metrics.update(
                {
                    "game_rounds": float(dbg.rounds),
                    "game_converged": float(1.0 if dbg.converged else 0.0),
                    "game_potential": float(dbg.potential),
                    "game_optimal_potential": float(dbg.optimal_potential),
                    "game_internal_opt_gap": float(dbg.optimality_gap),
                }
            )

        scene_metrics = self._scene_metrics()

        def _metrics(docked_num: int) -> Dict[str, float | str]:
            return {
                "docked": float(docked_num),
                "sim_time": float(self.time),
                **top_metrics,
                **scene_metrics,
            }

        train = ArticulatedTrain([head])

        # Move head so that its rear socket arrives at selected hub anchor.
        hub = decision.hub
        hub_dir = np.array([math.cos(hub.heading), math.sin(hub.heading)], dtype=float)
        head_goal_center = np.array([hub.x, hub.y], dtype=float) + head.spec.rear_port_offset * hub_dir
        head_goal = VehicleState(x=float(head_goal_center[0]), y=float(head_goal_center[1]), theta=hub.heading)

        static_for_head = [self.vehicles[vid] for vid in trailer_ids]
        ok, reason = self._plan_and_track(
            head,
            head_goal,
            static_for_head,
            phase="head_reposition",
            mode="path_replay",
        )
        if not ok:
            return SimulationResult(
                success=False,
                message=f"Head reposition failed ({reason})",
                failure_code=reason,
                decision=decision,
                frames=self.frames,
                world=self.world,
                vehicles=self.vehicles,
                metrics=_metrics(0),
            )

        free_trailers = {vid: self.vehicles[vid] for vid in trailer_ids}
        docked = 0

        while free_trailers:
            preferred = [vid for vid in decision.sequence if vid in free_trailers]
            preferred.extend([vid for vid in free_trailers.keys() if vid not in preferred])
            vid = self._select_next_trailer(train=train, free_trailers=free_trailers, preferred_order=preferred)
            if vid is None:
                return SimulationResult(
                    success=False,
                    message="Docking failed (PathPlanningFail)",
                    failure_code="PathPlanningFail",
                    decision=decision,
                    frames=self.frames,
                    world=self.world,
                    vehicles=self.vehicles,
                    metrics=_metrics(docked),
                )
            active = free_trailers[vid]
            other_static = [v for k, v in free_trailers.items() if k != vid]
            ok, reason = self._dock_vehicle(active=active, train=train, other_static=other_static, phase=f"dock_{vid}")
            if not ok:
                return SimulationResult(
                    success=False,
                    message=f"Docking failed for {vid} ({reason})",
                    failure_code=reason,
                    decision=decision,
                    frames=self.frames,
                    world=self.world,
                    vehicles=self.vehicles,
                    metrics=_metrics(docked),
                )
            train.attach_tail(active)
            docked += 1
            del free_trailers[vid]
            self._record(f"{vid}_attached", train=train)

        if docked != 4:
            return SimulationResult(
                success=False,
                message="Not all trailers docked",
                failure_code="PathPlanningFail",
                decision=decision,
                frames=self.frames,
                world=self.world,
                vehicles=self.vehicles,
                metrics=_metrics(docked),
            )

        motion_ok, reason = self._run_train_motion_demo(train=train)
        if not motion_ok:
            return SimulationResult(
                success=False,
                message=f"Final train motion failed ({reason})",
                failure_code=reason,
                decision=decision,
                frames=self.frames,
                world=self.world,
                vehicles=self.vehicles,
                metrics=_metrics(docked),
            )

        return SimulationResult(
            success=True,
            message="All 5 vehicles docked and train motion completed",
            failure_code=None,
            decision=decision,
            frames=self.frames,
            world=self.world,
            vehicles=self.vehicles,
            metrics={
                "frames": float(len(self.frames)),
                **_metrics(docked),
            },
        )
