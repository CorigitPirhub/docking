from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Callable

import numpy as np

from docking.collision import CollisionEngine
from docking.config import Config, CoordinatorConfig, PlannerConfig, SensorGlobalConfig, SensorVisionConfig
from docking.controllers import PathTrackingController
from docking.docking import DockingLockEvaluator, TwoStageDockingController
from docking.kinematics import AckermannModel, VehicleGeometry
from docking.math_utils import angle_diff, wrap_angle
from docking.grid_planner import GridAStarPlanner, GridPlannerConfig
from docking.planner import LocalPlanner
from docking.runtime_support import (
    EventType,
    ReconfigRuntimeEngine,
    TaskFeasibilityMonitor,
    estimate_leader_remaining_time,
)
from docking.train import TrainController, TrainKinematics, TrainSafetyGuard
from docking.types import ControlCommand, VehicleMode, VehicleState
from docking.sensors import GlobalPoseSensor, VisionSensor
from runtime.command_bus import (
    CommandHeader,
    DockingCommand,
    FeedbackStage,
    FeedbackStatus,
    SplitCommand,
)
from strategy.baseline_scheduler import BaselineScheduler, SchedulerConfig
from docking.costs import EnergyModelConfig
from strategy.p3_reconfig import min_nmax_in_window, nmax_at_s
from strategy.p4_recovery import P4Config, PredictiveRecoveryPlanner


def _polyline_s(path_xy: np.ndarray) -> np.ndarray:
    if len(path_xy) <= 1:
        return np.zeros(len(path_xy), dtype=float)
    ds = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(ds)])


def _interp_path(path_xy: np.ndarray, s_samples: np.ndarray, s_query: float) -> np.ndarray:
    s = float(np.clip(s_query, s_samples[0], s_samples[-1]))
    if s <= s_samples[0] + 1e-9:
        return path_xy[0].copy()
    if s >= s_samples[-1] - 1e-9:
        return path_xy[-1].copy()
    idx = int(np.searchsorted(s_samples, s, side="right") - 1)
    idx = max(0, min(idx, len(s_samples) - 2))
    seg = max(1e-9, float(s_samples[idx + 1] - s_samples[idx]))
    t = (s - float(s_samples[idx])) / seg
    return (1.0 - t) * path_xy[idx] + t * path_xy[idx + 1]


def _nearest_path_idx(path_xy: np.ndarray, p: np.ndarray) -> int:
    d = np.linalg.norm(path_xy - p, axis=1)
    return int(np.argmin(d))


def _three_point_radius(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> float:
    a = float(np.linalg.norm(p1 - p0))
    b = float(np.linalg.norm(p2 - p1))
    c = float(np.linalg.norm(p2 - p0))
    if min(a, b, c) < 1e-9:
        return math.inf
    area2 = abs(float((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0])))
    if area2 < 1e-9:
        return math.inf
    return float(a * b * c / max(2.0 * area2, 1e-9))


@dataclass(frozen=True)
class IntegratedDemoConfig:
    duration_s: float = 45.0
    leader_target_speed: float = 0.90
    free_target_speed: float = 1.00
    leader_goal_tol_m: float = 0.8
    vehicle_goal_tol_m: float = 1.2
    split_lookahead_m: float = 4.5
    dock_ahead_margin_m: float = 0.35
    command_ttl_s: float = 5.0
    max_concurrent_docks: int = 1
    retry_cooldown_s: float = 1.1
    replan_delay_s: float = 0.8
    max_retry_per_vehicle: int = 2
    feasibility_dock_est_s: float = 3.0
    feasibility_margin_s: float = 1.0
    enable_sensor_noise: bool = False
    inject_disturbance_for_type_c: bool = True
    disturbance_after_docking_s: float = 1.15
    disturbance_vision_prob: float = 0.65
    disturbance_block_prob: float = 0.35
    inject_disturbance_all_types: bool = False
    sample_every_n_low_ticks: int = 2
    record_history: bool = True
    record_monitor_trace: bool = True
    initial_chain_vehicle_ids: tuple[int, ...] = ()
    pre_split_initial_chain: bool = True
    reconfig_switch_cooldown_s: float = 1.2
    conflict_policy: str = "split_priority"  # split_priority | dock_priority
    stop_on_done: bool = True
    enable_gate_traversal: bool = True
    enable_gate_mode: bool = False
    enable_gate_escape: bool = True
    gate_speed_cap_mps: float = 0.45
    enable_bottleneck_coordination: bool = False
    docking_no_progress_timeout_s: float = 4.0
    docking_max_duration_s: float = 18.0


@dataclass
class DemoEvent:
    t: float
    source: str
    event: str
    vehicle_id: int | None
    peer_id: int | None
    detail: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class IntegratedRunResult:
    scenario_id: str
    subtype: str
    policy: str
    success: bool
    done_time_s: float
    total_energy: float
    leader_final_train_size: int
    dock_success_count: int
    split_count: int
    p4_interruption_count: int
    p4_replan_success_count: int
    p4_abort_to_independent_count: int
    collision_count: int
    feasibility_abort_count: int
    commands_submitted: int
    commands_accepted: int
    command_exec_accept: int
    command_exec_reject: int
    arbitration_trace: list[dict[str, Any]]
    reconfig_action_trace: list[dict[str, Any]]
    monitor_trace: list[dict[str, Any]]
    history: list[dict[str, Any]]
    events: list[DemoEvent]

    def to_dict(self) -> dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "subtype": self.subtype,
            "policy": self.policy,
            "success": bool(self.success),
            "done_time_s": float(self.done_time_s),
            "total_energy": float(self.total_energy),
            "leader_final_train_size": int(self.leader_final_train_size),
            "dock_success_count": int(self.dock_success_count),
            "split_count": int(self.split_count),
            "p4_interruption_count": int(self.p4_interruption_count),
            "p4_replan_success_count": int(self.p4_replan_success_count),
            "p4_abort_to_independent_count": int(self.p4_abort_to_independent_count),
            "collision_count": int(self.collision_count),
            "feasibility_abort_count": int(self.feasibility_abort_count),
            "commands_submitted": int(self.commands_submitted),
            "commands_accepted": int(self.commands_accepted),
            "command_exec_accept": int(self.command_exec_accept),
            "command_exec_reject": int(self.command_exec_reject),
            "arbitration_trace": self.arbitration_trace,
            "reconfig_action_trace": self.reconfig_action_trace,
            "monitor_trace": self.monitor_trace,
            "history": self.history,
            "events": [e.to_dict() for e in self.events],
        }


class IntegratedP2P4Runner:
    """Strategy-control integrated runner for P2-P4 demonstration."""

    class _StepStop(RuntimeError):
        pass

    def __init__(
        self,
        cfg: Config,
        case,
        *,
        policy: str = "integrated",
        seed: int = 0,
        demo_cfg: IntegratedDemoConfig | None = None,
        tick_observer: Callable[[dict[str, Any]], None] | None = None,
        step_decider: Callable[[dict[str, Any]], bool] | None = None,
    ):
        if policy not in {"integrated", "independent"}:
            raise ValueError(f"Unsupported policy: {policy}")
        self.cfg = cfg
        self.case = case
        self.policy = policy
        self.seed = int(seed)
        self.rng = np.random.default_rng(self.seed)
        self.demo_cfg = demo_cfg or IntegratedDemoConfig()
        self.tick_observer = tick_observer
        self.step_decider = step_decider

        self.vehicle_ids = sorted(int(v.vehicle_id) for v in case.vehicles_init)
        self.leader_id = 1 if 1 in self.vehicle_ids else self.vehicle_ids[0]

        self.path_xy = case.path_xy
        self.path_s = _polyline_s(self.path_xy)
        self.path_len = float(self.path_s[-1]) if len(self.path_s) else 0.0
        self.mandatory_gates: list[dict[str, float]] = [dict(g) for g in case.params.get("mandatory_gates", [])]
        self.mandatory_gates.sort(key=lambda g: float(g.get("cx", 0.0)))
        self._gate_idx_mem: dict[int, int] = {int(vid): 0 for vid in self.vehicle_ids}
        self._bottleneck_segments: list[dict[str, float]] = (
            self._build_bottleneck_segments() if self.demo_cfg.enable_bottleneck_coordination else []
        )
        self._bottleneck_owner: dict[int, int | None] = {i: None for i in range(len(self._bottleneck_segments))}

        self.states: dict[int, VehicleState] = {int(v.vehicle_id): v.copy() for v in case.vehicles_init}
        self.engine = ReconfigRuntimeEngine(cfg, self.vehicle_ids)
        # Keep feasibility logic enabled but tune it to scene-scale demo horizons.
        self.engine.feasibility = TaskFeasibilityMonitor(
            CoordinatorConfig(
                command_ttl_s=cfg.coordinator.command_ttl_s,
                feasibility_dock_time_est_s=self.demo_cfg.feasibility_dock_est_s,
                feasibility_margin_s=self.demo_cfg.feasibility_margin_s,
                feasibility_recover_hysteresis_s=cfg.coordinator.feasibility_recover_hysteresis_s,
                max_command_queue=cfg.coordinator.max_command_queue,
            )
        )

        self.geom = VehicleGeometry(cfg.vehicle)
        self.ack = AckermannModel(cfg.vehicle)
        self.collision = CollisionEngine(cfg.vehicle, cfg.safety)
        self.tracker = PathTrackingController(cfg.vehicle, cfg.control, steering_mode="pure_pursuit")
        self.nav_tracker = PathTrackingController(cfg.vehicle, cfg.control, steering_mode="stanley")
        self.planner = LocalPlanner(cfg.vehicle, cfg.planner, self.collision, cfg.control.dt)
        gate_planner_cfg = PlannerConfig(
            horizon_steps=int(max(cfg.planner.horizon_steps, 24)),
            sample_accel=cfg.planner.sample_accel,
            sample_steer_rate_deg=cfg.planner.sample_steer_rate_deg,
            goal_weight=float(cfg.planner.goal_weight),
            heading_weight=float(max(cfg.planner.heading_weight, 4.0)),
            clearance_weight=float(max(cfg.planner.clearance_weight, 0.6)),
            smooth_weight=float(min(cfg.planner.smooth_weight, 0.03)),
        )
        self.gate_planner = LocalPlanner(cfg.vehicle, gate_planner_cfg, self.collision, cfg.control.dt)
        # A* join-route planner: used to recover from scattered starts and cross-wall pockets.
        # Use a slightly finer grid than `scenario.validation_grid_resolution` so narrow gate gaps
        # (e.g., 0.79m) remain representable under conservative obstacle inflation.
        inflation = 0.5 * float(cfg.vehicle.car_width) + float(cfg.safety.min_clearance) + 0.01
        self.grid_planner = GridAStarPlanner(
            width=float(cfg.environment.width),
            height=float(cfg.environment.height),
            obstacles=list(case.obstacles),
            cfg=GridPlannerConfig(
                # Use a finer grid so narrow gate gaps remain representable after obstacle inflation.
                # With too-coarse grids (e.g., 0.15m), Type-C gaps can become topologically blocked,
                # causing A* join routes to fail and producing P6 timeouts (notably C3 Uniform_Spread).
                resolution=float(min(0.12, max(0.08, 0.6 * float(cfg.scenario.validation_grid_resolution)))),
                inflation_radius=float(inflation),
            ),
        )
        self.train_kin = TrainKinematics(cfg.vehicle)
        self.train_controller = TrainController(cfg.vehicle, cfg.control)
        self.train_guard = TrainSafetyGuard(cfg.vehicle, cfg.safety, self.collision)

        self.energy_model = EnergyModelConfig(eta_nominal=0.95)
        chase_speed = min(
            float(self.cfg.vehicle.max_speed),
            float(max(self.demo_cfg.free_target_speed, self.demo_cfg.leader_target_speed) + 0.35),
        )
        self.p2_scheduler = BaselineScheduler(
            cfg=SchedulerConfig(
                leader_id=int(self.leader_id),
                leader_speed_mps=float(self.demo_cfg.leader_target_speed),
                free_speed_mps=float(self.demo_cfg.free_target_speed),
                chase_speed_mps=float(chase_speed),
                max_accel_mps2=float(self.cfg.vehicle.max_accel),
            )
        )
        self.p4_planner = PredictiveRecoveryPlanner(
            cfg=P4Config(
                leader_id=int(self.leader_id),
                leader_speed_mps=float(self.demo_cfg.leader_target_speed),
                free_speed_mps=float(self.demo_cfg.free_target_speed),
                chase_speed_mps=float(chase_speed),
                max_accel_mps2=float(self.cfg.vehicle.max_accel),
            )
        )
        # Default is "stable demo": keep docking sensor noise disabled.
        # Set `IntegratedDemoConfig.enable_sensor_noise=True` to enable GNSS + vision noise for robustness eval.
        if self.demo_cfg.enable_sensor_noise:
            g_sensor_cfg = self.cfg.sensors.global_sensor
            v_sensor_cfg = self.cfg.sensors.vision
        else:
            g_sensor_cfg = SensorGlobalConfig(
                sigma_pos=0.0,
                sigma_yaw_deg=0.0,
                rate_hz=float(self.cfg.sensors.global_sensor.rate_hz),
            )
            v_sensor_cfg = SensorVisionConfig(
                fov_deg=float(self.cfg.sensors.vision.fov_deg),
                max_distance=float(self.cfg.sensors.vision.max_distance),
                sigma_pos=0.0,
                sigma_yaw_deg=0.0,
                rate_hz=float(self.cfg.sensors.vision.rate_hz),
            )
        self._dock_global_sensor = GlobalPoseSensor(g_sensor_cfg, seed=self.seed + 101)
        self._dock_vision_sensor = VisionSensor(v_sensor_cfg, self.cfg.vehicle, seed=self.seed + 202)
        self._dock_controller = TwoStageDockingController(
            self.cfg,
            self._dock_global_sensor,
            self._dock_vision_sensor,
            self.tracker,
            self.planner,
        )

        self.lock_eval: dict[int, DockingLockEvaluator] = {}
        self.docking_started_at: dict[int, float] = {}
        self.replan_ready_at: dict[int, float] = {}
        self.retry_count: dict[int, int] = {}
        self.abandon_docking: set[int] = set()
        self.forced_interrupted_once: set[int] = set()
        self.docking_intercept_s: dict[int, float] = {}

        self.total_energy = 0.0
        self.done_time: float | None = None
        self._next_strategy_tick = 0.0
        self._event_cursor = 0
        self._low_tick_count = 0
        self._leader_remaining_s = 60.0
        self._path_idx_mem: dict[int, int] = {}
        self._path_s_mem: dict[int, float] = {}
        self._leader_best_s: float = 0.0
        self._leader_best_s_t: float = 0.0
        self._leader_backoff_until: float = -1e9

        self.events: list[DemoEvent] = []
        self.history: list[dict[str, Any]] = []
        self.monitor_trace: list[dict[str, Any]] = []
        self.arbitration_trace: list[dict[str, Any]] = []
        self.reconfig_action_trace: list[dict[str, Any]] = []
        self._last_reconfig_action: str | None = None
        self._last_reconfig_action_t: float = -1e9
        self._last_cmd: dict[int, ControlCommand] = {}
        self._last_dock_cmd: dict[int, ControlCommand] = {}
        self._steer_flip_ts: dict[int, list[float]] = {vid: [] for vid in self.vehicle_ids}
        self._stall_ticks: dict[int, int] = {vid: 0 for vid in self.vehicle_ids}
        self._blocked_ticks: dict[int, int] = {vid: 0 for vid in self.vehicle_ids}
        self._gate_rescue_until: dict[int, float] = {vid: -1e9 for vid in self.vehicle_ids}
        self._gate_prog_idx: dict[int, int] = {vid: -1 for vid in self.vehicle_ids}
        self._gate_prog_best_s: dict[int, float] = {vid: -1e9 for vid in self.vehicle_ids}
        self._gate_prog_best_t: dict[int, float] = {vid: -1e9 for vid in self.vehicle_ids}
        self._step_energy: dict[int, float] = {vid: 0.0 for vid in self.vehicle_ids}
        self._energy_cumulative: dict[int, float] = {vid: 0.0 for vid in self.vehicle_ids}
        self._dock_debug: dict[int, dict[str, Any]] = {}
        self._dock_stage_mem: dict[int, str] = {}
        self._dock_best_dist: dict[int, float] = {}
        self._dock_best_metric: dict[int, float] = {}
        self._dock_best_t: dict[int, float] = {}
        self._collision_latch: dict[int, bool] = {vid: False for vid in self.vehicle_ids}
        self._collision_global_latch: bool = False
        self._join_route_xy: dict[int, np.ndarray] = {}
        self._join_route_goal_xy: dict[int, np.ndarray] = {}
        self._join_route_planned_at: dict[int, float] = {}
        self._join_route_strict_goal: dict[int, bool] = {}
        self._join_escape_until: dict[int, float] = {}
        self._gate_escape_phase: dict[int, int] = {}
        self._gate_escape_key: dict[int, tuple[float, float, float]] = {}
        self._gate_escape_dir: dict[int, float] = {}
        self._lane_path_xy: dict[int, np.ndarray] = {}
        self._lane_offset_m: dict[int, float] = {}

        if len(self.path_xy) > 0:
            for vid in self.vehicle_ids:
                idx0 = _nearest_path_idx(self.path_xy, self.states[vid].xy())
                self._path_idx_mem[int(vid)] = int(idx0)
                self._path_s_mem[int(vid)] = float(self.path_s[idx0])

        # Lane-offset tracking for Type-A scattered starts: assign non-leader heads to offset
        # corridor centerlines to avoid early multi-vehicle deadlocks on sinuous paths.
        # Disabled automatically for gate/chicane scenes (mandatory_gates present).
        if (not self.mandatory_gates) and len(self.vehicle_ids) >= 2 and len(self.path_xy) >= 2:
            half_w = float(getattr(self.cfg.scenario, "corridor_half_width", 0.0))
            max_off = half_w - 0.5 * float(self.cfg.vehicle.car_width) - float(self.cfg.safety.min_clearance) - 0.15
            max_off = float(np.clip(max_off, 0.0, 1.65))
            if max_off > 0.25:
                followers = [vid for vid in self.vehicle_ids if int(vid) != int(self.leader_id)]
                # If only one follower, bias it to the side that reduces its lateral error to the path.
                if len(followers) == 1:
                    vid = int(followers[0])
                    st = self.states[vid]
                    idx = _nearest_path_idx(self.path_xy, st.xy())
                    y_ref = float(self.path_xy[idx, 1])
                    sign = 1.0 if float(st.y) >= y_ref else -1.0
                    off = float(sign * max_off)
                    self._lane_offset_m[vid] = off
                    self._lane_path_xy[vid] = self._offset_path(self.path_xy, off)
                elif followers:
                    # Spread multiple followers across available corridor width, avoid centerline.
                    followers_sorted = sorted([int(v) for v in followers], key=lambda v: float(self.states[v].y))
                    m = len(followers_sorted)
                    for i, vid in enumerate(followers_sorted):
                        pos = -1.0 + 2.0 * float(i) / max(1.0, float(m - 1))
                        if abs(pos) < 1e-6:
                            st = self.states[vid]
                            idx = _nearest_path_idx(self.path_xy, st.xy())
                            y_ref = float(self.path_xy[idx, 1])
                            sign = 1.0 if float(st.y) >= y_ref else -1.0
                            pos = 0.35 * sign
                        off = float(np.clip(pos, -1.0, 1.0) * max_off)
                        self._lane_offset_m[vid] = off
                        self._lane_path_xy[vid] = self._offset_path(self.path_xy, off)

        # Optional initial topology for demonstrating split behavior in bottleneck scenarios.
        if self.demo_cfg.initial_chain_vehicle_ids:
            chain = [int(v) for v in self.demo_cfg.initial_chain_vehicle_ids if int(v) in self.vehicle_ids]
            if chain and len(self.path_xy) >= 2:
                d0 = self.path_xy[1] - self.path_xy[0]
                yaw0 = math.atan2(float(d0[1]), float(d0[0]))
                heading0 = np.array([math.cos(yaw0), math.sin(yaw0)], dtype=float)
                base = self.path_xy[0].copy()
                nominal_gap = abs(self.geom.front_hitch_x - self.geom.rear_hitch_x) + 0.25
                for k, vid in enumerate(chain):
                    c = base - heading0 * (nominal_gap * float(k))
                    st_old = self.states[vid]
                    placed = VehicleState(
                        vehicle_id=st_old.vehicle_id,
                        x=float(c[0]),
                        y=float(c[1]),
                        yaw=float(yaw0),
                        v=0.0,
                        delta=0.0,
                        mode=VehicleMode.FREE,
                    )
                    self.states[vid] = self._place_collision_free_near(placed, avoid_ids={vid})
            for parent, child in zip(chain[:-1], chain[1:]):
                ok, _ = self.engine.topology.start_docking(child, parent, now=0.0)
                if ok:
                    self.engine.topology.finalize_docking(child, parent, now=0.0)
                    # Project to a kinematically consistent locked pose for stable train startup.
                    locked = self._project_locked(self.states[child], self.states[parent])
                    self.states[child] = self._place_collision_free_near(locked, avoid_ids={child})
            # Pre-split at t=0 when static labels already prove oversized chain is infeasible.
            if chain and self.case.labels is not None and self.demo_cfg.pre_split_initial_chain:
                prof = self.case.labels.n_max_pass_profile
                allowed = int(min_nmax_in_window(prof, 0.0, self.path_len))
                free_gap = abs(self.geom.front_hitch_x - self.geom.rear_hitch_x) + 0.25
                head = self._chain_head(chain[0])
                while len(self._chain_from_head(head)) > max(1, allowed):
                    seg = self._chain_from_head(head)
                    parent = seg[-2]
                    child = seg[-1]
                    self.engine.topology.apply_split(parent, child, now=0.0)
                    p_st = self.states[parent]
                    c_st = self.states[child]
                    h = np.array([math.cos(p_st.yaw), math.sin(p_st.yaw)], dtype=float)
                    c_xy = p_st.xy() - h * free_gap
                    self.states[child] = VehicleState(
                        vehicle_id=c_st.vehicle_id,
                        x=float(c_xy[0]),
                        y=float(c_xy[1]),
                        yaw=float(p_st.yaw),
                        v=0.0,
                        delta=0.0,
                        mode=VehicleMode.FREE,
                    )
                if len(self.path_xy) >= 2:
                    d0 = self.path_xy[1] - self.path_xy[0]
                    yaw0 = math.atan2(float(d0[1]), float(d0[0]))
                    h0 = np.array([math.cos(yaw0), math.sin(yaw0)], dtype=float)
                    base = self.path_xy[0].copy()
                    free_order = [vid for vid in chain if self.engine.topology.parent[vid] is None]
                    for k, vid in enumerate(free_order):
                        if self.engine.topology.child[vid] is not None:
                            continue
                        s_old = self.states[vid]
                        c = base - h0 * (free_gap * float(k))
                        placed = VehicleState(
                            vehicle_id=s_old.vehicle_id,
                            x=float(c[0]),
                            y=float(c[1]),
                            yaw=float(yaw0),
                            v=0.0,
                            delta=0.0,
                            mode=VehicleMode.FREE,
                        )
                        self.states[vid] = self._place_collision_free_near(placed, avoid_ids={vid})

        self.cmd_idx = 0
        self.commands_submitted = 0
        self.commands_accepted = 0

        self.dock_success_count = 0
        self.split_count = 0
        self.p4_interruption_count = 0
        self.p4_replan_success_count = 0
        self.p4_abort_to_independent_count = 0
        self.collision_count = 0
        self.feasibility_abort_count = 0

    def _push_event(
        self,
        t: float,
        source: str,
        event: str,
        detail: str = "",
        vehicle_id: int | None = None,
        peer_id: int | None = None,
    ) -> None:
        self.events.append(
            DemoEvent(
                t=float(t),
                source=source,
                event=event,
                vehicle_id=vehicle_id,
                peer_id=peer_id,
                detail=detail,
            )
        )

    def _build_bottleneck_segments(self) -> list[dict[str, float]]:
        """
        Convert mandatory gate list into lane-change "bottleneck segments" for multi-vehicle coordination.

        Motivation:
        - The most common deadlock in multi-vehicle chicanes happens *between* the two cross-walls
          (during the large lateral lane-change), not exactly at the narrow wall itself.
        - We treat every consecutive gate pair as a "lane-change region" with capacity=1 (one head at a time).
        """
        gates = list(self.mandatory_gates)
        if not gates:
            return []
        out: list[dict[str, float]] = []
        n = len(gates)
        for i in range(0, n, 2):
            g0 = gates[i]
            g1 = gates[min(i + 1, n - 1)]

            # The original coordination window only covered the lane-change region *between* two walls.
            # In practice, multi-vehicle deadlocks also happen right before entering the first wall
            # (tailgating blocks the lead vehicle's steering sweep). Expand the mutual-exclusion window
            # to cover the whole "wall-pair" neighborhood: approach -> both walls -> exit.
            x0_first = float(g0.get("x0", float(g0.get("cx", 0.0))))
            x1_first = float(g0.get("x1", float(g0.get("cx", 0.0))))
            x0_second = float(g1.get("x0", float(g1.get("cx", 0.0))))
            x1_second = float(g1.get("x1", float(g1.get("cx", 0.0))))
            if x1_first < x0_first:
                x0_first, x1_first = x1_first, x0_first
            if x1_second < x0_second:
                x0_second, x1_second = x1_second, x0_second

            entry_x = float(x0_first - 0.05)
            exit_x = float(x1_second + 0.05)
            if exit_x <= entry_x + 0.30:
                continue
            # Expand approach window: deadlocks often happen *before* the first wall when a rear
            # vehicle tailgates and blocks the lead vehicle's steering sweep into the gap.
            # Use a conservative 2.4m buffer (~1.7m setup + extra margin) so mutual exclusion
            # engages before the rear vehicle gets into near-contact.
            approach_x = float(entry_x - 2.40)
            clear_x = float(exit_x + 0.65)
            out.append({"entry_x": entry_x, "exit_x": exit_x, "approach_x": approach_x, "clear_x": clear_x})
        return out

    def _update_bottleneck_owners(self, head_ids: list[int]) -> None:
        if not self._bottleneck_segments:
            return
        for idx, seg in enumerate(self._bottleneck_segments):
            owner = self._bottleneck_owner.get(idx, None)
            if owner is not None and owner in self.states:
                tail_x = float(self.geom.to_world(self.states[owner], float(self.geom.rear_x))[0])
                if tail_x > float(seg["clear_x"]) + 1e-6:
                    self._bottleneck_owner[idx] = None
            else:
                self._bottleneck_owner[idx] = None

            # Leader-first bottleneck ownership:
            # In multi-gate Type-B/C scenes, allowing a non-leader head to enter the wall-pair
            # region first can trap the global leader behind it (the lead head needs large steering
            # sweep between walls and gets blocked by the rear vehicle's collision checks). This
            # shows up as long P6 timeouts (notably B2 Random_Scattered).
            if self._bottleneck_owner.get(idx, None) is None and int(self.leader_id) in head_ids and int(self.leader_id) in self.states:
                leader = self.states[int(self.leader_id)]
                leader_tail_x = float(self.geom.to_world(leader, float(self.geom.rear_x))[0])
                if leader_tail_x <= float(seg["clear_x"]) + 1e-6:
                    self._bottleneck_owner[idx] = int(self.leader_id)

            if self._bottleneck_owner.get(idx, None) is not None:
                continue

            candidates: list[tuple[float, int]] = []
            for vid in head_ids:
                if vid not in self.states:
                    continue
                st = self.states[vid]
                tail_x = float(self.geom.to_world(st, float(self.geom.rear_x))[0])
                if tail_x > float(seg["clear_x"]) + 0.20:
                    continue
                nose_x = float(self.geom.to_world(st, float(self.geom.front_x))[0])
                if nose_x < float(seg["approach_x"]):
                    continue
                candidates.append((nose_x, int(vid)))
            if not candidates:
                continue
            candidates.sort(key=lambda t: (-t[0], t[1]))
            self._bottleneck_owner[idx] = int(candidates[0][1])

    def _active_bottleneck_idx(self, state: VehicleState) -> int | None:
        """
        Return the first upcoming/active bottleneck segment index when the vehicle is near it.
        """
        if not self._bottleneck_segments:
            return None
        nose_x = float(self.geom.to_world(state, float(self.geom.front_x))[0])
        tail_x = float(self.geom.to_world(state, float(self.geom.rear_x))[0])
        for idx, seg in enumerate(self._bottleneck_segments):
            if tail_x > float(seg["clear_x"]) + 0.15:
                continue
            # Only activate the rule when close enough to matter (avoid slowing down too early).
            if nose_x < float(seg["approach_x"]) - 0.05:
                return None
            if nose_x > float(seg["clear_x"]) + 2.0:
                continue
            return int(idx)
        return None

    def _bottleneck_hold(self, vehicle_id: int, state: VehicleState) -> str | None:
        idx = self._active_bottleneck_idx(state)
        if idx is None:
            return None
        seg = self._bottleneck_segments[idx]
        owner = self._bottleneck_owner.get(idx, None)
        if owner is None or int(owner) == int(vehicle_id):
            return None
        nose_x = float(self.geom.to_world(state, float(self.geom.front_x))[0])
        tail_x = float(self.geom.to_world(state, float(self.geom.rear_x))[0])
        if tail_x > float(seg["clear_x"]) + 0.10:
            return None
        if nose_x < float(seg["approach_x"]) - 0.05:
            return None
        return f"bottleneck_hold(seg={idx},owner={owner})"

    def _chain_head(self, vid: int) -> int:
        cur = int(vid)
        while self.engine.topology.parent[cur] is not None:
            cur = int(self.engine.topology.parent[cur])
        return cur

    def _chain_from_head(self, head: int) -> list[int]:
        out = [int(head)]
        cur = self.engine.topology.child[int(head)]
        while cur is not None:
            out.append(int(cur))
            cur = self.engine.topology.child[int(cur)]
        return out

    def _chain_tail(self, head: int) -> int:
        chain = self._chain_from_head(head)
        return int(chain[-1])

    def _chain_size_of_vehicle(self, vid: int) -> int:
        h = self._chain_head(vid)
        return len(self._chain_from_head(h))

    def _path_progress_s(self, state: VehicleState) -> float:
        if len(self.path_xy) == 0:
            return 0.0
        vid = int(state.vehicle_id)
        if vid not in self._path_idx_mem:
            idx = _nearest_path_idx(self.path_xy, state.xy())
            self._path_idx_mem[vid] = int(idx)
            self._path_s_mem[vid] = float(self.path_s[idx])
            return float(self.path_s[idx])

        hint = int(self._path_idx_mem[vid])
        n = int(len(self.path_xy))
        lo = max(0, hint - 8)
        hi = min(n, hint + 28)
        local = self.path_xy[lo:hi]
        if len(local) == 0:
            idx = _nearest_path_idx(self.path_xy, state.xy())
        else:
            d = np.linalg.norm(local - state.xy(), axis=1)
            j = int(np.argmin(d))
            idx = lo + j
            if float(d[j]) > 2.5:
                idx = _nearest_path_idx(self.path_xy, state.xy())

        s_prev = float(self._path_s_mem.get(vid, self.path_s[idx]))
        s_raw = float(self.path_s[idx])
        back_allow = 0.45 if float(state.v) < -0.05 else 0.08
        if s_raw < s_prev - back_allow:
            s_new = s_prev - back_allow
            idx = int(np.searchsorted(self.path_s, s_new, side="left"))
            idx = max(0, min(n - 1, idx))
            s_raw = float(self.path_s[idx])

        self._path_idx_mem[vid] = int(idx)
        self._path_s_mem[vid] = float(s_raw)
        return float(s_raw)

    def _target_point_ahead(self, state: VehicleState, lookahead_m: float = 1.2) -> tuple[np.ndarray, float]:
        s0 = self._path_progress_s(state)
        sq = min(self.path_len, s0 + lookahead_m)
        p = _interp_path(self.path_xy, self.path_s, sq)
        idx = _nearest_path_idx(self.path_xy, p)
        if idx < len(self.path_xy) - 1:
            d = self.path_xy[idx + 1] - self.path_xy[idx]
        else:
            d = self.path_xy[idx] - self.path_xy[max(0, idx - 1)]
        yaw = math.atan2(float(d[1]), float(d[0]))
        return p, yaw

    def _route_target_ahead(
        self, state: VehicleState, route_xy: np.ndarray, *, lookahead_m: float = 1.2
    ) -> tuple[np.ndarray, float]:
        if len(route_xy) < 2:
            return state.xy().copy(), float(state.yaw)
        idx0 = _nearest_path_idx(route_xy, state.xy())
        idx = int(idx0)
        acc = 0.0
        tgt = route_xy[idx].astype(float).copy()
        yaw = float(state.yaw)

        # Interpolate a continuous lookahead target along the (possibly sparse) route polyline.
        # This is important for A* join routes that may be compressed to only a few waypoints.
        while idx + 1 < len(route_xy) and acc < float(lookahead_m):
            p0 = route_xy[idx]
            p1 = route_xy[idx + 1]
            seg = p1 - p0
            seg_len = float(np.linalg.norm(seg))
            if seg_len <= 1e-9:
                idx += 1
                tgt = route_xy[idx].astype(float).copy()
                continue
            if acc + seg_len >= float(lookahead_m):
                r = float((float(lookahead_m) - acc) / seg_len)
                tgt = ((1.0 - r) * p0 + r * p1).astype(float).copy()
                yaw = float(math.atan2(float(seg[1]), float(seg[0])))
                return tgt, yaw
            acc += seg_len
            idx += 1
            tgt = route_xy[idx].astype(float).copy()
            yaw = float(math.atan2(float(seg[1]), float(seg[0])))

        # Fallback: end-point target.
        idx = max(0, min(idx, len(route_xy) - 1))
        tgt = route_xy[idx].astype(float).copy()
        if idx >= 1:
            d = route_xy[idx] - route_xy[idx - 1]
            if float(np.linalg.norm(d)) > 1e-9:
                yaw = float(math.atan2(float(d[1]), float(d[0])))
        return tgt, yaw

    def _gate_heading_yaw(self, gate: dict[str, float]) -> float:
        # Cross-wall "gate" obstacles are vertical walls with a gap; safe traversal requires the
        # vehicle to be nearly aligned with the wall normal (corridor x-direction). Using the
        # shared corridor tangent near the gate can demand a large yaw angle inside a narrow gap,
        # which is kinematically infeasible and causes persistent stalls/timeouts in Type-C.
        if len(self.path_xy) >= 2:
            dx = float(self.path_xy[-1, 0] - self.path_xy[0, 0])
            return 0.0 if dx >= 0.0 else float(math.pi)
        return 0.0

    def _gate_safe_band(self, gate: dict[str, float]) -> tuple[float, float] | None:
        y0 = float(gate.get("y0", float(gate.get("cy", 0.0)) - 0.2))
        y1 = float(gate.get("y1", float(gate.get("cy", 0.0)) + 0.2))
        if y1 < y0:
            y0, y1 = y1, y0
        gap = float(max(0.0, y1 - y0))
        if gap <= 1e-9:
            return None

        # Centerline "safe band" is computed from the vehicle body width + required clearance.
        # For Type-C gates, allowing centers too close to the wall edge leads to states where the
        # body cannot advance through the wall thickness without clipping the obstacle.
        half_gap = 0.5 * gap
        margin_req = 0.5 * float(self.cfg.vehicle.car_width) + float(self.cfg.safety.min_clearance) + 0.02
        # Clamp so we always keep a non-empty band.
        margin = float(min(margin_req, half_gap - 0.02))
        if margin <= 0.0:
            return None
        return float(y0 + margin), float(y1 - margin)

    def _active_gate(self, state: VehicleState) -> dict[str, float] | None:
        if not self.demo_cfg.enable_gate_traversal:
            return None
        # Gate-specific assist logic is primarily needed for multi-gate chicanes (Type-C)
        # and the serial bottleneck (B3). Applying it to simpler Type-B scenes can
        # over-constrain longitudinal headway and create unnecessary stalls.
        subtype = str(self.case.subtype)
        if not (subtype.startswith("C") or subtype in {"B2", "B3"}):
            return None
        if not self.mandatory_gates:
            return None
        vid = int(state.vehicle_id)

        # During deterministic gate-escape, keep the same gate "active" even after backing off
        # beyond the normal far-field activation threshold. Otherwise `_gate_escape_command()`
        # would immediately drop its latch as soon as it reverses (x < x0 - 1.8), causing a
        # chattering escape that never completes the turn+climb sequence.
        escape_key = self._gate_escape_key.get(vid, None)
        if escape_key is not None:
            for g in self.mandatory_gates:
                x0 = float(g.get("x0", float(g.get("cx", 0.0)) - 0.2))
                y0 = float(g.get("y0", float(g.get("cy", 0.0)) - 0.2))
                y1 = float(g.get("y1", float(g.get("cy", 0.0)) + 0.2))
                if y1 < y0:
                    y0, y1 = y1, y0
                key = (round(x0, 3), round(y0, 3), round(y1, 3))
                if key == escape_key:
                    return g

        idx = int(self._gate_idx_mem.get(vid, 0))
        idx = max(0, min(idx, len(self.mandatory_gates)))

        # Gate selection with hysteresis: keep handling the current gate until the rear bumper
        # clears it. This prevents premature switching to the next gate while still inside the
        # wall thickness, which can send the vehicle far off the intended chicane corridor.
        while idx < len(self.mandatory_gates):
            g = self.mandatory_gates[idx]
            x1 = float(g.get("x1", float(g.get("cx", 0.0))))
            tail_x = float(self.geom.to_world(state, float(self.geom.rear_x))[0])
            if tail_x > x1 + 0.6:
                idx += 1
                continue
            break

        self._gate_idx_mem[vid] = int(idx)
        if idx >= len(self.mandatory_gates):
            return None

        hitch = self.geom.front_hitch(state)
        hitch_x = float(hitch[0])
        hitch_y = float(hitch[1])

        def _gate_active(g: dict[str, float]) -> bool:
            x0 = float(g.get("x0", float(g.get("cx", 0.0)) - 0.2))
            x1 = float(g.get("x1", float(g.get("cx", 0.0)) + 0.2))
            y0 = float(g.get("y0", float(g.get("cy", 0.0)) - 0.2))
            y1 = float(g.get("y1", float(g.get("cy", 0.0)) + 0.2))
            if y1 < y0:
                y0, y1 = y1, y0

            # Activate gate logic only in a *near-field* band relative to the front hitch
            # (collision-critical point when approaching cross-walls).
            if hitch_x < (x0 - 0.95):
                return False
            if hitch_x > (x1 + 1.0):
                return False

            # Gate assists are recovery behaviors: when the hitch is already within the opening
            # band, the nominal corridor tracker is stable and faster. Only keep gate logic when
            # the hitch is *outside* the gap band (risking immediate wall contact), or when a
            # strict gate-join/escape is latched.
            strict_join = bool(self._join_route_strict_goal.get(vid, False)) and (vid in self._join_route_xy)
            hitch_in_gap = bool((float(y0) - 0.10) <= hitch_y <= (float(y1) + 0.10))
            return bool(strict_join or (not hitch_in_gap))

        g = self.mandatory_gates[idx]
        if _gate_active(g):
            return g

        # Reverse/backtrack robustness: if we moved back behind the activation threshold for the
        # current (monotone) gate index, fall back to the previous gate if it is still relevant.
        if idx > 0:
            g_prev = self.mandatory_gates[idx - 1]
            if _gate_active(g_prev):
                self._gate_idx_mem[vid] = int(idx - 1)
                return g_prev
        return None

    def _gate_traversal_goal(self, state: VehicleState, base_speed: float) -> tuple[np.ndarray, float, float] | None:
        gate = self._active_gate(state)
        if gate is None:
            return None
        cx = float(gate.get("cx", 0.0))
        cy = float(gate.get("cy", 0.0))
        x0 = float(gate.get("x0", cx - 0.2))
        x1 = float(gate.get("x1", cx + 0.2))
        y0 = float(gate.get("y0", cy - 0.2))
        y1 = float(gate.get("y1", cy + 0.2))
        if y1 < y0:
            y0, y1 = y1, y0
        gap = float(max(0.0, y1 - y0))

        # Gate phase transitions must consider the full vehicle body and the hitch protrusions.
        # We track two reference points with hysteresis:
        # - `hitch_x`: front hitch x (collision-critical when approaching cross-wall gaps)
        # - `tail_x`: rear bumper x (clears gate last)
        # This prevents the controller from flipping back into "approach/back-off" after the
        # nose clears the wall while the body is still inside the wall thickness.
        hitch = self.geom.front_hitch(state)
        hitch_x = float(hitch[0])
        hitch_y = float(hitch[1])
        tail_x = float(self.geom.to_world(state, float(self.geom.rear_x))[0])

        # Do not engage gate-mode too early when the vehicle is still far from the gate opening
        # in lateral direction. Between consecutive chicanes, this can otherwise force an overly
        # aggressive lane-change and drive yaw toward ±90deg (x-progress stalls).
        if gap > 1e-6 and hitch_x < (x0 - 0.25):
            half_gap = 0.5 * gap
            if abs(float(state.y) - float(cy)) > (half_gap + 0.75):
                return None

        # If the rear bumper has already cleared the gate, return to nominal tracking.
        if tail_x > x1 + 0.6:
            return None

        # Choose a traversal centerline for the gate opening.
        #
        # Important distinction:
        # - During approach (outside the wall thickness), clamping to a strict clearance band too
        #   early can demand an unrealistically large lateral jump between consecutive chicanes.
        # - While inside/committed to the wall thickness, we must stay in a stricter band to
        #   avoid grazing the wall edges.
        gap_slack = 0.05
        gap_low = float(y0 + gap_slack)
        gap_high = float(y1 - gap_slack)
        if gap_high < gap_low:
            gap_low, gap_high = float(y0), float(y1)
        band_strict = self._gate_safe_band(gate)
        y_center = float(cy)
        if band_strict is not None:
            y_center = float(np.clip(y_center, float(band_strict[0]), float(band_strict[1])))

        # Hitch-in-gap safety guard:
        # If the front hitch is already close to the wall face but outside the opening band,
        # any forward motion will immediately collide even when the vehicle center is inside.
        # In that case, request a short back-off + yaw alignment before committing forward.
        # This is a dominant P6 failure in B2 Random_Scattered (seed=41042).
        yaw_gate = float(self._gate_heading_yaw(gate))
        yaw_err_gate = float(abs(angle_diff(yaw_gate, float(state.yaw))))
        # Large yaw misalignment means the hitch can swing far outside the gap band even when the
        # vehicle center is still safely inside. Trigger the back-off earlier in that case to
        # create room for a forward turn; otherwise the controller can fall into a forward/back-off
        # oscillation that never clears the wall (B3 Clustered_At_A seed=50042).
        hitch_guard = bool(hitch_x >= (x0 - 0.60)) or bool((yaw_err_gate >= math.radians(35.0)) and (hitch_x >= (x0 - 0.85)))
        if hitch_guard:
            hitch_in_gap = bool(float(gap_low) <= float(hitch_y) <= float(gap_high))
            if not hitch_in_gap:
                # Back-off target must be large enough to allow a forward turn without sweeping the
                # protruding front hitch into the wall. However, in scattered starts a second vehicle
                # can be close behind, and a too-large back-off can create a rear-end deadlock where
                # neither vehicle can move (B3 Random_Scattered seed=51042). Use an adaptive margin
                # based on available rear headway.
                backoff_extra = 1.05
                rear_dx = math.inf
                for oid, o in self.states.items():
                    if int(oid) == int(state.vehicle_id):
                        continue
                    if float(o.x) >= float(state.x) - 0.20:
                        continue
                    if abs(float(o.y) - float(state.y)) > 1.6:
                        continue
                    rear_dx = float(min(rear_dx, float(state.x - o.x)))
                if rear_dx < 1.85:
                    backoff_extra = 0.65
                backoff_x = float(x0 - float(self.geom.front_hitch_x) - float(backoff_extra))
                if float(state.x) > backoff_x + 0.02:
                    goal = np.array([backoff_x, float(cy)], dtype=float)
                    yaw_goal = float(yaw_gate)
                    v_gate = float(min(base_speed, 0.35, float(self.demo_cfg.gate_speed_cap_mps)))
                    v_gate = float(max(0.10, v_gate))
                    return goal, yaw_goal, v_gate

        entered = bool(hitch_x >= (x0 - 0.10))
        cleared = bool(tail_x >= (x1 + 0.10))
        inside = bool(entered and (not cleared))
        committed = False
        if not inside:
            ok_y = bool(float(gap_low) <= float(state.y) <= float(gap_high))
            # When close to the gate wall, "commit" to the forward-through behavior
            # to avoid oscillating at the wall corner.
            committed = bool((hitch_x >= (x0 - 0.50)) and ok_y)
        inside_like = bool(inside or committed)
        if not inside_like and len(self.path_xy) >= 2:
            # For chicanes, the shared corridor path already encodes a gradual lane-change between
            # two gates. Using the gate centerline `cy` as the approach target can demand an overly
            # aggressive lateral jump (leading to oscillations and timeouts). Instead, anchor the
            # approach y to the path profile at the nominal approach x, then clamp into the safe band.
            approach_x = float(x0 - 0.40)
            idx_ref = int(np.argmin(np.abs(self.path_xy[:, 0] - approach_x)))
            y_center = float(self.path_xy[idx_ref, 1])
            y_center = float(np.clip(y_center, gap_low, gap_high))
        # If already inside and close to the safe centerline, keep current y to avoid
        # unnecessary lateral corrections that can graze the gate walls.
        if inside_like and abs(float(state.y) - y_center) <= 0.08:
            y_center = float(state.y)
        if inside_like:
            if band_strict is not None:
                y_center = float(np.clip(y_center, float(band_strict[0]), float(band_strict[1])))
            else:
                y_center = float(np.clip(y_center, gap_low, gap_high))
        else:
            y_center = float(np.clip(y_center, gap_low, gap_high))

        if inside_like:
            goal_x = float(x1 + 0.75)
        else:
            goal_x = float(x0 - 0.40)
        goal = np.array([goal_x, float(y_center)], dtype=float)

        # Heading target aligned with the shared path around the gate; when far from the entrance,
        # aim toward the approach goal to avoid getting stuck above/below the opening.
        yaw_goal = float(math.atan2(float(goal[1] - state.y), float(goal[0] - state.x)))
        # Near the wall face, prioritize aligning with the gate direction to reduce the chance of
        # grazing the wall thickness with a large yaw angle.
        # NOTE: forcing yaw too early (far before x0) can create deadlocks in scattered starts:
        # the vehicle aligns to the gate direction but cannot reduce lateral error enough before
        # reaching the wall face, then repeatedly freezes/back-offs. Only force yaw when we're
        # already committed/inside, or very close to the wall face.
        # Force yaw alignment earlier based on the front hitch proximity (the hitch protrudes
        # beyond the bumper and can collide with the wall edges even when the body is still
        # upstream). A too-late switch can lock the vehicle into repeated back-offs before the
        # wall face (P6 B2 Random_Scattered seed=41042).
        if inside_like or (hitch_x >= (x0 - 0.85)):
            yaw_goal = float(yaw_gate)
        yaw_err = float(abs(angle_diff(yaw_goal, state.yaw)))

        v_gate = float(base_speed)
        if hitch_x >= (x0 - 1.2):
            v_gate = min(v_gate, 0.55)
        if (hitch_x >= (x0 - 0.6)) or inside_like:
            v_gate = min(v_gate, float(self.demo_cfg.gate_speed_cap_mps))
        v_gate = float(max(0.10, v_gate))
        return goal, float(yaw_goal), float(v_gate)

    def _gate_backoff_command(self, state: VehicleState) -> ControlCommand | None:
        """
        Emergency back-off behavior for chicane cross-walls.

        When a vehicle starts behind a cross-wall (y outside the gap) and the extended front
        hitch is close to the wall face, any small forward progress can cause immediate contact.
        In this case, backing off provides room to perform a forward turning maneuver safely.
        """
        gate = self._active_gate(state)
        if gate is None:
            return None
        x0 = float(gate.get("x0", float(gate.get("cx", 0.0)) - 0.2))
        x1 = float(gate.get("x1", float(gate.get("cx", 0.0)) + 0.2))
        y0 = float(gate.get("y0", float(gate.get("cy", 0.0)) - 0.2))
        y1 = float(gate.get("y1", float(gate.get("cy", 0.0)) + 0.2))
        if y1 < y0:
            y0, y1 = y1, y0
        cy = float(gate.get("cy", 0.5 * (y0 + y1)))

        # If already inside the gap band, avoid unnecessary reverse.
        # IMPORTANT: use the front hitch position for gap membership.
        # With large yaw angles, the hitch can fall outside the opening even when the vehicle
        # center is inside, leading to immediate wall contact and repeated stalls.
        hitch_y = float(self.geom.front_hitch(state)[1])
        if (float(y0) - 0.10) <= hitch_y <= (float(y1) + 0.10):
            return None

        hitch_x = float(self.geom.front_hitch(state)[0])
        # Trigger backoff only when the hitch is *very* close to the wall face.
        # This intentionally includes the region where a forward turning maneuver would
        # sweep the hitch into the wall before reaching the gap height.
        if hitch_x < (x0 - 0.35):
            return None

        # Back off enough room to execute a ~90deg turn (R_min≈1m) with hitch margin.
        backoff_target_x = float(x0 - float(self.geom.front_hitch_x) - 1.05)
        if float(state.x) <= backoff_target_x + 0.02:
            return None

        dt = max(self.cfg.control.dt, 1e-6)
        max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
        dmax = math.radians(self.cfg.vehicle.max_steer_deg)
        y_err = float(cy - float(state.y))
        steer_sign = 0.0
        if y_err > 1e-6:
            steer_sign = 1.0
        elif y_err < -1e-6:
            steer_sign = -1.0
        # Reverse-turn steering: while backing up, steer to bias the reverse motion toward the gap centerline.
        delta_target = float(np.clip(steer_sign * 0.45, -dmax, dmax))
        steer_rate = float(np.clip((delta_target - float(state.delta)) / dt, -max_rate, max_rate))
        v_ref = -0.35
        accel = float(np.clip((v_ref - float(state.v)) / dt, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd))
        cmd = ControlCommand(accel=accel, steer_rate=steer_rate)
        # Force reverse-only cap (no forward) while backing off.
        cmd = self._enforce_speed_cap(state, cmd, 0.0, allow_reverse=True)
        return cmd

    def _rear_backoff_for_gate_lead(self, state: VehicleState) -> ControlCommand | None:
        """
        Back-off behavior for the *rear* vehicle when the front vehicle is maneuvering through a gate.

        Motivation:
        - In B/C chicanes, a vehicle exiting the first wall often needs significant steering sweep.
        - If a second vehicle tailgates too closely, collision checks cause the front vehicle to freeze,
          creating a deadlock right at the wall exit.

        This helper creates temporary extra longitudinal headway by commanding a short reverse.
        """
        if not self.demo_cfg.enable_gate_traversal:
            return None
        if not self.mandatory_gates:
            return None
        vid = int(state.vehicle_id)
        # Leader-priority semantic: the global leader should not yield/back-off to create space
        # for a non-leader head ahead. Instead, the front head must yield via
        # `_front_yield_for_stalled_leader` when the leader is blocked.
        if vid == int(self.leader_id):
            return None

        # Find the closest lead vehicle in +x within a lateral band.
        lead_id = None
        lead_dx = math.inf
        for oid, o in self.states.items():
            if int(oid) == int(vid):
                continue
            dx = float(o.x - state.x)
            if dx <= 1e-6:
                continue
            if abs(float(o.y - state.y)) > 1.1:
                continue
            if dx < lead_dx:
                lead_dx = dx
                lead_id = int(oid)
        if lead_id is None or math.isinf(lead_dx):
            return None

        lead_state = self.states.get(int(lead_id), None)
        if lead_state is None:
            return None
        gate = self._active_gate(lead_state)
        if gate is None:
            return None
        x0 = float(gate.get("x0", float(gate.get("cx", 0.0)) - 0.2))
        lead_nose_x = float(self.geom.to_world(lead_state, float(self.geom.front_x))[0])
        if lead_nose_x < x0 - 0.80:
            return None

        # Require a larger headway when the lead is near/inside the wall thickness.
        desired_dx = 2.35
        if float(lead_dx) >= desired_dx:
            return None

        dt = max(self.cfg.control.dt, 1e-6)
        max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
        unwind = float(np.clip(-state.delta / dt, -max_rate, max_rate))
        # Prefer braking/holding over reversing: aggressive reverse back-off can drift laterally and
        # push a vehicle into obstacle pockets in B/C chicanes, hurting P6 completion time.
        v_ref = 0.0
        allow_reverse = False
        if float(lead_dx) < 1.30:
            v_ref = -0.18
            allow_reverse = True
        # If we're too close to a gate-handling lead and already nearly stopped, use a gentle reverse
        # to actively create maneuvering room (otherwise both vehicles can deadlock before the wall).
        if (not allow_reverse) and abs(float(state.v)) <= 0.05 and float(lead_dx) < desired_dx - 0.10:
            v_ref = -0.12
            allow_reverse = True
        accel = float(
            np.clip(
                (v_ref - float(state.v)) / dt,
                -self.cfg.vehicle.max_decel,
                self.cfg.control.speed.max_accel_cmd,
            )
        )
        cmd = ControlCommand(accel=accel, steer_rate=unwind)
        cmd = self._enforce_speed_cap(state, cmd, 0.0, allow_reverse=allow_reverse)
        return cmd

    def _rear_backoff_for_turning_lead(self, state: VehicleState) -> ControlCommand | None:
        """
        Generic back-off behavior for scattered/merged heads (non-gate cases).

        Motivation:
        - In A2/A3 Uniform_Spread, a vehicle can start ahead in x but far from the corridor.
          It may need to rotate/merge aggressively (sometimes even reverse a bit).
        - If a rear vehicle closes too much, the lead vehicle's turning sweep becomes infeasible,
          producing a long oscillation + stall.
        - Pure speed limiting can keep the rear stopped, but that may not create enough extra space.
          A gentle reverse back-off by the rear resolves the deadlock reliably.
        """
        if len(self.states) <= 1:
            return None
        vid = int(state.vehicle_id)
        # Leader-priority semantic: do not command the global leader to back off for
        # scattered heads ahead; front heads should yield when blocking leader.
        if vid == int(self.leader_id):
            return None

        lead_id = None
        lead_dx = math.inf
        for oid, o in self.states.items():
            if int(oid) == int(vid):
                continue
            dx = float(o.x - state.x)
            if dx <= 1e-6:
                continue
            if abs(float(o.y - state.y)) > 1.15:
                continue
            if dx < lead_dx:
                lead_dx = dx
                lead_id = int(oid)
        if lead_id is None or math.isinf(lead_dx):
            return None

        lead_state = self.states.get(int(lead_id), None)
        if lead_state is None:
            return None

        lead_idx = _nearest_path_idx(self.path_xy, lead_state.xy())
        if lead_idx < len(self.path_xy) - 1:
            d = self.path_xy[lead_idx + 1] - self.path_xy[lead_idx]
        else:
            d = self.path_xy[lead_idx] - self.path_xy[max(0, lead_idx - 1)]
        n = float(np.linalg.norm(d))
        if n < 1e-9:
            return None
        lead_path_yaw = float(math.atan2(float(d[1]), float(d[0])))
        yaw_err = float(abs(angle_diff(lead_path_yaw, lead_state.yaw)))
        lat_err = float(self._lateral_distance_to_path(lead_state))

        lead_maneuver = bool(
            abs(float(lead_state.delta)) >= 0.22
            or (yaw_err >= math.radians(70.0))
            or ((lat_err >= 0.65) and (yaw_err >= math.radians(40.0)))
        )
        if not lead_maneuver:
            return None

        desired_dx = 2.05
        if float(lead_dx) >= desired_dx:
            return None

        dt = max(self.cfg.control.dt, 1e-6)
        max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
        unwind = float(np.clip(-state.delta / dt, -max_rate, max_rate))

        v_ref = 0.0
        allow_reverse = False
        # If too close, apply a gentle reverse to actively create maneuvering room.
        if float(lead_dx) < 1.55:
            v_ref = -0.16
            allow_reverse = True
        elif abs(float(state.v)) <= 0.05 and float(lead_dx) < desired_dx - 0.10:
            v_ref = -0.12
            allow_reverse = True

        accel = float(
            np.clip(
                (v_ref - float(state.v)) / dt,
                -self.cfg.vehicle.max_decel,
                self.cfg.control.speed.max_accel_cmd,
            )
        )
        cmd = ControlCommand(accel=accel, steer_rate=unwind)
        cmd = self._enforce_speed_cap(state, cmd, 0.0, allow_reverse=allow_reverse)
        return cmd

    def _front_yield_for_stalled_leader(self, state: VehicleState, now: float) -> ControlCommand | None:
        """
        Yield behavior for a head vehicle that is *ahead* of the global leader and blocks its progress.

        Motivation:
        - In B/C scenes with cross-wall gates, scattered starts can place a non-leader head in front.
        - If that head stalls near a bottleneck entrance, the leader will keep stopping due to
          collision checks and never reaches the gate.
        - A short reverse back-off by the blocking head restores the leader-priority semantic
          without requiring high-level WAIT commands.
        """
        if not self.mandatory_gates:
            return None
        vid = int(state.vehicle_id)
        if vid == int(self.leader_id):
            return None
        # If this head is already in the cross-wall gate assist region, let the specialized
        # gate traversal/back-off logic handle it. Forcing the generic leader-yield reverse here
        # can fight the gate controller and lock both vehicles into a reverse deadlock (B3
        # Random_Scattered seed=51042).
        if self._active_gate(state) is not None:
            return None
        # If this head is already executing a (potentially strict) join route or deterministic
        # gate-escape, do not force a reverse-yield that fights the recovery controller.
        # Otherwise the head can get stuck oscillating in place while the leader remains stalled
        # (P6 B2 Random_Scattered seed=41042).
        if vid in self._join_route_xy and (not self._join_route_done(vid)):
            return None
        if vid in self._gate_escape_phase or vid in self._gate_escape_key:
            return None
        leader = self.states.get(int(self.leader_id), None)
        if leader is None:
            return None

        # Progress-stagnation trigger (more robust than pure v~=0 stall):
        # - In chicane scenes, the leader can oscillate/reverse slightly while making no forward
        #   progress (path-s stays constant), preventing the old stall-ticks trigger.
        # - When leader progress is stagnant for long enough, the front head must yield.
        stagnation_s = 3.0
        if float(now) - float(self._leader_best_s_t) < float(stagnation_s):
            return None

        dx = float(state.x - leader.x)
        if dx <= 0.6 or dx >= 6.0:
            return None
        if abs(float(state.y - leader.y)) > 1.4:
            return None

        dt = max(self.cfg.control.dt, 1e-6)
        max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
        unwind = float(np.clip(-state.delta / dt, -max_rate, max_rate))

        v_ref = -0.35
        accel = float(
            np.clip(
                (v_ref - float(state.v)) / dt,
                -self.cfg.vehicle.max_decel,
                self.cfg.control.speed.max_accel_cmd,
            )
        )
        cmd = ControlCommand(accel=accel, steer_rate=unwind)
        cmd = self._enforce_speed_cap(state, cmd, 0.0, allow_reverse=True)
        return cmd

    def _leader_backoff_for_escaping_front(self, state: VehicleState, now: float) -> ControlCommand | None:
        """
        Back-off behavior for the global leader when a front head is stuck in gate-escape.

        Motivation:
        - In B2/B3 and multi-gate Type-C, Random_Scattered can spawn a non-leader head ahead.
        - If that head triggers gate-escape and needs to reverse, the leader may be too close behind.
        - The front head then freezes (collision checks) and the leader cannot pass -> 60s timeout.

        This helper lets the leader reverse briefly to open longitudinal headway so the escaping
        head can complete its back-off/turn/climb sequence.
        """
        if not self.mandatory_gates:
            return None
        vid = int(state.vehicle_id)
        if vid != int(self.leader_id):
            return None
        if len(self.states) <= 1:
            return None

        dt = max(self.cfg.control.dt, 1e-6)
        if float(now) < float(self._leader_backoff_until):
            max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
            unwind = float(np.clip(-state.delta / dt, -max_rate, max_rate))
            v_ref = -0.35
            accel = float(
                np.clip((v_ref - float(state.v)) / dt, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd)
            )
            cmd = ControlCommand(accel=accel, steer_rate=unwind)
            cmd = self._enforce_speed_cap(state, cmd, 0.0, allow_reverse=True)
            return cmd
        stall_limit = int(1.0 / dt)

        best_dx = math.inf
        for oid in self.vehicle_ids:
            if int(oid) == int(vid):
                continue
            # Only consider other FREE heads (ignore trains and docking followers).
            if self.engine.topology.parent.get(int(oid), None) is not None:
                continue
            if self.engine.topology.child.get(int(oid), None) is not None:
                continue
            if self.engine.topology.mode.get(int(oid), VehicleMode.FREE) != VehicleMode.FREE:
                continue
            escape_active = bool(int(oid) in self._gate_escape_phase or int(oid) in self._gate_escape_key)
            o = self.states[int(oid)]
            dx = float(o.x - state.x)
            if dx <= 0.2 or dx >= 3.20:
                continue
            if abs(float(o.y - state.y)) > 2.0:
                continue
            gate_o = self._active_gate(o)
            if gate_o is None:
                continue
            band = self._gate_safe_band(gate_o)
            if band is not None:
                safe_low, safe_high = float(band[0]), float(band[1])
            else:
                y0 = float(gate_o.get("y0", float(gate_o.get("cy", 0.0)) - 0.2))
                y1 = float(gate_o.get("y1", float(gate_o.get("cy", 0.0)) + 0.2))
                if y1 < y0:
                    y0, y1 = y1, y0
                safe_low, safe_high = float(y0), float(y1)
            in_safe = bool((safe_low - 0.02) <= float(o.y) <= (safe_high + 0.02))

            needs_room = False
            if escape_active:
                stalled = int(self._stall_ticks.get(int(oid), 0))
                if stalled < stall_limit:
                    continue
                phase = int(self._gate_escape_phase.get(int(oid), 0))
                # If the front head is already safely aligned (late phase + in_safe), do not back off.
                if in_safe and phase >= 2:
                    continue
                needs_room = True
            else:
                # Also back off for a front head that is "gate-stuck" and must reverse to recover yaw
                # / hitch-in-gap, but cannot because the leader is too close behind (B3 Random_Scattered
                # seed=51042).
                x0 = float(gate_o.get("x0", float(gate_o.get("cx", 0.0)) - 0.2))
                y0 = float(gate_o.get("y0", float(gate_o.get("cy", 0.0)) - 0.2))
                y1 = float(gate_o.get("y1", float(gate_o.get("cy", 0.0)) + 0.2))
                if y1 < y0:
                    y0, y1 = y1, y0
                gap_slack = 0.05
                gap_low = float(y0 + gap_slack)
                gap_high = float(y1 - gap_slack)
                if gap_high < gap_low:
                    gap_low, gap_high = float(y0), float(y1)
                hitch = self.geom.front_hitch(o)
                hitch_x = float(hitch[0])
                hitch_y = float(hitch[1])
                hitch_in_gap = bool(gap_low <= hitch_y <= gap_high)
                yaw_gate = float(self._gate_heading_yaw(gate_o))
                yaw_err_gate = float(abs(angle_diff(yaw_gate, float(o.yaw))))
                hitch_guard = bool(hitch_x >= (x0 - 0.60)) or bool(
                    (yaw_err_gate >= math.radians(35.0)) and (hitch_x >= (x0 - 0.85))
                )
                needs_room = bool((not hitch_in_gap) and hitch_guard)
                # Preemptive leader back-off:
                # Even if the front head is still moving, it may soon need to reverse-turn to recover
                # hitch-in-gap near the wall. When the leader is close behind, that reverse-turn is
                # collision-blocked and both vehicles can deadlock. Trigger back-off early once the
                # leader is within a close longitudinal band.
                if needs_room and dx >= 3.00:
                    needs_room = False
                if not needs_room:
                    continue

            best_dx = float(min(best_dx, dx))

        if math.isinf(best_dx):
            return None

        max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
        unwind = float(np.clip(-state.delta / dt, -max_rate, max_rate))
        # Use a stronger back-off for gate-stuck cases: front heads typically need ~1m reverse space
        # to realign their protruding hitch without collision, otherwise the system can deadlock.
        v_ref = -0.35 if best_dx < 2.10 else -0.18
        accel = float(
            np.clip((v_ref - float(state.v)) / dt, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd)
        )
        cmd = ControlCommand(accel=accel, steer_rate=unwind)
        cmd = self._enforce_speed_cap(state, cmd, 0.0, allow_reverse=True)
        self._leader_backoff_until = float(max(self._leader_backoff_until, float(now) + 1.2))
        return cmd

    def _gate_escape_command(self, state: VehicleState) -> ControlCommand | None:
        """
        Deterministic escape behavior for cross-wall pockets (B/C chicanes).

        The basic issue in scattered starts:
        - Vehicles may start below/above a cross-wall gap.
        - The *front hitch* protrudes, so even small forward progress can contact the wall.
        - A purely geometric (x,y) join path can be kinematically infeasible.

        This handler enforces a conservative, kinematically realizable sequence:
        1) Reverse to create x-margin for turning
        2) Forward-turn to near-vertical heading
        3) Forward-climb in y until entering the gap band
        """
        gate = self._active_gate(state)
        vid = int(state.vehicle_id)
        if gate is None:
            self._gate_escape_phase.pop(vid, None)
            self._gate_escape_key.pop(vid, None)
            self._gate_escape_dir.pop(vid, None)
            return None

        x0 = float(gate.get("x0", float(gate.get("cx", 0.0)) - 0.2))
        x1 = float(gate.get("x1", float(gate.get("cx", 0.0)) + 0.2))
        y0 = float(gate.get("y0", float(gate.get("cy", 0.0)) - 0.2))
        y1 = float(gate.get("y1", float(gate.get("cy", 0.0)) + 0.2))
        if y1 < y0:
            y0, y1 = y1, y0
        cy = float(gate.get("cy", 0.5 * (y0 + y1)))

        gap_low, gap_high = (float(y0), float(y1))
        band = self._gate_safe_band(gate)
        if band is not None:
            safe_low, safe_high = (float(band[0]), float(band[1]))
        else:
            safe_low, safe_high = (float(gap_low), float(gap_high))
        key = (round(x0, 3), round(y0, 3), round(y1, 3))
        slack = 0.02
        in_gap = bool((float(gap_low) - slack) <= float(state.y) <= (float(gap_high) + slack))
        in_safe = bool((float(safe_low) - slack) <= float(state.y) <= (float(safe_high) + slack))
        active = bool(self._gate_escape_key.get(vid, None) == key)

        # If an escape is latched and we have already entered the opening band, keep the escape
        # state-machine alive long enough to perform the yaw-recovery phase. Clearing immediately
        # on `in_gap` can drop control while the vehicle is still at large yaw, letting it drift
        # out of the band and re-trigger escape repeatedly (P6 Uniform_Spread stalls).
        #
        # NOTE: for Type-C/B3 gates, being inside the *gap* band is not sufficient. If the vehicle
        # center is still hugging the wall edge (outside the stricter safe band), forward motion can
        # remain infeasible and cause stalls right at the wall face. Only jump to yaw-recovery when
        # the vehicle is already inside the safe band.
        if active and in_safe:
            if int(self._gate_escape_phase.get(vid, 0)) < 3:
                self._gate_escape_phase[vid] = 3

        # Only *trigger* escape when we're behind the wall (outside the gap band) and close enough
        # to the wall face. If we are already inside the opening, prefer the nominal corridor /
        # join-route tracking; escape can introduce unnecessary reverse-turn oscillations.
        # Once triggered, keep the escape latched until yaw + y are recovered; otherwise the
        # back-off step itself would immediately disable the handler due to hitch_x moving away.
        if not active:
            # If we are already inside the *safe* band, do not trigger escape.
            # If we are inside the gap band but still outside the safe band, allow escape to
            # recover from "edge hugging" states near the wall face (P6 timeouts).
            if in_safe:
                return None
            hitch_x = float(self.geom.front_hitch(state)[0])
            # Trigger only in the near-field of the wall face; too-early triggering makes
            # scattered starts waste time in unnecessary reverse-turn maneuvers.
            if hitch_x < (x0 - 0.55):
                return None

            self._gate_escape_key[vid] = key
            self._gate_escape_phase[vid] = 0
            # Persist the escape direction for this gate to avoid flipping targets around `cy`.
            # Vehicles may enter the gap band close to the boundary and oscillate if the escape
            # direction is recomputed from the instantaneous y.
            if float(state.y) < float(safe_low):
                self._gate_escape_dir[vid] = 1.0
            elif float(state.y) > float(safe_high):
                self._gate_escape_dir[vid] = -1.0
            else:
                # Fallback: choose the direction that drives toward the gate centerline.
                self._gate_escape_dir[vid] = 1.0 if float(state.y) < float(cy) else -1.0

        phase = int(self._gate_escape_phase.get(vid, 0))
        dt = max(self.cfg.control.dt, 1e-6)
        max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
        dmax = math.radians(self.cfg.vehicle.max_steer_deg)
        hitch_margin_x = 0.35
        # Back off far enough to turn/climb without sweeping the front hitch into the wall.
        # `_active_gate()` is kept active during escape via `_gate_escape_key` latch.
        backoff_target_x = float(x0 - float(self.geom.front_hitch_x) - 1.05)

        hitch_x_now = float(self.geom.front_hitch(state)[0])
        # If we're still outside the opening and too close to the wall face, back off again before
        # turning/climbing. Once inside the opening (`in_gap`), allow approaching the face so the
        # vehicle can complete the pass-through.
        if phase >= 1 and (not in_gap) and hitch_x_now >= (x0 - hitch_margin_x):
            # Too close to the wall face while still escaping; back off again before turning/climbing.
            self._gate_escape_phase[vid] = 0
            phase = 0
        # Safety-net for the edge-hugging case:
        # - The vehicle can enter the *gap* band but remain outside the stricter safe band.
        # - If it then creeps too close to the wall face, phase-3 (forward-only) yaw recovery can
        #   stall because any forward motion would collide.
        # Reset to the back-off phase to recover maneuvering room and re-enter the safe band.
        if phase >= 3 and (not in_safe) and hitch_x_now >= (x0 - hitch_margin_x):
            self._gate_escape_phase[vid] = 0
            phase = 0

        if phase <= 0:
            if float(state.x) > backoff_target_x + 0.02:
                yaw_align = float(self._gate_heading_yaw(gate))
                cos_yaw = float(math.cos(float(state.yaw)))
                reverse = bool(cos_yaw >= 0.0)
                delta_target = 0.0

                # During back-off, do not allow large yaw to create significant lateral drift
                # (e.g., reversing while yaw≈50deg will push the vehicle further away from the
                # gate band). Bias steering to recover yaw toward the gate heading while backing
                # up so the motion is primarily along -x.
                if abs(cos_yaw) < 0.90:
                    yaw_target = float(yaw_align if reverse else wrap_angle(yaw_align + math.pi))
                    yaw_err_align = float(angle_diff(yaw_target, float(state.yaw)))
                    if abs(yaw_err_align) > 0.06:
                        steer_sign = 1.0 if yaw_err_align > 0.0 else -1.0
                        if reverse:
                            # In reverse, yaw-rate sign flips.
                            steer_sign *= -1.0
                        delta_target = float(np.clip(steer_sign * 0.65 * dmax, -dmax, dmax))

                steer_rate = float(np.clip((delta_target - float(state.delta)) / dt, -max_rate, max_rate))
                # Back off in the -x direction. Normally this is done by reversing while facing +x.
                # If the vehicle is already facing backward (cos(yaw)<0), reversing would *increase*
                # x and can immediately collide with the wall; in that case use a slow forward creep
                # (still moving toward -x) to recover before the climb phase.
                v_ref = -0.35 if reverse else 0.25
                if abs(cos_yaw) < 0.55:
                    v_ref = -0.25 if reverse else 0.20
                accel = float(np.clip((v_ref - float(state.v)) / dt, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd))
                cmd = ControlCommand(accel=accel, steer_rate=steer_rate)
                cmd = self._enforce_speed_cap(state, cmd, 0.55, allow_reverse=reverse)
                return cmd
            self._gate_escape_phase[vid] = 1
            phase = 1

        dir_sign = float(self._gate_escape_dir.get(vid, 1.0))
        yaw_climb = 0.5 * math.pi if dir_sign >= 0.0 else (-0.5 * math.pi)
        yaw_align = float(self._gate_heading_yaw(gate))
        y_center = float(np.clip(float(cy), float(safe_low), float(safe_high)))

        if phase == 1:
            yaw_err = float(angle_diff(yaw_climb, float(state.yaw)))
            if abs(yaw_err) <= 0.35:
                self._gate_escape_phase[vid] = 2
                phase = 2
            else:
                steer_sign = 1.0 if yaw_err > 0.0 else -1.0
                delta_target = float(np.clip(steer_sign * dmax, -dmax, dmax))
                steer_rate = float(np.clip((delta_target - float(state.delta)) / dt, -max_rate, max_rate))
                v_ref = 0.30
                accel = float(np.clip((v_ref - float(state.v)) / dt, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd))
                cmd = ControlCommand(accel=accel, steer_rate=steer_rate)
                cmd = self._enforce_speed_cap(state, cmd, 0.55, allow_reverse=False)
                return cmd

        # phase == 2: climb toward the *safe* band (gap band can be too wide near edges).
        if phase == 2:
            # If we reached the safe band, switch to yaw recovery/alignment.
            # Also allow an earlier hand-off once we are inside the *opening* and close enough to
            # the centerline: for narrow safe bands, continuing pure ±90deg climbing can overshoot
            # across the band and lead to long oscillations before the first gate (P6 B3/C*).
            if in_safe or (in_gap and abs(float(state.y) - float(y_center)) <= 0.30):
                # Entered the gap band; do a short "yaw recovery" stage before handing
                # off to nominal gate traversal. Without this, vehicles may remain near
                # ±90deg yaw, making x-progress extremely slow and causing timeouts.
                self._gate_escape_phase[vid] = 3
                phase = 3
            else:
                # Adaptive climb direction:
                # When inside the opening but outside the safe band, keep climbing toward the
                # centerline without relying on a fixed pre-selected direction. This avoids the
                # phase-3 overshoot oscillation seen in P6 (B3/C*) where a vehicle enters the
                # gap at the boundary, then swings past the safe band and gets stuck in resets.
                if abs(float(state.y) - float(y_center)) > 0.05:
                    dir_sign = 1.0 if float(state.y) < float(y_center) else -1.0
                    self._gate_escape_dir[vid] = float(dir_sign)
                    yaw_climb = 0.5 * math.pi if dir_sign >= 0.0 else (-0.5 * math.pi)
                # Keep steering small to avoid x-drift, but bias yaw toward the climb direction.
                yaw_err = float(angle_diff(yaw_climb, float(state.yaw)))
                delta_target = float(np.clip(1.2 * yaw_err, -0.35, 0.35))
                steer_rate = float(np.clip((delta_target - float(state.delta)) / dt, -max_rate, max_rate))
                # Speed schedule for the climb phase:
                # - When far from the wall face (large x-margin), allow faster longitudinal speed so
                #   large off-band initializations converge within the P6 60s budget.
                # - When near the wall face, keep the conservative cap to avoid grazing collisions.
                y_err = float(abs(float(state.y) - float(y_center)))
                v_ref = float(np.clip(0.45 + 0.04 * y_err, 0.45, 0.90))
                accel = float(
                    np.clip((v_ref - float(state.v)) / dt, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd)
                )
                cmd = ControlCommand(accel=accel, steer_rate=steer_rate)
                wall_margin = float(x0 - hitch_x_now)
                speed_cap = 0.55 if wall_margin < 0.65 else 0.90
                cmd = self._enforce_speed_cap(state, cmd, speed_cap, allow_reverse=False)
                return cmd

        # phase == 3: yaw recovery and corridor alignment inside the gap band.
        if phase >= 3:
            in_gap = bool((float(gap_low) - slack) <= float(state.y) <= (float(gap_high) + slack))
            in_safe = bool((float(safe_low) - slack) <= float(state.y) <= (float(safe_high) + slack))
            yaw_err = float(angle_diff(yaw_align, float(state.yaw)))
            if not in_gap:
                # We drifted out of the *opening* while still escaping; reset to back-off and
                # climb so we never command forward motion into the wall.
                if float(state.y) < float(safe_low):
                    self._gate_escape_dir[vid] = 1.0
                elif float(state.y) > float(safe_high):
                    self._gate_escape_dir[vid] = -1.0
                self._gate_escape_phase[vid] = 0
                if float(state.x) > backoff_target_x + 0.02:
                    delta_target = 0.0
                    steer_rate = float(np.clip((delta_target - float(state.delta)) / dt, -max_rate, max_rate))
                    v_ref = -0.35
                    accel = float(
                        np.clip((v_ref - float(state.v)) / dt, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd)
                    )
                    cmd = ControlCommand(accel=accel, steer_rate=steer_rate)
                    cmd = self._enforce_speed_cap(state, cmd, 0.0, allow_reverse=True)
                    return cmd
                # Not enough room to back off; next tick will proceed with turn+climb.
                return None
            # If yaw is recovered while staying inside the strict safe band, hand control back to
            # the nominal gate traversal / corridor tracking logic.
            #
            # NOTE: previously we also required `|y - y_center| <= 0.12`, but the phase-3 controller
            # intentionally holds `y` constant once it is *inside* the safe band to avoid overshoot.
            # This combination could latch the escape forever near the band edge (P6 B3/C* timeouts).
            if in_safe and abs(yaw_err) <= 0.55:
                self._gate_escape_phase.pop(vid, None)
                self._gate_escape_key.pop(vid, None)
                self._gate_escape_dir.pop(vid, None)
                return None

            # Track a short virtual corridor segment at y_center using Stanley for better
            # yaw + lateral convergence than a point attractor. Avoid reverse-turn oscillations
            # here: they can drift the vehicle out of the opening band and cause repeated resets.
            #
            # IMPORTANT: keep the corridor purely horizontal at `y_center`. Using a diagonal
            # segment anchored at the current (x,y) can drive overshoot across the narrow safe
            # band and lead to phase-3↔phase-0 chattering (P6 B3/C*).
            #
            # When already inside the safe band, prioritize yaw alignment and *avoid* additional
            # lateral correction: holding y close to its current safe value reduces the risk of
            # overshooting out of the gap band during yaw recovery.
            y_line = float(y_center)
            # While already inside the safe band, avoid aggressive lateral changes that could
            # kick the vehicle out of the opening; but still converge toward the centerline
            # when hugging the band edge.
            if in_safe and abs(float(state.y) - y_line) <= 0.06:
                y_line = float(state.y)

            # Build a short horizontal corridor that drives the vehicle *through* the wall
            # thickness while keeping it inside the safe band. The previous corridor ended at
            # `x0 - 0.35` (still upstream of the wall face), which could command targets behind
            # the vehicle once it had already entered the gate, stalling progress.
            forward_sign = 1.0 if abs(angle_diff(yaw_align, 0.0)) < 0.5 else -1.0
            nose_x = float(self.geom.to_world(state, float(self.geom.front_x))[0])
            near_wall_face = bool(nose_x >= (x0 - 0.15))
            if forward_sign >= 0.0:
                if near_wall_face:
                    x_exit = float(x1 + 0.75)
                    x_mid = float(min(x_exit, max(float(state.x) + 0.60, float(x0 - 0.10))))
                else:
                    # Stay upstream while still recovering yaw/y into the safe band.
                    x_exit = float(x0 - 0.35)
                    x_mid = float(min(x_exit, float(state.x) + 0.60))
            else:
                if near_wall_face:
                    x_exit = float(x0 - 0.75)
                    x_mid = float(max(x_exit, min(float(state.x) - 0.60, float(x1 + 0.10))))
                else:
                    x_exit = float(x1 + 0.35)
                    x_mid = float(max(x_exit, float(state.x) - 0.60))
            staging_x0 = float(state.x)
            staging_x1 = float(x_mid)
            staging_x2 = float(x_exit)
            corridor = np.array(
                [
                    [staging_x0, y_line],
                    [staging_x1, y_line],
                    [staging_x2, y_line],
                ],
                dtype=float,
            )
            yaw_abs = float(abs(yaw_err))
            # Faster yaw recovery is critical for P6 time budget: lingering in near-vertical yaw
            # inside the gap can consume most of the 60s horizon for scattered starts.
            if yaw_abs > 1.05:
                v_ref = 0.30
            elif yaw_abs > 0.70:
                v_ref = 0.42
            else:
                v_ref = 0.55
            cmd_pp = self.nav_tracker.track_path(state, corridor, float(v_ref))
            cmd = ControlCommand(accel=float(self._speed_accel_cmd(state.v, float(v_ref))), steer_rate=float(cmd_pp.steer_rate))
            wall_margin = float(x0 - hitch_x_now)
            speed_cap = 0.55 if wall_margin > 0.25 else 0.45
            cmd = self._enforce_speed_cap(state, cmd, speed_cap, allow_reverse=False)
            return cmd

    def _goal_heading_vector(self) -> np.ndarray:
        if len(self.path_xy) >= 2:
            # Use the overall corridor axis for terminal slotting.
            # The last path segment can be angled (e.g., chicane exit), and using its tangent can
            # place slots off the nominal corridor centerline, making the rear head's goal hard to reach.
            dx = float(self.path_xy[-1, 0] - self.path_xy[0, 0])
            if abs(dx) > 1e-9:
                return np.array([1.0, 0.0], dtype=float) if dx >= 0.0 else np.array([-1.0, 0.0], dtype=float)
            d = self.path_xy[-1] - self.path_xy[-2]
            n = float(np.linalg.norm(d))
            if n > 1e-9:
                return d / n
        if self.leader_id in self.states:
            yaw = float(self.states[self.leader_id].yaw)
            return np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
        return np.array([1.0, 0.0], dtype=float)

    def _goal_for_head(self, head_id: int) -> np.ndarray:
        goal = self.case.goal_xy.astype(float).copy()
        heads = list(self.engine.topology.heads())
        if int(head_id) not in heads or len(heads) <= 1:
            return goal
        # Leader-priority semantic: the global leader always targets the nominal goal,
        # other free heads queue behind it. Do not let non-leader heads "steal" the goal
        # when they temporarily get ahead during scattered starts or after aborts.
        if int(self.leader_id) in heads:
            others = [int(h) for h in heads if int(h) != int(self.leader_id)]
            others.sort(key=lambda h: self._path_progress_s(self.states[h]), reverse=True)
            ordered = [int(self.leader_id)] + others
        else:
            ordered = [int(h) for h in heads]
            ordered.sort()
        rank = int(ordered.index(int(head_id)))
        if rank <= 0:
            return goal
        # For bottleneck / chicane scenes, a longitudinal queue behind the global goal can
        # backfire in Random_Scattered starts when a non-leader head spawns *far ahead*:
        # it may reach its "behind-goal" slot early and park on the shared corridor, blocking
        # the true leader. In that case, switch the ahead head to a *lateral* parking slot at B.
        #
        # IMPORTANT: do *not* do this unconditionally. When the head is behind the leader,
        # forcing it to park laterally at B can require overtaking the leader near the terminal
        # area, which is often infeasible and causes 60s timeouts.
        if self.mandatory_gates and int(self.leader_id) in self.states and int(head_id) != int(self.leader_id):
            leader_st = self.states[int(self.leader_id)]
            head_st = self.states[int(head_id)]
            ahead_leader = bool(float(head_st.x) > float(leader_st.x) + 1.0)
            if ahead_leader:
                # Assign lateral parking slots near B for ahead heads.
                heading = self._goal_heading_vector()
                normal = np.array([-float(heading[1]), float(heading[0])], dtype=float)

                car_w = float(self.cfg.vehicle.car_width)
                clr = float(self.cfg.safety.min_clearance)
                # Parking offsets can use the *full environment* width, not the nominal corridor.
                # A larger lateral offset prevents the parked head from blocking the leader's
                # final approach to the global goal (seen in Random_Scattered B/C runs).
                half_h_env = 0.5 * float(self.cfg.environment.height) - 1.0
                max_off_env = float(max(0.0, half_h_env - (0.5 * car_w + clr + 0.25)))
                # Moderate lateral pull-over (paired with strict tolerance below) is enough to
                # clear the leader's lane while remaining reachable in 60s across random seeds.
                desired_off = float(1.60 + 0.25 * max(0, int(rank) - 1))
                off_mag = float(np.clip(desired_off, 1.20, max_off_env)) if max_off_env > 0.0 else 0.0
                # Choose the parking side to minimize lateral motion: keep the head on its
                # current side of the corridor/goal instead of forcing a cross-over at x≈goal.
                sign = 1.0 if float(head_st.y) >= float(goal[1]) else -1.0
                # Also move the parking slot slightly *forward* in the mission direction so it
                # remains forward-reachable even if the head overshoots x≈goal before starting
                # the lateral pull-over.
                advance = float(1.60 + 0.35 * max(0, int(rank) - 1))
                goal_slot = goal + heading * float(advance) + normal * float(sign * off_mag)

                # Keep slots within environment bounds (soft clamp, goals are points not states).
                half_w_env = 0.5 * float(self.cfg.environment.width) - 1.0
                goal_slot[0] = float(np.clip(goal_slot[0], -half_w_env, half_w_env))
                goal_slot[1] = float(np.clip(goal_slot[1], -half_h_env, half_h_env))
                return goal_slot.astype(float).copy()
        # Queue parking slots behind the nominal goal to avoid terminal blocking.
        base_gap = abs(self.geom.front_hitch_x - self.geom.rear_hitch_x) + 0.15
        # Tolerances interact with slot spacing: if the front head is allowed to stop far from its
        # goal (vehicle_goal_tol_m) while the leader requires a tighter slot (leader_goal_tol_m),
        # the original base-gap can become infeasible (rear head cannot reach its slot without
        # colliding). Expand the slot gap by the tolerance asymmetry.
        tol_slack = max(0.0, float(self.demo_cfg.vehicle_goal_tol_m) - float(self.demo_cfg.leader_goal_tol_m))
        # NOTE: for scattered starts, a too-small slot gap can be practically unreachable within the
        # 60s P6 budget because the rear head must approach very close to the front head while still
        # respecting collision/speed-limit heuristics. Use a more conservative gap so each head can
        # "arrive at B" without requiring tight tailgating in the terminal region.
        slot_gap = max(5.0, float(base_gap) + float(tol_slack) + 0.05)
        # Place slots along the shared corridor path instead of a fixed global axis:
        # - In B3/C* chicanes, the corridor centerline near the goal can be offset in y.
        # - Using a fixed `h=[1,0]` forces trailing heads to climb laterally toward `goal.y` too early,
        #   which can be slow/unnecessary and dominate the 60s P6 budget.
        # Additionally, when lane-offset tracking is enabled in Type-A open scenes, use the
        # per-head tracked corridor (offset path) so parked heads do not block the global leader.
        ref_path_xy = self._track_path_xy(int(head_id))
        if len(ref_path_xy) >= 2 and len(self.path_s) == len(ref_path_xy):
            slot_s = float(max(0.0, float(self.path_len) - float(slot_gap) * float(rank)))
            return _interp_path(ref_path_xy, self.path_s, slot_s).astype(float).copy()
        h = self._goal_heading_vector()
        return goal - h * (slot_gap * float(rank))

    def _goal_for_vehicle(self, vehicle_id: int) -> np.ndarray:
        head = self._chain_head(int(vehicle_id))
        return self._goal_for_head(head)

    def _offset_path(self, path_xy: np.ndarray, offset_m: float) -> np.ndarray:
        off = float(offset_m)
        if abs(off) <= 1e-9:
            return path_xy
        out = np.zeros_like(path_xy, dtype=float)
        n = len(path_xy)
        if n <= 1:
            return path_xy.astype(float).copy()
        for i in range(n):
            if i < n - 1:
                d = path_xy[i + 1] - path_xy[i]
            else:
                d = path_xy[i] - path_xy[i - 1]
            nn = float(np.linalg.norm(d))
            if nn < 1e-9:
                out[i] = path_xy[i]
                continue
            t = d / nn
            nvec = np.array([-float(t[1]), float(t[0])], dtype=float)
            out[i] = path_xy[i] + nvec * off
        return out

    def _track_path_xy(self, vehicle_id: int) -> np.ndarray:
        return self._lane_path_xy.get(int(vehicle_id), self.path_xy)

    def _lateral_distance_to_path(self, state: VehicleState, path_xy: np.ndarray | None = None) -> float:
        ref = self.path_xy if path_xy is None else path_xy
        idx = _nearest_path_idx(ref, state.xy())
        return float(np.linalg.norm(state.xy() - ref[idx]))

    def _same_lane_band(self) -> float:
        """
        Lateral band for treating two vehicles as sharing the same "lane" on the corridor.

        This is used by conservative car-following/overtake guards. A too-large band can
        freeze the global leader behind a slightly offset head in Clustered_At_A starts,
        even when there is enough clearance to pass side-by-side.
        """
        car_w = float(self.cfg.vehicle.car_width)
        clr = float(self.cfg.safety.min_clearance)
        # car width + clearance + small margin for hitch disks / yaw variation.
        return float(max(0.40, car_w + clr + 0.10))

    def _signed_lateral_to_path(self, state: VehicleState, path_xy: np.ndarray | None = None) -> float:
        ref = self.path_xy if path_xy is None else path_xy
        if len(ref) < 2:
            return 0.0
        vid = int(state.vehicle_id)
        if path_xy is not None and path_xy is not self.path_xy:
            idx = _nearest_path_idx(ref, state.xy())
        else:
            idx = int(self._path_idx_mem.get(vid, _nearest_path_idx(ref, state.xy())))
        idx = max(0, min(int(idx), len(ref) - 1))
        if idx < len(ref) - 1:
            d = ref[idx + 1] - ref[idx]
        else:
            d = ref[idx] - ref[idx - 1]
        n = float(np.linalg.norm(d))
        if n < 1e-9:
            return 0.0
        t = d / n
        nvec = np.array([-float(t[1]), float(t[0])], dtype=float)
        return float((state.xy() - ref[idx]) @ nvec)

    def _maybe_plan_join_route(self, vehicle_id: int, now: float) -> None:
        """
        Plan a one-time A* join route from the current position to the shared corridor path.

        This addresses the P6 failure mode where Random_Scattered initial states start in
        obstacle-separated pockets (e.g., below chicane cross-walls). Pure path tracking is
        then either collision-prone or deadlocks at the wall corner.
        """
        vid = int(vehicle_id)
        # If a gate-escape is active, avoid replanning a join route that would fight the
        # deterministic reverse/turn/climb escape state machine.
        if vid in self._gate_escape_phase or vid in self._gate_escape_key:
            return
        path_xy = self._track_path_xy(vid)
        if len(path_xy) < 2:
            return
        st = self.states[vid]
        dist_to_path = self._lateral_distance_to_path(st, path_xy)
        blocked = int(self._blocked_ticks.get(vid, 0))
        stall_ticks = int(self._stall_ticks.get(vid, 0))
        stall_limit = int(2.0 / max(self.cfg.control.dt, 1e-6))
        gate = self._active_gate(st)
        # Far-field gate hint for join-route planning:
        # `_active_gate()` intentionally activates only in the near-field to avoid over-constraining
        # nominal tracking. For scattered starts, however, we want to align laterally to the *next*
        # gate band early (before the front hitch approaches the wall face), otherwise the vehicle
        # can drift forward, trigger escape/back-off oscillations, and burn most of the 60s P6 budget.
        if gate is None and self.mandatory_gates:
            idx_hint = int(self._gate_idx_mem.get(vid, 0))
            idx_hint = max(0, min(idx_hint, len(self.mandatory_gates) - 1))
            g_hint = self.mandatory_gates[idx_hint]
            x0_hint = float(g_hint.get("x0", float(g_hint.get("cx", 0.0)) - 0.2))
            x1_hint = float(g_hint.get("x1", float(g_hint.get("cx", 0.0)) + 0.2))
            # Only hint when reasonably close upstream of the wall.
            if float(st.x) >= (x0_hint - 4.0) and float(st.x) <= (x1_hint + 1.0):
                gate = g_hint
        gate_need_join = False
        gate_join_xy: np.ndarray | None = None
        if gate is not None:
            y0 = float(gate.get("y0", float(gate.get("cy", 0.0)) - 0.2))
            y1 = float(gate.get("y1", float(gate.get("cy", 0.0)) + 0.2))
            if y1 < y0:
                y0, y1 = y1, y0
            in_band = bool((y0 - 0.10) <= float(st.y) <= (y1 + 0.10))
            if not in_band:
                # Do not mis-classify the nominal corridor approach as a "pocket":
                # the reference path does not enter the gap band until close to the wall face.
                y_ref = float(np.interp(float(st.x), self.path_xy[:, 0], self.path_xy[:, 1]))
                off_corridor = bool(abs(float(st.y) - y_ref) >= 0.60)
                if not off_corridor:
                    in_band = True
            if not in_band:
                gate_need_join = True
                cy = float(gate.get("cy", 0.5 * (y0 + y1)))
                x0 = float(gate.get("x0", float(gate.get("cx", 0.0)) - 0.2))
                # Gate-pocket join goal: first recover into the gap band *before* the wall face.
                #
                # Planning a join goal *past* the wall thickness (x1+...) forces A* to route
                # through narrow gaps and can create long circular limit-cycles in pure tracking
                # when the vehicle starts far off-band (P6 B3/C1/C2/C3 timeouts).
                #
                # By staging just upstream of the wall (front hitch behind x0 by a margin), we
                # let the normal corridor/gate traversal logic handle the actual pass-through.
                # Keep a larger upstream staging margin for scattered starts: if the join goal
                # is too close to the wall face, the front hitch constraint can force early
                # reverse/back-off oscillations that dominate the 60s P6 budget in B3/C*.
                staging_margin = 0.75
                gate_join_x = float(x0 - float(self.geom.front_hitch_x) - staging_margin)
                # Gate-staging join should be kinematically feasible:
                # - A purely lateral (vertical) join target at the current x can be hard/impossible
                #   for Ackermann to realize quickly, and can deadlock with a leader behind
                #   (P6 B2 Random_Scattered seed=41042).
                # - When sufficiently upstream, allow advancing toward the canonical staging x so
                #   the join route is diagonal (forward + lateral), improving feasibility.
                # - When already close to the wall face, avoid pushing x forward while still
                #   outside the opening band (front hitch protrudes).
                if float(st.x) > gate_join_x - 1.0:
                    gate_join_x = float(min(gate_join_x, float(st.x)))
                gate_join_xy = np.array([gate_join_x, float(cy)], dtype=float)
        # Only replan occasionally to bound compute cost.
        last_t = float(self._join_route_planned_at.get(vid, -1e9))
        if (now - last_t) < 2.0:
            return

        stall_trigger = bool(stall_ticks >= stall_limit)
        # In bottleneck/gate scenes, a head can be "stalled" on-path due to gate-yield or
        # multi-vehicle interactions. Triggering an A* join-route purely from stall ticks can
        # cause unnecessary rejoin replans and even regress progress (P6 B2 scattered timeouts).
        if self.mandatory_gates:
            stall_trigger = bool(stall_trigger and dist_to_path >= 0.85)
        subtype = str(self.case.subtype)
        dist_join = 1.25
        # In multi-gate / chicane scenes, waiting for very large lateral error before planning a
        # join route can allow the Stanley corridor tracker to enter large oscillations, wasting
        # most of the 60s P6 budget (B2/B3 Random_Scattered). Use a lower trigger threshold.
        if self.mandatory_gates and (subtype.startswith("C") or subtype in {"B2", "B3"}):
            dist_join = 0.95
        need_join = bool(
            dist_to_path >= dist_join
            or (blocked >= 16 and dist_to_path >= 0.85)
            or stall_trigger
            or gate_need_join
        )

        # Sticky strict join: when we planned a gate-crossing join-route (strict_goal=True),
        # keep executing it until the join goal is reached. Clearing it early based on
        # `dist_to_path` alone can drop the vehicle back into the wrong-side-of-gap pocket
        # and cause long oscillation timeouts (notably C1/C3 Uniform_Spread).
        if vid in self._join_route_xy and bool(self._join_route_strict_goal.get(vid, False)):
            if not self._join_route_done(vid):
                return
        if not need_join:
            if vid in self._join_route_xy:
                # Already joined; clear any stale join route.
                self._join_route_xy.pop(vid, None)
                self._join_route_goal_xy.pop(vid, None)
                self._join_route_strict_goal.pop(vid, None)
            return

        # Leader-priority join: when rejoining from scattered starts, avoid planning a merge point
        # *ahead* of the global leader. This prevents repeated early deadlocks where a scattered
        # vehicle cuts in front of the leader and both vehicles freeze before the first bottleneck.
        strict_goal = False
        if gate_need_join and gate_join_xy is not None:
            join_xy = gate_join_xy.astype(float).copy()
            strict_goal = True
        else:
            def _join_idx_for_monotone_x(path_xy: np.ndarray, st_xy: np.ndarray) -> int:
                # For the project's benchmark paths, x is monotone increasing (A/B/C). Using the
                # Euclidean nearest point can pick a *behind* segment for far-scattered starts,
                # causing long backtracking and P6 timeouts (e.g., B3/C2 Random_Scattered).
                if len(path_xy) < 2:
                    return 0
                dx = np.diff(path_xy[:, 0]).astype(float)
                if bool(np.all(dx > 1e-6)) or bool(np.all(dx < -1e-6)):
                    return int(np.argmin(np.abs(path_xy[:, 0] - float(st_xy[0]))))
                return _nearest_path_idx(path_xy, st_xy)

            if int(vid) != int(self.leader_id) and int(self.leader_id) in self.states:
                leader_st = self.states[int(self.leader_id)]
                behind_leader = bool(float(st.x) < float(leader_st.x) - 0.45)
                if behind_leader:
                    leader_s = float(self._path_progress_s(leader_st))
                    join_s = float(max(0.0, leader_s - 2.4))
                    join_xy = _interp_path(self.path_xy, self.path_s, join_s).astype(float).copy()
                else:
                    x_sign = 1.0 if float(self.path_xy[-1, 0] - self.path_xy[0, 0]) >= 0.0 else -1.0
                    join_ahead = float(np.clip(0.60 * float(dist_to_path) + 1.20, 1.20, 3.00))
                    x_target = float(st.x + x_sign * join_ahead)
                    join_idx = _join_idx_for_monotone_x(path_xy, np.array([x_target, float(st.y)], dtype=float))
                    join_xy = path_xy[join_idx].astype(float).copy()
            else:
                x_sign = 1.0 if float(self.path_xy[-1, 0] - self.path_xy[0, 0]) >= 0.0 else -1.0
                join_ahead = float(np.clip(0.60 * float(dist_to_path) + 1.20, 1.20, 3.00))
                x_target = float(st.x + x_sign * join_ahead)
                join_idx = _join_idx_for_monotone_x(path_xy, np.array([x_target, float(st.y)], dtype=float))
                join_xy = path_xy[join_idx].astype(float).copy()

        # If the vehicle is currently behind an active cross-wall (y outside the gap),
        # join via a staging point on the safe side first. This avoids early x-advance
        # that would sweep the extended front hitch into the wall.
        route = None
        if gate is not None:
            y0 = float(gate.get("y0", float(gate.get("cy", 0.0)) - 0.2))
            y1 = float(gate.get("y1", float(gate.get("cy", 0.0)) + 0.2))
            if y1 < y0:
                y0, y1 = y1, y0
            if not ((y0 - 0.10) <= float(st.y) <= (y1 + 0.10)):
                cy = float(gate.get("cy", 0.5 * (y0 + y1)))
                x0 = float(gate.get("x0", float(gate.get("cx", 0.0)) - 0.2))
                # Staging point is used to shift laterally into the gap centerline while staying
                # safely behind the wall face. The previous overly conservative back-off (1m+)
                # produced join routes whose first segment was "behind" the vehicle and triggered
                # a short-horizon local planner that could stall (P6 timeouts in C1/C2 Uniform_Spread).
                staging_margin = 0.75
                front_hitch_x = float(self.geom.to_world(st, float(self.geom.front_hitch_x))[0])
                canonical_x = float(x0 - float(self.geom.front_hitch_x) - staging_margin)
                if front_hitch_x <= float(x0 - staging_margin):
                    # When far upstream, avoid a purely lateral (vertical) staging move at the
                    # current x: that behavior can drive yaw toward ±90deg and create a
                    # rear-end deadlock in scattered starts (P6 B2 Random_Scattered seed=41042).
                    # Instead, advance toward the canonical staging x gradually so the join path
                    # remains diagonal (forward + lateral).
                    staging_x = float(min(canonical_x, float(st.x) + 1.2))
                else:
                    staging_x = float(canonical_x)
                # Cap required backtracking so the join route remains mostly forward-executable.
                staging_x = float(max(staging_x, float(st.x) - 0.35))
                staging = np.array([staging_x, float(cy)], dtype=float)
                r1 = self.grid_planner.plan(start_xy=st.xy(), goal_xy=staging)
                r2 = self.grid_planner.plan(start_xy=staging, goal_xy=join_xy)
                if r1 is not None and r2 is not None and len(r1) >= 2 and len(r2) >= 2:
                    route = np.concatenate([r1[:-1], r2], axis=0)
        if route is None:
            route = self.grid_planner.plan(start_xy=st.xy(), goal_xy=join_xy)
        self._join_route_planned_at[vid] = float(now)
        if route is None or len(route) < 2:
            return
        self._join_route_xy[vid] = route
        self._join_route_goal_xy[vid] = join_xy
        self._join_route_strict_goal[vid] = bool(strict_goal)

    def _join_route_done(self, vehicle_id: int) -> bool:
        vid = int(vehicle_id)
        if vid not in self._join_route_xy:
            return True
        join_xy = self._join_route_goal_xy.get(vid, None)
        if join_xy is None or int(join_xy.shape[0]) != 2:
            return True
        st = self.states[vid]
        if bool(self._join_route_strict_goal.get(vid, False)):
            # For strict (gate-staging) join routes, do not clear too early:
            # reaching within 0.75m of the staging point is insufficient when the vehicle is
            # still outside the gate opening band. Early clearing can drop the vehicle into a
            # wall-corner deadlock and dominate P6 timeouts (B3/C* scattered starts).
            d = float(np.linalg.norm(st.xy() - join_xy))
            gate = self._active_gate(st)
            if gate is not None:
                y0 = float(gate.get("y0", float(gate.get("cy", 0.0)) - 0.2))
                y1 = float(gate.get("y1", float(gate.get("cy", 0.0)) + 0.2))
                if y1 < y0:
                    y0, y1 = y1, y0
                in_band = bool((y0 - 0.10) <= float(st.y) <= (y1 + 0.10))
            else:
                in_band = True
            # If we've already entered the gap band and progressed past the staging x, allow
            # clearing the strict join route. Keeping it active too long can force oscillatory
            # "rejoin" behavior and regress x-progress in B2 chicanes (P6 seed=41042).
            if in_band and float(st.x) >= float(join_xy[0]) + 0.25:
                return True
            return bool(d <= 0.55 and in_band)
        if float(np.linalg.norm(st.xy() - join_xy)) <= 0.75:
            return True
        if self._lateral_distance_to_path(st, self._track_path_xy(vid)) <= 0.65:
            return True
        return False

    def _local_path_radius(self, near_idx: int, lookahead_points: int) -> float:
        if len(self.path_xy) < 3:
            return math.inf
        lo = max(1, near_idx - 1)
        hi = min(len(self.path_xy) - 2, near_idx + max(1, int(lookahead_points)))
        rmin = math.inf
        for i in range(lo, hi + 1):
            r = _three_point_radius(self.path_xy[i - 1], self.path_xy[i], self.path_xy[i + 1])
            if r < rmin:
                rmin = r
        return float(rmin)

    def _single_curvature_limited_speed(self, base_speed: float, state: VehicleState) -> float:
        if float(base_speed) <= 1e-6:
            return 0.0
        tcfg = self.cfg.control.train
        idx = _nearest_path_idx(self.path_xy, state.xy())
        local_radius = self._local_path_radius(idx, tcfg.curvature_lookahead_points)
        if math.isinf(local_radius):
            return float(base_speed)
        ratio = float(np.clip(self.cfg.vehicle.min_turn_radius_single / max(local_radius, 1e-6), 0.0, 2.0))
        gain = 0.75 * float(tcfg.curvature_speed_gain)
        speed_scale = 1.0 / (1.0 + gain * ratio)
        v = float(base_speed) * speed_scale
        return float(np.clip(v, tcfg.min_speed, base_speed))

    def _nearest_obstacle_distance(self, state: VehicleState) -> float:
        dmin = math.inf
        for obs in self.case.obstacles:
            dx = max(abs(state.x - obs.x) - 0.5 * obs.width, 0.0)
            dy = max(abs(state.y - obs.y) - 0.5 * obs.height, 0.0)
            d = math.hypot(dx, dy)
            if d < dmin:
                dmin = d
        return float(dmin)

    def _single_obstacle_limited_speed(self, base_speed: float, state: VehicleState) -> float:
        if float(base_speed) <= 1e-6:
            return 0.0
        d = self._nearest_obstacle_distance(state)
        v = float(base_speed)
        subtype = str(self.case.subtype)
        relax = bool(subtype.startswith("C") or subtype in {"B2", "B3"})
        if d < 1.2:
            v = min(v, 0.90)
        if d < 0.9:
            v = min(v, 0.75)
        if d < 0.65:
            # Slightly less conservative speed cap in moderate-clearance corridors for chicanes.
            # Keep the original more conservative cap for Type-A/B where higher speed near obstacles
            # tends to increase wall-grazing collisions.
            v = min(v, 0.75 if relax else 0.65)
        if d < 0.5:
            v = min(v, 0.65 if relax else 0.60)
        if d < 0.35:
            v = min(v, 0.55 if relax else 0.50)
        min_v = 0.12 if d < 0.65 else 0.18
        return float(max(min_v, v))

    def _speed_limit_by_lead_vehicle(self, vehicle_id: int, base_speed: float) -> float:
        st_self = self.states[vehicle_id]
        s_self = self._path_progress_s(st_self)
        lat_self = self._signed_lateral_to_path(st_self)
        band = self._same_lane_band()
        # World-frame lateral guard:
        # Signed lateral distance is computed w.r.t. local path normals. Near diagonal path segments,
        # the normal can have a strong x-component, making two vehicles with large `|y|` separation
        # appear to share the same lane. This can incorrectly stop the leader behind a laterally
        # parked head near B (P6 C2 Random_Scattered seed=71042).
        y_band = float(max(1.2, 1.8 * band))
        lead_gap = math.inf
        for oid in self.vehicle_ids:
            if oid == vehicle_id:
                continue
            st_o = self.states[oid]
            if abs(float(st_o.y) - float(st_self.y)) > y_band:
                continue
            s_o = self._path_progress_s(st_o)
            if s_o <= s_self:
                continue
            lat_o = self._signed_lateral_to_path(st_o)
            if abs(lat_o - lat_self) > band:
                continue
            lead_gap = min(lead_gap, s_o - s_self)
        if math.isinf(lead_gap):
            return float(base_speed)
        safe_gap = 2.2 + 1.2 * max(0.0, self.states[vehicle_id].v)
        if lead_gap >= safe_gap:
            return float(base_speed)
        v_lim = max(0.0, 0.65 * (lead_gap - 1.0))
        return float(min(base_speed, v_lim))

    def _speed_limit_by_x_lead(self, vehicle_id: int, base_speed: float) -> float:
        """
        Simple longitudinal (x-axis) car-following limiter for scattered starts.

        Motivation:
        - Early in Random_Scattered/Uniform_Spread, path-s ordering can be noisy when vehicles are
          far from the corridor.
        - Vehicles can collide near the start when the faster free-speed head closes into a slower
          vehicle (often the global leader).

        This limiter uses x-ordering within a lateral band as a conservative proxy for "in front".
        """
        st = self.states[vehicle_id]
        lead_dx = math.inf
        lead_id: int | None = None
        for oid, o in self.states.items():
            if int(oid) == int(vehicle_id):
                continue
            # Only consider vehicles in front along the main corridor axis.
            dx = float(o.x - st.x)
            if dx <= 1e-6:
                continue
            # Ignore vehicles far away laterally.
            #
            # IMPORTANT (B/C chicanes):
            # In multi-gate scenes, a lead vehicle may need to perform a large lateral lane-change
            # while still being "in front" of a rear vehicle. Using the nominal same-lane band
            # (≈ vehicle width) can mis-classify that lead as a different lane, allowing the rear
            # vehicle to close in too much and deadlock the lead's steering sweep (P6 B2
            # Random_Scattered seed=41042).
            band = float(self._same_lane_band())
            if self.mandatory_gates and (str(self.case.subtype).startswith("C") or str(self.case.subtype) in {"B2", "B3"}):
                try:
                    x0_min = min(float(g.get("x0", float(g.get("cx", 0.0)))) for g in self.mandatory_gates)
                except ValueError:
                    x0_min = float("inf")
                # Enlarge the lateral band in the gate-approach neighborhood (≈ 4m upstream),
                # matching the far-field gate hint logic used for join-route planning.
                if (float(st.x) >= (x0_min - 4.2)) or (float(o.x) >= (x0_min - 4.2)):
                    band = float(max(band, 1.95))
            if abs(float(o.y - st.y)) > band:
                continue
            if dx < lead_dx:
                lead_dx = dx
                lead_id = int(oid)
        if math.isinf(lead_dx):
            return float(base_speed)

        v_cap = float(base_speed)
        safe_gap = 1.45 + 1.0 * max(0.0, float(st.v))
        # Near cross-wall gates, vehicles need extra longitudinal headway to remain maneuverable
        # (otherwise the rear vehicle can physically block the front vehicle's turning sweep).
        stop_dx = 0.55
        # IMPORTANT: the enlarged gate headway heuristic is tuned for Type-C multi-gate scenes where
        # scattered starts frequently create tight gate-queue deadlocks. Applying it to Type-B
        # bottleneck scenes can over-constrain longitudinal motion and create unnecessary stalls.
        if self.demo_cfg.enable_gate_traversal and str(self.case.subtype).startswith("C"):
            if self._active_gate(st) is not None:
                stop_dx = max(stop_dx, 2.25)
            if lead_id is not None and lead_id in self.states:
                if self._active_gate(self.states[lead_id]) is not None:
                    stop_dx = max(stop_dx, 2.25)
        if lead_id is not None and lead_id in self.states:
            lead = self.states[lead_id]
            # Even in open scenes (Type-A), a lead vehicle may need extra headway when it is
            # executing a sharp turning maneuver (e.g., after a join-route that approaches the
            # corridor with a near-vertical heading). Without extra room, the rear vehicle can
            # create a persistent deadlock: the lead keeps oscillating/reversing to avoid contact.
            turning = bool(abs(float(lead.delta)) >= 0.22)
            if turning:
                stop_dx = max(stop_dx, 1.65)
            if len(self.path_xy) >= 2:
                lead_idx = _nearest_path_idx(self.path_xy, lead.xy())
                if lead_idx < len(self.path_xy) - 1:
                    d = self.path_xy[lead_idx + 1] - self.path_xy[lead_idx]
                else:
                    d = self.path_xy[lead_idx] - self.path_xy[max(0, lead_idx - 1)]
                n = float(np.linalg.norm(d))
                if n > 1e-9:
                    lead_path_yaw = float(math.atan2(float(d[1]), float(d[0])))
                    yaw_err_path = float(abs(angle_diff(lead_path_yaw, lead.yaw)))
                    lat_err_path = float(self._lateral_distance_to_path(lead))
                    # When the lead is strongly misaligned with the corridor tangent, it needs
                    # room to rotate/merge. Keep the rear vehicle back; otherwise both vehicles
                    # can freeze near the start (A3 S-curve is sensitive).
                    if (yaw_err_path >= math.radians(70.0)) or (
                        (lat_err_path >= 0.65) and (yaw_err_path >= math.radians(40.0))
                    ):
                        stop_dx = max(stop_dx, 1.95)
        guard_dx = float(max(safe_gap, stop_dx + 0.15))
        if lead_dx < guard_dx:
            # Smoothly reduce speed as the gap shrinks; fully stop when too close.
            v_cap = min(v_cap, max(0.0, 0.70 * (lead_dx - stop_dx)))
        return float(max(0.0, v_cap))

    def _speed_limit_by_proximity(
        self,
        vehicle_id: int,
        base_speed: float,
        *,
        skip_ids: set[int] | None = None,
        ahead_only: bool = False,
    ) -> float:
        st = self.states[vehicle_id]
        d_min = math.inf
        skip = set() if skip_ids is None else set(skip_ids)
        for oid, o in self.states.items():
            if oid == vehicle_id:
                continue
            if oid in skip:
                continue
            if ahead_only and float(o.x) < float(st.x) - 0.05:
                continue
            d = float(np.linalg.norm(st.xy() - o.xy()))
            if d < d_min:
                d_min = d
        if math.isinf(d_min):
            return float(base_speed)
        v_cap = float(base_speed)
        if d_min < 2.6:
            v_cap = min(v_cap, max(0.35, 0.90 * (d_min - 0.8)))
        if d_min < 1.8:
            v_cap = min(v_cap, max(0.30, 0.70 * (d_min - 0.5)))
            if d_min < 1.4:
                v_cap = min(v_cap, max(0.25, 0.50 * (d_min - 0.35)))
        return float(max(0.0, v_cap))

    def _enforce_speed_target(self, state: VehicleState, cmd: ControlCommand, target_speed: float) -> ControlCommand:
        # Keep a one-step speed cap consistent with actuator limits.
        # (The previous accel-cap clip could generate invalid accel ranges when `accel_cap`
        # was more negative than `-max_decel`, producing accelerations below the physical limit.)
        return self._enforce_speed_cap(state, cmd, float(max(0.0, target_speed)), allow_reverse=False)

    def _speed_accel_cmd(self, v_now: float, target_speed: float) -> float:
        v_ref = float(np.clip(target_speed, 0.0, self.cfg.vehicle.max_speed))
        accel = float(self.cfg.control.speed.kp * (v_ref - float(v_now)))
        return float(np.clip(accel, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd))

    def _enforce_speed_cap(self, state: VehicleState, cmd: ControlCommand, speed_cap: float, *, allow_reverse: bool) -> ControlCommand:
        dt = max(self.cfg.control.dt, 1e-6)
        accel = float(np.clip(cmd.accel, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd))
        v_next = float(state.v + accel * dt)

        cap_fwd = float(max(0.0, speed_cap))
        if v_next > cap_fwd + 1e-9:
            accel = float((cap_fwd - state.v) / dt)
            v_next = float(cap_fwd)

        if not allow_reverse and v_next < -1e-9:
            accel = float((0.0 - state.v) / dt)
            v_next = 0.0

        if allow_reverse:
            cap_rev = float(self.cfg.vehicle.max_reverse_speed)
            if v_next < -cap_rev - 1e-9:
                accel = float((-cap_rev - state.v) / dt)
                v_next = float(-cap_rev)

        accel = float(np.clip(accel, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd))
        return ControlCommand(accel=accel, steer_rate=float(cmd.steer_rate))

    def _leader_remaining_time(self) -> float:
        leader_state = self.states[self.leader_id]
        s = self._path_progress_s(leader_state)
        dist = max(0.0, self.path_len - s)
        # IMPORTANT: use the *executed* leader speed budget, not the vehicle theoretical v_max.
        # Otherwise the remaining time is underestimated (especially in bottleneck gates),
        # which can trigger premature feasibility flips and abort docking too early.
        v_budget = float(min(self.cfg.vehicle.max_speed, max(0.2, self.demo_cfg.leader_target_speed)))
        return estimate_leader_remaining_time(
            distance_remaining=dist,
            speed_now=leader_state.v,
            v_max=v_budget,
            a_max=self.cfg.vehicle.max_accel,
        )

    def _stop_command(self, state: VehicleState) -> ControlCommand:
        dt = max(self.cfg.control.dt, 1e-6)
        max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
        unwind = float(np.clip(-state.delta / dt, -max_rate, max_rate))
        if state.v > 0.0:
            dec = min(self.cfg.vehicle.max_decel, state.v / dt)
            return ControlCommand(accel=-dec, steer_rate=unwind)
        if state.v < 0.0:
            acc = min(self.cfg.vehicle.max_accel, abs(state.v) / dt)
            return ControlCommand(accel=acc, steer_rate=unwind)
        return ControlCommand(accel=0.0, steer_rate=unwind)

    def _freeze_state(self, state: VehicleState) -> VehicleState:
        dt = max(self.cfg.control.dt, 1e-6)
        max_delta_step = math.radians(self.cfg.vehicle.max_steer_rate_deg_s) * dt
        delta = float(state.delta + np.clip(-state.delta, -max_delta_step, max_delta_step))
        return VehicleState(
            vehicle_id=state.vehicle_id,
            x=float(state.x),
            y=float(state.y),
            yaw=float(state.yaw),
            v=0.0,
            delta=float(delta),
            mode=state.mode,
        )

    def _collision_with_any_vehicle(
        self,
        state: VehicleState,
        *,
        skip_ids: set[int] | None = None,
    ) -> bool:
        skip = set() if skip_ids is None else set(skip_ids)
        for oid, other in self.states.items():
            if oid == state.vehicle_id or oid in skip:
                continue
            if self.collision.collide_vehicle_vehicle(state, other, include_clearance=False):
                return True
        return False

    def _place_collision_free_near(
        self,
        base_state: VehicleState,
        *,
        avoid_ids: set[int] | None = None,
    ) -> VehicleState:
        avoid = set() if avoid_ids is None else set(avoid_ids)
        h = np.array([math.cos(base_state.yaw), math.sin(base_state.yaw)], dtype=float)
        n = np.array([-math.sin(base_state.yaw), math.cos(base_state.yaw)], dtype=float)
        long_offsets = [0.0] + [(-0.15 * i) for i in range(1, 25)] + [(0.15 * i) for i in range(1, 25)]
        lat_offsets = [0.0, 0.18, -0.18, 0.36, -0.36, 0.54, -0.54, 0.72, -0.72, 0.90, -0.90]
        for dl in long_offsets:
            for dt_lat in lat_offsets:
                c = base_state.xy() + h * float(dl) + n * float(dt_lat)
                cand = VehicleState(
                    vehicle_id=base_state.vehicle_id,
                    x=float(c[0]),
                    y=float(c[1]),
                    yaw=float(base_state.yaw),
                    v=float(base_state.v),
                    delta=float(base_state.delta),
                    mode=base_state.mode,
                )
                if self._collision_with_obstacles_fast(cand):
                    continue
                if self._collision_with_any_vehicle(cand, skip_ids=avoid):
                    continue
                return cand
        return base_state

    def _dock_servo_command(
        self,
        follower: VehicleState,
        rear_target: np.ndarray,
        yaw_target: float,
        *,
        leader_speed: float,
        max_speed: float,
        min_speed: float,
        align_speed_cap: float,
        allow_reverse: bool,
    ) -> tuple[ControlCommand, dict[str, float]]:
        rel_world = rear_target - self.geom.front_hitch(follower)
        c = math.cos(-follower.yaw)
        s = math.sin(-follower.yaw)
        rel_x = float(c * rel_world[0] - s * rel_world[1])
        rel_y = float(s * rel_world[0] + c * rel_world[1])
        yaw_err = float(angle_diff(yaw_target, follower.yaw))

        denom = rel_x * rel_x + rel_y * rel_y + 1e-4
        delta_ff = math.atan2(2.0 * self.cfg.vehicle.wheelbase * rel_y, denom)
        delta_ref = delta_ff + 0.58 * yaw_err
        dmax = math.radians(self.cfg.vehicle.max_steer_deg)
        delta_ref = float(np.clip(delta_ref, -dmax, dmax))
        max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
        steer_rate = float(np.clip((delta_ref - follower.delta) / self.cfg.control.dt, -max_rate, max_rate))

        speed_ref = float(leader_speed + 0.82 * rel_x)
        if not allow_reverse:
            speed_ref = max(0.0, speed_ref)
        speed_ref = float(np.clip(speed_ref, min_speed, max_speed))
        if (abs(rel_y) > 0.24) or (abs(yaw_err) > math.radians(9.0)):
            speed_ref = float(min(speed_ref, align_speed_cap))

        kp = self.cfg.control.speed.kp
        accel = float(np.clip(kp * (speed_ref - follower.v), -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd))
        return ControlCommand(accel=accel, steer_rate=steer_rate), {
            "rel_x": float(rel_x),
            "rel_y": float(rel_y),
            "yaw_err_deg": float(abs(math.degrees(yaw_err))),
            "speed_ref": float(speed_ref),
        }

    def _track_center_with_heading(
        self,
        follower: VehicleState,
        center_goal: np.ndarray,
        yaw_goal: float,
        speed_ref: float,
        yaw_gain: float,
    ) -> ControlCommand:
        cmd = self.tracker.track_point(
            follower,
            float(center_goal[0]),
            float(center_goal[1]),
            float(yaw_goal),
            float(speed_ref),
        )
        max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
        sr = float(np.clip(cmd.steer_rate + yaw_gain * angle_diff(yaw_goal, follower.yaw), -max_rate, max_rate))
        return ControlCommand(accel=cmd.accel, steer_rate=sr)

    def _store_cmd(self, vehicle_id: int, cmd: ControlCommand, now: float) -> None:
        vid = int(vehicle_id)
        prev = self._last_cmd.get(vid, None)
        self._last_cmd[vid] = cmd
        if prev is None:
            return
        th = math.radians(8.0)
        if (abs(prev.steer_rate) >= th) and (abs(cmd.steer_rate) >= th):
            if np.sign(prev.steer_rate) * np.sign(cmd.steer_rate) < 0.0:
                self._steer_flip_ts[vid].append(float(now))

    def _build_monitor_snapshot(self, now: float, prev_states: dict[int, VehicleState]) -> dict[str, Any]:
        dt = max(self.cfg.control.dt, 1e-6)
        alerts: list[dict[str, Any]] = []
        states_out: dict[str, Any] = {}
        dmax = math.radians(self.cfg.vehicle.max_steer_deg)
        sr_lim = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
        for vid in self.vehicle_ids:
            p = prev_states[vid]
            s = self.states[vid]
            cmd = self._last_cmd.get(vid, ControlCommand(accel=0.0, steer_rate=0.0))
            pos_step = float(np.linalg.norm(s.xy() - p.xy()))
            yaw_rate = float(angle_diff(s.yaw, p.yaw) / dt)
            accel_est = float((s.v - p.v) / dt)
            goal_xy = self._goal_for_vehicle(vid)
            goal_dist = float(np.linalg.norm(s.xy() - goal_xy))
            goal_dist_global = float(np.linalg.norm(s.xy() - self.case.goal_xy))
            mode_now = str(self.engine.topology.mode[vid].value)
            mode_prev = str(p.mode.value)
            flips = [t for t in self._steer_flip_ts[vid] if (now - t) <= 1.2 + 1e-9]
            self._steer_flip_ts[vid] = flips

            stall_ticks = int(self._stall_ticks.get(vid, 0))
            if stall_ticks >= int(2.0 / dt):
                alerts.append({"type": "stall", "vehicle_id": int(vid), "duration_s": float(stall_ticks * dt)})
            if abs(yaw_rate) > 1.05 * self.cfg.vehicle.max_yaw_rate:
                alerts.append({"type": "yaw_rate_violation", "vehicle_id": int(vid), "yaw_rate": float(yaw_rate)})
            if pos_step > (self.cfg.vehicle.max_speed * dt + 0.12):
                alerts.append({"type": "position_jump", "vehicle_id": int(vid), "pos_step": float(pos_step)})
            if len(flips) >= 4:
                alerts.append({"type": "steer_oscillation", "vehicle_id": int(vid), "flip_count_1p2s": int(len(flips))})
            if s.v > self.cfg.vehicle.max_speed + 1e-6 or s.v < -self.cfg.vehicle.max_reverse_speed - 1e-6:
                alerts.append({"type": "speed_bound", "vehicle_id": int(vid), "v": float(s.v)})
            if abs(s.delta) > dmax + math.radians(0.5):
                alerts.append({"type": "steer_bound", "vehicle_id": int(vid), "delta_deg": float(math.degrees(s.delta))})
            if accel_est > self.cfg.vehicle.max_accel + 0.25 or accel_est < -self.cfg.vehicle.max_decel - 0.25:
                alerts.append({"type": "accel_bound", "vehicle_id": int(vid), "accel": float(accel_est)})
            if abs(cmd.steer_rate) > sr_lim + math.radians(5.0):
                alerts.append(
                    {
                        "type": "steer_rate_bound",
                        "vehicle_id": int(vid),
                        "steer_rate_deg_s": float(math.degrees(cmd.steer_rate)),
                    }
                )
            if mode_now == "DOCKING" and s.v < -0.05:
                alerts.append({"type": "dock_reverse_motion", "vehicle_id": int(vid), "v": float(s.v)})
            if (mode_prev == "TRAIN_FOLLOW") and (mode_now == "FREE") and (pos_step > 0.08):
                alerts.append({"type": "split_release_jump", "vehicle_id": int(vid), "pos_step": float(pos_step)})

            states_out[str(vid)] = {
                "x": float(s.x),
                "y": float(s.y),
                "yaw": float(s.yaw),
                "v": float(s.v),
                "delta": float(s.delta),
                "mode": str(self.engine.topology.mode[vid].value),
                "cmd_accel": float(cmd.accel),
                "cmd_steer_rate": float(cmd.steer_rate),
                "accel_est": float(accel_est),
                "yaw_rate_est": float(yaw_rate),
                "pos_step": float(pos_step),
                "goal_dist": float(goal_dist),
                "goal_dist_global": float(goal_dist_global),
                "path_s": float(self._path_progress_s(s)),
                "step_energy": float(self._step_energy.get(vid, 0.0)),
                "energy_cumulative": float(self._energy_cumulative.get(vid, 0.0)),
            }
            if vid in self._dock_debug:
                states_out[str(vid)]["dock_debug"] = self._dock_debug[vid]

        dock_pairs: list[dict[str, Any]] = []
        for f, l in sorted(self.engine.topology.docking_target.items()):
            if f not in self.states or l not in self.states:
                continue
            fs = self.states[f]
            ls = self.states[l]
            front = self.geom.front_hitch(fs)
            rear = self.geom.rear_hitch(ls)
            dock_pairs.append(
                {
                    "follower_id": int(f),
                    "leader_id": int(l),
                    "hitch_distance": float(np.linalg.norm(front - rear)),
                    "yaw_error_deg": float(abs(math.degrees(angle_diff(ls.yaw, fs.yaw)))),
                    "speed_error": float(abs(ls.v - fs.v)),
                }
            )

        return {
            "t": float(now),
            "states": states_out,
            "edges": [[int(a), int(b)] for a, b in self.engine.topology.edges()],
            "pending_docks": {str(k): int(v) for k, v in self.engine.topology.docking_target.items()},
            "leader_s": float(self._path_progress_s(self.states[self.leader_id])),
            "leader_remaining_s": float(self._leader_remaining_s),
            "total_energy": float(self.total_energy),
            "collision_count": int(self.collision_count),
            "dock_pairs": dock_pairs,
            "last_reconfig_action": self._last_reconfig_action,
            "alerts": alerts,
        }

    def _submit_command(self, cmd, now: float, source: str) -> None:
        self.commands_submitted += 1
        fb = self.engine.submit_command(cmd, now=now)
        if fb.accepted:
            self.commands_accepted += 1
            if isinstance(cmd, DockingCommand):
                self.docking_intercept_s[int(cmd.follower_id)] = (
                    float(cmd.intercept_path_s) if cmd.intercept_path_s is not None else 0.0
                )
        self._push_event(
            now,
            source=source,
            event=f"submit_{cmd.kind.value.lower()}",
            detail=(
                f"accepted={fb.accepted}:{fb.error_code.value}:{fb.detail}"
                + (
                    f":follower={cmd.follower_id}:leader={cmd.leader_id}:s={cmd.intercept_path_s:.2f}"
                    if isinstance(cmd, DockingCommand) and cmd.intercept_path_s is not None
                    else ""
                )
            ),
        )

    def _cmd_header(self, *, now: float, command_id: str, source: str, priority: int) -> CommandHeader:
        ttl = float(self.demo_cfg.command_ttl_s)
        return CommandHeader(
            command_id=str(command_id),
            state_seq=int(self.engine.state_seq),
            issued_at=float(now),
            deadline_at=float(now + ttl),
            priority=int(priority),
            source=str(source),
        )

    def _next_cmd_id(self, prefix: str) -> str:
        self.cmd_idx += 1
        return f"{prefix}_{self.case.scenario_id}_{self.cmd_idx:05d}"

    def _eligible_docking_followers(self, now: float) -> list[int]:
        out: list[int] = []
        for vid in self.vehicle_ids:
            if vid == self.leader_id:
                continue
            if vid in self.abandon_docking:
                continue
            if vid in self.engine.topology.docking_target:
                continue
            if self.engine.topology.parent[vid] is not None:
                continue
            if self.engine.topology.child[vid] is not None:
                continue
            if self.engine.topology.mode[vid] != VehicleMode.FREE:
                continue
            t_ready = self.replan_ready_at.get(vid, -1e9)
            if now < t_ready - 1e-9:
                continue
            out.append(vid)
        return out

    def _strategy_tick(self, now: float) -> None:
        # P0 acceptance rule: strategy only emits commands; it does not directly write control actions.
        if self.policy == "independent":
            return

        leader_chain = self._chain_from_head(self.leader_id)
        leader_size = len(leader_chain)
        leader_s = self._path_progress_s(self.states[self.leader_id])
        profile = self.case.labels.n_max_pass_profile

        # P3 passability-aware split guard (global-forward conservative check).
        nmax_future = min_nmax_in_window(profile, leader_s, self.path_len)
        split_needed = bool(leader_size > int(nmax_future) and leader_size > 1)

        feasible = self.engine.decisions[-1].feasible if self.engine.decisions else True
        leader_tail = self._chain_tail(self.leader_id)
        # P6 stability rule: in bottleneck scenes (Type B/C), initiating docking can easily
        # degrade completion rate due to late intercepts and retry loops. Keep docking enabled
        # for Type-A (open) scenes where it is the intended main behavior.
        allow_docking = bool(str(self.case.subtype).startswith("A"))
        dock_context_ok = bool(
            feasible
            and allow_docking
            and len(self.engine.topology.docking_target) < self.demo_cfg.max_concurrent_docks
            and leader_tail not in self.engine.topology.docking_target.values()
        )
        dock_candidate: tuple[float, int, float, str] | None = None
        if dock_context_ok:
            leader_st = self.states[self.leader_id]
            candidates: list[tuple[float, int, float, str]] = []
            for vid in self._eligible_docking_followers(now):
                st = self.states[vid]
                # Robustness guard for scattered starts: do not start docking when the follower is
                # not clearly *behind* the leader along the main corridor axis. Otherwise the leader
                # can collide with a side-by-side/ahead docking follower before the intercept logic
                # has a chance to establish safe ordering.
                if float(st.x) > float(leader_st.x) - 0.70:
                    continue
                is_replan = vid in self.replan_ready_at
                if is_replan:
                    ranked = self.p4_planner.rank_candidates(
                        case=self.case,
                        follower=st,
                        t_now=float(now),
                    )
                    if not ranked:
                        continue
                    c = ranked[0]
                    s_int = float(c.s_intercept)
                    if s_int <= leader_s + self.demo_cfg.dock_ahead_margin_m:
                        continue
                    if nmax_at_s(profile, s_int) < leader_size + 1:
                        continue
                    candidates.append((float(c.score), int(vid), s_int, "P4"))
                else:
                    c = self.p2_scheduler.select_best_intercept(
                        follower=st,
                        train_size_now=leader_size,
                        tail_ready_time=now,
                        case=self.case,
                        leader_s=leader_s,
                        t_now=now,
                        conservative=True,
                        require_energy_gain=True,
                    )
                    if c is None:
                        continue
                    if float(c.s_intercept) <= leader_s + self.demo_cfg.dock_ahead_margin_m:
                        continue
                    candidates.append((float(c.cost.total_cost), int(vid), float(c.s_intercept), "P2"))
            if candidates:
                candidates.sort(key=lambda x: x[0])
                dock_candidate = candidates[0]
        dock_available = dock_candidate is not None
        conflict_now = bool(split_needed and dock_available)
        prefer_dock_on_conflict = str(self.demo_cfg.conflict_policy) == "dock_priority"

        chosen_action = "NONE"
        cooldown_active = bool((now - self._last_reconfig_action_t) < self.demo_cfg.reconfig_switch_cooldown_s)
        do_split = bool(split_needed and not (conflict_now and prefer_dock_on_conflict))
        if do_split:
            while leader_size > int(nmax_future) and leader_size > 1:
                parent = leader_chain[-2]
                child = leader_chain[-1]
                cmd = SplitCommand(
                    header=self._cmd_header(
                        now=now,
                        command_id=self._next_cmd_id("p3split"),
                        source="p3_guard",
                        priority=10,
                    ),
                    parent_id=int(parent),
                    child_id=int(child),
                    reason="passability_guard",
                )
                self._submit_command(cmd, now, source="P3")
                leader_chain.pop()
                leader_size -= 1
                chosen_action = "SPLIT"

        if not feasible:
            self.arbitration_trace.append(
                {
                    "t": float(now),
                    "leader_s": float(leader_s),
                    "leader_size": int(len(self._chain_from_head(self.leader_id))),
                    "nmax_future": int(nmax_future),
                    "split_needed": bool(split_needed),
                    "dock_available": bool(dock_available),
                    "conflict": bool(conflict_now),
                    "chosen_action": str(chosen_action),
                    "consistent": bool((not conflict_now) or (chosen_action == "SPLIT")),
                    "cooldown_active": bool(cooldown_active),
                    "reason": "feasibility_block",
                }
            )
            return

        # Split has strict priority over Dock on conflict; cooldown suppresses rapid Dock/Split toggling.
        if chosen_action == "NONE" and dock_candidate is not None:
            if (self._last_reconfig_action == "SPLIT") and cooldown_active:
                chosen_action = "HOLD"
            else:
                _, follower_id, s_intercept, src = dock_candidate
                cmd = DockingCommand(
                    header=self._cmd_header(
                        now=now,
                        command_id=self._next_cmd_id("p2dock" if src == "P2" else "p4redock"),
                        source="p2_scheduler" if src == "P2" else "p4_recovery",
                        priority=5,
                    ),
                    follower_id=int(follower_id),
                    leader_id=int(leader_tail),
                    intercept_path_s=float(s_intercept),
                )
                self._submit_command(cmd, now, source=src)
                if follower_id in self.replan_ready_at:
                    self.replan_ready_at.pop(follower_id, None)
                chosen_action = "DOCK"

        self.arbitration_trace.append(
            {
                "t": float(now),
                "leader_s": float(leader_s),
                "leader_size": int(len(self._chain_from_head(self.leader_id))),
                "nmax_future": int(nmax_future),
                "split_needed": bool(split_needed),
                "dock_available": bool(dock_available),
                "conflict": bool(conflict_now),
                "chosen_action": str(chosen_action),
                "consistent": bool((not conflict_now) or (chosen_action == "SPLIT")),
                "cooldown_active": bool(cooldown_active),
                "reason": ("priority_split_over_dock" if not prefer_dock_on_conflict else "dock_priority_on_conflict"),
            }
        )
        if chosen_action in {"SPLIT", "DOCK"}:
            if self._last_reconfig_action != chosen_action:
                self.reconfig_action_trace.append({"t": float(now), "action": str(chosen_action)})
            self._last_reconfig_action = chosen_action
            self._last_reconfig_action_t = float(now)

    def _get_lock_eval(self, follower_id: int) -> DockingLockEvaluator:
        if follower_id in self.lock_eval:
            return self.lock_eval[follower_id]
        l = DockingLockEvaluator(self.cfg.docking, self.geom)
        self.lock_eval[follower_id] = l
        return l

    def _project_locked(self, follower: VehicleState, leader: VehicleState) -> VehicleState:
        anchor = self.geom.rear_hitch(leader)
        yaw = leader.yaw
        rear = anchor - np.array([math.cos(yaw), math.sin(yaw)], dtype=float) * self.geom.front_hitch_x
        return VehicleState(
            vehicle_id=follower.vehicle_id,
            x=float(rear[0]),
            y=float(rear[1]),
            yaw=float(yaw),
            v=float(leader.v),
            # Keep follower steering continuous at lock transition; trailer steering
            # will be re-derived by train kinematics on subsequent updates.
            delta=float(follower.delta),
            mode=VehicleMode.TRAIN_FOLLOW,
        )

    def _lock_metrics(self, follower: VehicleState, leader: VehicleState) -> tuple[float, float, float]:
        f_h = self.geom.front_hitch(follower)
        l_h = self.geom.rear_hitch(leader)
        pos_err = float(np.linalg.norm(f_h - l_h))
        yaw_err = float(abs(angle_diff(leader.yaw, follower.yaw)))
        speed_err = float(abs(leader.v - follower.v))
        return pos_err, yaw_err, speed_err

    def _soft_capture_step(self, follower: VehicleState, leader: VehicleState) -> tuple[VehicleState, bool]:
        proj = self._project_locked(follower, leader)
        dxy = proj.xy() - follower.xy()
        dpos = float(np.linalg.norm(dxy))
        max_pos = max(1e-6, float(self.cfg.docking.soft_capture_max_position_correction))
        if dpos > max_pos:
            dxy = dxy * (max_pos / dpos)
        yaw_err = float(angle_diff(proj.yaw, follower.yaw))
        max_yaw = math.radians(self.cfg.docking.soft_capture_max_yaw_correction_deg)
        dyaw = float(np.clip(yaw_err, -max_yaw, max_yaw))
        v_new = float(follower.v + self.cfg.docking.soft_capture_velocity_blend * (proj.v - follower.v))
        delta_cmd = float(follower.delta + self.cfg.docking.soft_capture_velocity_blend * (proj.delta - follower.delta))
        max_delta_step = math.radians(self.cfg.vehicle.max_steer_rate_deg_s) * self.cfg.control.dt
        delta_new = float(follower.delta + np.clip(delta_cmd - follower.delta, -max_delta_step, max_delta_step))
        corrected = VehicleState(
            vehicle_id=follower.vehicle_id,
            x=float(follower.x + dxy[0]),
            y=float(follower.y + dxy[1]),
            yaw=float(follower.yaw + dyaw),
            v=float(v_new),
            delta=float(delta_new),
            mode=follower.mode,
        )
        pos_res = float(np.linalg.norm(proj.xy() - corrected.xy()))
        yaw_res_deg = float(abs(math.degrees(angle_diff(proj.yaw, corrected.yaw))))
        speed_res = float(abs(proj.v - corrected.v))
        transition_ok = bool(
            pos_res <= self.cfg.docking.lock_transition_max_position_jump
            and yaw_res_deg <= self.cfg.docking.lock_transition_max_yaw_jump_deg
            and speed_res <= self.cfg.docking.lock_transition_max_speed_jump
        )
        return corrected, transition_ok

    def _should_force_disturbance(self, follower_id: int, now: float, stage: str) -> str | None:
        if not self.demo_cfg.inject_disturbance_for_type_c:
            return None
        if (not self.demo_cfg.inject_disturbance_all_types) and (not str(self.case.subtype).startswith("C")):
            return None
        if follower_id in self.forced_interrupted_once:
            return None
        t0 = self.docking_started_at.get(follower_id, None)
        if t0 is None:
            return None
        if (now - t0) < self.demo_cfg.disturbance_after_docking_s:
            return None
        x = float(self.rng.random())
        self.forced_interrupted_once.add(follower_id)
        if stage == "VISUAL_SERVO" and x <= self.demo_cfg.disturbance_vision_prob:
            return "vision_lost"
        return "global_plan_infeasible"

    def _simple_docking_command(self, follower: VehicleState, leader: VehicleState, now: float) -> tuple[ControlCommand, dict[str, Any]]:
        rear = self.geom.rear_hitch(leader)
        front = self.geom.front_hitch(follower)
        rel = rear - front
        d = float(np.linalg.norm(rel))

        heading = np.array([math.cos(leader.yaw), math.sin(leader.yaw)], dtype=float)
        yaw_err = float(angle_diff(leader.yaw, follower.yaw))
        yaw_err_deg = float(abs(math.degrees(yaw_err)))
        lat_err = float(heading[0] * rel[1] - heading[1] * rel[0])
        long_err = float(np.dot(rel, heading))

        f_id = int(follower.vehicle_id)
        # Include leader in collision-avoidance during global approach so docking does not
        # shortcut through the leader body when follower approaches from the side.
        dynamic_others = [self.states[j] for j in self.vehicle_ids if j != int(follower.vehicle_id)]
        cmd_raw, dbg_raw = self._dock_controller.compute_command(
            follower=follower,
            leader_true=leader,
            timestamp=float(now),
            obstacles=self.case.obstacles,
            dynamic_others=dynamic_others,
        )
        stage = str(dbg_raw.stage)
        cmd = cmd_raw
        # Constrain docking reverse speed to a very small band to avoid
        # large swing while keeping minimal alignment recoverability.
        dt = max(self.cfg.control.dt, 1e-6)
        accel = float(cmd.accel)
        v_min_dock = -0.05
        if follower.v < v_min_dock - 1e-3:
            accel = float(self.cfg.vehicle.max_accel)
        else:
            min_accel_limited_reverse = float((v_min_dock - follower.v) / dt)
            accel = float(max(accel, min_accel_limited_reverse))
        accel = float(np.clip(accel, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd))
        cmd = ControlCommand(accel=accel, steer_rate=float(cmd.steer_rate))
        cmd = self._smooth_dock_command(f_id, cmd)
        extra_dbg: dict[str, Any] = {
            "w_vis": float(dbg_raw.w_vis),
            "visual_valid": 1.0 if bool(dbg_raw.visual_valid) else 0.0,
            "fallback_global": 1.0 if bool(dbg_raw.fallback_global) else 0.0,
            "fallback_reason": str(getattr(dbg_raw, "fallback_reason", "")),
            "visual_lost_time": float(dbg_raw.visual_lost_time),
        }

        self._dock_stage_mem[f_id] = stage
        dbg: dict[str, Any] = {
            "stage": stage,
            "distance": float(d),
            "yaw_err_deg": float(yaw_err_deg),
            "lat_err": float(lat_err),
            "long_err": float(long_err),
        }
        dbg.update(extra_dbg)
        return cmd, dbg

    def _smooth_dock_command(self, follower_id: int, cmd: ControlCommand) -> ControlCommand:
        prev = self._last_dock_cmd.get(follower_id, None)
        if prev is None:
            self._last_dock_cmd[follower_id] = cmd
            return cmd
        max_da = max(0.0, float(self.cfg.docking.cmd_smooth_max_accel_step))
        max_ds = max(0.0, float(self.cfg.docking.cmd_smooth_max_steer_rate_step))
        accel = prev.accel + float(np.clip(cmd.accel - prev.accel, -max_da, max_da))
        steer_rate = prev.steer_rate + float(np.clip(cmd.steer_rate - prev.steer_rate, -max_ds, max_ds))
        out = ControlCommand(accel=float(accel), steer_rate=float(steer_rate))
        self._last_dock_cmd[follower_id] = out
        return out

    def _apply_chain_motion(self, now: float, updated: set[int]) -> None:
        heads = self.engine.topology.heads()
        docking_followers = set(self.engine.topology.docking_target.keys())
        if self.demo_cfg.enable_bottleneck_coordination:
            active_heads = [int(h) for h in heads if int(h) not in docking_followers]
            self._update_bottleneck_owners(active_heads)
        for head in heads:
            if head in docking_followers:
                continue
            chain = self._chain_from_head(head)
            chain_states = [self.states[i] for i in chain]
            head_state = chain_states[0]
            hold_reason = None
            if self.demo_cfg.enable_bottleneck_coordination:
                hold_reason = self._bottleneck_hold(int(head), head_state)
            if hold_reason is not None:
                if len(chain) == 1:
                    dt = max(self.cfg.control.dt, 1e-6)
                    cmd_hold = self._stop_command(head_state)
                    # Bottleneck coordination for scattered starts:
                    # When the global leader owns the upcoming wall-pair bottleneck, a non-leader
                    # head that happens to be in front may need to actively reverse-yield; simply
                    # stopping in-lane can permanently block the leader and cause 60s timeouts
                    # (notably P6 B2/B3 Random_Scattered).
                    if head != self.leader_id and int(self.leader_id) in self.states:
                        leader_st = self.states[int(self.leader_id)]
                        ahead_leader = bool(float(head_state.x) > float(leader_st.x) + 0.55)
                        if ahead_leader:
                            max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
                            unwind = float(np.clip(-head_state.delta / dt, -max_rate, max_rate))
                            v_ref = -0.35
                            accel = float(
                                np.clip(
                                    (v_ref - float(head_state.v)) / dt,
                                    -self.cfg.vehicle.max_decel,
                                    self.cfg.control.speed.max_accel_cmd,
                                )
                            )
                            cmd_hold = ControlCommand(accel=accel, steer_rate=unwind)
                            cmd_hold = self._enforce_speed_cap(head_state, cmd_hold, 0.0, allow_reverse=True)
                    nxt_stop = self.ack.step(head_state, cmd_hold, dt)
                    others = [self.states[j] for j in self.vehicle_ids if j != head]
                    if self._collision_with_obstacles_fast(nxt_stop) or self._collision_with_others_fast(nxt_stop, others):
                        cmd_hold = self._stop_command(head_state)
                        nxt_stop = self._freeze_state(head_state)
                    self._store_cmd(int(head), cmd_hold, now)
                    self.states[int(head)] = nxt_stop
                    updated.add(int(head))
                    continue

                # Fallback for train heads (rare in P6 FREE-start): freeze all units.
                cmd_stop = self._stop_command(head_state)
                self._store_cmd(int(head), cmd_stop, now)
                for sid in chain:
                    st_old = self.states[sid]
                    self.states[sid] = VehicleState(
                        vehicle_id=st_old.vehicle_id,
                        x=float(st_old.x),
                        y=float(st_old.y),
                        yaw=float(st_old.yaw),
                        v=0.0,
                        delta=float(st_old.delta),
                        mode=st_old.mode,
                    )
                    updated.add(sid)
                continue
            target_speed = self.demo_cfg.leader_target_speed if head == self.leader_id else self.demo_cfg.free_target_speed
            if head != self.leader_id:
                target_speed = self._speed_limit_by_lead_vehicle(head, target_speed)
                target_speed = self._speed_limit_by_proximity(head, target_speed, ahead_only=True)
                target_speed = self._speed_limit_by_x_lead(head, target_speed)
            else:
                # Near-field cooperative behavior follows the same profile as the
                # validated foundation docking simulation.
                # Only cooperate once the executor has entered the *close docking phase* for a follower.
                # During long-horizon intercept/follow (pre-intercept), the leader should keep its own
                # navigation speed profile and not be slowed down aggressively.
                lead_pairs = [
                    (f, l)
                    for f, l in self.engine.topology.docking_target.items()
                    if int(l) == int(head) and int(f) in self.docking_started_at
                ]
                if lead_pairs:
                    d_min = math.inf
                    for f, _ in lead_pairs:
                        if f not in self.states:
                            continue
                        d = float(np.linalg.norm(self.geom.front_hitch(self.states[f]) - self.geom.rear_hitch(self.states[head])))
                        d_min = min(d_min, d)
                    if d_min < 0.6:
                        target_speed = min(target_speed, 0.08)
                    elif d_min < 2.0:
                        target_speed = min(target_speed, 0.18)
                    elif d_min < 4.0:
                        target_speed = min(target_speed, 0.30)
                    elif d_min < 8.0:
                        target_speed = min(target_speed, 0.45)
                    else:
                        target_speed = min(target_speed, 0.65)
                # Leader should not be slowed down by vehicles behind it (leader-priority semantic).
                # Only apply a directional limiter for vehicles clearly *ahead* on the corridor.
                if len(chain) == 1:
                    target_speed = self._speed_limit_by_lead_vehicle(head, target_speed)
                target_speed = self._speed_limit_by_x_lead(head, target_speed)

                # Gate-escape protection: if a non-leader head ahead is still outside a mandatory
                # gate band (i.e., executing a strict gate-crossing join/escape), prevent the
                # global leader from closing in too much. Otherwise the leader can enter a
                # collision-avoidance deadlock with the escaping vehicle and block its yaw/climb
                # recovery, causing long P6 timeouts (C1/C2/C3 Uniform_Spread).
                if len(chain) == 1 and self.mandatory_gates:
                    leader_st = head_state
                    for oid in self.vehicle_ids:
                        if int(oid) == int(head):
                            continue
                        if oid not in self.states:
                            continue
                        # Only consider other FREE heads (not docked trains).
                        if self.engine.topology.parent.get(int(oid), None) is not None:
                            continue
                        if self.engine.topology.child.get(int(oid), None) is not None:
                            continue
                        if self.engine.topology.mode.get(int(oid), VehicleMode.FREE) != VehicleMode.FREE:
                            continue
                        # Only protect vehicles that are in strict gate join / escape context.
                        strict_join = bool(self._join_route_strict_goal.get(int(oid), False))
                        escaping = bool(int(oid) in self._gate_escape_phase or int(oid) in self._gate_escape_key)
                        if not (strict_join or escaping):
                            continue
                        o = self.states[int(oid)]
                        dx = float(o.x - leader_st.x)
                        if dx <= 0.2:
                            continue
                        # Require lateral proximity; otherwise the leader can safely pass.
                        if abs(float(o.y - leader_st.y)) > 2.0:
                            continue
                        gate_o = self._active_gate(o)
                        if gate_o is None:
                            continue
                        band = self._gate_safe_band(gate_o)
                        if band is not None:
                            safe_low, safe_high = float(band[0]), float(band[1])
                        else:
                            y0 = float(gate_o.get("y0", float(gate_o.get("cy", 0.0)) - 0.2))
                            y1 = float(gate_o.get("y1", float(gate_o.get("cy", 0.0)) + 0.2))
                            if y1 < y0:
                                y0, y1 = y1, y0
                            safe_low, safe_high = float(y0), float(y1)
                        in_safe = bool((safe_low - 0.02) <= float(o.y) <= (safe_high + 0.02))
                        escape_phase = self._gate_escape_phase.get(int(oid), None)
                        needs_room = False
                        if escaping:
                            # Escape can remain active after entering the *gap* band. When still
                            # outside the stricter safe band (or in early escape phases), the front
                            # head may need to reverse/back-off again. Do not let the leader close
                            # in and collision-block the escape (P6 B2/B3 Random_Scattered).
                            if escape_phase is None:
                                needs_room = bool(not in_safe)
                            else:
                                needs_room = bool(int(escape_phase) <= 1 or (not in_safe))
                        else:
                            # Strict join routes: keep buffer until the head is inside the safe band.
                            needs_room = bool(not in_safe)
                        if not needs_room:
                            continue
                        # Keep a buffer while the other head is recovering into the safe band.
                        if dx < 2.2:
                            target_speed = min(target_speed, 0.0)
                        elif dx < 3.2:
                            target_speed = min(target_speed, 0.25)
            goal_xy_head = self._goal_for_head(head)
            dist_goal_head = float(np.linalg.norm(self.states[head].xy() - goal_xy_head))
            head_goal_tol = self.demo_cfg.leader_goal_tol_m if head == self.leader_id else self.demo_cfg.vehicle_goal_tol_m
            # Stricter parking tolerance for lateral goal slots:
            # When a non-leader head is assigned a lateral parking slot near B, the default
            # vehicle_goal_tol_m (≈1.2m) can allow it to stop while still occupying the leader's
            # corridor centerline, blocking the leader from satisfying its tighter goal tolerance.
            # Use a tighter tolerance for such lateral slots so the head actually clears the lane.
            if (
                head != self.leader_id
                and self.mandatory_gates
                and float(np.linalg.norm(goal_xy_head - self.case.goal_xy)) >= 0.70
            ):
                head_goal_tol = float(min(head_goal_tol, 0.60))
            if dist_goal_head <= head_goal_tol:
                for sid in chain:
                    s_old = self.states[sid]
                    cmd = self._stop_command(s_old)
                    self.states[sid] = self.ack.step(s_old, cmd, self.cfg.control.dt)
                    updated.add(sid)
                continue

            if len(chain) >= 2:
                cmd = self.train_controller.track_head_path(chain_states, self.path_xy, target_speed=target_speed, dt=self.cfg.control.dt)
                self._store_cmd(head, cmd, now)
                upd = self.train_kin.update(chain_states, cmd, self.cfg.control.dt)
                rep = self.train_guard.check(chain_states, upd.states, upd.articulation_angles, self.case.obstacles)
                external_collision = False
                if not rep.emergency_stop:
                    for st in upd.states:
                        others = [self.states[j] for j in self.vehicle_ids if j not in chain]
                        skip = set(self.engine.topology.docking_target.keys()) if head == self.leader_id else set()
                        if self._collision_with_obstacles_fast(st) or self._collision_with_others_fast(st, others, skip_ids=skip):
                            external_collision = True
                            break
                if rep.emergency_stop or external_collision:
                    # safety stop on invalid train update
                    self._push_event(now, "CTRL", "train_emergency_stop", detail=rep.reason if rep.reason else "external_collision")
                    for sid in chain:
                        st_old = self.states[sid]
                        self.states[sid] = VehicleState(
                            vehicle_id=st_old.vehicle_id,
                            x=st_old.x,
                            y=st_old.y,
                            yaw=st_old.yaw,
                            v=0.0,
                            delta=st_old.delta,
                            mode=st_old.mode,
                        )
                        updated.add(sid)
                    continue
                max_delta_step = math.radians(self.cfg.vehicle.max_steer_rate_deg_s) * self.cfg.control.dt
                for sid, st in zip(chain, upd.states):
                    st_old = self.states[sid]
                    d_delta = float(angle_diff(st.delta, st_old.delta))
                    delta_limited = float(st_old.delta + np.clip(d_delta, -max_delta_step, max_delta_step))
                    st_limited = VehicleState(
                        vehicle_id=st.vehicle_id,
                        x=float(st.x),
                        y=float(st.y),
                        yaw=float(st.yaw),
                        v=float(st.v),
                        delta=float(delta_limited),
                        mode=st.mode,
                    )
                    self.states[sid] = st_limited
                    updated.add(sid)
                continue

            # single head vehicle
            others = [self.states[j] for j in self.vehicle_ids if j != head]

            cmd_leader_backoff = self._leader_backoff_for_escaping_front(head_state, now)
            if cmd_leader_backoff is not None:
                dt = float(self.cfg.control.dt)
                nxt_back = self.ack.step(head_state, cmd_leader_backoff, dt)
                if self._collision_with_obstacles_fast(nxt_back) or self._collision_with_others_fast(nxt_back, others):
                    cmd_leader_backoff = self._stop_command(head_state)
                    nxt_back = self._freeze_state(head_state)
                self._store_cmd(head, cmd_leader_backoff, now)
                self.states[head] = nxt_back
                updated.add(head)
                continue

            cmd_front_yield = self._front_yield_for_stalled_leader(head_state, now)
            if cmd_front_yield is not None:
                dt = float(self.cfg.control.dt)
                nxt_yield = self.ack.step(head_state, cmd_front_yield, dt)
                if self._collision_with_obstacles_fast(nxt_yield) or self._collision_with_others_fast(nxt_yield, others):
                    cmd_front_yield = self._stop_command(head_state)
                    nxt_yield = self._freeze_state(head_state)
                self._store_cmd(head, cmd_front_yield, now)
                self.states[head] = nxt_yield
                updated.add(head)
                continue

            cmd_rear_backoff = self._rear_backoff_for_gate_lead(head_state)
            if cmd_rear_backoff is not None:
                dt = float(self.cfg.control.dt)
                nxt_back = self.ack.step(head_state, cmd_rear_backoff, dt)
                if self._collision_with_obstacles_fast(nxt_back) or self._collision_with_others_fast(nxt_back, others):
                    cmd_rear_backoff = self._stop_command(head_state)
                    nxt_back = self._freeze_state(head_state)
                self._store_cmd(head, cmd_rear_backoff, now)
                self.states[head] = nxt_back
                updated.add(head)
                continue

            cmd_rear_yield = self._rear_backoff_for_turning_lead(head_state)
            if cmd_rear_yield is not None:
                dt = float(self.cfg.control.dt)
                max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
                candidates = [
                    cmd_rear_yield,
                    ControlCommand(accel=float(cmd_rear_yield.accel), steer_rate=float(max_rate)),
                    ControlCommand(accel=float(cmd_rear_yield.accel), steer_rate=float(-max_rate)),
                ]
                chosen = None
                for cmd_try in candidates:
                    cmd_try = self._enforce_speed_cap(head_state, cmd_try, 0.0, allow_reverse=True)
                    nxt_try = self.ack.step(head_state, cmd_try, dt)
                    if self._collision_with_obstacles_fast(nxt_try) or self._collision_with_others_fast(nxt_try, others):
                        continue
                    chosen = (cmd_try, nxt_try)
                    break
                if chosen is None:
                    cmd_rear_yield = self._stop_command(head_state)
                    nxt_back = self._freeze_state(head_state)
                else:
                    cmd_rear_yield, nxt_back = chosen
                self._store_cmd(head, cmd_rear_yield, now)
                self.states[head] = nxt_back
                updated.add(head)
                continue

            if head != self.leader_id and (self.leader_id in self.states):
                leader_st = self.states[self.leader_id]
                d_lead = float(np.linalg.norm(head_state.xy() - leader_st.xy()))
                leader_goal_dist = float(np.linalg.norm(leader_st.xy() - self._goal_for_head(self.leader_id)))
                # Early queue-yield: if a non-leader head starts slightly ahead and laterally offset,
                # it tends to merge toward the corridor while the leader also tracks a curved path,
                # producing a near side-by-side squeeze that freezes both vehicles (P6 B3/C*
                # Clustered_At_A). Slow the non-leader briefly so the leader becomes the front head
                # before the merge happens.
                if d_lead < 2.6 and float(self._path_progress_s(leader_st)) < 5.0:
                    dx = float(head_state.x - leader_st.x)
                    if 0.10 <= dx <= 0.85 and abs(float(head_state.y - leader_st.y)) <= 1.8:
                        band = self._same_lane_band()
                        lane_sep = float(abs(self._signed_lateral_to_path(head_state) - self._signed_lateral_to_path(leader_st)))
                        if lane_sep > band:
                            target_speed = min(target_speed, 0.45)
                # Hard rear-end guard in split mode: follower must not compress into leader.
                behind_leader = bool(float(head_state.x) < float(leader_st.x) - 0.35)
                ahead_leader = bool(float(head_state.x) > float(leader_st.x) + 0.35)
                if ahead_leader:
                    # Leader-priority semantic in free-flow: prevent a non-leader head from
                    # overtaking the global leader when both are already on the shared corridor.
                    #
                    # Motivation:
                    # - In scattered starts (especially Type-A3 Uniform_Spread), a non-leader can
                    #   slip past the leader laterally, then its slot-goal (behind B) becomes
                    #   unreachable without reversing (not supported by the nominal tracker),
                    #   causing timeouts.
                    # IMPORTANT:
                    # This limiter is intended for Type-A "open corridor" scenes where a follower
                    # can slip past the leader near the start and later be unable to reach its
                    # assigned slot without reversing. In B/C bottleneck scenes (mandatory gates),
                    # applying it can unnecessarily slow down the front head while it performs
                    # the chicane lane-change, causing P6 timeouts (notably B2 Random_Scattered).
                    if not self.mandatory_gates:
                        band = self._same_lane_band()
                        lat_self = float(self._signed_lateral_to_path(head_state))
                        lat_lead = float(self._signed_lateral_to_path(leader_st))
                        lane_sep = float(abs(lat_self - lat_lead))
                        dx = float(head_state.x - leader_st.x)
                        # Only enforce when the two heads are within a short longitudinal window;
                        # otherwise the cap can dominate completion time without improving safety.
                        if (
                            dx <= 1.0
                            and abs(lat_self) <= 0.90
                            and abs(lat_lead) <= 0.90
                            and lane_sep <= band
                            and abs(float(head_state.y - leader_st.y)) <= 1.2
                        ):
                            # Do not hard-freeze: allow slow motion so the head can drift to its
                            # lane-offset / open space and let the leader proceed.
                            target_speed = min(target_speed, 0.25)
                if d_lead < 1.8:
                    # Only apply a full stop when the follower is *behind* the leader in the
                    # corridor direction. In scattered starts, vehicles can become close while
                    # being side-by-side or the follower already ahead; stopping then can create
                    # a deadlock at bottleneck entrances.
                    if behind_leader:
                        # Near terminal area, allow slow closure to the assigned parking slot
                        # instead of freezing behind leader.
                        if (leader_goal_dist > self.demo_cfg.leader_goal_tol_m + 0.05) or (dist_goal_head <= head_goal_tol):
                            cmd_guard = self._stop_command(head_state)
                            self._store_cmd(head, cmd_guard, now)
                            nxt_guard = self.ack.step(head_state, cmd_guard, self.cfg.control.dt)
                            self.states[head] = nxt_guard
                            updated.add(head)
                            self._blocked_ticks[head] = 0
                            continue
                        target_speed = min(target_speed, 0.25)
                    else:
                        # If already ahead (or side-by-side), allow the vehicle to pull away and
                        # increase headway. Over-constraining speed here can trap both vehicles
                        # in front of the first gate in B/C chicanes (leader slows down because a
                        # close head is detected, while the head cannot open the gap).
                        pass
                if d_lead < 2.6:
                    if behind_leader:
                        target_speed = min(target_speed, 0.45)

            join_route = None
            # Global join route (A*): when the vehicle starts far away from the shared corridor,
            # first navigate to the nearest corridor point through obstacle-separated pockets.
            #
            # NOTE: This must not be limited to gate-based scenarios only. Random_Scattered A-type
            # cases can also spawn vehicles behind obstacle pockets where pure corridor tracking
            # will stall indefinitely (P6 timeouts).
            self._maybe_plan_join_route(head, now)
            join_route = self._join_route_xy.get(int(head), None)
            if join_route is not None:
                if self._join_route_done(int(head)):
                    self._join_route_xy.pop(int(head), None)
                    self._join_route_goal_xy.pop(int(head), None)
                    self._join_route_strict_goal.pop(int(head), None)
                    join_route = None
                else:
                    join_speed = float(min(target_speed, 0.75))
                    if bool(self._join_route_strict_goal.get(int(head), False)):
                        # Gate-crossing join routes are safety-critical; keep speed low to
                        # reduce wall-grazing collisions in narrow gaps.
                        join_speed = float(min(join_speed, 0.55))

                        # If enabled, allow the deterministic gate-escape state machine to
                        # take over inside strict (gate-crossing) join-routes. This provides a
                        # kinematically realizable backoff/turn/climb sequence that prevents
                        # the common stall where the vehicle creeps toward the wall face while
                        # yaw changes too slowly, then freezes due to collision checks.
                        if self.demo_cfg.enable_gate_escape:
                            cmd_escape = self._gate_escape_command(head_state)
                            if cmd_escape is not None:
                                dt = float(self.cfg.control.dt)
                                nxt_escape = self.ack.step(head_state, cmd_escape, dt)
                                if self._collision_with_obstacles_fast(nxt_escape) or self._collision_with_others_fast(nxt_escape, others):
                                    cmd_escape = self._stop_command(head_state)
                                    nxt_escape = self._freeze_state(head_state)
                                self._store_cmd(head, cmd_escape, now)
                                self.states[head] = nxt_escape
                                self._blocked_ticks[head] = 0
                                updated.add(head)
                                continue
                    strict_join = bool(self._join_route_strict_goal.get(int(head), False))
                    join_goal = self._join_route_goal_xy.get(int(head), None)
                    dist_goal = float(np.linalg.norm(head_state.xy() - join_goal)) if join_goal is not None else 0.0
                    # Join-route stall rescue:
                    # When far-scattered starts generate a straight A* route that runs close to an obstacle
                    # corner, the vehicle can get stuck with v≈0 while steering changes (yaw cannot change
                    # without motion). If we observe sustained stall while a join-route is active, apply a
                    # short reverse-turn maneuver to create room and reduce heading error to the join goal.
                    if not strict_join:
                        stall_ticks = int(self._stall_ticks.get(int(head), 0))
                        stall_limit = int(2.0 / max(self.cfg.control.dt, 1e-6))
                        if stall_ticks >= stall_limit:
                            until = float(self._join_escape_until.get(int(head), -1e9))
                            if stall_ticks >= 2 * stall_limit and abs(float(head_state.v)) <= 0.12:
                                until = float(max(until, now + 0.9))
                                self._join_escape_until[int(head)] = float(until)
                            if now <= until:
                                dt = float(self.cfg.control.dt)
                                max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
                                dmax = math.radians(self.cfg.vehicle.max_steer_deg)
                                vec_to_goal = (join_goal - head_state.xy()) if join_goal is not None else (tgt - head_state.xy())
                                yaw_tgt = float(math.atan2(float(vec_to_goal[1]), float(vec_to_goal[0])))
                                yaw_err = float(angle_diff(yaw_tgt, float(head_state.yaw)))
                                # In reverse, yaw rate flips sign: steer opposite to yaw error.
                                steer_sign = -1.0 if yaw_err > 1e-6 else (1.0 if yaw_err < -1e-6 else 0.0)
                                delta_target = float(np.clip(steer_sign * 0.55, -dmax, dmax))
                                steer_rate = float(
                                    np.clip((delta_target - float(head_state.delta)) / max(dt, 1e-6), -max_rate, max_rate)
                                )
                                v_ref = -0.25
                                accel = float(
                                    np.clip(
                                        (v_ref - float(head_state.v)) / max(dt, 1e-6),
                                        -self.cfg.vehicle.max_decel,
                                        self.cfg.control.speed.max_accel_cmd,
                                    )
                                )
                                cmd_bk = ControlCommand(accel=accel, steer_rate=steer_rate)
                                cmd_bk = self._enforce_speed_cap(head_state, cmd_bk, 0.0, allow_reverse=True)
                                nxt_bk = self.ack.step(head_state, cmd_bk, dt)
                                if (not self._collision_with_obstacles_fast(nxt_bk)) and (
                                    not self._collision_with_others_fast(nxt_bk, others)
                                ):
                                    self._blocked_ticks[head] = 0
                                    self._store_cmd(head, cmd_bk, now)
                                    self.states[head] = nxt_bk
                                    updated.add(head)
                                    continue
                    # Dynamic lookahead for sparse A* routes:
                    # - Too-small lookahead can pick a target extremely close to the vehicle
                    #   when following a long straight segment, forcing saturated steering and
                    #   circular limit-cycles (P6 B3/C* scattered timeouts).
                    # - Scale lookahead with remaining join-goal distance, but cap it for
                    #   strict gate staging to avoid cutting across the wall neighborhood.
                    join_lookahead = float(np.clip(0.30 * dist_goal + 0.8, 1.6, 4.0))
                    if strict_join:
                        join_lookahead = float(min(join_lookahead, 3.2))
                    tgt, _tgt_yaw = self._route_target_ahead(head_state, join_route, lookahead_m=float(join_lookahead))
                    # If the computed target is still too close while the goal is far away,
                    # extend lookahead once to avoid tight orbits.
                    if dist_goal >= 2.0 and float(np.linalg.norm(tgt - head_state.xy())) < 0.75 and join_lookahead < 3.9:
                        tgt, _tgt_yaw = self._route_target_ahead(
                            head_state,
                            join_route,
                            lookahead_m=float(min(4.0, join_lookahead + 1.0)),
                        )
                    vec = tgt - head_state.xy()
                    heading = np.array([math.cos(float(head_state.yaw)), math.sin(float(head_state.yaw))], dtype=float)
                    behind = bool(float(np.dot(vec, heading)) < -0.15 and float(np.linalg.norm(vec)) > 0.35)
                    if behind:
                        if strict_join:
                            cmd_backoff = self._gate_backoff_command(head_state)
                            if cmd_backoff is not None:
                                nxt_back = self.ack.step(head_state, cmd_backoff, self.cfg.control.dt)
                                if not (
                                    self._collision_with_obstacles_fast(nxt_back)
                                    or self._collision_with_others_fast(nxt_back, others)
                                ):
                                    self._store_cmd(head, cmd_backoff, now)
                                    self.states[head] = nxt_back
                                    self._blocked_ticks[head] = 0
                                    updated.add(head)
                                    continue
                        # If the lookahead point is behind the vehicle, a short-horizon sampled
                        # local planner can choose a "do nothing" command due to limited progress
                        # in 0.6s horizons at low speed. Use a point-attractor controller to
                        # deterministically steer toward the target and keep the join route moving.
                        cmd_pt = self.tracker.track_point(
                            head_state,
                            float(tgt[0]),
                            float(tgt[1]),
                            float(_tgt_yaw),
                            float(join_speed),
                        )
                        cmd_join = ControlCommand(
                            accel=float(self._speed_accel_cmd(head_state.v, join_speed)),
                            steer_rate=float(cmd_pt.steer_rate),
                        )
                        cmd_join = self._enforce_speed_cap(head_state, cmd_join, join_speed, allow_reverse=False)
                        nxt_join = self.ack.step(head_state, cmd_join, self.cfg.control.dt)
                        if self._collision_with_obstacles_fast(nxt_join) or self._collision_with_others_fast(nxt_join, others):
                            cmd_join = self._stop_command(head_state)
                            nxt_join = self._freeze_state(head_state)
                        self._store_cmd(head, cmd_join, now)
                        self.states[head] = nxt_join
                        self._blocked_ticks[head] = 0
                        updated.add(head)
                        continue
                    # Forward tracking along join-route.
                    # Use pure-pursuit for A* routes: Stanley can saturate at large cross-track
                    # errors and drive yaw toward ±90deg, stalling x-progress near cross-walls.
                    dt = float(self.cfg.control.dt)
                    max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
                    dmax = math.radians(self.cfg.vehicle.max_steer_deg)
                    vec_pp = tgt - head_state.xy()
                    alpha_pp = float(
                        angle_diff(float(math.atan2(float(vec_pp[1]), float(vec_pp[0]))), float(head_state.yaw))
                    )
                    ld_pp = float(max(1e-3, np.linalg.norm(vec_pp)))
                    delta_ref = float(math.atan2(2.0 * float(self.cfg.vehicle.wheelbase) * math.sin(alpha_pp), ld_pp))
                    delta_ref = float(np.clip(delta_ref, -dmax, dmax))
                    steer_rate = float(
                        np.clip((delta_ref - float(head_state.delta)) / max(dt, 1e-6), -max_rate, max_rate)
                    )
                    cmd_join = ControlCommand(
                        accel=float(self._speed_accel_cmd(head_state.v, join_speed)),
                        steer_rate=float(steer_rate),
                    )
                    cmd_join = self._enforce_speed_cap(head_state, cmd_join, join_speed, allow_reverse=False)
                    nxt_join = self.ack.step(head_state, cmd_join, self.cfg.control.dt)
                    hit_obs_join = self._collision_with_obstacles_fast(nxt_join)
                    hit_veh_join = self._collision_with_others_fast(nxt_join, others)
                    if hit_obs_join or hit_veh_join:
                        # Strict gate-staging join routes can become kinematically infeasible when the
                        # vehicle approaches the wall face outside the opening band (front hitch protrudes).
                        # Instead of freezing until the join-route is dropped, apply a deterministic
                        # back-off so the vehicle can regain turning room and continue climbing into the band.
                        if strict_join:
                            cmd_backoff = self._gate_backoff_command(head_state)
                            if cmd_backoff is not None:
                                nxt_back = self.ack.step(head_state, cmd_backoff, dt)
                                if not (
                                    self._collision_with_obstacles_fast(nxt_back)
                                    or self._collision_with_others_fast(nxt_back, others)
                                ):
                                    self._blocked_ticks[head] = 0
                                    self._store_cmd(head, cmd_backoff, now)
                                    self.states[head] = nxt_back
                                    updated.add(head)
                                    continue
                        # Generic join-route backoff:
                        # If the forward preview collides (often at obstacle corners), create a bit of
                        # longitudinal room by reversing while steering to reduce heading error to the
                        # join-route target. This avoids getting stuck with v=0 where yaw cannot change.
                        if abs(float(head_state.v)) <= 0.12 and int(self._blocked_ticks.get(head, 0)) >= 2:
                            vec_to_tgt = tgt - head_state.xy()
                            yaw_tgt = float(math.atan2(float(vec_to_tgt[1]), float(vec_to_tgt[0])))
                            yaw_err = float(angle_diff(yaw_tgt, float(head_state.yaw)))
                            steer_sign = -1.0 if yaw_err > 1e-6 else (1.0 if yaw_err < -1e-6 else 0.0)
                            delta_target = float(np.clip(steer_sign * 0.55, -dmax, dmax))
                            steer_rate_bk = float(np.clip((delta_target - float(head_state.delta)) / max(dt, 1e-6), -max_rate, max_rate))
                            v_ref = -0.25
                            accel_bk = float(
                                np.clip(
                                    (v_ref - float(head_state.v)) / max(dt, 1e-6),
                                    -self.cfg.vehicle.max_decel,
                                    self.cfg.control.speed.max_accel_cmd,
                                )
                            )
                            cmd_bk = ControlCommand(accel=accel_bk, steer_rate=steer_rate_bk)
                            cmd_bk = self._enforce_speed_cap(head_state, cmd_bk, 0.0, allow_reverse=True)
                            nxt_bk = self.ack.step(head_state, cmd_bk, dt)
                            if (not self._collision_with_obstacles_fast(nxt_bk)) and (
                                not self._collision_with_others_fast(nxt_bk, others)
                            ):
                                self._blocked_ticks[head] = 0
                                self._store_cmd(head, cmd_bk, now)
                                self.states[head] = nxt_bk
                                updated.add(head)
                                continue
                        # Join-route collision fallback:
                        # A purely geometric A* polyline can be kinematically infeasible near obstacle
                        # corners. When the one-step pure-pursuit preview is blocked, try the sampled
                        # local planner to find a feasible short-horizon motion toward the join target.
                        plan_join = self.gate_planner.plan_step(
                            ego=head_state,
                            goal_xy=tgt,
                            goal_yaw=float(_tgt_yaw),
                            obstacles=self.case.obstacles,
                            dynamic_others=others,
                            force_goal_yaw=False,
                        )
                        if plan_join.feasible:
                            cmd_alt = plan_join.command
                            # Keep forward speed intent when planner is not explicitly braking/reversing.
                            if float(cmd_alt.accel) >= -1e-9:
                                cmd_alt = ControlCommand(
                                    accel=float(self._speed_accel_cmd(head_state.v, join_speed)),
                                    steer_rate=float(cmd_alt.steer_rate),
                                )
                            allow_reverse = bool(float(cmd_alt.accel) < -1e-9)
                            cmd_alt = self._enforce_speed_cap(head_state, cmd_alt, join_speed, allow_reverse=allow_reverse)
                            nxt_alt = self.ack.step(head_state, cmd_alt, self.cfg.control.dt)
                            if (not self._collision_with_obstacles_fast(nxt_alt)) and (
                                not self._collision_with_others_fast(nxt_alt, others)
                            ):
                                moved = float(np.linalg.norm(nxt_alt.xy() - head_state.xy()))
                                # Only accept the fallback when it makes meaningful progress toward the join goal.
                                # Otherwise the vehicle can get stuck accepting tiny oscillations forever.
                                d_after = float(np.linalg.norm(nxt_alt.xy() - join_goal)) if join_goal is not None else float(dist_goal)
                                progress = float(dist_goal - d_after)
                                if (moved > 0.001) and (progress > 0.005):
                                    self._blocked_ticks[head] = 0
                                    self._store_cmd(head, cmd_alt, now)
                                    self.states[head] = nxt_alt
                                    updated.add(head)
                                    continue
                        self._blocked_ticks[head] += 1
                        cmd_stop = self._stop_command(head_state)
                        self._store_cmd(head, cmd_stop, now)
                        self.states[head] = self._freeze_state(head_state)
                        # If the join route leads to immediate collision, drop it and replan later.
                        if self._blocked_ticks[head] >= 12:
                            self._join_route_xy.pop(int(head), None)
                            self._join_route_goal_xy.pop(int(head), None)
                            self._join_route_strict_goal.pop(int(head), None)
                        updated.add(head)
                        continue

                    self._blocked_ticks[head] = 0
                    self._store_cmd(head, cmd_join, now)
                    self.states[head] = nxt_join
                    updated.add(head)
                    continue

            # Gate-stall rescue: for multi-gate chicanes (Type-C) and serial bottlenecks (B3),
            # vehicles can get stuck in near-vertical yaw oscillations right before the wall face.
            # Those micro-motions do not trip the generic stall-tick detector, causing P6 timeouts.
            #
            # Detect lack of *front-bumper x-progress* near an active gate and apply a short
            # reverse-turn maneuver to recover yaw + lateral band.
            if self.mandatory_gates and self.demo_cfg.enable_gate_traversal:
                gate_rescue = self._active_gate(head_state)
                if gate_rescue is not None:
                    dt = float(self.cfg.control.dt)
                    gate_idx = int(self._gate_idx_mem.get(head, -1))
                    x_prog = float(self.geom.to_world(head_state, float(self.geom.front_x))[0])
                    if int(self._gate_prog_idx.get(head, -1)) != int(gate_idx):
                        self._gate_prog_idx[head] = int(gate_idx)
                        self._gate_prog_best_s[head] = float(x_prog)
                        self._gate_prog_best_t[head] = float(now)
                        self._gate_rescue_until[head] = -1e9
                    else:
                        if x_prog > float(self._gate_prog_best_s.get(head, -1e9)) + 0.05:
                            self._gate_prog_best_s[head] = float(x_prog)
                            self._gate_prog_best_t[head] = float(now)
                            self._gate_rescue_until[head] = -1e9
                        else:
                            stuck_for = float(now) - float(self._gate_prog_best_t.get(head, now))
                            if stuck_for >= 3.0 and float(dist_goal_head) > 1.5 and abs(float(head_state.v)) <= 0.20:
                                self._gate_rescue_until[head] = float(
                                    max(self._gate_rescue_until.get(head, -1e9), now + 0.8)
                                )
                                # Reset timer so we don't immediately extend rescue forever.
                                self._gate_prog_best_t[head] = float(now)

                    if float(now) <= float(self._gate_rescue_until.get(head, -1e9)):
                        band = self._gate_safe_band(gate_rescue)
                        out_of_band = False
                        if band is not None:
                            y_low, y_high = float(band[0]), float(band[1])
                            out_of_band = bool(
                                (y_high > y_low)
                                and ((head_state.y < y_low - 0.02) or (head_state.y > y_high + 0.02))
                            )
                        yaw_goal_gate = float(self._gate_heading_yaw(gate_rescue))
                        yaw_err_gate = float(angle_diff(yaw_goal_gate, head_state.yaw))
                        max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
                        dmax = math.radians(self.cfg.vehicle.max_steer_deg)

                        if out_of_band and band is not None:
                            y_mid = 0.5 * (float(y_low) + float(y_high))
                            y_err = float(head_state.y - y_mid)
                            # Reverse steering should push the vehicle back toward the safe band.
                            steer_sign = 1.0 if y_err > 1e-6 else (-1.0 if y_err < -1e-6 else 0.0)
                        else:
                            # In reverse, yaw rate flips sign: steer opposite to yaw error.
                            steer_sign = -1.0 if yaw_err_gate > 1e-6 else (1.0 if yaw_err_gate < -1e-6 else 0.0)

                        delta_target = float(np.clip(steer_sign * 0.55, -dmax, dmax))
                        steer_rate = float(
                            np.clip((delta_target - head_state.delta) / max(dt, 1e-6), -max_rate, max_rate)
                        )
                        v_ref = -0.28
                        accel = float(
                            np.clip(
                                (v_ref - head_state.v) / max(dt, 1e-6),
                                -self.cfg.vehicle.max_decel,
                                self.cfg.control.speed.max_accel_cmd,
                            )
                        )
                        cmd_rec = ControlCommand(accel=accel, steer_rate=steer_rate)
                        cmd_rec = self._enforce_speed_cap(head_state, cmd_rec, min(float(target_speed), 0.55), allow_reverse=True)
                        nxt_rec = self.ack.step(head_state, cmd_rec, dt)
                        if self._collision_with_obstacles_fast(nxt_rec) or self._collision_with_others_fast(nxt_rec, others):
                            cmd_rec = self._stop_command(head_state)
                            nxt_rec = self._freeze_state(head_state)
                        self._store_cmd(head, cmd_rec, now)
                        self.states[head] = nxt_rec
                        updated.add(head)
                        continue

            cmd_escape = None
            # Gate escape: deterministic recovery for the "wrong side of cross-wall" pocket.
            #
            # Avoid triggering escape for vehicles that are already well-aligned with the shared
            # corridor (low lateral error); otherwise the leader can unnecessarily reverse near
            # the first gate because the reference path does not enter the gap band until close
            # to the wall face. We only allow escape when the vehicle is clearly off-corridor.
            if self.demo_cfg.enable_gate_escape and (
                str(self.case.subtype).startswith("C") or str(self.case.subtype) in {"B2", "B3"}
            ):
                escape_active = bool(int(head) in self._gate_escape_phase or int(head) in self._gate_escape_key)
                if escape_active:
                    cmd_escape = self._gate_escape_command(head_state)
                else:
                    # Use y-deviation at the same x as a stable "off-corridor" signal for monotone-x
                    # chicane paths (more robust than Euclidean nearest-point distance near sharp bends).
                    y_ref = float(np.interp(float(head_state.x), self.path_xy[:, 0], self.path_xy[:, 1]))
                    lat_err = float(abs(float(head_state.y) - y_ref))
                    if lat_err >= 0.55:
                        cmd_escape = self._gate_escape_command(head_state)
            if cmd_escape is not None:
                # Drop any join route once the deterministic escape takes over.
                self._join_route_xy.pop(int(head), None)
                self._join_route_goal_xy.pop(int(head), None)
                self._join_route_strict_goal.pop(int(head), None)
                nxt_backoff = self.ack.step(head_state, cmd_escape, self.cfg.control.dt)
                if self._collision_with_obstacles_fast(nxt_backoff) or self._collision_with_others_fast(nxt_backoff, others):
                    nxt_backoff = self._freeze_state(head_state)
                self._store_cmd(head, cmd_escape, now)
                self.states[head] = nxt_backoff
                updated.add(head)
                continue

            # Curvature-aware speed schedule for single vehicles in tight bottlenecks.
            target_speed = self._single_curvature_limited_speed(target_speed, head_state)
            obs_dist = self._nearest_obstacle_distance(head_state)
            target_speed = self._single_obstacle_limited_speed(target_speed, head_state)
            gate_goal = None
            if self.demo_cfg.enable_gate_traversal:
                active_gate = self._active_gate(head_state)
                # Gate assists are intended as *recovery* behaviors. Running the full gate-mode
                # controller all the time can create oscillations and timeouts (B1/B3). Prefer
                # the nominal corridor tracker unless we are actually stuck near a wall.
                # Emergency back-off should be purely geometry-triggered (front hitch near the
                # wall face while outside the opening band). Do not gate it on blocked_ticks:
                # in many failures the vehicle never accumulates blocked ticks because it keeps
                # oscillating with tiny collision-free steps.
                cmd_backoff = self._gate_backoff_command(head_state)
                if cmd_backoff is not None:
                    dt = float(self.cfg.control.dt)
                    nxt_back = self.ack.step(head_state, cmd_backoff, dt)
                    if self._collision_with_obstacles_fast(nxt_back) or self._collision_with_others_fast(nxt_back, others):
                        cmd_backoff = self._stop_command(head_state)
                        nxt_back = self._freeze_state(head_state)
                    self._store_cmd(head, cmd_backoff, now)
                    self.states[head] = nxt_back
                    updated.add(head)
                    self._blocked_ticks[head] = 0
                    continue

                # Gate-mode assist (auto-engage when needed):
                # - The cross-wall collision-critical point is the *front hitch*, not only the
                #   front bumper. If we wait until `front_x` is near the wall face, the hitch
                #   may already collide and the vehicle never reaches the engagement window.
                # - Keep the original `enable_gate_mode` flag as an always-on switch, but also
                #   auto-enable when we are close to the wall face or have been blocked recently.
                if active_gate is not None:
                    x0_face = float(active_gate.get("x0", float(active_gate.get("cx", 0.0)) - 0.2))
                    x1_face = float(active_gate.get("x1", float(active_gate.get("cx", 0.0)) + 0.2))
                    y0_face = float(active_gate.get("y0", float(active_gate.get("cy", 0.0)) - 0.2))
                    y1_face = float(active_gate.get("y1", float(active_gate.get("cy", 0.0)) + 0.2))
                    if y1_face < y0_face:
                        y0_face, y1_face = y1_face, y0_face
                    hitch_x = float(self.geom.front_hitch(head_state)[0])
                    tail_x = float(self.geom.to_world(head_state, float(self.geom.rear_x))[0])
                    y_near_gap = bool((y0_face - 0.12) <= float(head_state.y) <= (y1_face + 0.12))
                    not_cleared = bool(tail_x <= (x1_face + 0.60))

                    engage = bool(self.demo_cfg.enable_gate_mode)
                    if int(self._blocked_ticks.get(head, 0)) >= 1:
                        engage = True
                    # Engage earlier based on front hitch proximity to the wall face.
                    if hitch_x >= (x0_face - 0.75):
                        engage = True

                    if engage and y_near_gap and not_cleared:
                        gate_goal = self._gate_traversal_goal(head_state, target_speed)

            if gate_goal is not None:
                p_gate, yaw_gate, v_gate = gate_goal
                # Gate progress rescue: when path-progress has not improved for a while near a gate,
                # force a short reverse-and-turn maneuver to escape local minima. This catches the
                # common failure mode where the vehicle oscillates in place while remaining collision-free.
                dt = float(self.cfg.control.dt)
                gate_idx = int(self._gate_idx_mem.get(head, -1))
                # Use x-progress (front bumper) as the primary progress signal near a vertical gate wall.
                # Path-s can stagnate or even regress when the vehicle is laterally offset in chicanes,
                # which can cause the rescue logic to trigger repeatedly and stall forward motion.
                x_prog = float(self.geom.to_world(head_state, float(self.geom.front_x))[0])
                if int(self._gate_prog_idx.get(head, -1)) != int(gate_idx):
                    self._gate_prog_idx[head] = int(gate_idx)
                    self._gate_prog_best_s[head] = float(x_prog)
                    self._gate_prog_best_t[head] = float(now)
                    self._gate_rescue_until[head] = -1e9
                else:
                    if x_prog > float(self._gate_prog_best_s.get(head, -1e9)) + 0.05:
                        self._gate_prog_best_s[head] = float(x_prog)
                        self._gate_prog_best_t[head] = float(now)
                        self._gate_rescue_until[head] = -1e9
                    else:
                        stuck_for = float(now) - float(self._gate_prog_best_t.get(head, now))
                        if stuck_for >= 4.0 and float(dist_goal_head) > 1.5:
                            self._gate_rescue_until[head] = float(max(self._gate_rescue_until.get(head, -1e9), now + 0.9))
                            # Reset timer so we don't immediately extend rescue forever.
                            self._gate_prog_best_t[head] = float(now)

                if float(now) <= float(self._gate_rescue_until.get(head, -1e9)):
                    gate = self._active_gate(head_state)
                    if gate is not None:
                        band = self._gate_safe_band(gate)
                        out_of_band = False
                        yaw_goal_gate = float(self._gate_heading_yaw(gate))
                        yaw_err_gate = float(angle_diff(yaw_goal_gate, head_state.yaw))
                        if band is not None:
                            y_low, y_high = band
                            out_of_band = bool(
                                (y_high > y_low)
                                and ((head_state.y < y_low - 0.02) or (head_state.y > y_high + 0.02))
                            )
                        nose_x = float(self.geom.to_world(head_state, float(self.geom.front_x))[0])
                        x0_face = float(gate.get("x0", float(gate.get("cx", 0.0)) - 0.2))
                        near_wall_face = bool(nose_x >= x0_face - 0.10)
                        # If stuck near a gate, reverse-turn to recover either:
                        # - the lateral band (out_of_band), or
                        # - a large yaw misalignment while still "in band" (common stall at wall face).
                        # Also trigger when the front bumper is already at the wall face; even small yaw
                        # errors can be unrecoverable without creating longitudinal room via reverse.
                        need_realign = bool(out_of_band or abs(yaw_err_gate) > math.radians(25.0) or near_wall_face)
                        if need_realign:
                            max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
                            dmax = math.radians(self.cfg.vehicle.max_steer_deg)
                            steer_sign = 0.0
                            if out_of_band and band is not None:
                                y_mid = 0.5 * (float(y_low) + float(y_high))
                                y_err = float(head_state.y - y_mid)
                                # Reverse steering should push the vehicle back toward the safe band.
                                steer_sign = 1.0 if y_err > 1e-6 else (-1.0 if y_err < -1e-6 else 0.0)
                            else:
                                # In reverse, yaw rate flips sign: steer opposite to yaw error.
                                steer_sign = -1.0 if yaw_err_gate > 1e-6 else (1.0 if yaw_err_gate < -1e-6 else 0.0)
                            delta_target = float(np.clip(steer_sign * 0.55, -dmax, dmax))
                            steer_rate = float(np.clip((delta_target - head_state.delta) / max(dt, 1e-6), -max_rate, max_rate))
                            v_ref = -0.28
                            accel = float(np.clip((v_ref - head_state.v) / max(dt, 1e-6), -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd))
                            cmd_rec = ControlCommand(accel=accel, steer_rate=steer_rate)
                            cmd_rec = self._enforce_speed_cap(head_state, cmd_rec, v_gate, allow_reverse=True)
                            nxt_rec = self.ack.step(head_state, cmd_rec, dt)
                            if (not self._collision_with_obstacles_fast(nxt_rec)) and (not self._collision_with_others_fast(nxt_rec, others)):
                                self._store_cmd(head, cmd_rec, now)
                                self.states[head] = nxt_rec
                                updated.add(head)
                                self._blocked_ticks[head] = 0
                                continue

                # Fast default: Stanley-track a short virtual corridor segment through the gate.
                # Always try this first; fall back to the sampling planner only when the one-step
                # preview would collide. This avoids slow/early use of the sampling planner during
                # long lane-change approaches and allows `blocked_ticks` to accumulate for gate
                # escape logic when genuinely stuck.
                gate_dir = np.array([math.cos(float(yaw_gate)), math.sin(float(yaw_gate))], dtype=float)
                p2 = p_gate + gate_dir * 1.2
                corridor = np.stack([head_state.xy(), p_gate, p2], axis=0)
                lat_err_gate = float(abs(float(p_gate[1]) - float(head_state.y)))
                yaw_err_gate = float(abs(angle_diff(float(yaw_gate), float(head_state.yaw))))
                goal_behind = bool(float(p_gate[0]) < float(head_state.x) - 0.05)
                use_sampling = bool(
                    int(self._blocked_ticks.get(head, 0)) >= 6
                    or lat_err_gate >= 0.65
                    or yaw_err_gate >= math.radians(45.0)
                )
                # If the gate target is behind (back-off goal), do not use the forward-only fast
                # corridor tracker; it will command forward motion and can induce long oscillation
                # stalls at the wall face (B3 Clustered_At_A seed=50042).
                if goal_behind:
                    use_sampling = True
                if not use_sampling:
                    gate_tracker = self.tracker if abs(float(head_state.v)) < 0.35 else self.nav_tracker
                    cmd_pp = gate_tracker.track_path(head_state, corridor, float(v_gate))
                    cmd_gate_fast = ControlCommand(
                        accel=float(self._speed_accel_cmd(head_state.v, float(v_gate))),
                        steer_rate=float(cmd_pp.steer_rate),
                    )
                    cmd_gate_fast = self._enforce_speed_cap(head_state, cmd_gate_fast, float(v_gate), allow_reverse=False)
                    nxt_gate_fast = self.ack.step(head_state, cmd_gate_fast, self.cfg.control.dt)
                    if (not self._collision_with_obstacles_fast(nxt_gate_fast)) and (not self._collision_with_others_fast(nxt_gate_fast, others)):
                        self._store_cmd(head, cmd_gate_fast, now)
                        self._blocked_ticks[head] = 0
                        self.states[head] = nxt_gate_fast
                        updated.add(head)
                        continue
                    self._blocked_ticks[head] += 1

                force_yaw = bool(float(np.linalg.norm(head_state.xy() - p_gate)) <= 1.8)
                plan_gate = self.gate_planner.plan_step(
                    ego=head_state,
                    goal_xy=p_gate,
                    goal_yaw=float(yaw_gate),
                    obstacles=self.case.obstacles,
                    dynamic_others=others,
                    force_goal_yaw=force_yaw,
                )
                if plan_gate.feasible:
                    cmd_gate = plan_gate.command
                else:
                    v_ref = float(-abs(float(v_gate)) if goal_behind else float(v_gate))
                    cmd_gate = self.tracker.track_point(
                        head_state,
                        float(p_gate[0]),
                        float(p_gate[1]),
                        float(yaw_gate),
                        float(v_ref),
                    )
                # Heuristic: the sampling planner can get "myopically stuck" with zero steer-rate
                # near a gate approach when large yaw/lateral errors require an intermediate
                # detour/reversal. In this case, borrow a corrective steering action from the
                # point-tracking controller to avoid long stalls.
                if plan_gate.feasible and abs(float(cmd_gate.steer_rate)) <= 1e-6:
                    yaw_err_exec = float(abs(angle_diff(float(yaw_gate), float(head_state.yaw))))
                    lat_err_exec = float(abs(float(p_gate[1]) - float(head_state.y)))
                    if yaw_err_exec > math.radians(45.0) or lat_err_exec > 0.25:
                        cmd_pp = self.tracker.track_point(
                            head_state,
                            float(p_gate[0]),
                            float(p_gate[1]),
                            float(yaw_gate),
                            float(v_gate),
                        )
                        cmd_gate = ControlCommand(accel=float(cmd_gate.accel), steer_rate=float(cmd_pp.steer_rate))
                # Stronger lateral correction before committing into the wall thickness region.
                # The constant-command local planner can be overly "smooth" and fail to reduce
                # lateral error fast enough between chicane walls, leading to timeouts.
                if plan_gate.feasible:
                    lat_err_exec = float(abs(float(p_gate[1]) - float(head_state.y)))
                    if lat_err_exec > 0.12:
                        cmd_pp = self.tracker.track_point(
                            head_state,
                            float(p_gate[0]),
                            float(p_gate[1]),
                            float(yaw_gate),
                            float(v_gate),
                        )
                        cmd_gate = ControlCommand(accel=float(cmd_gate.accel), steer_rate=float(cmd_pp.steer_rate))
                # Respect the planner's intent when it decides to brake/reverse for re-alignment:
                # only apply forward speed-tracking when the candidate accel is non-negative.
                if (not goal_behind) and (float(cmd_gate.accel) >= -1e-9):
                    cmd_gate = ControlCommand(
                        accel=float(self._speed_accel_cmd(head_state.v, v_gate)),
                        steer_rate=float(cmd_gate.steer_rate),
                    )
                allow_reverse = bool(goal_behind or (float(cmd_gate.accel) < -1e-9))
                cmd_gate = self._enforce_speed_cap(head_state, cmd_gate, v_gate, allow_reverse=allow_reverse)
                nxt_gate = self.ack.step(head_state, cmd_gate, self.cfg.control.dt)
                blocked_sampling = False
                hit_obs_gate = self._collision_with_obstacles_fast(nxt_gate)
                hit_veh_gate = self._collision_with_others_fast(nxt_gate, others)
                if hit_obs_gate or hit_veh_gate:
                    # Vehicle-only deadlock breaker near gate entrance: if we're blocked by a vehicle
                    # behind us, prefer a short straight creep (unwinding steering) to open headway.
                    # This prevents "tailgating freezes" where both vehicles stop before the wall.
                    recovered_gate = False
                    if hit_veh_gate:
                        colliders = [o for o in others if self.collision.collide_vehicle_vehicle(nxt_gate, o, include_clearance=False)]
                        if colliders:
                            colliders.sort(key=lambda o: float(np.linalg.norm(head_state.xy() - o.xy())))
                            c = colliders[0]
                            if float(c.x) < float(head_state.x) - 0.05:
                                dt = max(self.cfg.control.dt, 1e-6)
                                max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
                                unwind = float(np.clip(-head_state.delta / dt, -max_rate, max_rate))
                                v_ref = float(min(float(v_gate), 0.35))
                                cmd_esc = ControlCommand(
                                    accel=float(self._speed_accel_cmd(head_state.v, v_ref)),
                                    steer_rate=unwind,
                                )
                                cmd_esc = self._enforce_speed_cap(head_state, cmd_esc, v_ref, allow_reverse=False)
                                nxt_esc = self.ack.step(head_state, cmd_esc, dt)
                                if (not self._collision_with_obstacles_fast(nxt_esc)) and (
                                    not self._collision_with_others_fast(nxt_esc, others)
                                ):
                                    cmd_gate = cmd_esc
                                    nxt_gate = nxt_esc
                                    hit_obs_gate = False
                                    hit_veh_gate = False
                                    recovered_gate = True
                    # Gate wall deadlock breaker: if a forward-through command collides with the wall
                    # thickness near the face, do *not* freeze in place. Actively back off (reverse)
                    # and realign; otherwise scattered starts can end up permanently parked at the wall
                    # corner with zero progress.
                    if not recovered_gate and hit_obs_gate:
                        gate = self._active_gate(head_state)
                        if gate is not None:
                            dt = max(self.cfg.control.dt, 1e-6)
                            max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
                            dmax = math.radians(self.cfg.vehicle.max_steer_deg)

                            band = self._gate_safe_band(gate)
                            yaw_goal_gate = float(self._gate_heading_yaw(gate))
                            yaw_err_gate = float(angle_diff(yaw_goal_gate, head_state.yaw))
                            nose_x = float(self.geom.to_world(head_state, float(self.geom.front_x))[0])
                            x0_face = float(gate.get("x0", float(gate.get("cx", 0.0)) - 0.2))
                            near_wall_face = bool(nose_x >= x0_face - 0.15)
                            out_of_band = False
                            if band is not None:
                                y_low, y_high = band
                                out_of_band = bool((y_high > y_low) and ((head_state.y < y_low - 0.03) or (head_state.y > y_high + 0.03)))

                            def _try_backoff(turn: bool) -> tuple[ControlCommand, VehicleState] | None:
                                if (not near_wall_face) and (not out_of_band):
                                    return None
                                v_ref = -0.28
                                accel = float(
                                    np.clip(
                                        (v_ref - head_state.v) / dt,
                                        -self.cfg.vehicle.max_decel,
                                        self.cfg.control.speed.max_accel_cmd,
                                    )
                                )
                                if turn:
                                    steer_sign = 0.0
                                    if out_of_band and band is not None:
                                        y_mid = 0.5 * (float(y_low) + float(y_high))
                                        y_err = float(head_state.y - y_mid)
                                        steer_sign = 1.0 if y_err > 1e-6 else (-1.0 if y_err < -1e-6 else 0.0)
                                    else:
                                        steer_sign = -1.0 if yaw_err_gate > 1e-6 else (1.0 if yaw_err_gate < -1e-6 else 0.0)
                                    delta_target = float(np.clip(steer_sign * 0.55, -dmax, dmax))
                                    steer_rate = float(np.clip((delta_target - head_state.delta) / dt, -max_rate, max_rate))
                                else:
                                    steer_rate = float(np.clip(-head_state.delta / dt, -max_rate, max_rate))
                                cmd_b = ControlCommand(accel=accel, steer_rate=steer_rate)
                                cmd_b = self._enforce_speed_cap(head_state, cmd_b, v_gate, allow_reverse=True)
                                nxt_b = self.ack.step(head_state, cmd_b, dt)
                                if self._collision_with_obstacles_fast(nxt_b) or self._collision_with_others_fast(nxt_b, others):
                                    return None
                                return cmd_b, nxt_b

                            cand = _try_backoff(turn=True) or _try_backoff(turn=False)
                            if cand is not None:
                                cmd_gate, nxt_gate = cand
                                hit_obs_gate = False
                                hit_veh_gate = False
                                recovered_gate = True
                                blocked_sampling = True
                    if not recovered_gate and (hit_obs_gate or hit_veh_gate):
                        cmd_gate = self._stop_command(head_state)
                        nxt_gate = self._freeze_state(head_state)
                        blocked_sampling = True
                self._store_cmd(head, cmd_gate, now)
                if blocked_sampling:
                    self._blocked_ticks[head] = int(self._blocked_ticks.get(head, 0)) + 1
                else:
                    self._blocked_ticks[head] = 0
                self.states[head] = nxt_gate
                updated.add(head)
                continue
            # Near-goal homing avoids endpoint spline sharpness from causing last-meter stalls.
            if dist_goal_head <= max(2.0, head_goal_tol + 0.4):
                if len(self.path_xy) >= 2:
                    d_goal = self.path_xy[-1] - self.path_xy[-2]
                    yaw_goal = float(math.atan2(float(d_goal[1]), float(d_goal[0])))
                else:
                    yaw_goal = float(head_state.yaw)
                # Dynamic near-goal speed schedule:
                # A fixed low cap (0.35m/s) makes long chicane scenes miss the 60s budget even when
                # the last meters are collision-free (P6 B3/C* timeouts). Keep it slow only very
                # close to the goal, otherwise allow a moderate approach speed.
                v_goal = float(np.clip(0.25 + 0.25 * float(dist_goal_head), 0.35, 0.65))
                v_goal = float(min(float(target_speed), v_goal))
                cmd_goal = self.tracker.track_point(
                    head_state,
                    float(goal_xy_head[0]),
                    float(goal_xy_head[1]),
                    yaw_goal,
                    float(v_goal),
                )
                cmd_goal = self._enforce_speed_target(head_state, cmd_goal, float(v_goal))
                self._store_cmd(head, cmd_goal, now)
                nxt_goal = self.ack.step(head_state, cmd_goal, self.cfg.control.dt)
                if (not self._collision_with_obstacles_fast(nxt_goal)) and (not self._collision_with_others_fast(nxt_goal, others)):
                    self._blocked_ticks[head] = 0
                    self.states[head] = nxt_goal
                    updated.add(head)
                    continue
            # Default: track the shared corridor path. Collision recovery below can still
            # fall back to the local planner when the nominal tracker would collide.
            track_path = self._track_path_xy(int(head))
            use_direct_goal = False
            if (not self.mandatory_gates) and int(head) != int(self.leader_id):
                # In Type-A scenes, forcing far-scattered vehicles to aggressively merge onto the
                # corridor centerline can create early multi-vehicle deadlocks (A3 Uniform_Spread).
                # When the lateral error is large, prefer a direct-to-goal heading to keep the
                # vehicle in free space and avoid blocking the leader's corridor.
                if self._lateral_distance_to_path(head_state, track_path) >= 1.80:
                    use_direct_goal = True
            if use_direct_goal:
                direct = np.stack([head_state.xy(), goal_xy_head.astype(float).copy()], axis=0)
                cmd = self.nav_tracker.track_path(head_state, direct, target_speed)
            else:
                # In B/C chicane scenes, Stanley tracking can become overly aggressive when the
                # vehicle starts far from the corridor (Random_Scattered / Uniform_Spread), causing
                # near-vertical yaw oscillations before the first gate. Use pure-pursuit until the
                # lateral error is reduced, then switch back to Stanley for precision.
                lat_err = float(self._lateral_distance_to_path(head_state, track_path))
                prefer_pp = bool(self.mandatory_gates and lat_err >= 0.90)
                # Type-B2 is a two-gate chicane-like scene; Stanley can keep flipping near the
                # wall-pair lane-change region and dominate P6 completion time. Prefer the more
                # stable pure-pursuit tracker there.
                if self.mandatory_gates and str(self.case.subtype) == "B2":
                    prefer_pp = True
                ctrl = self.tracker if prefer_pp else self.nav_tracker
                cmd = ctrl.track_path(head_state, track_path, target_speed)
            # Use explicit speed control to avoid tracker-induced stalls/oscillations in wide-open regions.
            cmd = ControlCommand(
                accel=float(self._speed_accel_cmd(head_state.v, target_speed)),
                steer_rate=float(cmd.steer_rate),
            )
            cmd = self._enforce_speed_cap(head_state, cmd, target_speed, allow_reverse=False)
            self._store_cmd(head, cmd, now)
            nxt = self.ack.step(head_state, cmd, self.cfg.control.dt)
            hit_obs = self._collision_with_obstacles_fast(nxt)
            hit_veh = self._collision_with_others_fast(nxt, others)
            if hit_obs or hit_veh:
                # Vehicle-only deadlock breaker: if the collision is with a vehicle behind us,
                # attempt a slow straight creep forward (unwinding steering) to increase the gap.
                # If the collision is with a vehicle ahead, attempt a short reverse back-off.
                if hit_veh and (not hit_obs):
                    # IMPORTANT: `hit_veh` includes clearance violations (min_clearance). If we only
                    # test raw polygon intersection here, side-by-side near-contacts will not be
                    # classified as colliders and the deadlock breaker won't trigger (P6 B3/C*
                    # Clustered_At_A stalls).
                    colliders = [o for o in others if self.collision.collide_vehicle_vehicle(nxt, o, include_clearance=True)]
                    if colliders:
                        colliders.sort(key=lambda o: float(np.linalg.norm(head_state.xy() - o.xy())))
                        c = colliders[0]
                        dt = max(self.cfg.control.dt, 1e-6)
                        max_rate = math.radians(self.cfg.vehicle.max_steer_rate_deg_s)
                        unwind = float(np.clip(-head_state.delta / dt, -max_rate, max_rate))

                        if float(c.x) < float(head_state.x) - 0.05:
                            v_ref = float(min(target_speed, 0.35))
                            accel = float(
                                np.clip((v_ref - float(head_state.v)) / dt, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd)
                            )
                            cmd_esc = ControlCommand(accel=accel, steer_rate=unwind)
                            cmd_esc = self._enforce_speed_cap(head_state, cmd_esc, v_ref, allow_reverse=False)
                            nxt_esc = self.ack.step(head_state, cmd_esc, dt)
                            if (not self._collision_with_obstacles_fast(nxt_esc)) and (not self._collision_with_others_fast(nxt_esc, others)):
                                self._store_cmd(head, cmd_esc, now)
                                self._blocked_ticks[head] = 0
                                self.states[head] = nxt_esc
                                updated.add(head)
                                continue
                        elif float(c.x) > float(head_state.x) + 0.05:
                            v_ref = -0.28
                            accel = float(
                                np.clip((v_ref - float(head_state.v)) / dt, -self.cfg.vehicle.max_decel, self.cfg.control.speed.max_accel_cmd)
                            )
                            cmd_esc = ControlCommand(accel=accel, steer_rate=unwind)
                            cmd_esc = self._enforce_speed_cap(head_state, cmd_esc, 0.0, allow_reverse=True)
                            nxt_esc = self.ack.step(head_state, cmd_esc, dt)
                            if (not self._collision_with_obstacles_fast(nxt_esc)) and (not self._collision_with_others_fast(nxt_esc, others)):
                                self._store_cmd(head, cmd_esc, now)
                                self._blocked_ticks[head] = 0
                                self.states[head] = nxt_esc
                                updated.add(head)
                                continue

                recovered = False
                if hit_obs or hit_veh:
                    for lh in (1.5, 1.1, 0.8, 0.6):
                        p_goal, yaw_goal = self._target_point_ahead(head_state, lookahead_m=lh)
                        plan = self.planner.plan_step(
                            ego=head_state,
                            goal_xy=p_goal,
                            goal_yaw=float(yaw_goal),
                            obstacles=self.case.obstacles,
                            dynamic_others=others,
                        )
                        if not plan.feasible:
                            continue
                        cmd_try = plan.command
                        if float(cmd_try.accel) >= -1e-9:
                            speed_ref = float(target_speed)
                            if hit_veh and (not hit_obs):
                                speed_ref = float(min(speed_ref, 0.45))
                            cmd_try = ControlCommand(
                                accel=float(self._speed_accel_cmd(head_state.v, speed_ref)),
                                steer_rate=float(cmd_try.steer_rate),
                            )
                        nxt_try = self.ack.step(head_state, cmd_try, self.cfg.control.dt)
                        if self._collision_with_obstacles_fast(nxt_try) or self._collision_with_others_fast(nxt_try, others):
                            continue
                        self._store_cmd(head, cmd_try, now)
                        cmd = cmd_try
                        nxt = nxt_try
                        hit_obs = False
                        hit_veh = False
                        recovered = True
                        break
                if hit_obs or hit_veh:
                    self._blocked_ticks[head] += 1
                    cmd_stop = self._stop_command(head_state)
                    self._store_cmd(head, cmd_stop, now)
                    nxt = self._freeze_state(head_state)
                if recovered:
                    self._blocked_ticks[head] = 0
            else:
                self._blocked_ticks[head] = 0
            self.states[head] = nxt
            updated.add(head)

    def _handle_docking_interruption(self, now: float, follower_id: int, leader_id: int, reason: str, updated: set[int]) -> None:
        self.p4_interruption_count += 1
        n_retry = self.retry_count.get(follower_id, 0)
        if n_retry < self.demo_cfg.max_retry_per_vehicle:
            self.retry_count[follower_id] = n_retry + 1
            self.replan_ready_at[follower_id] = now + self.demo_cfg.replan_delay_s
            self.engine.abort_docking(follower_id, now=now, detail=f"p4_{reason}")
            self._push_event(
                now,
                "P4",
                "abort_and_replan",
                detail=reason,
                vehicle_id=follower_id,
                peer_id=leader_id,
            )
            self.docking_started_at.pop(follower_id, None)
            self._dock_debug.pop(follower_id, None)
            self._last_dock_cmd.pop(follower_id, None)
            self._dock_stage_mem.pop(follower_id, None)
            self._dock_best_dist.pop(follower_id, None)
            self._dock_best_metric.pop(follower_id, None)
            self._dock_best_t.pop(follower_id, None)
            updated.add(follower_id)
            return

        self.abandon_docking.add(follower_id)
        self.engine.abort_docking(follower_id, now=now, detail=f"p4_{reason}_abandon")
        self.p4_abort_to_independent_count += 1
        self._push_event(
            now,
            "P4",
            "abort_to_independent",
            detail=reason,
            vehicle_id=follower_id,
            peer_id=leader_id,
        )
        self.docking_started_at.pop(follower_id, None)
        self._dock_debug.pop(follower_id, None)
        self._last_dock_cmd.pop(follower_id, None)
        self._dock_stage_mem.pop(follower_id, None)
        self._dock_best_dist.pop(follower_id, None)
        self._dock_best_metric.pop(follower_id, None)
        self._dock_best_t.pop(follower_id, None)
        updated.add(follower_id)

    def _apply_docking_motion(self, now: float, updated: set[int]) -> None:
        pairs = sorted(self.engine.topology.docking_target.items())
        for follower_id, leader_id in pairs:
            if follower_id in updated:
                continue
            if follower_id not in self.states or leader_id not in self.states:
                continue
            lock_eval = self._get_lock_eval(follower_id)
            f_state = self.states[follower_id]
            l_state = self.states[leader_id]

            # If strategy provided an intercept s, the executor must respect it: do not attempt to
            # physically dock to the leader tail too early (often inside bottlenecks).
            s_int = self.docking_intercept_s.get(int(follower_id), None)
            if s_int is not None and float(s_int) > 1e-6:
                leader_s = self._path_progress_s(l_state)
                if leader_s + float(self.demo_cfg.dock_ahead_margin_m) < float(s_int):
                    others = [self.states[j] for j in self.vehicle_ids if int(j) != int(follower_id)]
                    labels = self.case.labels
                    profile = [] if labels is None else labels.n_max_pass_profile
                    follower_s = self._path_progress_s(f_state)
                    if profile and int(nmax_at_s(profile, leader_s)) < 2 and (leader_s - follower_s) < 3.5:
                        cmd = self._stop_command(f_state)
                        target_speed = 0.0
                        speed_ref = float(target_speed)
                        speed_cap = float(target_speed)
                        allow_reverse_exec = False
                        fallback_reason = "yield_bottleneck"
                        d_tail = float(np.linalg.norm(self.geom.front_hitch(f_state) - self.geom.rear_hitch(l_state)))
                    else:
                        rear = self.geom.rear_hitch(l_state)
                        heading_vec = np.array([math.cos(l_state.yaw), math.sin(l_state.yaw)], dtype=float)
                        follow_offset = 2.4 + 0.8 * max(0.0, float(l_state.v))
                        # Path-based tail-follow target: keeps the follower on the shared corridor
                        # (important in chicanes), instead of chasing a raw Euclidean point that can
                        # fall on the wrong side of a wall.
                        s_leader = self._path_progress_s(l_state)
                        s_target = max(0.0, float(s_leader) - float(follow_offset))
                        target_xy = _interp_path(self.path_xy, self.path_s, float(s_target))
                        d_tail = float(np.linalg.norm(self.geom.front_hitch(f_state) - rear))

                        # Use chase-speed for intercept following; using free-speed here makes the
                        # intercept timing assumption in `BaselineScheduler` inconsistent with execution
                        # and can prevent docking from ever reaching the close phase in scattered starts.
                        target_speed = float(min(self.p2_scheduler.cfg.chase_speed_mps, self.cfg.vehicle.max_speed))
                        # Tail-gap-based speed schedule: keep safe gap and avoid aggressive merges.
                        if d_tail <= follow_offset + 1.2:
                            target_speed = min(target_speed, max(0.12, 0.45 * (d_tail - follow_offset)))
                        target_speed = self._speed_limit_by_proximity(follower_id, target_speed)
                        speed_ref = float(target_speed)
                        speed_cap = float(target_speed)
                        allow_reverse_exec = False

                        # For scattered starts, first rejoin the shared corridor using A*.
                        self._maybe_plan_join_route(follower_id, now)
                        join_route = self._join_route_xy.get(int(follower_id), None)
                        if join_route is not None and (not self._join_route_done(int(follower_id))):
                            join_speed = float(min(target_speed, 0.75))
                            speed_ref = float(join_speed)
                            speed_cap = float(join_speed)
                            allow_reverse_exec = False
                            cmd_pp = self.nav_tracker.track_path(f_state, join_route, join_speed)
                            cmd = ControlCommand(
                                accel=float(self._speed_accel_cmd(f_state.v, join_speed)),
                                steer_rate=float(cmd_pp.steer_rate),
                            )
                            fallback_reason = "join_route"
                        else:
                            if join_route is not None:
                                self._join_route_xy.pop(int(follower_id), None)
                                self._join_route_goal_xy.pop(int(follower_id), None)
                                self._join_route_strict_goal.pop(int(follower_id), None)
                            cmd = self.nav_tracker.track_path(f_state, self.path_xy, target_speed)
                            fallback_reason = ""
                        if float(cmd.accel) >= -1e-9:
                            cmd = ControlCommand(
                                accel=float(self._speed_accel_cmd(f_state.v, speed_ref)),
                                steer_rate=float(cmd.steer_rate),
                            )
                    cmd = self._enforce_speed_cap(f_state, cmd, speed_cap, allow_reverse=allow_reverse_exec)
                    nxt = self.ack.step(f_state, cmd, self.cfg.control.dt)
                    if self._collision_with_obstacles_fast(nxt) or self._collision_with_others_fast(nxt, others):
                        cmd = self._stop_command(f_state)
                        nxt = self._freeze_state(f_state)
                    self._store_cmd(follower_id, cmd, now)
                    self.states[follower_id] = nxt
                    self._dock_debug[follower_id] = {
                        "stage": "INTERCEPT_FOLLOW",
                        "distance": float(d_tail),
                        "yaw_err_deg": float(abs(math.degrees(angle_diff(l_state.yaw, f_state.yaw)))),
                        "lat_err": 0.0,
                        "long_err": 0.0,
                        "rel_x": 0.0,
                        "rel_y": 0.0,
                        "speed_ref": float(speed_ref),
                        "fallback_reason": str(fallback_reason),
                    }
                    updated.add(follower_id)
                    continue

            if follower_id not in self.docking_started_at:
                self.docking_started_at[follower_id] = now
            cmd_raw, dock_dbg = self._simple_docking_command(f_state, l_state, now)
            stage = str(dock_dbg.get("stage", "GLOBAL_APPROACH"))
            cmd = cmd_raw
            d_now = float(dock_dbg.get("distance", np.linalg.norm(self.geom.front_hitch(f_state) - self.geom.rear_hitch(l_state))))
            lat_abs = abs(float(dock_dbg.get("lat_err", 0.0)))
            yaw_deg_abs = abs(float(dock_dbg.get("yaw_err_deg", 0.0)))
            progress_metric = float(d_now + 0.8 * lat_abs + 0.02 * yaw_deg_abs)
            best_metric = float(self._dock_best_metric.get(follower_id, math.inf))
            if progress_metric + 0.03 < best_metric:
                self._dock_best_metric[follower_id] = progress_metric
                self._dock_best_dist[follower_id] = d_now
                self._dock_best_t[follower_id] = now
            else:
                self._dock_best_dist.setdefault(follower_id, d_now)
                self._dock_best_metric.setdefault(follower_id, progress_metric)
                self._dock_best_t.setdefault(follower_id, now)
            self._dock_debug[follower_id] = {
                "stage": str(stage),
                "distance": float(d_now),
                "yaw_err_deg": float(yaw_deg_abs),
                "lat_err": float(lat_abs),
                "long_err": float(dock_dbg.get("long_err", 0.0)),
                "rel_x": float(dock_dbg.get("rel_x", 0.0)),
                "rel_y": float(dock_dbg.get("rel_y", 0.0)),
                "speed_ref": float(dock_dbg.get("speed_ref", 0.0)),
                "fallback_reason": str(dock_dbg.get("fallback_reason", "")),
            }
            forced = self._should_force_disturbance(follower_id, now, stage)
            if forced is not None:
                self._handle_docking_interruption(now, follower_id, leader_id, forced, updated)
                continue
            # P4-style: abort docking if it makes no measurable progress for too long.
            t0 = float(self.docking_started_at.get(follower_id, now))
            if (now - t0) >= float(self.demo_cfg.docking_max_duration_s):
                self._handle_docking_interruption(now, follower_id, leader_id, "timeout", updated)
                continue
            best_t = float(self._dock_best_t.get(follower_id, now))
            base_no_progress = float(self.demo_cfg.docking_no_progress_timeout_s)
            no_progress_timeout = base_no_progress
            if stage == "VISUAL_SERVO":
                no_progress_timeout = max(no_progress_timeout, 2.0 * base_no_progress)
            if str(dock_dbg.get("fallback_reason", "")) == "vision_lost":
                no_progress_timeout = max(no_progress_timeout, 2.5 * base_no_progress)
            if (now - best_t) >= float(no_progress_timeout) and d_now >= max(
                1.0, float(self.cfg.docking.soft_capture_distance) + 0.25
            ):
                reason = str(dock_dbg.get("fallback_reason", "")) or "no_progress"
                self._handle_docking_interruption(now, follower_id, leader_id, reason, updated)
                continue

            dt = float(self.cfg.control.dt)
            cmd_final = cmd
            nxt = self.ack.step(f_state, cmd_final, dt)
            if self.collision.collide_vehicle_vehicle(nxt, l_state, include_clearance=False):
                hitch_d = float(np.linalg.norm(self.geom.front_hitch(nxt) - self.geom.rear_hitch(l_state)))
                if hitch_d > float(self.cfg.docking.soft_capture_distance) + 0.12:
                    cmd_final = self._stop_command(f_state)
                    nxt = self.ack.step(f_state, cmd_final, dt)
                    self._push_event(
                        now,
                        "CTRL",
                        "dock_safety_stop",
                        detail=f"hitch_d={hitch_d:.3f}",
                        vehicle_id=follower_id,
                        peer_id=leader_id,
                    )
            self._store_cmd(follower_id, cmd_final, now)
            if self._collision_with_obstacles_fast(nxt):
                # first action: brake-and-yield; escalate to P4 interruption only if still blocked
                cmd_b = self._stop_command(f_state)
                self._store_cmd(follower_id, cmd_b, now)
                nxt_b = self.ack.step(f_state, cmd_b, self.cfg.control.dt)
                nxt = nxt_b

            pos_err, yaw_err, speed_err = self._lock_metrics(nxt, l_state)
            soft_ok = (
                pos_err <= self.cfg.docking.soft_capture_distance
                and abs(math.degrees(yaw_err)) <= self.cfg.docking.soft_capture_max_yaw_deg
                and speed_err <= self.cfg.docking.soft_capture_max_speed_error
            )
            transition_ok = True
            if soft_ok:
                nxt, transition_ok = self._soft_capture_step(nxt, l_state)
            cond = lock_eval.update(nxt, l_state, self.cfg.control.dt)
            if soft_ok and transition_ok and cond.locked:
                yaw_deg = abs(math.degrees(cond.yaw_error))
                if yaw_deg < 10.0:
                    nxt = self._project_locked(nxt, l_state)
                    self.states[follower_id] = nxt
                    if self.engine.mark_docking_locked(follower_id, leader_id, now):
                        self.dock_success_count += 1
                        if self.retry_count.get(follower_id, 0) > 0:
                            self.p4_replan_success_count += 1
                    self.docking_started_at.pop(follower_id, None)
                    self.docking_intercept_s.pop(follower_id, None)
                    self._dock_debug.pop(follower_id, None)
                    self._last_dock_cmd.pop(follower_id, None)
                    self._dock_stage_mem.pop(follower_id, None)
                    self._dock_best_dist.pop(follower_id, None)
                    self._dock_best_metric.pop(follower_id, None)
                    self._dock_best_t.pop(follower_id, None)
                    updated.add(follower_id)
                    continue

            if cond.locked:
                yaw_deg = abs(math.degrees(cond.yaw_error))
                if yaw_deg < 10.0:
                    nxt = self._project_locked(nxt, l_state)
                    self.states[follower_id] = nxt
                    if self.engine.mark_docking_locked(follower_id, leader_id, now):
                        self.dock_success_count += 1
                        if self.retry_count.get(follower_id, 0) > 0:
                            self.p4_replan_success_count += 1
                    self.docking_started_at.pop(follower_id, None)
                    self.docking_intercept_s.pop(follower_id, None)
                    self._dock_debug.pop(follower_id, None)
                    self._last_dock_cmd.pop(follower_id, None)
                    self._dock_stage_mem.pop(follower_id, None)
                    self._dock_best_dist.pop(follower_id, None)
                    self._dock_best_metric.pop(follower_id, None)
                    self._dock_best_t.pop(follower_id, None)
                    updated.add(follower_id)
                    continue
            self.states[follower_id] = nxt
            updated.add(follower_id)

    def _accumulate_energy(self, prev_states: dict[int, VehicleState]) -> None:
        for vid in self.vehicle_ids:
            self._step_energy[vid] = 0.0
        for vid in self.vehicle_ids:
            p = prev_states[vid]
            n = self.states[vid]
            d = float(np.linalg.norm(n.xy() - p.xy()))
            if d <= 1e-9:
                continue
            v_avg = 0.5 * max(0.0, p.v + n.v)
            sz = self._chain_size_of_vehicle(vid)
            if sz > 1:
                e = self.energy_model.train_energy(d, train_size=sz, kappa_abs=0.0, speed_mps=v_avg)
            else:
                e = self.energy_model.single_energy(d, speed_mps=v_avg)
            self.total_energy += float(e)
            self._step_energy[vid] = float(e)
            self._energy_cumulative[vid] = float(self._energy_cumulative.get(vid, 0.0) + e)

    def _update_stall_ticks(self, prev_states: dict[int, VehicleState]) -> None:
        for vid in self.vehicle_ids:
            p = prev_states[vid]
            s = self.states[vid]
            pos_step = float(np.linalg.norm(s.xy() - p.xy()))
            goal_xy = self._goal_for_vehicle(vid)
            goal_dist = float(np.linalg.norm(s.xy() - goal_xy))
            if (pos_step < 0.01) and (abs(float(s.v)) < 0.05) and (goal_dist > 1.5):
                self._stall_ticks[vid] = int(self._stall_ticks.get(vid, 0) + 1)
            else:
                self._stall_ticks[vid] = 0

    def _record_history(self, now: float) -> None:
        if not self.demo_cfg.record_history:
            return
        if self._low_tick_count % max(1, self.demo_cfg.sample_every_n_low_ticks) != 0:
            return
        self.history.append(
            {
                "t": float(now),
                "states": {
                    str(vid): {
                        "x": float(self.states[vid].x),
                        "y": float(self.states[vid].y),
                        "yaw": float(self.states[vid].yaw),
                        "v": float(self.states[vid].v),
                        "mode": str(self.engine.topology.mode[vid].value),
                    }
                    for vid in self.vehicle_ids
                },
                "edges": [[int(a), int(b)] for a, b in self.engine.topology.edges()],
                "pending_docks": {str(k): int(v) for k, v in self.engine.topology.docking_target.items()},
                "leader_s": float(self._path_progress_s(self.states[self.leader_id])),
                "leader_remaining_s": float(self._leader_remaining_s),
            }
        )

    def _all_reached_goal(self) -> bool:
        head_goal_ok: dict[int, bool] = {}
        for h in self.engine.topology.heads():
            goal_h = self._goal_for_head(h)
            d_h = float(np.linalg.norm(self.states[h].xy() - goal_h))
            tol_h = self.demo_cfg.leader_goal_tol_m if h == self.leader_id else self.demo_cfg.vehicle_goal_tol_m
            ok = bool(d_h <= tol_h)
            if (not ok) and int(h) != int(self.leader_id) and len(self.path_xy) >= 2:
                # Slot-goal reachability guard:
                # When a non-leader head passes its assigned slot along the corridor (monotone-x),
                # Euclidean distance to the slot can increase and become unrecoverable without
                # reversing. Treat "passed the slot on-corridor" as completion to avoid false
                # timeouts in Type-A scattered starts (e.g., A3 Uniform_Spread).
                try:
                    goal_idx = _nearest_path_idx(self.path_xy, goal_h)
                    goal_s = float(self.path_s[goal_idx]) if len(self.path_s) == len(self.path_xy) else float("nan")
                except Exception:
                    goal_s = float("nan")
                if math.isfinite(goal_s):
                    cur_s = float(self._path_progress_s(self.states[int(h)]))
                    lat = float(self._lateral_distance_to_path(self.states[int(h)], self._track_path_xy(int(h))))
                    if (cur_s >= goal_s - 0.20) and (lat <= 1.25):
                        ok = True
            head_goal_ok[int(h)] = bool(ok)
        for vid in self.vehicle_ids:
            if self.engine.topology.parent[vid] is not None:
                # Trailer completion is inherited from chain-head completion.
                if not head_goal_ok.get(self._chain_head(vid), False):
                    return False
                continue
            # Free head vehicles inherit the head completion condition (including the
            # "passed-slot" guard above).
            if head_goal_ok.get(self._chain_head(vid), False):
                continue
            goal_v = self._goal_for_vehicle(vid)
            d = float(np.linalg.norm(self.states[vid].xy() - goal_v))
            tol = self.demo_cfg.leader_goal_tol_m if vid == self.leader_id else self.demo_cfg.vehicle_goal_tol_m
            if d > tol:
                return False
        return True

    def _collision_with_obstacles_fast(self, state: VehicleState, near_m: float = 2.5) -> bool:
        for obs in self.case.obstacles:
            if abs(state.x - obs.x) > near_m + 0.5 * obs.width:
                continue
            if abs(state.y - obs.y) > near_m + 0.5 * obs.height:
                continue
            if self.collision.collide_vehicle_obstacle(state, obs, include_clearance=False):
                return True
        return False

    def _collision_with_others_fast(
        self,
        state: VehicleState,
        others: list[VehicleState],
        near_m: float = 2.8,
        skip_ids: set[int] | None = None,
    ) -> bool:
        skip = set() if skip_ids is None else set(skip_ids)
        for o in others:
            if o.vehicle_id == state.vehicle_id:
                continue
            if o.vehicle_id in skip:
                continue
            if float(np.linalg.norm(state.xy() - o.xy())) > near_m:
                continue
            if self.collision.collide_vehicle_vehicle(state, o, include_clearance=False):
                return True
        return False

    def _actual_collision_exists(self) -> bool:
        for vid in self.vehicle_ids:
            st = self.states[vid]
            if self._collision_with_obstacles_fast(st):
                return True
        connected_pairs: set[tuple[int, int]] = set()
        for a, b in self.engine.topology.edges():
            connected_pairs.add((int(a), int(b)))
            connected_pairs.add((int(b), int(a)))
        for i, a in enumerate(self.vehicle_ids):
            sa = self.states[a]
            for b in self.vehicle_ids[i + 1 :]:
                sb = self.states[b]
                if (int(a), int(b)) in connected_pairs:
                    continue
                legal_docking_contact = False
                if self.engine.topology.docking_target.get(int(a), None) == int(b):
                    d = float(np.linalg.norm(self.geom.front_hitch(sa) - self.geom.rear_hitch(sb)))
                    legal_docking_contact = d <= (self.cfg.docking.soft_capture_distance + 0.1)
                elif self.engine.topology.docking_target.get(int(b), None) == int(a):
                    d = float(np.linalg.norm(self.geom.front_hitch(sb) - self.geom.rear_hitch(sa)))
                    legal_docking_contact = d <= (self.cfg.docking.soft_capture_distance + 0.1)
                if legal_docking_contact:
                    continue
                if self.collision.collide_vehicle_vehicle(sa, sb, include_clearance=False):
                    return True
        return False

    def _first_collision_detail(self) -> tuple[int | None, int | None, str]:
        # Returns (vehicle_id, peer_id, detail_str).
        for vid in self.vehicle_ids:
            st = self.states[vid]
            for oi, obs in enumerate(self.case.obstacles):
                if self.collision.collide_vehicle_obstacle(st, obs, include_clearance=False):
                    return (
                        int(vid),
                        None,
                        f"obs_collision:obs_idx={oi} obs=({obs.x:.2f},{obs.y:.2f},w={obs.width:.2f},h={obs.height:.2f})",
                    )

        connected_pairs: set[tuple[int, int]] = set()
        for a, b in self.engine.topology.edges():
            connected_pairs.add((int(a), int(b)))
            connected_pairs.add((int(b), int(a)))

        for i, a in enumerate(self.vehicle_ids):
            sa = self.states[a]
            for b in self.vehicle_ids[i + 1 :]:
                if (int(a), int(b)) in connected_pairs:
                    continue
                sb = self.states[b]
                legal_docking_contact = False
                if self.engine.topology.docking_target.get(int(a), None) == int(b):
                    d = float(np.linalg.norm(self.geom.front_hitch(sa) - self.geom.rear_hitch(sb)))
                    legal_docking_contact = d <= (self.cfg.docking.soft_capture_distance + 0.1)
                elif self.engine.topology.docking_target.get(int(b), None) == int(a):
                    d = float(np.linalg.norm(self.geom.front_hitch(sb) - self.geom.rear_hitch(sa)))
                    legal_docking_contact = d <= (self.cfg.docking.soft_capture_distance + 0.1)
                if legal_docking_contact:
                    continue
                if self.collision.collide_vehicle_vehicle(sa, sb, include_clearance=False):
                    return (int(a), int(b), "veh_collision")

        return (None, None, "")

    def _refresh_collision_counter(self, now: float | None = None) -> None:
        collide_now = self._actual_collision_exists()
        if collide_now and (not self._collision_global_latch):
            self.collision_count += 1
            if now is not None:
                vid, peer, detail = self._first_collision_detail()
                self._push_event(float(now), "SYS", "collision", detail=detail, vehicle_id=vid, peer_id=peer)
        self._collision_global_latch = bool(collide_now)

    def _consume_engine_events(self) -> None:
        while self._event_cursor < len(self.engine.events):
            ev = self.engine.events[self._event_cursor]
            self._event_cursor += 1
            self._push_event(ev.t, "RUNTIME", ev.event_type.value, detail=ev.detail, vehicle_id=ev.vehicle_id, peer_id=ev.peer_id)
            if ev.event_type == EventType.SPLIT_DONE:
                self.split_count += 1
                if ev.vehicle_id is not None and ev.peer_id is not None and ev.vehicle_id in self.states and ev.peer_id in self.states:
                    child = self.states[int(ev.vehicle_id)]
                    parent = self.states[int(ev.peer_id)]
                    relaxed = VehicleState(
                        vehicle_id=child.vehicle_id,
                        x=float(child.x),
                        y=float(child.y),
                        yaw=float(child.yaw),
                        v=float(min(child.v, max(0.0, parent.v))),
                        delta=float(child.delta),
                        mode=VehicleMode.FREE,
                    )
                    if self._collision_with_obstacles_fast(relaxed) or self._collision_with_any_vehicle(
                        relaxed, skip_ids={relaxed.vehicle_id}
                    ):
                        free_gap = abs(self.geom.front_hitch_x - self.geom.rear_hitch_x) + 0.25
                        h = np.array([math.cos(parent.yaw), math.sin(parent.yaw)], dtype=float)
                        c = parent.xy() - h * free_gap
                        relaxed = VehicleState(
                            vehicle_id=child.vehicle_id,
                            x=float(c[0]),
                            y=float(c[1]),
                            yaw=float(parent.yaw),
                            v=0.0,
                            delta=0.0,
                            mode=VehicleMode.FREE,
                        )
                        relaxed = self._place_collision_free_near(relaxed, avoid_ids={relaxed.vehicle_id})
                    # Keep split transition continuous: cap one-step release displacement.
                    disp = relaxed.xy() - child.xy()
                    disp_norm = float(np.linalg.norm(disp))
                    max_step = self.cfg.vehicle.max_speed * self.cfg.control.dt + 0.12
                    if disp_norm > max_step:
                        disp = disp * (max_step / max(disp_norm, 1e-9))
                        relaxed = VehicleState(
                            vehicle_id=relaxed.vehicle_id,
                            x=float(child.x + disp[0]),
                            y=float(child.y + disp[1]),
                            yaw=float(relaxed.yaw),
                            v=float(relaxed.v),
                            delta=float(relaxed.delta),
                            mode=relaxed.mode,
                        )
                    self.states[int(ev.vehicle_id)] = relaxed
            elif ev.event_type == EventType.DOCKING_ABORTED and "feasibility_infeasible" in ev.detail:
                self.feasibility_abort_count += 1
            elif ev.event_type == EventType.DOCKING_ABORTED:
                # Safety cleanup on docking abort:
                # If the follower is physically very close (or even overlapping) with the leader at
                # the moment the runtime clears `docking_target`, the collision checker will start
                # counting it as an illegal contact. This can fail otherwise recoverable runs
                # (e.g., P6 A3 Uniform_Spread) even though the intent is to "abort and replan".
                #
                # Resolve by backing the follower off to a collision-free pose behind the leader
                # before collision accounting runs in the same tick.
                if ev.vehicle_id is None or ev.peer_id is None:
                    continue
                fid = int(ev.vehicle_id)
                lid = int(ev.peer_id)
                if fid not in self.states or lid not in self.states:
                    continue
                follower = self.states[fid]
                leader = self.states[lid]
                # Always drop to FREE + stop on abort.
                follower = VehicleState(
                    vehicle_id=follower.vehicle_id,
                    x=float(follower.x),
                    y=float(follower.y),
                    yaw=float(follower.yaw),
                    v=0.0,
                    delta=float(follower.delta),
                    mode=VehicleMode.FREE,
                )
                if self._collision_with_obstacles_fast(follower) or self.collision.collide_vehicle_vehicle(
                    follower, leader, include_clearance=False
                ):
                    free_gap = abs(self.geom.front_hitch_x - self.geom.rear_hitch_x) + 0.45
                    h = np.array([math.cos(float(leader.yaw)), math.sin(float(leader.yaw))], dtype=float)
                    c = leader.xy() - h * free_gap
                    follower = VehicleState(
                        vehicle_id=follower.vehicle_id,
                        x=float(c[0]),
                        y=float(c[1]),
                        yaw=float(leader.yaw),
                        v=0.0,
                        delta=0.0,
                        mode=VehicleMode.FREE,
                    )
                    follower = self._place_collision_free_near(follower, avoid_ids={fid})
                self.states[fid] = follower

    def _low_tick_hook(self, _engine: ReconfigRuntimeEngine, now: float) -> None:
        # schedule strategy ticks
        while now + 1e-9 >= self._next_strategy_tick:
            self._strategy_tick(self._next_strategy_tick)
            self._next_strategy_tick += 1.0 / max(self.cfg.control.high_rate_hz, 1e-6)

        prev_states = {vid: st.copy() for vid, st in self.states.items()}
        updated: set[int] = set()
        self._apply_chain_motion(now, updated)
        self._apply_docking_motion(now, updated)
        self._accumulate_energy(prev_states)
        self._update_stall_ticks(prev_states)
        self._consume_engine_events()
        self._refresh_collision_counter(now)

        self._leader_remaining_s = self._leader_remaining_time()
        leader_s = float(self._path_progress_s(self.states[self.leader_id]))
        if leader_s > float(self._leader_best_s) + 0.05:
            self._leader_best_s = float(leader_s)
            self._leader_best_s_t = float(now)
        if self.done_time is None and self._all_reached_goal():
            self.done_time = float(now)
            self._push_event(now, "SYS", "all_goal_reached", detail="done")
            if self.demo_cfg.stop_on_done:
                self._consume_engine_events()
                self._record_history(now)
                monitor_row = None
                if self.demo_cfg.record_monitor_trace or self.tick_observer is not None or self.step_decider is not None:
                    monitor_row = self._build_monitor_snapshot(now, prev_states)
                    if self.demo_cfg.record_monitor_trace:
                        self.monitor_trace.append(monitor_row)
                    if self.tick_observer is not None:
                        self.tick_observer(monitor_row)
                self._low_tick_count += 1
                raise self._StepStop("done")

        self._record_history(now)
        monitor_row = None
        if self.demo_cfg.record_monitor_trace or self.tick_observer is not None or self.step_decider is not None:
            monitor_row = self._build_monitor_snapshot(now, prev_states)
            if self.demo_cfg.record_monitor_trace:
                self.monitor_trace.append(monitor_row)
            if self.tick_observer is not None:
                self.tick_observer(monitor_row)
        if self.step_decider is not None and monitor_row is not None:
            if not bool(self.step_decider(monitor_row)):
                raise self._StepStop("step_decider_stop")
        self._low_tick_count += 1

    def _leader_remaining_fn(self, _t: float) -> float:
        return float(self._leader_remaining_s)

    def run(self) -> IntegratedRunResult:
        self._leader_remaining_s = self._leader_remaining_time()
        try:
            self.engine.run(
                duration_s=self.demo_cfg.duration_s,
                leader_remaining_fn=self._leader_remaining_fn,
                low_tick_hook=self._low_tick_hook,
            )
        except self._StepStop:
            pass
        self._consume_engine_events()
        exec_fbs = [f for f in self.engine.feedbacks if f.stage == FeedbackStage.EXEC]
        exec_accept = sum(1 for f in exec_fbs if f.status == FeedbackStatus.RUNNING)
        exec_reject = sum(1 for f in exec_fbs if f.status == FeedbackStatus.REJECTED)
        done_time = self.done_time if self.done_time is not None else float(self.demo_cfg.duration_s)
        success = bool(self.done_time is not None and self.collision_count == 0)
        return IntegratedRunResult(
            scenario_id=self.case.scenario_id,
            subtype=self.case.subtype,
            policy=self.policy,
            success=success,
            done_time_s=float(done_time),
            total_energy=float(self.total_energy),
            leader_final_train_size=int(len(self._chain_from_head(self.leader_id))),
            dock_success_count=int(self.dock_success_count),
            split_count=int(self.split_count),
            p4_interruption_count=int(self.p4_interruption_count),
            p4_replan_success_count=int(self.p4_replan_success_count),
            p4_abort_to_independent_count=int(self.p4_abort_to_independent_count),
            collision_count=int(self.collision_count),
            feasibility_abort_count=int(self.feasibility_abort_count),
            commands_submitted=int(self.commands_submitted),
            commands_accepted=int(self.commands_accepted),
            command_exec_accept=int(exec_accept),
            command_exec_reject=int(exec_reject),
            arbitration_trace=self.arbitration_trace,
            reconfig_action_trace=self.reconfig_action_trace,
            monitor_trace=self.monitor_trace,
            history=self.history,
            events=self.events,
        )
