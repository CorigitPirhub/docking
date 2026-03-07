from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .bcfd import BCFDConfig
from .collision import CollisionEngine
from .config import Config
from .coop_docking import CooperativeStagingPlanner, StagingPlan, StagingTracker
from .kinematics import AckermannModel, VehicleGeometry
from .math_utils import angle_diff, clamp, wrap_angle
from .sensors import GlobalPoseSensor, VisionSensor, line_blocked_by_obstacles
from .types import ControlCommand, Obstacle, VehicleState


def _sigmoid(x: float) -> float:
    xx = float(x)
    if xx >= 30.0:
        return 1.0
    if xx <= -30.0:
        return 0.0
    return 1.0 / (1.0 + math.exp(-xx))


@dataclass
class DockSkillDebug:
    stage: str
    follower_substage: str
    leader_relocated: bool
    d_tail: float
    w_vis: float
    visual_valid: bool
    fusion_gamma: float
    nis: float
    fallback_global: bool
    fallback_reason: str
    min_clearance: float
    visual_lost_time: float
    replan_count: int = 0


@dataclass(frozen=True)
class DockSkillOptions:
    enable_cooperative_staging: bool = True
    fusion_mode: str = "belief_consistent"  # belief_consistent | dist_blend | hard_switch | global_only
    enable_capture_funnel: bool = True
    enable_micro_replan: bool = True
    enable_fallback: bool = True
    enable_safety_projection: bool = True
    enable_fast_lane: bool = True


class CooperativeDockingSkill:
    """
    Co-BCFD Docking Skill:
    - optional cooperative staging (leader relocation)
    - belief-consistent GNSS↔Vision fusion
    - capture-funnel near-field MPPI
    - safety projection
    """

    def __init__(
        self,
        cfg: Config,
        *,
        seed: int,
        obstacles: list[Obstacle],
        leader_init: VehicleState,
        follower_init: VehicleState,
        prefer_static_leader: bool = False,
        plan: StagingPlan | None = None,
        options: DockSkillOptions | None = None,
    ) -> None:
        self.cfg = cfg
        self.seed = int(seed)
        self.obstacles = list(obstacles)
        self.options = options or DockSkillOptions()

        self.geom = VehicleGeometry(cfg.vehicle)
        self.model = AckermannModel(cfg.vehicle)
        self.collision = CollisionEngine(cfg.vehicle, cfg.safety)

        self._staging_planner = CooperativeStagingPlanner(cfg, seed=self.seed + 31)
        self._tracker = StagingTracker(cfg)

        self._prefer_static_leader = bool(prefer_static_leader or (not self.options.enable_cooperative_staging))
        if plan is not None:
            plan = self._canonicalize_plan(plan, leader_ref=leader_init, follower_ref=follower_init)
        self._plan: StagingPlan | None = None if plan is None else plan
        self._fixed_plan: StagingPlan | None = None if plan is None else plan
        self._leader_goal: VehicleState | None = None
        self._follower_goal: VehicleState | None = None
        self._leader_path_xy: np.ndarray | None = None
        self._follower_path_xy: np.ndarray | None = None
        self._leader_relocated = False

        # Follower sensors observing the leader target.
        self._global_sensor = GlobalPoseSensor(cfg.sensors.global_sensor, seed=self.seed + 11)
        self._vision_sensor = VisionSensor(cfg.sensors.vision, cfg.vehicle, seed=self.seed + 17)
        self._last_global = None
        self._last_vision = None
        self._visual_lost_time = 0.0

        self._stage: str = "INIT"
        self._follower_substage: str = "INIT"
        self._near_field_active = False
        self._staging_time = 0.0
        self._dock_best_tail = float("inf")
        self._dock_stall_time = 0.0
        self._replan_count = 0
        self._fast_lane_certified = False
        self._fast_lane_certified = False

        self._bcfd_cfg = BCFDConfig()

        # Cache initial states for planning.
        self._leader_init = leader_init.copy()
        self._follower_init = follower_init.copy()

        if plan is not None:
            self._apply_plan(plan)

    def reset(self) -> None:
        self._plan = None
        self._leader_goal = None
        self._follower_goal = None
        self._leader_path_xy = None
        self._follower_path_xy = None
        self._leader_relocated = False

        self._last_global = None
        self._last_vision = None
        self._visual_lost_time = 0.0
        self._stage = "INIT"
        self._follower_substage = "INIT"
        self._near_field_active = False
        self._staging_time = 0.0
        self._dock_best_tail = float("inf")
        self._dock_stall_time = 0.0
        self._replan_count = 0

        if self._fixed_plan is not None:
            self._apply_plan(self._fixed_plan)

    def _canonicalize_plan(
        self,
        plan: StagingPlan,
        *,
        leader_ref: VehicleState,
        follower_ref: VehicleState,
    ) -> StagingPlan:
        if self._prefer_static_leader:
            return plan
        move_dist = float(np.linalg.norm(plan.leader_goal.xy() - leader_ref.xy()))
        if move_dist >= 0.35:
            return plan
        try:
            static_plan = self._staging_planner.plan(
                obstacles=self.obstacles,
                leader=leader_ref.copy(),
                follower=follower_ref.copy(),
                prefer_static_leader=True,
            )
        except Exception:
            return plan
        static_move = float(np.linalg.norm(static_plan.leader_goal.xy() - leader_ref.xy()))
        if static_move > 1e-6:
            return plan
        if not math.isfinite(float(static_plan.score)) and str(static_plan.reason) != 'ok':
            return plan
        return static_plan

    def _dock_zone_clearance(self, leader_state: VehicleState) -> float:
        heading = np.array([math.cos(float(leader_state.yaw)), math.sin(float(leader_state.yaw))], dtype=float)
        rear = self.geom.rear_hitch(leader_state)
        clearances: list[float] = []
        for standoff in (0.30, 0.55, 0.80):
            center = rear - heading * float(self.geom.front_hitch_x + float(standoff))
            probe = VehicleState(vehicle_id=-1, x=float(center[0]), y=float(center[1]), yaw=float(leader_state.yaw), v=0.0, delta=0.0)
            clearances.append(float(self.collision.min_clearance_vehicle_obstacles(probe, self.obstacles)))
        return float(min(clearances) if clearances else 1e6)

    def _compute_fast_lane_certificate(self, *, leader_ref: VehicleState, follower_ref: VehicleState) -> bool:
        if not self.options.enable_fast_lane:
            return False
        if self._leader_goal is None or self._follower_goal is None:
            return False
        if self._leader_relocated:
            return False
        if abs(float(angle_diff(float(leader_ref.yaw), float(follower_ref.yaw)))) > math.radians(12.0):
            return False
        if line_blocked_by_obstacles(self.geom.front_hitch(follower_ref), self.geom.rear_hitch(leader_ref), self.obstacles):
            return False
        if float(self.collision.min_clearance_vehicle_obstacles(leader_ref, self.obstacles)) < 0.9:
            return False
        if float(self.collision.min_clearance_vehicle_obstacles(follower_ref, self.obstacles)) < 0.9:
            return False
        return self._dock_zone_clearance(self._leader_goal) >= 1.35

    def _apply_plan(self, plan: StagingPlan, *, leader_ref: VehicleState | None = None) -> None:
        self._plan = plan
        self._leader_goal = plan.leader_goal.copy()
        self._follower_goal = plan.follower_goal.copy()
        self._leader_path_xy = plan.leader_path_xy.astype(float).copy()
        self._follower_path_xy = plan.follower_path_xy.astype(float).copy()

        ref = self._leader_init if leader_ref is None else leader_ref
        move_dist = float(np.linalg.norm(self._leader_goal.xy() - ref.xy()))
        self._leader_relocated = (not self._prefer_static_leader) and (move_dist >= 0.35)
        self._fast_lane_certified = self._compute_fast_lane_certificate(leader_ref=ref, follower_ref=self._follower_init)

        self._stage = "STAGING" if self._leader_relocated else "DOCKING"
        self._follower_substage = "APPROACH_PATH"
        self._near_field_active = False
        self._staging_time = 0.0
        self._dock_best_tail = float("inf")
        self._dock_stall_time = 0.0

    def _maybe_plan(self) -> None:
        if self._plan is not None:
            return
        plan = self._staging_planner.plan(
            obstacles=self.obstacles,
            leader=self._leader_init,
            follower=self._follower_init,
            prefer_static_leader=self._prefer_static_leader,
        )
        plan = self._canonicalize_plan(plan, leader_ref=self._leader_init, follower_ref=self._follower_init)
        self._apply_plan(plan)

    def _maybe_replan_from_current(self, *, leader: VehicleState, follower: VehicleState, d_tail: float) -> None:
        if not self.options.enable_micro_replan:
            self._dock_best_tail = float("inf")
            self._dock_stall_time = 0.0
            return
        if self._stage != "DOCKING":
            self._dock_best_tail = float("inf")
            self._dock_stall_time = 0.0
            return
        if float(d_tail) < self._dock_best_tail - 0.02:
            self._dock_best_tail = float(d_tail)
            self._dock_stall_time = 0.0
            return
        self._dock_stall_time += float(self.cfg.control.dt)
        if self._dock_stall_time < 5.0:
            return
        if self._replan_count >= 2:
            return
        if float(d_tail) <= max(0.45, 3.0 * float(self.cfg.docking.lock_position_tol)):
            return
        try:
            plan = self._staging_planner.plan(
                obstacles=self.obstacles,
                leader=leader.copy(),
                follower=follower.copy(),
                prefer_static_leader=False,
            )
            plan = self._canonicalize_plan(plan, leader_ref=leader, follower_ref=follower)
            self._apply_plan(plan, leader_ref=leader)
            self._replan_count += 1
            self._follower_substage = "REPLAN_STAGING"
        except Exception:
            self._dock_stall_time = 0.0
            return

    def _rear_hitch_from_global_meas(self, meas) -> tuple[np.ndarray, float]:
        yaw = float(meas.yaw)
        rear_x = float(self.geom.rear_x - self.cfg.vehicle.hitch_length)
        rear = np.array([float(meas.x) + math.cos(yaw) * rear_x, float(meas.y) + math.sin(yaw) * rear_x], dtype=float)
        return rear, float(yaw)

    def _rear_hitch_from_vision(self, follower: VehicleState, vis_meas) -> tuple[np.ndarray, float]:
        cam = self.geom.front_hitch(follower)
        c = math.cos(float(follower.yaw))
        s = math.sin(float(follower.yaw))
        rel = np.array([float(vis_meas.rel_x), float(vis_meas.rel_y)], dtype=float)
        rel_world = np.array([c * rel[0] - s * rel[1], s * rel[0] + c * rel[1]], dtype=float)
        rear = cam + rel_world
        yaw = wrap_angle(float(follower.yaw) + float(vis_meas.rel_yaw))
        return rear, float(yaw)

    def _fusion(
        self,
        *,
        follower: VehicleState,
        leader_true: VehicleState,
        d_true: float,
        timestamp: float,
    ) -> tuple[np.ndarray, float, float, float, float, bool, bool]:
        g = self._global_sensor.observe(leader_true, timestamp)
        if g is not None:
            self._last_global = g
        v = self._vision_sensor.observe_rear_hitch(follower, leader_true, self.obstacles, timestamp)
        if v is not None:
            self._last_vision = v

        if self._last_global is None:
            return (
                self.geom.rear_hitch(leader_true),
                float(leader_true.yaw),
                0.0,
                0.0,
                float("inf"),
                False,
                False,
            )

        rear_g, yaw_g = self._rear_hitch_from_global_meas(self._last_global)
        visual_valid = bool(self._last_vision and bool(getattr(self._last_vision, "valid", False)))
        if not visual_valid:
            self._visual_lost_time += float(self.cfg.control.dt)
            return rear_g, float(yaw_g), 0.0, 0.0, 0.0, True, False

        rear_v, yaw_v = self._rear_hitch_from_vision(follower, self._last_vision)
        self._visual_lost_time = 0.0

        diff = rear_v - rear_g
        sg = float(self.cfg.sensors.global_sensor.sigma_pos)
        sv = float(self.cfg.sensors.vision.sigma_pos)
        rg = (sg * sg) * np.eye(2, dtype=float)
        rv = (sv * sv) * np.eye(2, dtype=float)
        ssum = rg + rv
        try:
            inv = np.linalg.inv(ssum)
            nis = float(diff.T @ inv @ diff)
        except np.linalg.LinAlgError:
            nis = float("inf")

        # distance schedule on the required fusion band (avoid hard switch)
        d_min = float(self.cfg.docking.blend_distance_min)
        d_max = float(self.cfg.docking.blend_distance_max)
        if d_true <= d_min:
            s_dist = 1.0
        elif d_true >= d_max:
            s_dist = 0.0
        else:
            s_dist = float((d_max - d_true) / max(d_max - d_min, 1e-9))

        info_w = float(np.trace(rg) / max(np.trace(rg) + np.trace(rv), 1e-9))
        fusion_mode = str(self.options.fusion_mode)
        if fusion_mode == "global_only":
            gamma = 0.0
            w_vis = 0.0
        elif fusion_mode == "hard_switch":
            gamma = 1.0 if float(d_true) <= float(self.cfg.docking.stage_switch_distance) else 0.0
            w_vis = float(info_w if gamma > 0.5 else 0.0)
        elif fusion_mode == "dist_blend":
            gamma = float(clamp(s_dist, 0.0, 1.0))
            w_vis = float(clamp(gamma * info_w, 0.0, 1.0))
        else:
            gamma_dist = _sigmoid(8.0 * (s_dist - 0.35))
            gamma_nis = _sigmoid(2.2 * (9.21 - float(nis)))  # chi^2_0.99 for 2D
            gamma = float(clamp(gamma_dist * gamma_nis, 0.0, 1.0))
            w_vis = float(clamp(gamma * info_w, 0.0, 1.0))

        rear = (1.0 - w_vis) * rear_g + w_vis * rear_v
        yaw_err = angle_diff(float(yaw_v), float(yaw_g))
        yaw = wrap_angle(float(yaw_g) + float(w_vis) * float(yaw_err))
        return rear, float(yaw), float(w_vis), float(gamma), float(nis), True, True

    def _in_bounds(self, state: VehicleState, margin: float = 0.55) -> bool:
        half_w = 0.5 * float(self.cfg.environment.width) - float(margin)
        half_h = 0.5 * float(self.cfg.environment.height) - float(margin)
        return (-half_w <= float(state.x) <= half_w) and (-half_h <= float(state.y) <= half_h)

    def _path_remaining_distance(self, state: VehicleState, path_xy: np.ndarray | None) -> float:
        if path_xy is None or len(path_xy) < 2:
            return 0.0
        p = state.xy()
        deltas = path_xy - p[None, :]
        idx = int(np.argmin(np.sum(deltas * deltas, axis=1)))
        rem = float(np.linalg.norm(path_xy[idx] - p))
        if idx < len(path_xy) - 1:
            rem += float(np.sum(np.linalg.norm(np.diff(path_xy[idx:], axis=0), axis=1)))
        return rem

    def _bcfd_near_field(
        self,
        *,
        follower: VehicleState,
        leader: VehicleState,
        rear_target_xy: np.ndarray,
        yaw_target: float,
    ) -> ControlCommand:
        cfg = self._bcfd_cfg

        fh = self.geom.front_hitch(follower)
        rel_world = np.asarray(rear_target_xy, dtype=float) - fh
        lc = math.cos(float(yaw_target))
        ls = math.sin(float(yaw_target))
        x_l = float(lc * float(rel_world[0]) + ls * float(rel_world[1]))
        y_l = float(-ls * float(rel_world[0]) + lc * float(rel_world[1]))
        yaw_diff = float(angle_diff(float(yaw_target), float(follower.yaw)))
        yaw_err = abs(yaw_diff)
        dist = float(np.linalg.norm(rel_world))

        fast_lane = bool(self._fast_lane_certified)
        if dist > float(cfg.align_enter_dist_m):
            x_ref = float(clamp(0.70 + 0.18 * dist, float(cfg.standoff_approach_min), float(cfg.standoff_approach_max)))
            v_cap = float(1.30 if fast_lane else cfg.v_cap_approach)
        elif dist > float(cfg.dock_enter_dist_m):
            x_ref = float(cfg.standoff_align)
            v_cap = float(0.42 if fast_lane else cfg.v_cap_align)
        else:
            x_ref = 0.0
            v_cap = float(0.18 if fast_lane else cfg.v_cap_dock)

        ex = float(x_l - x_ref)
        v_ref = float(leader.v) + float(cfg.k_x) * float(ex)
        v_ref = float(clamp(v_ref, -float(self.cfg.vehicle.max_reverse_speed), float(v_cap)))

        # Contact gate: if too close but misaligned, back off to create room.
        gate_lat = float(cfg.gate_lat_m * (1.8 if fast_lane else 1.0))
        gate_yaw = math.radians(float(cfg.gate_yaw_deg * (1.8 if fast_lane else 1.0)))
        if dist <= float(cfg.gate_contact_dist_m):
            if abs(y_l) > gate_lat or yaw_err > gate_yaw:
                v_ref = float(-0.08 if fast_lane else cfg.gate_backoff_speed)
        # Strong backoff when entering close range with large misalignment.
        if dist <= 1.0 and yaw_err > math.radians(22.0):
            v_ref = min(float(v_ref), -0.18)
        if dist <= 0.9 and abs(y_l) > 0.20:
            v_ref = min(float(v_ref), -0.14)

        v_term = abs(float(follower.v)) + float(cfg.stanley_softening)
        delta_ref = float(yaw_diff + math.atan2(float(cfg.k_cte) * float(y_l), float(v_term)))
        dmax = math.radians(float(self.cfg.vehicle.max_steer_deg))
        delta_ref = float(clamp(delta_ref, -dmax, dmax))

        max_rate = math.radians(float(self.cfg.vehicle.max_steer_rate_deg_s))
        steer_rate = float(clamp((delta_ref - float(follower.delta)) / float(self.cfg.control.dt), -max_rate, max_rate))

        accel = float(
            clamp(
                float(self.cfg.control.speed.kp) * (float(v_ref) - float(follower.v)),
                -float(self.cfg.vehicle.max_decel),
                float(self.cfg.control.speed.max_accel_cmd),
            )
        )
        return ControlCommand(accel=accel, steer_rate=steer_rate)

    def _project_safe(
        self,
        *,
        ego: VehicleState,
        cmd: ControlCommand,
        other: VehicleState,
        allow_contact: bool,
    ) -> ControlCommand:
        # Try nominal, then a small candidate set biased toward braking.
        candidates: list[ControlCommand] = [cmd]
        candidates.append(ControlCommand(accel=-float(self.cfg.vehicle.max_decel), steer_rate=0.0))
        candidates.append(ControlCommand(accel=float(self.cfg.vehicle.max_accel), steer_rate=0.0))
        candidates.append(ControlCommand(accel=min(0.0, float(cmd.accel)), steer_rate=float(cmd.steer_rate)))
        for sr_deg in (-25.0, 25.0):
            candidates.append(ControlCommand(accel=-0.8, steer_rate=math.radians(sr_deg)))
        max_rate = math.radians(float(self.cfg.vehicle.max_steer_rate_deg_s))
        candidates.append(ControlCommand(accel=-float(self.cfg.vehicle.max_decel), steer_rate=float(max_rate)))
        candidates.append(ControlCommand(accel=-float(self.cfg.vehicle.max_decel), steer_rate=float(-max_rate)))

        best: ControlCommand | None = None
        best_dev = float("inf")
        best_clear = -1e9
        for c in candidates:
            nxt = self.model.step(ego, c, float(self.cfg.control.dt))
            if not self._in_bounds(nxt, margin=0.55):
                continue
            collide = False
            for obs in self.obstacles:
                if self.collision.collide_vehicle_obstacle(nxt, obs, include_clearance=True):
                    collide = True
                    break
            if collide:
                continue
            if not allow_contact and self.collision.collide_vehicle_vehicle(nxt, other, include_clearance=True):
                continue
            clr_obs = self.collision.min_clearance_vehicle_obstacles(nxt, self.obstacles)
            clr_veh = self.collision.min_clearance_vehicle_vehicle(nxt, other) if not allow_contact else 1e6
            clr = float(min(clr_obs, clr_veh))
            dev = float((float(c.accel) - float(cmd.accel)) ** 2 + (float(c.steer_rate) - float(cmd.steer_rate)) ** 2)
            if (dev < best_dev - 1e-12) or (abs(dev - best_dev) <= 1e-12 and clr > best_clear):
                best = c
                best_dev = dev
                best_clear = clr
        if best is None:
            return ControlCommand(accel=-float(self.cfg.vehicle.max_decel), steer_rate=0.0)
        return best

    def compute_commands(
        self,
        *,
        leader: VehicleState,
        follower: VehicleState,
        timestamp: float,
    ) -> tuple[ControlCommand, ControlCommand, DockSkillDebug]:
        self._maybe_plan()
        assert self._leader_goal is not None and self._follower_goal is not None
        assert self._leader_path_xy is not None and self._follower_path_xy is not None

        d_tail = float(np.linalg.norm(self.geom.front_hitch(follower) - self.geom.rear_hitch(leader)))
        self._maybe_replan_from_current(leader=leader, follower=follower, d_tail=float(d_tail))
        if self._stage == "STAGING":
            self._near_field_active = False
        allow_contact_dist = float(max(0.45, float(self.cfg.docking.soft_capture_distance) + 0.05))
        allow_contact = d_tail <= allow_contact_dist

        # Fusion (exactly once per tick; the function has internal timers/state).
        rear_est, yaw_est, w_vis, gamma, nis, have_global, visual_valid = self._fusion(
            follower=follower,
            leader_true=leader,
            d_true=d_tail,
            timestamp=float(timestamp),
        )
        if not have_global:
            w_vis = 0.0
            gamma = 0.0
            nis = float("inf")
            visual_valid = False

        # Default commands.
        leader_cmd = ControlCommand(accel=0.0, steer_rate=0.0)
        follower_cmd = ControlCommand(accel=0.0, steer_rate=0.0)
        fallback_global = False
        fallback_reason = ""

        # Leader staging.
        if self._stage == "STAGING":
            self._staging_time += float(self.cfg.control.dt)
            leader_rem = self._path_remaining_distance(leader, self._leader_path_xy)
            follower_rem = self._path_remaining_distance(follower, self._follower_path_xy)
            sync_ratio = leader_rem / max(follower_rem, 1e-3)
            leader_speed = float(clamp(0.45 + 0.18 * sync_ratio, 0.32, 0.72))
            follower_speed = float(clamp(0.55 + 0.55 * sync_ratio, 0.38, 1.12))
            leader_cmd, leader_done = self._tracker.command(
                state=leader,
                goal=self._leader_goal,
                path_xy=self._leader_path_xy,
                target_speed=leader_speed,
                obstacles=self.obstacles,
                other=follower,
            )
            follower_cmd, _ = self._tracker.command(
                state=follower,
                goal=self._follower_goal,
                path_xy=self._follower_path_xy,
                target_speed=follower_speed,
                obstacles=self.obstacles,
                other=leader,
            )
            self._follower_substage = "APPROACH_PATH"
            follower_goal_dist = float(np.linalg.norm(follower.xy() - self._follower_goal.xy()))
            follower_ready = bool(
                follower_goal_dist <= 0.70
                or d_tail <= float(self.cfg.docking.stage_switch_distance) + 0.25
            )
            forced_limit = 18.0 if self._leader_relocated else 14.0
            forced_done = bool(self._staging_time >= forced_limit and follower_ready)
            if leader_done and follower_ready:
                self._stage = "DOCKING"
            elif forced_done:
                self._stage = "DOCKING"
                self._follower_substage = "FORCED_DOCK"
        else:
            # Keep leader steady during final docking.
            if abs(float(leader.v)) > 0.02:
                leader_cmd = ControlCommand(
                    accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(leader.v))),
                    steer_rate=0.0,
                )
            else:
                leader_cmd = ControlCommand(accel=0.0, steer_rate=float(-leader.delta / max(self.cfg.control.dt, 1e-3)))

            # Follower approach path until near-field.
            d_goal = float(np.linalg.norm(follower.xy() - self._follower_goal.xy()))
            if (not self._near_field_active) and d_goal > 0.45 and d_tail > float(self.cfg.docking.stage_switch_distance) + 0.20:
                approach_speed = 1.38 if self._fast_lane_certified else 1.05
                follower_cmd, _ = self._tracker.command(
                    state=follower,
                    goal=self._follower_goal,
                    path_xy=self._follower_path_xy,
                    target_speed=float(approach_speed),
                    obstacles=self.obstacles,
                    other=leader,
                )
                self._follower_substage = "APPROACH_PATH"
            else:
                self._near_field_active = True
                # Near-field: capture-funnel + MPPI micro-maneuver + lock assist.
                if not have_global:
                    follower_cmd = ControlCommand(accel=0.0, steer_rate=0.0)
                    fallback_global = True
                    fallback_reason = "missing_global"
                else:
                    # Leader-frame capture test.
                    fh = self.geom.front_hitch(follower)
                    rel_w = rear_est - fh
                    lc = math.cos(float(yaw_est))
                    ls = math.sin(float(yaw_est))
                    x_l = float(lc * float(rel_w[0]) + ls * float(rel_w[1]))
                    y_l = float(-ls * float(rel_w[0]) + lc * float(rel_w[1]))
                    yaw_diff = float(angle_diff(float(yaw_est), float(follower.yaw)))
                    yaw_err = abs(float(yaw_diff))

                    if self.options.enable_micro_replan and d_tail <= 1.0 and float(yaw_err) > math.radians(30.0):
                        follower_cmd = ControlCommand(accel=-0.9, steer_rate=0.0)
                        self._follower_substage = "YAW_BACKOFF"
                    else:
                        if self.options.enable_capture_funnel:
                            follower_cmd = self._bcfd_near_field(
                                follower=follower,
                                leader=leader,
                                rear_target_xy=rear_est,
                                yaw_target=float(yaw_est),
                            )
                            self._follower_substage = "FUNNEL_ALIGN"
                        else:
                            target_speed = float(clamp(0.20 + 0.35 * d_tail, 0.10, 0.55))
                            follower_cmd = self._tracker.tracker.track_point(
                                follower,
                                float(rear_est[0]),
                                float(rear_est[1]),
                                float(yaw_est),
                                target_speed,
                            )
                            self._follower_substage = "DIRECT_TRACK"

                    if self.options.enable_fallback and d_tail <= float(self.cfg.docking.stage_switch_distance) and (not visual_valid):
                        fallback_global = True
                        fallback_reason = "vision_lost"
                        follower_cmd = ControlCommand(
                            accel=min(0.0, float(follower_cmd.accel)),
                            steer_rate=float(follower_cmd.steer_rate),
                        )

                    # Hard speed clamp in contact neighborhood to prevent "high-speed touch-and-pass".
                    if d_tail <= 0.55:
                        v_cap = 0.16 if d_tail > 0.30 else 0.08
                        if abs(float(follower.v)) > float(v_cap):
                            follower_cmd = ControlCommand(
                                accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(follower.v))),
                                steer_rate=float(follower_cmd.steer_rate),
                            )
                            self._follower_substage = "SPEED_CLAMP"

                    # Contact lock-assist: prioritize speed/yaw matching in the final centimeters.
                    if self.options.enable_capture_funnel and d_tail <= max(0.18 if self._fast_lane_certified else 0.25, 2.5 * float(self.cfg.docking.lock_position_tol)):
                        max_rate = math.radians(float(self.cfg.vehicle.max_steer_rate_deg_s))
                        dmax = math.radians(float(self.cfg.vehicle.max_steer_deg))
                        v_ref = float(leader.v) + 1.1 * float(clamp(float(x_l), -0.08, 0.08))
                        if abs(float(x_l)) < 0.03 and abs(float(y_l)) < 0.02 and float(yaw_err) < math.radians(3.0):
                            v_ref = float(leader.v)
                        v_ref = float(clamp(v_ref, -0.12, 0.12))
                        v_term = abs(float(follower.v)) + 0.25
                        delta_ref = float(yaw_diff + math.atan2(2.2 * float(y_l), float(v_term)))
                        delta_ref = float(clamp(delta_ref, -dmax, dmax))
                        steer_rate = float(clamp((delta_ref - float(follower.delta)) / max(float(self.cfg.control.dt), 1e-3), -max_rate, max_rate))
                        accel = float(
                            clamp(
                                float(self.cfg.control.speed.kp) * (float(v_ref) - float(follower.v)),
                                -float(self.cfg.vehicle.max_decel),
                                float(self.cfg.control.speed.max_accel_cmd),
                            )
                        )
                        follower_cmd = ControlCommand(accel=accel, steer_rate=steer_rate)
                        self._follower_substage = "LOCK_ASSIST"

        # Safety projection (exact collision checks, 1-step).
        if self.options.enable_safety_projection:
            leader_cmd = self._project_safe(ego=leader, cmd=leader_cmd, other=follower, allow_contact=False)
            follower_cmd = self._project_safe(ego=follower, cmd=follower_cmd, other=leader, allow_contact=allow_contact)

        # Joint safety: both vehicles move in the same tick, so validate the pairwise next-state too.
        leader_nxt = self.model.step(leader, leader_cmd, float(self.cfg.control.dt))
        follower_nxt = self.model.step(follower, follower_cmd, float(self.cfg.control.dt))
        d_tail_nxt = float(np.linalg.norm(self.geom.front_hitch(follower_nxt) - self.geom.rear_hitch(leader_nxt)))
        allow_contact_nxt = d_tail_nxt <= allow_contact_dist
        if self.options.enable_safety_projection and (not allow_contact_nxt) and self.collision.collide_vehicle_vehicle(follower_nxt, leader_nxt, include_clearance=True):
            rel = leader.xy() - follower.xy()
            lat = float(-math.sin(float(follower.yaw)) * float(rel[0]) + math.cos(float(follower.yaw)) * float(rel[1]))
            steer_escape = math.radians(float(self.cfg.vehicle.max_steer_rate_deg_s)) * (1.0 if lat >= 0.0 else -1.0)
            leader_cmd = ControlCommand(
                accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(leader.v))),
                steer_rate=0.0,
            )
            follower_cmd = ControlCommand(
                accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(follower.v))),
                steer_rate=float(steer_escape),
            )

        min_clear = float(
            min(
                self.collision.min_clearance_vehicle_obstacles(follower, self.obstacles),
                self.collision.min_clearance_vehicle_obstacles(leader, self.obstacles),
            )
        )

        dbg = DockSkillDebug(
            stage=str(self._stage),
            follower_substage=str(self._follower_substage),
            leader_relocated=bool(self._leader_relocated),
            d_tail=float(d_tail),
            w_vis=float(w_vis),
            visual_valid=bool(visual_valid),
            fusion_gamma=float(gamma),
            nis=float(nis),
            fallback_global=bool(fallback_global),
            fallback_reason=str(fallback_reason),
            min_clearance=float(min_clear),
            visual_lost_time=float(self._visual_lost_time),
            replan_count=int(self._replan_count),
        )
        return leader_cmd, follower_cmd, dbg
