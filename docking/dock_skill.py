from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .bcfd import BCFDConfig
from .collision import CollisionEngine
from .config import Config
from .coop_docking import CooperativeStagingPlanner, StagingPlan, StagingTracker
from .kinematics import AckermannModel, VehicleGeometry
from .lc_corridor import CorridorReciprocityExecutor
from .stage25_subskills import (
    TerminalTubePlan,
    TerminalViabilityTubeSkill,
    VisibilityPersistentRestagingSkill,
    VisibilityRestagePlan,
)
from .time_compression import PlanCompressionCertificate, PlanCompressionCertifier
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
    terminal_capture_boost: bool = False


@dataclass(frozen=True)
class DockSkillOptions:
    enable_cooperative_staging: bool = True
    enable_corridor_reciprocity: bool = True
    enable_lc_hybrid_search: bool = True
    fusion_mode: str = "belief_consistent"  # belief_consistent | dist_blend | hard_switch | global_only
    enable_capture_funnel: bool = True
    enable_micro_replan: bool = True
    enable_fallback: bool = True
    enable_safety_projection: bool = True
    enable_fast_lane: bool = True
    enable_terminal_viability_tube: bool = True
    enable_visibility_persistent_restaging: bool = True


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
        lane_hints: dict[str, Any] | None = None,
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
        self._corridor_executor = CorridorReciprocityExecutor(cfg)
        self._time_certifier = PlanCompressionCertifier(cfg)
        self._tvt_skill = TerminalViabilityTubeSkill(cfg)
        self._vpcr_skill = VisibilityPersistentRestagingSkill(cfg)

        self._prefer_static_leader = bool(prefer_static_leader or (not self.options.enable_cooperative_staging))
        self._lane_hints = dict(lane_hints or {})
        if plan is not None:
            plan = self._canonicalize_plan(plan, leader_ref=leader_init, follower_ref=follower_init)
        self._plan: StagingPlan | None = None if plan is None else plan
        self._fixed_plan: StagingPlan | None = None if plan is None else plan
        self._leader_goal: VehicleState | None = None
        self._follower_goal: VehicleState | None = None
        self._leader_path_xy: np.ndarray | None = None
        self._follower_path_xy: np.ndarray | None = None
        self._leader_relocated = False
        self._active_branch: str | None = None
        self._tvt_plan: TerminalTubePlan | None = None
        self._vpcr_plan: VisibilityRestagePlan | None = None
        self._tvt_used = False
        self._vpcr_used = 0
        self._vpcr_loss_attempted = False
        self._had_near_field_visual = False
        self._plan_time_cert: PlanCompressionCertificate | None = None
        self._semantic_lane_plan = False
        self._terminal_capture_boost = False
        self._lane_heading_target: float | None = None
        self._leader_post_align_active = False
        self._leader_post_align_attempts = 0
        self._leader_post_align_time = 0.0
        self._leader_post_align_total_time = 0.0
        self._leader_post_align_best_error = float("inf")
        self._leader_post_align_entry_error = float("inf")
        self._leader_post_align_prev_error = float("inf")
        self._leader_post_align_diverge_count = 0
        self._leader_post_align_failed = False
        self._leader_post_align_failure_reason = ""

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
        self._active_branch = None
        self._tvt_plan = None
        self._vpcr_plan = None
        self._tvt_used = False
        self._vpcr_used = 0
        self._vpcr_loss_attempted = False
        self._had_near_field_visual = False
        self._plan_time_cert: PlanCompressionCertificate | None = None
        self._terminal_capture_boost = False
        self._lane_heading_target = None
        self._leader_post_align_active = False
        self._leader_post_align_attempts = 0
        self._leader_post_align_time = 0.0
        self._leader_post_align_total_time = 0.0
        self._leader_post_align_best_error = float("inf")
        self._leader_post_align_entry_error = float("inf")
        self._leader_post_align_prev_error = float("inf")
        self._leader_post_align_diverge_count = 0
        self._leader_post_align_failed = False
        self._leader_post_align_failure_reason = ""
        self._vpcr_skill.reset()

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
        self._corridor_executor.reset()

        if self._fixed_plan is not None:
            self._apply_plan(self._fixed_plan)

    def _canonicalize_plan(
        self,
        plan: StagingPlan,
        *,
        leader_ref: VehicleState,
        follower_ref: VehicleState,
    ) -> StagingPlan:
        if self._prefer_static_leader or str(plan.control_mode) == "corridor_reciprocal":
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

    def _lc_terminal_lock_command(self, *, follower: VehicleState, leader: VehicleState) -> ControlCommand:
        if float(follower.v) > float(leader.v) + 0.01:
            accel = float(-float(self.cfg.vehicle.max_decel))
        else:
            accel = float(
                clamp(
                    float(self.cfg.control.speed.kp) * (float(leader.v) - float(follower.v)),
                    -float(self.cfg.vehicle.max_decel),
                    float(self.cfg.control.speed.max_accel_cmd),
                )
            )
        steer_rate = float(-float(follower.delta) / max(float(self.cfg.control.dt), 1e-3))
        command = ControlCommand(accel=accel, steer_rate=steer_rate)
        projected = self.model.step(follower, command, float(self.cfg.control.dt))
        clearance_m = float(self.collision.min_clearance_vehicle_obstacles(projected, self.obstacles))
        if clearance_m < float(self.cfg.safety.min_clearance) + 0.008:
            return ControlCommand(
                accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(follower.v))) if abs(float(follower.v)) > 1e-3 else 0.0,
                steer_rate=steer_rate,
            )
        return command

    def _freeze_vehicle(self, state: VehicleState) -> ControlCommand:
        if abs(float(state.v)) > 0.02:
            return ControlCommand(
                accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(state.v))),
                steer_rate=0.0,
            )
        return ControlCommand(
            accel=0.0,
            steer_rate=float(-float(state.delta) / max(float(self.cfg.control.dt), 1e-3)),
        )

    def _resolve_lane_heading_target(self, *, plan: StagingPlan | None) -> float | None:
        if plan is not None and isinstance(plan.metadata, dict) and "lane_heading" in plan.metadata:
            return float(plan.metadata["lane_heading"])
        centerline = self._lane_hints.get("centerline", [])
        if isinstance(centerline, list) and len(centerline) >= 2:
            start_xy = np.asarray(centerline[0], dtype=float)
            end_xy = np.asarray(centerline[-1], dtype=float)
            segment_xy = end_xy - start_xy
            if float(np.linalg.norm(segment_xy)) > 1e-9:
                return float(math.atan2(float(segment_xy[1]), float(segment_xy[0])))
        return None

    def _corridor_parallel_yaw_error(self, *, yaw: float) -> float:
        if self._lane_heading_target is None:
            return 0.0
        lane_heading = float(self._lane_heading_target)
        return float(
            min(
                abs(float(angle_diff(float(yaw), lane_heading))),
                abs(float(angle_diff(float(yaw), float(wrap_angle(lane_heading + math.pi))))),
            )
        )

    def _corridor_parallel_signed_error(self, *, yaw: float) -> float:
        if self._lane_heading_target is None:
            return 0.0
        lane_heading = float(self._lane_heading_target)
        candidate_a = float(angle_diff(lane_heading, float(yaw)))
        candidate_b = float(angle_diff(float(wrap_angle(lane_heading + math.pi)), float(yaw)))
        return float(candidate_a if abs(candidate_a) <= abs(candidate_b) else candidate_b)

    def _project_locked_follower_state(self, *, follower: VehicleState, leader: VehicleState) -> VehicleState | None:
        anchor_xy = self.geom.rear_hitch(leader)
        heading_xy = np.array([math.cos(float(leader.yaw)), math.sin(float(leader.yaw))], dtype=float)
        center_xy = anchor_xy - heading_xy * float(self.geom.front_hitch_x)
        projected = VehicleState(
            vehicle_id=int(follower.vehicle_id),
            x=float(center_xy[0]),
            y=float(center_xy[1]),
            yaw=float(leader.yaw),
            v=float(leader.v),
            delta=float(leader.delta),
            mode=follower.mode,
        )
        clearance_floor = float(self.cfg.safety.min_clearance) + 0.005
        if float(self.collision.min_clearance_vehicle_obstacles(projected, self.obstacles)) < clearance_floor:
            return None
        if float(self.collision.min_clearance_vehicle_obstacles(leader, self.obstacles)) < clearance_floor:
            return None
        return projected

    def _leader_post_align_command(
        self,
        *,
        leader: VehicleState,
        follower: VehicleState,
    ) -> tuple[ControlCommand, float, bool, bool]:
        current_error = float(self._corridor_parallel_yaw_error(yaw=float(leader.yaw)))
        if self._lane_heading_target is None or current_error <= math.radians(15.0):
            return self._freeze_vehicle(leader), float(current_error), True, False
        steer_soft = math.radians(18.0)
        steer_mid = math.radians(24.0)
        signed_gap = float(self._corridor_parallel_signed_error(yaw=float(leader.yaw)))
        target_sign = 1.0 if signed_gap >= 0.0 else -1.0
        dt = float(self.cfg.control.dt)
        max_rate = math.radians(float(self.cfg.vehicle.max_steer_rate_deg_s))
        phase_duration_s = 0.35
        forward_phase = bool(int(self._leader_post_align_time / max(phase_duration_s, dt)) % 2 == 0)
        steer_target = float(target_sign * (steer_mid if abs(signed_gap) > math.radians(18.0) else steer_soft))
        target_speed = float(0.10 if forward_phase else -0.08)
        target_delta = float(steer_target if forward_phase else -steer_target)
        leader_roll = leader.copy()
        follower_roll = follower.copy()
        min_clearance_m = float("inf")
        blocked = False
        for _ in range(8):
            accel = float(
                clamp(
                    float(self.cfg.control.speed.kp) * (float(target_speed) - float(leader_roll.v)),
                    -float(self.cfg.vehicle.max_decel),
                    float(self.cfg.control.speed.max_accel_cmd),
                )
            )
            steer_rate = float(clamp((float(target_delta) - float(leader_roll.delta)) / max(dt, 1e-3), -max_rate, max_rate))
            cmd = ControlCommand(accel=accel, steer_rate=steer_rate)
            leader_next = self.model.step(leader_roll, cmd, dt)
            follower_next = self._project_locked_follower_state(follower=follower_roll, leader=leader_next)
            if follower_next is None:
                blocked = True
                break
            min_clearance_m = float(
                min(
                    min_clearance_m,
                    self.collision.min_clearance_vehicle_obstacles(leader_next, self.obstacles),
                    self.collision.min_clearance_vehicle_obstacles(follower_next, self.obstacles),
                )
            )
            leader_roll = leader_next
            follower_roll = follower_next
        if blocked:
            return self._freeze_vehicle(leader), float(current_error), False, True
        yaw_error = float(self._corridor_parallel_yaw_error(yaw=float(leader_roll.yaw)))
        align_done = bool(yaw_error <= math.radians(15.0))
        command = ControlCommand(
            accel=float(
                clamp(
                    float(self.cfg.control.speed.kp) * (float(target_speed) - float(leader.v)),
                    -float(self.cfg.vehicle.max_decel),
                    float(self.cfg.control.speed.max_accel_cmd),
                )
            ),
            steer_rate=float(clamp((float(target_delta) - float(leader.delta)) / max(dt, 1e-3), -max_rate, max_rate)),
        )
        return command, float(yaw_error), align_done, False

    def _apply_plan(self, plan: StagingPlan, *, leader_ref: VehicleState | None = None, follower_ref: VehicleState | None = None) -> None:
        self._plan = plan
        self._corridor_executor.reset()
        self._semantic_lane_plan = str(plan.control_mode) == "corridor_reciprocal" or str(plan.reason) in {"semantic_anchor", "lc_corridor_reciprocity"}
        self._leader_goal = plan.leader_goal.copy()
        self._follower_goal = plan.follower_goal.copy()
        self._leader_path_xy = plan.leader_path_xy.astype(float).copy()
        self._follower_path_xy = plan.follower_path_xy.astype(float).copy()

        ref = self._leader_init if leader_ref is None else leader_ref
        follower_ref_eff = self._follower_init if follower_ref is None else follower_ref
        move_dist = float(np.linalg.norm(self._leader_goal.xy() - ref.xy()))
        self._leader_relocated = (not self._prefer_static_leader) and (move_dist >= 0.35)
        self._fast_lane_certified = self._compute_fast_lane_certificate(leader_ref=ref, follower_ref=follower_ref_eff)

        self._stage = "STAGING" if self._leader_relocated else "DOCKING"
        self._follower_substage = "APPROACH_PATH"
        self._near_field_active = False
        self._staging_time = 0.0
        self._dock_best_tail = float("inf")
        self._dock_stall_time = 0.0
        self._lane_heading_target = self._resolve_lane_heading_target(plan=plan)
        self._leader_post_align_active = False
        self._leader_post_align_attempts = 0
        self._leader_post_align_time = 0.0
        self._leader_post_align_total_time = 0.0
        self._leader_post_align_best_error = float("inf")
        self._leader_post_align_entry_error = float("inf")
        self._leader_post_align_prev_error = float("inf")
        self._leader_post_align_diverge_count = 0
        self._leader_post_align_failed = False
        self._leader_post_align_failure_reason = ""
        self._plan_time_cert = self._time_certifier.certify(
            leader_ref=ref,
            follower_ref=follower_ref_eff,
            leader_goal=self._leader_goal,
            follower_goal=self._follower_goal,
            leader_path_xy=self._leader_path_xy,
            follower_path_xy=self._follower_path_xy,
            obstacles=self.obstacles,
        )

    def _stage_plan_from_branch(self, plan: TerminalTubePlan | VisibilityRestagePlan) -> StagingPlan:
        return StagingPlan(
            leader_goal=plan.leader_goal.copy(),
            follower_goal=plan.follower_goal.copy(),
            leader_path_xy=plan.leader_path_xy.astype(float).copy(),
            follower_path_xy=plan.follower_path_xy.astype(float).copy(),
            score=float(plan.score),
            reason=str(plan.reason),
        )

    def _activate_branch_plan(
        self,
        *,
        branch: str,
        leader_ref: VehicleState,
        follower_ref: VehicleState,
        plan: TerminalTubePlan | VisibilityRestagePlan,
    ) -> None:
        self._apply_plan(self._stage_plan_from_branch(plan), leader_ref=leader_ref, follower_ref=follower_ref)
        self._active_branch = str(branch)
        self._stage = "STAGING"
        self._near_field_active = False
        self._staging_time = 0.0
        self._dock_best_tail = float("inf")
        self._dock_stall_time = 0.0
        self._follower_substage = "TUBE_TRACK" if branch == "TVT" else "RESTAGE_PATH"
        if branch == "VPCR":
            self._vpcr_skill.reset()

    def _maybe_activate_tvt(self, *, leader: VehicleState, follower: VehicleState, d_tail: float) -> None:
        if self._active_branch is not None or self._tvt_used:
            return
        if not self.options.enable_terminal_viability_tube:
            return
        if self._stage != "STAGING":
            return
        if line_blocked_by_obstacles(self.geom.front_hitch(follower), self.geom.rear_hitch(leader), self.obstacles):
            return
        if abs(float(angle_diff(float(leader.yaw), float(follower.yaw)))) > math.radians(25.0):
            return
        if self._leader_goal is None or self._follower_goal is None or self._leader_path_xy is None or self._follower_path_xy is None:
            return
        plan = self._tvt_skill.propose(
            obstacles=self.obstacles,
            leader=leader,
            follower=follower,
            nominal_leader_goal=self._leader_goal,
            nominal_follower_goal=self._follower_goal,
            nominal_leader_path_xy=self._leader_path_xy,
            nominal_follower_path_xy=self._follower_path_xy,
            dock_zone_clearance_m=float(self._dock_zone_clearance(self._leader_goal)),
        )
        if plan is None:
            return
        self._tvt_plan = plan
        self._activate_branch_plan(branch="TVT", leader_ref=leader, follower_ref=follower, plan=plan)
        self._tvt_used = True
        self._replan_count += 1

    def _maybe_activate_vpcr(self, *, leader: VehicleState, follower: VehicleState, d_tail: float, visual_valid: bool) -> None:
        if self._active_branch is not None or self._vpcr_used >= 2 or self._vpcr_loss_attempted:
            return
        if not self.options.enable_visibility_persistent_restaging:
            return
        if self._stage != "DOCKING":
            return
        if not self.options.enable_fallback:
            return
        if not self._had_near_field_visual:
            return
        if bool(visual_valid):
            return
        vpcr_loss_tau = float(max(0.35, 0.45 * float(self.cfg.docking.recovery_visual_loss_timeout_s)))
        if self._visual_lost_time < vpcr_loss_tau:
            return
        if d_tail > float(self.cfg.docking.stage_switch_distance) + 0.75:
            return
        self._vpcr_loss_attempted = True
        plan = self._vpcr_skill.propose(
            obstacles=self.obstacles,
            leader=leader,
            follower=follower,
        )
        if plan is None:
            return
        self._vpcr_plan = plan
        self._activate_branch_plan(branch="VPCR", leader_ref=leader, follower_ref=follower, plan=plan)
        self._vpcr_used += 1
        self._replan_count += 1

    def _maybe_plan(self) -> None:
        if self._plan is not None:
            return
        lane_hints = self._lane_hints if self.options.enable_corridor_reciprocity else {}
        plan = self._staging_planner.plan(
            obstacles=self.obstacles,
            leader=self._leader_init,
            follower=self._follower_init,
            prefer_static_leader=self._prefer_static_leader,
            semantic_hints=lane_hints,
            use_lc_hybrid_search=bool(self.options.enable_lc_hybrid_search),
        )
        plan = self._canonicalize_plan(plan, leader_ref=self._leader_init, follower_ref=self._follower_init)
        self._apply_plan(plan)

    def _maybe_replan_from_current(self, *, leader: VehicleState, follower: VehicleState, d_tail: float) -> None:
        if self._active_branch is not None:
            self._dock_best_tail = float("inf")
            self._dock_stall_time = 0.0
            return
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
            lane_hints = self._lane_hints if self.options.enable_corridor_reciprocity else {}
            plan = self._staging_planner.plan(
                obstacles=self.obstacles,
                leader=leader.copy(),
                follower=follower.copy(),
                prefer_static_leader=False,
                semantic_hints=lane_hints,
                use_lc_hybrid_search=bool(self.options.enable_lc_hybrid_search),
            )
            plan = self._canonicalize_plan(plan, leader_ref=leader, follower_ref=follower)
            self._apply_plan(plan, leader_ref=leader, follower_ref=follower)
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
        terminal_boost = bool(self._terminal_capture_boost and dist <= 1.50 and yaw_err <= math.radians(12.0) and abs(y_l) <= 0.26)
        if dist > float(cfg.align_enter_dist_m):
            x_ref = float(clamp(0.70 + 0.18 * dist, float(cfg.standoff_approach_min), float(cfg.standoff_approach_max)))
            v_cap = float(1.30 if fast_lane else (1.18 if terminal_boost else cfg.v_cap_approach))
        elif dist > float(cfg.dock_enter_dist_m):
            x_ref = float(cfg.standoff_align)
            v_cap = float(0.42 if fast_lane else (0.46 if terminal_boost else cfg.v_cap_align))
        else:
            x_ref = 0.0
            v_cap = float(0.18 if fast_lane else (0.24 if terminal_boost else cfg.v_cap_dock))

        ex = float(x_l - x_ref)
        v_ref = float(leader.v) + float(cfg.k_x) * float(ex)
        v_ref = float(clamp(v_ref, -float(self.cfg.vehicle.max_reverse_speed), float(v_cap)))

        # Contact gate: if too close but misaligned, back off to create room.
        gate_scale = 1.8 if fast_lane else (1.35 if terminal_boost else 1.0)
        gate_lat = float(cfg.gate_lat_m * gate_scale)
        gate_yaw = math.radians(float(cfg.gate_yaw_deg * gate_scale))
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
        if visual_valid and d_tail <= float(self.cfg.docking.stage_switch_distance) + 0.45:
            self._had_near_field_visual = True
            self._vpcr_loss_attempted = False

        self._maybe_activate_tvt(leader=leader, follower=follower, d_tail=float(d_tail))
        self._maybe_activate_vpcr(leader=leader, follower=follower, d_tail=float(d_tail), visual_valid=bool(visual_valid))
        if self._active_branch is None:
            self._maybe_replan_from_current(leader=leader, follower=follower, d_tail=float(d_tail))

        # Default commands.
        leader_cmd = ControlCommand(accel=0.0, steer_rate=0.0)
        follower_cmd = ControlCommand(accel=0.0, steer_rate=0.0)
        fallback_global = False
        fallback_reason = ""
        debug_stage = str(self._active_branch) if self._active_branch is not None else str(self._stage)
        debug_substage = str(self._follower_substage)

        if self._active_branch == "TVT" and self._tvt_plan is not None:
            leader_cmd, follower_cmd, branch_done, branch_substage = self._tvt_skill.command(
                leader=leader,
                follower=follower,
                plan=self._tvt_plan,
                obstacles=self.obstacles,
            )
            debug_stage = "TVT"
            debug_substage = str(branch_substage)
            self._follower_substage = str(branch_substage)
            if branch_done:
                self._active_branch = None
                self._tvt_plan = None
                self._stage = "DOCKING"
                self._near_field_active = False
                self._dock_best_tail = float("inf")
                self._dock_stall_time = 0.0
                self._follower_substage = "APPROACH_PATH"
        elif self._active_branch == "VPCR" and self._vpcr_plan is not None:
            leader_cmd, follower_cmd, branch_done, branch_substage = self._vpcr_skill.command(
                leader=leader,
                follower=follower,
                plan=self._vpcr_plan,
                obstacles=self.obstacles,
                visual_valid=bool(visual_valid),
            )
            debug_stage = "VPCR"
            debug_substage = str(branch_substage)
            self._follower_substage = str(branch_substage)
            if branch_done:
                self._active_branch = None
                self._vpcr_plan = None
                self._stage = "DOCKING"
                self._near_field_active = True
                self._terminal_capture_boost = True
                self._dock_best_tail = float("inf")
                self._dock_stall_time = 0.0
                self._follower_substage = "VPCR_HANDOFF"
                debug_substage = "VPCR_HANDOFF"
        elif self._stage == "STAGING":
            self._staging_time += float(self.cfg.control.dt)
            if self._plan is not None and str(self._plan.control_mode) == "corridor_reciprocal":
                leader_cmd, follower_cmd, stage_done, lc_substage = self._corridor_executor.command(
                    leader=leader,
                    follower=follower,
                    plan=self._plan,
                    obstacles=self.obstacles,
                )
                self._follower_substage = str(lc_substage)
                debug_stage = "STAGING_LC"
                debug_substage = str(lc_substage)
                if stage_done:
                    self._stage = "DOCKING"
            else:
                leader_rem = self._path_remaining_distance(leader, self._leader_path_xy)
                follower_rem = self._path_remaining_distance(follower, self._follower_path_xy)
                sync_ratio = leader_rem / max(follower_rem, 1e-3)
                time_cert = self._plan_time_cert
                progress_compress = bool(
                    time_cert is not None
                    and bool(time_cert.active)
                    and self._active_branch is None
                    and (not self._vpcr_loss_attempted)
                )
                if self._semantic_lane_plan:
                    leader_speed = 0.16
                    follower_speed = 0.24
                elif progress_compress:
                    leader_speed = float(clamp((0.45 + 0.18 * sync_ratio) * float(time_cert.stage_leader_scale), 0.32, 1.10))
                    follower_speed = float(clamp((0.55 + 0.55 * sync_ratio) * float(time_cert.stage_follower_scale), 0.38, 1.55))
                else:
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
                debug_stage = "STAGING"
                debug_substage = "APPROACH_PATH"
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
                debug_substage = "FORCED_DOCK"
        else:
            corridor_yaw_error = float(self._corridor_parallel_yaw_error(yaw=float(leader.yaw)))
            post_align_needed = bool(
                self._semantic_lane_plan
                and self._lane_heading_target is not None
                and d_tail <= 1.25
                and corridor_yaw_error > math.radians(12.0)
            )
            if post_align_needed and self._leader_post_align_total_time >= 5.0:
                self._leader_post_align_failed = True
            if (
                (not self._leader_post_align_active)
                and post_align_needed
                and (not self._leader_post_align_failed)
                and self._leader_post_align_attempts < 6
            ):
                self._leader_post_align_active = True
                self._leader_post_align_attempts += 1
                self._leader_post_align_time = 0.0
                self._leader_post_align_best_error = float(corridor_yaw_error)
                self._leader_post_align_entry_error = float(corridor_yaw_error)
                self._leader_post_align_prev_error = float(corridor_yaw_error)
                self._leader_post_align_diverge_count = 0
                self._leader_post_align_failure_reason = ""
            lc_terminal_hold = bool(
                self._semantic_lane_plan
                and d_tail <= float(self.cfg.docking.lock_position_tol) + 0.08
                and (not self._leader_post_align_active)
                and (corridor_yaw_error <= math.radians(15.0) or self._leader_post_align_failed)
            )
            # Keep leader steady during final docking.
            if self._leader_post_align_active:
                leader_cmd, align_error, align_done, align_blocked = self._leader_post_align_command(leader=leader, follower=follower)
                self._leader_post_align_time += float(self.cfg.control.dt)
                self._leader_post_align_total_time += float(self.cfg.control.dt)
                self._leader_post_align_best_error = float(min(self._leader_post_align_best_error, align_error))
                if float(corridor_yaw_error) > float(self._leader_post_align_best_error) + math.radians(1.5):
                    self._leader_post_align_diverge_count += 1
                else:
                    self._leader_post_align_diverge_count = max(0, int(self._leader_post_align_diverge_count) - 1)
                if float(corridor_yaw_error) > float(self._leader_post_align_entry_error) + math.radians(3.0):
                    self._leader_post_align_diverge_count = max(2, int(self._leader_post_align_diverge_count))
                if align_done:
                    self._leader_post_align_active = False
                    self._leader_post_align_time = 0.0
                    self._leader_post_align_prev_error = float(align_error)
                elif align_blocked:
                    self._leader_post_align_active = False
                    self._leader_post_align_failed = True
                    self._leader_post_align_time = 0.0
                    self._leader_post_align_failure_reason = "LEADER_POST_ALIGN_BLOCKED"
                    leader_cmd = self._freeze_vehicle(leader)
                elif self._leader_post_align_diverge_count >= 2:
                    self._leader_post_align_active = False
                    self._leader_post_align_failed = True
                    self._leader_post_align_time = 0.0
                    self._leader_post_align_failure_reason = "LEADER_POST_ALIGN_DIVERGE"
                    leader_cmd = self._freeze_vehicle(leader)
                elif self._leader_post_align_total_time >= 5.0:
                    self._leader_post_align_active = False
                    self._leader_post_align_failed = True
                    self._leader_post_align_time = 0.0
                    self._leader_post_align_failure_reason = "LEADER_POST_ALIGN_TIMEOUT"
                self._leader_post_align_prev_error = float(corridor_yaw_error)
            elif lc_terminal_hold and abs(float(leader.v)) > 1e-3:
                leader_cmd = ControlCommand(
                    accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(leader.v))),
                    steer_rate=float(-leader.delta / max(self.cfg.control.dt, 1e-3)),
                )
            elif abs(float(leader.v)) > 0.02:
                leader_cmd = ControlCommand(
                    accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(leader.v))),
                    steer_rate=0.0,
                )
            else:
                leader_cmd = ControlCommand(accel=0.0, steer_rate=float(-leader.delta / max(self.cfg.control.dt, 1e-3)))

            # Follower approach path until near-field.
            d_goal = float(np.linalg.norm(follower.xy() - self._follower_goal.xy()))
            if (not self._near_field_active) and d_goal > 0.45 and d_tail > float(self.cfg.docking.stage_switch_distance) + 0.20:
                time_cert = self._plan_time_cert
                approach_compress = bool(
                    time_cert is not None
                    and bool(time_cert.active)
                    and self._active_branch is None
                    and (not self._vpcr_loss_attempted)
                )
                approach_speed = 1.38 if self._fast_lane_certified else (float(time_cert.approach_speed_mps) if approach_compress else 1.05)
                follower_cmd, _ = self._tracker.command(
                    state=follower,
                    goal=self._follower_goal,
                    path_xy=self._follower_path_xy,
                    target_speed=float(approach_speed),
                    obstacles=self.obstacles,
                    other=leader,
                )
                self._follower_substage = "APPROACH_PATH"
                debug_substage = "APPROACH_PATH"
            else:
                self._near_field_active = True
                # Near-field: capture-funnel + MPPI micro-maneuver + lock assist.
                if self._leader_post_align_active:
                    follower_cmd = ControlCommand(accel=0.0, steer_rate=0.0)
                    self._follower_substage = "LEADER_POST_ALIGN"
                    debug_substage = "LEADER_POST_ALIGN"
                elif post_align_needed and self._leader_post_align_failed:
                    follower_cmd = ControlCommand(accel=0.0, steer_rate=0.0)
                    failure_reason = str(self._leader_post_align_failure_reason or "LEADER_POST_ALIGN_TIMEOUT")
                    self._follower_substage = failure_reason
                    debug_substage = failure_reason
                elif not have_global:
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

                    if lc_terminal_hold:
                        follower_cmd = self._lc_terminal_lock_command(follower=follower, leader=leader)
                        self._follower_substage = "LC_LOCK_HOLD"
                        debug_substage = "LC_LOCK_HOLD"
                    elif self.options.enable_micro_replan and d_tail <= 1.0 and float(yaw_err) > math.radians(30.0):
                        follower_cmd = ControlCommand(accel=-0.9, steer_rate=0.0)
                        self._follower_substage = "YAW_BACKOFF"
                        debug_substage = "YAW_BACKOFF"
                    else:
                        if self.options.enable_capture_funnel:
                            follower_cmd = self._bcfd_near_field(
                                follower=follower,
                                leader=leader,
                                rear_target_xy=rear_est,
                                yaw_target=float(yaw_est),
                            )
                            self._follower_substage = "FUNNEL_ALIGN"
                            debug_substage = "FUNNEL_ALIGN"
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
                            debug_substage = "DIRECT_TRACK"

                    if self.options.enable_fallback and d_tail <= float(self.cfg.docking.stage_switch_distance) and (not visual_valid):
                        fallback_global = True
                        fallback_reason = "vision_lost"
                        follower_cmd = ControlCommand(
                            accel=min(0.0, float(follower_cmd.accel)),
                            steer_rate=float(follower_cmd.steer_rate),
                        )

                    # Hard speed clamp in contact neighborhood to prevent "high-speed touch-and-pass".
                    if d_tail <= 0.55:
                        basin_boost = bool(self._terminal_capture_boost and visual_valid and yaw_err <= math.radians(12.0) and abs(y_l) <= 0.26)
                        v_cap = (0.45 if d_tail > 0.30 else 0.28) if basin_boost else (0.16 if d_tail > 0.30 else 0.08)
                        if self._follower_substage != "LC_LOCK_HOLD" and abs(float(follower.v)) > float(v_cap):
                            follower_cmd = ControlCommand(
                                accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(follower.v))),
                                steer_rate=float(follower_cmd.steer_rate),
                            )
                            self._follower_substage = "SPEED_CLAMP"
                            debug_substage = "SPEED_CLAMP"

                    # Contact lock-assist: prioritize speed/yaw matching in the final centimeters.
                    lock_enter = max(0.18 if self._fast_lane_certified else (1.10 if self._terminal_capture_boost and visual_valid and yaw_err <= math.radians(12.0) and abs(y_l) <= 0.26 else 0.25), 2.5 * float(self.cfg.docking.lock_position_tol))
                    if self.options.enable_capture_funnel and d_tail <= lock_enter and self._follower_substage != "LC_LOCK_HOLD":
                        max_rate = math.radians(float(self.cfg.vehicle.max_steer_rate_deg_s))
                        dmax = math.radians(float(self.cfg.vehicle.max_steer_deg))
                        v_span = 0.38 if self._terminal_capture_boost else 0.08
                        v_ref = float(leader.v) + 1.1 * float(clamp(float(x_l), -v_span, v_span))
                        if abs(float(x_l)) < 0.03 and abs(float(y_l)) < 0.02 and float(yaw_err) < math.radians(3.0):
                            v_ref = float(leader.v)
                        v_ref = float(clamp(v_ref, -0.16 if self._terminal_capture_boost else -0.12, 0.50 if self._terminal_capture_boost else 0.12))
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
                        debug_substage = "LOCK_ASSIST"
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
        certified_terminal_projection = False
        if self._semantic_lane_plan:
            rear_hitch = self.geom.rear_hitch(leader)
            front_hitch = self.geom.front_hitch(follower)
            rel_world = rear_hitch - front_hitch
            lc_now = math.cos(float(leader.yaw))
            ls_now = math.sin(float(leader.yaw))
            y_l_now = float(-ls_now * float(rel_world[0]) + lc_now * float(rel_world[1]))
            certified_terminal_projection = bool(
                float(d_tail) <= 0.86
                and abs(float(angle_diff(float(leader.yaw), float(follower.yaw)))) <= math.radians(13.0)
                and abs(float(y_l_now)) <= 0.30
                and float(min_clear) >= float(self.cfg.safety.min_clearance) + 0.01
            )

        dbg = DockSkillDebug(
            stage=str(debug_stage),
            follower_substage=str(debug_substage),
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
            terminal_capture_boost=bool(self._terminal_capture_boost or certified_terminal_projection),
        )
        return leader_cmd, follower_cmd, dbg
