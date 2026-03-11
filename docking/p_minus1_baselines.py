from __future__ import annotations

import math
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any

import numpy as np

from .bcfd import BaselineDockingController
from .collision import CollisionEngine
from .config import Config
from .controllers import PathTrackingController
from .coop_docking import CooperativeStagingPlanner, StagingPlan, StagingTracker
from .dock_skill import CooperativeDockingSkill, DockSkillOptions
from .kinematics import VehicleGeometry
from .math_utils import clamp
from .planner import LocalPlanner
from .sensors import GlobalPoseSensor, VisionSensor
from .types import ControlCommand, Obstacle, VehicleState


@dataclass(frozen=True)
class PMinus1MethodSpec:
    method_id: str
    family: str
    summary: str


METHOD_SPECS: dict[str, PMinus1MethodSpec] = {
    "co_bcfd": PMinus1MethodSpec("co_bcfd", "full", "Co-BCFD mainline with cooperative staging, CGFL, gated TVT/VPCR, plan-certified SC/EC time compression, terminal capture boost, and belief-consistent funnel control."),
    "S25_cgfl_only": PMinus1MethodSpec("S25_cgfl_only", "stage25", "Stage-2.5 baseline with CGFL only; TVT/VPCR disabled."),
    "S25_tvt_only": PMinus1MethodSpec("S25_tvt_only", "stage25", "Stage-2.5 branch eval: TVT enabled, VPCR disabled."),
    "S25_vpcr_only": PMinus1MethodSpec("S25_vpcr_only", "stage25", "Stage-2.5 branch eval: VPCR enabled, TVT disabled."),
    "S25_tvt_vpcr": PMinus1MethodSpec("S25_tvt_vpcr", "stage25", "Stage-2.5 branch eval: TVT and VPCR both enabled."),
    "T_global_only": PMinus1MethodSpec("T_global_only", "weak", "Global-only two-stage interception without visual feedback in-loop."),
    "T_hard_switch": PMinus1MethodSpec("T_hard_switch", "weak", "Two-stage baseline with hard GNSS-to-vision switching."),
    "T_pure_pursuit": PMinus1MethodSpec("T_pure_pursuit", "weak", "Pure geometric go-to-point / pursuit baseline without strong local planning."),
    "T_lattice_pbvs": PMinus1MethodSpec("T_lattice_pbvs", "strong", "Static-leader predock path planning + PBVS-like blended servo."),
    "T_parking_hierarchical": PMinus1MethodSpec("T_parking_hierarchical", "strong", "Static-leader predock path + local parking-style planner + final servo."),
    "T_coop_hard_switch": PMinus1MethodSpec("T_coop_hard_switch", "capability", "Cooperative staging allowed, but hard switching and no belief-consistent gate."),
    "T_coop_dist_blend": PMinus1MethodSpec("T_coop_dist_blend", "capability", "Cooperative staging + distance-only fusion, no NIS-consistency gate."),
    "A_no_stage": PMinus1MethodSpec("A_no_stage", "ablation", "Disable cooperative staging / leader relocation."),
    "A_no_belief_gate": PMinus1MethodSpec("A_no_belief_gate", "ablation", "Replace belief-consistent fusion with distance-only blend."),
    "A_no_funnel_gate": PMinus1MethodSpec("A_no_funnel_gate", "ablation", "Disable capture-funnel/contact gating in the near field."),
    "A_no_micro_maneuver": PMinus1MethodSpec("A_no_micro_maneuver", "ablation", "Disable near-field micro replan / backoff recovery."),
    "A_no_corridor_reciprocity": PMinus1MethodSpec("A_no_corridor_reciprocity", "ablation", "Disable lane-constrained corridor reciprocity and fall back to generic cooperative staging."),
    "A_no_lc_hybrid_search": PMinus1MethodSpec("A_no_lc_hybrid_search", "ablation", "Keep lane-constrained reciprocal staging, but replace the hybrid primitive search + basin certificate with the retained legacy reciprocal planner."),
    "A_no_fallback": PMinus1MethodSpec("A_no_fallback", "ablation", "Disable visual-loss fallback logic."),
    "A_no_safety_projection": PMinus1MethodSpec("A_no_safety_projection", "ablation", "Disable one-step safety projection layer."),
    "A_no_stage_no_belief": PMinus1MethodSpec("A_no_stage_no_belief", "ablation", "Disable staging and belief-consistent fusion together."),
    "A_no_funnel_no_micro": PMinus1MethodSpec("A_no_funnel_no_micro", "ablation", "Disable capture funnel and micro-maneuver recovery together."),
}

FULL_METHOD_IDS = [
    "co_bcfd",
    "T_global_only",
    "T_hard_switch",
    "T_pure_pursuit",
    "T_lattice_pbvs",
    "T_parking_hierarchical",
    "T_coop_hard_switch",
    "T_coop_dist_blend",
]

ABLATION_METHOD_IDS = [
    "A_no_stage",
    "A_no_belief_gate",
    "A_no_funnel_gate",
    "A_no_micro_maneuver",
    "A_no_corridor_reciprocity",
    "A_no_lc_hybrid_search",
    "A_no_fallback",
    "A_no_safety_projection",
    "A_no_stage_no_belief",
    "A_no_funnel_no_micro",
]

CORE_ABLATION_METHOD_IDS = [
    "A_no_stage",
    "A_no_belief_gate",
    "A_no_funnel_gate",
    "A_no_micro_maneuver",
    "A_no_corridor_reciprocity",
    "A_no_lc_hybrid_search",
]

STRONG_METHOD_IDS = [mid for mid, spec in METHOD_SPECS.items() if spec.family == "strong"]
CAPABILITY_METHOD_IDS = [mid for mid, spec in METHOD_SPECS.items() if spec.family == "capability"]


def freeze_leader_command(cfg: Config, leader: VehicleState) -> ControlCommand:
    if abs(float(leader.v)) > 0.02:
        return ControlCommand(accel=-float(cfg.vehicle.max_decel), steer_rate=0.0)
    return ControlCommand(accel=0.0, steer_rate=float(-leader.delta / max(cfg.control.dt, 1e-3)))


class SkillRunner:
    def __init__(self, skill: CooperativeDockingSkill) -> None:
        self.skill = skill

    def reset(self) -> None:
        self.skill.reset()

    def compute_commands(self, *, leader: VehicleState, follower: VehicleState, timestamp: float, obstacles: list[Obstacle]):
        _ = obstacles
        return self.skill.compute_commands(leader=leader, follower=follower, timestamp=timestamp)


class StaticLeaderRunner:
    def __init__(self, cfg: Config, follower_controller: BaselineDockingController) -> None:
        self.cfg = cfg
        self.ctrl = follower_controller

    def reset(self) -> None:
        if hasattr(self.ctrl, "reset"):
            self.ctrl.reset()

    def compute_commands(self, *, leader: VehicleState, follower: VehicleState, timestamp: float, obstacles: list[Obstacle]):
        cmd_f, dbg = self.ctrl.compute_command(
            follower=follower,
            leader_true=leader,
            timestamp=timestamp,
            obstacles=obstacles,
            dynamic_others=[],
        )
        return ControlCommand(accel=0.0, steer_rate=0.0), cmd_f, dbg


class CoopBaselineRunner:
    def __init__(
        self,
        cfg: Config,
        follower_controller: BaselineDockingController,
        *,
        tag: str,
        staging_plan: StagingPlan,
    ) -> None:
        self.cfg = cfg
        self.ctrl = follower_controller
        self.tag = str(tag)
        self.staging_plan = staging_plan
        self.staging_tracker = StagingTracker(cfg)
        self.stage = "STAGING"

    def reset(self) -> None:
        self.stage = "STAGING"
        if hasattr(self.ctrl, "reset"):
            self.ctrl.reset()

    def compute_commands(self, *, leader: VehicleState, follower: VehicleState, timestamp: float, obstacles: list[Obstacle]):
        _ = timestamp
        if self.stage == "STAGING":
            semantic_lane = str(self.staging_plan.reason) == "semantic_anchor"
            cmd_l, leader_done = self.staging_tracker.command(
                state=leader,
                goal=self.staging_plan.leader_goal,
                path_xy=self.staging_plan.leader_path_xy,
                target_speed=0.16 if semantic_lane else 0.55,
                obstacles=obstacles,
                other=follower,
            )
            cmd_f, _ = self.staging_tracker.command(
                state=follower,
                goal=self.staging_plan.follower_goal,
                path_xy=self.staging_plan.follower_path_xy,
                target_speed=0.24 if semantic_lane else 0.78,
                obstacles=obstacles,
                other=leader,
            )
            follower_goal_dist = float(np.linalg.norm(follower.xy() - self.staging_plan.follower_goal.xy()))
            follower_ready = bool(follower_goal_dist <= 0.70)
            if leader_done and follower_ready:
                self.stage = "DOCKING"
            dbg = SimpleNamespace(
                stage=f"STAGING_{self.tag}",
                follower_substage="APPROACH_PATH",
                w_vis=0.0,
                fallback_global=False,
                fallback_reason="",
                visual_lost_time=0.0,
                visual_valid=True,
                fusion_gamma=0.0,
                nis=0.0,
                min_clearance=1e6,
                replan_count=0,
            )
            return cmd_l, cmd_f, dbg

        cmd_l = freeze_leader_command(self.cfg, leader)
        cmd_f, dbg0 = self.ctrl.compute_command(
            follower=follower,
            leader_true=leader,
            timestamp=timestamp,
            obstacles=obstacles,
            dynamic_others=[],
        )
        dbg = SimpleNamespace(**dbg0.__dict__)
        dbg.stage = f"COOP_{self.tag}:{getattr(dbg0, 'stage', 'baseline')}"
        dbg.follower_substage = "BASELINE_DOCK"
        dbg.replan_count = 0
        return cmd_l, cmd_f, dbg


class StaticPredockRunner:
    def __init__(
        self,
        cfg: Config,
        *,
        seed: int,
        obstacles: list[Obstacle],
        leader_init: VehicleState,
        follower_init: VehicleState,
        mode: str,
        lane_hints: dict[str, Any] | None = None,
    ) -> None:
        self.cfg = cfg
        self.mode = str(mode)
        self.obstacles = list(obstacles)
        self.geom = VehicleGeometry(cfg.vehicle)
        self.tracker = StagingTracker(cfg)
        self.path_tracker = PathTrackingController(cfg.vehicle, cfg.control, steering_mode="stanley")
        self.collision = CollisionEngine(cfg.vehicle, cfg.safety)
        self.local_planner = LocalPlanner(cfg.vehicle, cfg.planner, self.collision, cfg.control.dt)
        self.plan = CooperativeStagingPlanner(cfg, seed=int(seed)).plan(
            obstacles=self.obstacles,
            leader=leader_init,
            follower=follower_init,
            prefer_static_leader=True,
            semantic_hints=lane_hints,
        )
        self.stage = "PREDOCK"
        self.gs = GlobalPoseSensor(cfg.sensors.global_sensor, seed=int(seed) + 11)
        self.vs = VisionSensor(cfg.sensors.vision, cfg.vehicle, seed=int(seed) + 17)
        self.final_servo = BaselineDockingController(
            cfg,
            global_sensor=self.gs,
            vision_sensor=self.vs,
            tracker=PathTrackingController(cfg.vehicle, cfg.control, steering_mode="pure_pursuit"),
            collision=self.collision,
            mode="dist_blend",
        )
        self.micro_fail_count = 0

    def reset(self) -> None:
        self.stage = "PREDOCK"
        self.micro_fail_count = 0
        self.final_servo.reset()

    def _debug(self, stage: str, sub: str, *, w_vis: float = 0.0, visual_valid: bool = True, fusion_gamma: float = 0.0, min_clearance: float = 1e6) -> Any:
        return SimpleNamespace(
            stage=stage,
            follower_substage=sub,
            w_vis=float(w_vis),
            visual_valid=bool(visual_valid),
            fusion_gamma=float(fusion_gamma),
            nis=0.0,
            fallback_global=False,
            fallback_reason="",
            visual_lost_time=0.0,
            min_clearance=float(min_clearance),
            replan_count=int(self.micro_fail_count),
        )

    def compute_commands(self, *, leader: VehicleState, follower: VehicleState, timestamp: float, obstacles: list[Obstacle]):
        _ = obstacles
        cmd_l = freeze_leader_command(self.cfg, leader)
        d_tail = float(np.linalg.norm(self.geom.front_hitch(follower) - self.geom.rear_hitch(leader)))
        predock_goal = VehicleState(
            vehicle_id=int(follower.vehicle_id),
            x=float(self.plan.follower_goal.x),
            y=float(self.plan.follower_goal.y),
            yaw=float(leader.yaw),
            v=0.0,
            delta=0.0,
            mode=follower.mode,
        )

        if self.stage == "PREDOCK":
            cmd_f, done = self.tracker.command(
                state=follower,
                goal=predock_goal,
                path_xy=self.plan.follower_path_xy,
                target_speed=0.88,
                obstacles=self.obstacles,
                other=leader,
            )
            if done or d_tail <= float(self.cfg.docking.stage_switch_distance) + 0.40:
                self.stage = "MICRO" if self.mode == "parking_hierarchical" else "FINAL"
            return cmd_l, cmd_f, self._debug(f"STATIC_{self.mode}", "PREDOCK_PATH")

        if self.stage == "MICRO":
            rear = self.geom.rear_hitch(leader)
            heading = np.array([math.cos(float(leader.yaw)), math.sin(float(leader.yaw))], dtype=float)
            approach_xy = rear - heading * float(clamp(0.28 + 0.22 * d_tail, 0.25, 0.75))
            plan = self.local_planner.plan_step(
                ego=follower,
                goal_xy=approach_xy,
                goal_yaw=float(leader.yaw),
                obstacles=self.obstacles,
                dynamic_others=[],
                force_goal_yaw=True,
            )
            if plan.feasible:
                cmd_f = plan.command
                self.micro_fail_count = 0
            else:
                self.micro_fail_count += 1
                cmd_f = self.path_tracker.track_point(
                    follower,
                    float(approach_xy[0]),
                    float(approach_xy[1]),
                    float(leader.yaw),
                    0.28,
                )
            if d_tail <= 0.70 or np.linalg.norm(follower.xy() - approach_xy) <= 0.28:
                self.stage = "FINAL"
            return cmd_l, cmd_f, self._debug(f"STATIC_{self.mode}", "MICRO_PARK")

        cmd_f, dbg0 = self.final_servo.compute_command(
            follower=follower,
            leader_true=leader,
            timestamp=timestamp,
            obstacles=self.obstacles,
            dynamic_others=[],
        )
        yaw_gap = abs(float(math.degrees(math.atan2(math.sin(float(leader.yaw - follower.yaw)), math.cos(float(leader.yaw - follower.yaw))))))
        if d_tail <= 0.55:
            v_cap = 0.16 if d_tail > 0.30 else 0.08
            if abs(float(follower.v)) > float(v_cap):
                cmd_f = ControlCommand(
                    accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(follower.v))),
                    steer_rate=float(cmd_f.steer_rate),
                )
        if d_tail <= 0.22 and yaw_gap <= 6.0:
            cmd_f = ControlCommand(
                accel=float(min(float(cmd_f.accel), -0.2 if float(follower.v) > float(leader.v) else 0.0)),
                steer_rate=0.0,
            )
        dbg = self._debug(
            f"STATIC_{self.mode}:{getattr(dbg0, 'stage', 'FINAL')}",
            "FINAL_SERVO",
            w_vis=float(getattr(dbg0, "w_vis", 0.0)),
            visual_valid=bool(getattr(dbg0, "visual_valid", True)),
            fusion_gamma=float(getattr(dbg0, "fusion_gamma", 0.0)),
            min_clearance=float(getattr(dbg0, "min_clearance", 1e6)),
        )
        dbg.visual_lost_time = float(getattr(dbg0, "visual_lost_time", 0.0))
        return cmd_l, cmd_f, dbg


def build_method_runners(
    cfg: Config,
    *,
    seed: int,
    obstacles: list[Obstacle],
    leader_init: VehicleState,
    follower_init: VehicleState,
    staging_plan: StagingPlan,
    lane_hints: dict[str, Any] | None = None,
) -> dict[str, Any]:
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    tracker = PathTrackingController(cfg.vehicle, cfg.control, steering_mode="pure_pursuit")

    def make_baseline(mode: str, *, seed_offset: int) -> BaselineDockingController:
        gs = GlobalPoseSensor(cfg.sensors.global_sensor, seed=int(seed) + 11 + int(seed_offset))
        vs = VisionSensor(cfg.sensors.vision, cfg.vehicle, seed=int(seed) + 17 + int(seed_offset))
        return BaselineDockingController(
            cfg,
            global_sensor=gs,
            vision_sensor=vs,
            tracker=tracker,
            collision=collision,
            mode=mode,
        )

    def make_skill(options: DockSkillOptions) -> CooperativeDockingSkill:
        plan = staging_plan
        prefer_static = not options.enable_cooperative_staging
        if (not options.enable_corridor_reciprocity) and str(getattr(plan, "control_mode", "generic")) == "corridor_reciprocal":
            plan = CooperativeStagingPlanner(cfg, seed=int(seed) + 451).plan(
                obstacles=obstacles,
                leader=leader_init,
                follower=follower_init,
                prefer_static_leader=False,
                semantic_hints={},
            )
        if bool(options.enable_corridor_reciprocity) and (not bool(options.enable_lc_hybrid_search)) and str(getattr(plan, "control_mode", "generic")) == "corridor_reciprocal":
            plan = CooperativeStagingPlanner(cfg, seed=int(seed) + 461).plan(
                obstacles=obstacles,
                leader=leader_init,
                follower=follower_init,
                prefer_static_leader=False,
                semantic_hints=lane_hints,
                use_lc_hybrid_search=False,
            )
        if prefer_static:
            plan = CooperativeStagingPlanner(cfg, seed=int(seed) + 401).plan(
                obstacles=obstacles,
                leader=leader_init,
                follower=follower_init,
                prefer_static_leader=True,
                semantic_hints=lane_hints,
            )
        return CooperativeDockingSkill(
            cfg,
            seed=int(seed),
            obstacles=obstacles,
            leader_init=leader_init,
            follower_init=follower_init,
            prefer_static_leader=prefer_static,
            plan=plan,
            lane_hints=lane_hints,
            options=options,
        )

    runners: dict[str, Any] = {
        "co_bcfd": SkillRunner(make_skill(DockSkillOptions())),
        "S25_cgfl_only": SkillRunner(make_skill(DockSkillOptions(enable_terminal_viability_tube=False, enable_visibility_persistent_restaging=False))),
        "S25_tvt_only": SkillRunner(make_skill(DockSkillOptions(enable_terminal_viability_tube=True, enable_visibility_persistent_restaging=False))),
        "S25_vpcr_only": SkillRunner(make_skill(DockSkillOptions(enable_terminal_viability_tube=False, enable_visibility_persistent_restaging=True))),
        "S25_tvt_vpcr": SkillRunner(make_skill(DockSkillOptions(enable_terminal_viability_tube=True, enable_visibility_persistent_restaging=True))),
        "T_global_only": StaticLeaderRunner(cfg, make_baseline("global_only", seed_offset=0)),
        "T_hard_switch": StaticLeaderRunner(cfg, make_baseline("hard_switch", seed_offset=10)),
        "T_pure_pursuit": StaticLeaderRunner(cfg, make_baseline("pure_pursuit", seed_offset=20)),
        "T_lattice_pbvs": StaticPredockRunner(
            cfg,
            seed=int(seed) + 101,
            obstacles=obstacles,
            leader_init=leader_init,
            follower_init=follower_init,
            mode="lattice_pbvs",
            lane_hints=lane_hints,
        ),
        "T_parking_hierarchical": StaticPredockRunner(
            cfg,
            seed=int(seed) + 151,
            obstacles=obstacles,
            leader_init=leader_init,
            follower_init=follower_init,
            mode="parking_hierarchical",
            lane_hints=lane_hints,
        ),
        "T_coop_hard_switch": CoopBaselineRunner(
            cfg,
            make_baseline("hard_switch", seed_offset=30),
            tag="hard_switch",
            staging_plan=staging_plan,
        ),
        "T_coop_dist_blend": CoopBaselineRunner(
            cfg,
            make_baseline("dist_blend", seed_offset=40),
            tag="dist_blend",
            staging_plan=staging_plan,
        ),
        "A_no_stage": SkillRunner(make_skill(DockSkillOptions(enable_cooperative_staging=False))),
        "A_no_belief_gate": SkillRunner(make_skill(DockSkillOptions(fusion_mode="dist_blend"))),
        "A_no_funnel_gate": SkillRunner(make_skill(DockSkillOptions(enable_capture_funnel=False))),
        "A_no_micro_maneuver": SkillRunner(make_skill(DockSkillOptions(enable_micro_replan=False))),
        "A_no_corridor_reciprocity": SkillRunner(make_skill(DockSkillOptions(enable_corridor_reciprocity=False))),
        "A_no_lc_hybrid_search": SkillRunner(make_skill(DockSkillOptions(enable_lc_hybrid_search=False))),
        "A_no_fallback": SkillRunner(make_skill(DockSkillOptions(enable_fallback=False))),
        "A_no_safety_projection": SkillRunner(make_skill(DockSkillOptions(enable_safety_projection=False))),
        "A_no_stage_no_belief": SkillRunner(
            make_skill(DockSkillOptions(enable_cooperative_staging=False, fusion_mode="dist_blend"))
        ),
        "A_no_funnel_no_micro": SkillRunner(
            make_skill(DockSkillOptions(enable_capture_funnel=False, enable_micro_replan=False))
        ),
    }
    return runners
