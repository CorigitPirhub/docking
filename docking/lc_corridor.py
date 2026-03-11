from __future__ import annotations

import heapq
import itertools
import math
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .bcfd import BCFDConfig
from .collision import CollisionEngine
from .config import Config
from .controllers import PathTrackingController
from .grid_planner import GridAStarPlanner, GridPlannerConfig
from .kinematics import AckermannModel, VehicleGeometry
from .math_utils import angle_diff, clamp, wrap_angle
from .planner import LocalPlanner
from .types import ControlCommand, Obstacle, VehicleMode, VehicleState


@dataclass(frozen=True)
class CorridorReciprocalPlan:
    leader_goal: VehicleState
    follower_goal: VehicleState
    leader_path_xy: np.ndarray
    follower_path_xy: np.ndarray
    leader_primitive_states: np.ndarray | None
    leader_primitive_cmd: np.ndarray | None
    leader_primitive_steps: int
    score: float
    reason: str
    metadata: dict[str, Any]


@dataclass(frozen=True)
class PrimitivePhase:
    target_speed: float
    steer_target: float
    duration_s: float


@dataclass(frozen=True)
class MotionPrimitive:
    name: str
    phases: tuple[PrimitivePhase, ...]
    cost_bias: float
    gear_switches: int


@dataclass(frozen=True)
class PrimitiveRollout:
    primitive: MotionPrimitive
    end_state: VehicleState
    states: np.ndarray
    commands: np.ndarray
    path_length_m: float


@dataclass(frozen=True)
class BasinReadyCertificate:
    anchor: VehicleState
    predock: VehicleState
    follower_path_xy: np.ndarray
    sigma_geom: float
    follower_path_clearance_m: float
    anchor_clearance_m: float
    handoff_clearance_m: float
    dock_zone_clearance_m: float
    alignment_quality: float
    metadata: dict[str, Any]


@dataclass(frozen=True)
class TerminalClosureCertificate:
    sigma: float
    tube_clearance_m: float
    axial_score: float
    lateral_score: float
    yaw_score: float
    x_l_m: float
    y_l_m: float
    yaw_gap_deg: float
    witness_sigma: float
    witness_name: str
    metadata: dict[str, Any]


@dataclass(order=True)
class _SearchNode:
    priority: float
    counter: int
    state: VehicleState = field(compare=False)
    depth: int = field(compare=False)
    g_cost: float = field(compare=False)
    min_clearance_m: float = field(compare=False)
    path_states: tuple[tuple[float, float, float], ...] = field(compare=False)
    path_commands: tuple[tuple[float, float, float], ...] = field(compare=False)


@dataclass(frozen=True)
class RightBayConstraintContext:
    bay_depth_m: float
    bay_width_m: float
    leader_offset_m: float
    terminal_side: float
    lane_heading: float
    anchor_hint_xy: tuple[float, float]


@dataclass(frozen=True)
class MacroPrimitiveTemplate:
    name: str
    primitive_names: tuple[str, ...]
    metadata: dict[str, Any]


@dataclass(frozen=True)
class MacroTemplateEvaluation:
    plan: CorridorReciprocalPlan
    sigma_post: float
    soft_path_cost: float
    front_clearance_m: float
    longitudinal_offset_m: float
    exec_nominal_sigma: float
    exec_robust_sigma: float
    terminal_nominal_sigma: float
    terminal_robust_sigma: float
    combined_nominal_sigma: float
    combined_robust_sigma: float
    min_clearance_m: float
    legacy_priority: float


@dataclass(frozen=True)
class EnvironmentDominantDirection:
    heading: float
    concentration: float
    axis_ratio: float
    source: str


class EnvironmentDominantDirectionExtractor:
    def extract(
        self,
        *,
        lane: dict[str, Any],
        leader: VehicleState,
        anchor_hint_xy: np.ndarray,
        corridor_polygons: list[np.ndarray],
    ) -> EnvironmentDominantDirection:
        centerline = lane.get("centerline", [])
        if isinstance(centerline, list) and len(centerline) >= 2:
            points_xy = np.asarray(centerline, dtype=float)
            segment = points_xy[-1] - points_xy[0]
            if float(np.linalg.norm(segment)) > 1e-9:
                return EnvironmentDominantDirection(
                    heading=float(math.atan2(float(segment[1]), float(segment[0]))),
                    concentration=0.98,
                    axis_ratio=1.0,
                    source="centerline",
                )
        local_points: list[np.ndarray] = []
        probe_center = 0.5 * (leader.xy() + np.asarray(anchor_hint_xy, dtype=float))
        for polygon in corridor_polygons:
            polygon_xy = np.asarray(polygon, dtype=float)
            if len(polygon_xy) < 3:
                continue
            centroid_xy = np.mean(polygon_xy, axis=0)
            if float(np.linalg.norm(centroid_xy - probe_center)) <= 6.0:
                local_points.append(polygon_xy)
        if local_points:
            stacked_xy = np.vstack(local_points)
            centered_xy = stacked_xy - np.mean(stacked_xy, axis=0, keepdims=True)
            cov = np.cov(centered_xy.T)
            eigvals, eigvecs = np.linalg.eigh(cov)
            major_idx = int(np.argmax(eigvals))
            minor_idx = 1 - major_idx
            major_vec = eigvecs[:, major_idx]
            heading = float(math.atan2(float(major_vec[1]), float(major_vec[0])))
            major_val = float(max(eigvals[major_idx], 1e-9))
            minor_val = float(max(eigvals[minor_idx], 1e-9))
            axis_ratio = float(major_val / minor_val)
            concentration = float(clamp((axis_ratio - 1.0) / 6.0, 0.0, 0.95))
            return EnvironmentDominantDirection(
                heading=float(heading),
                concentration=float(concentration),
                axis_ratio=float(axis_ratio),
                source="pca",
            )
        fallback_heading = float(math.atan2(float(anchor_hint_xy[1] - leader.y), float(anchor_hint_xy[0] - leader.x)))
        if not math.isfinite(fallback_heading):
            fallback_heading = float(leader.yaw)
        return EnvironmentDominantDirection(
            heading=float(fallback_heading),
            concentration=0.10,
            axis_ratio=1.0,
            source="goal_bearing",
        )


class UnifiedCertificateField:
    def __init__(self, cfg: Config, *, planner: "CorridorReciprocityPlanner", executor: "CorridorReciprocityExecutor" | None = None) -> None:
        self.cfg = cfg
        self.planner = planner
        self.executor = executor

    def sigma_post(self, *, leader_pose: VehicleState, corridor_info: dict[str, Any]) -> float:
        lane_heading = float(corridor_info.get("lane_heading", leader_pose.yaw))
        yaw_gap = float(angle_diff(float(leader_pose.yaw), float(lane_heading)))
        return float(clamp(abs(math.cos(yaw_gap)), 0.0, 1.0))

    def sigma_c_offline(
        self,
        *,
        anchor: VehicleState,
        follower: VehicleState,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
        lane_heading: float,
    ) -> tuple[float, float]:
        nominal_sigma, robust_sigma, _ = self.planner._robust_execution_certificate(
            anchor=anchor,
            follower=follower,
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
            lane_heading=lane_heading,
            allow_grid_fallback=False,
        )
        return float(nominal_sigma), float(robust_sigma)

    def sigma_t_offline(
        self,
        *,
        anchor: VehicleState,
        follower: VehicleState,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
    ) -> tuple[float, float]:
        nominal_sigma, robust_sigma, _ = self.planner._robust_terminal_safety_certificate(
            anchor=anchor,
            follower=follower,
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
        )
        return float(nominal_sigma), float(robust_sigma)

    def sigma_c_runtime(
        self,
        *,
        leader_pose: VehicleState,
        follower_pose: VehicleState,
        corridor_info: dict[str, Any],
        obstacles: list[Obstacle],
    ) -> tuple[float, float]:
        runtime = self.executor if self.executor is not None else None
        if runtime is None:
            return 0.0, 0.0
        nominal_sigma, robust_sigma, _ = runtime._robust_execution_certificate(
            leader_pose=leader_pose,
            follower_pose=follower_pose,
            corridor_info=corridor_info,
            obstacles=obstacles,
        )
        return float(nominal_sigma), float(robust_sigma)

    def sigma_t_runtime(
        self,
        *,
        leader_pose: VehicleState,
        follower_pose: VehicleState,
        corridor_info: dict[str, Any],
        obstacles: list[Obstacle],
    ) -> tuple[float, dict[str, Any]]:
        runtime = self.executor if self.executor is not None else None
        if runtime is None:
            return 0.0, {}
        return runtime.compute_terminal_closure_certificate(
            leader_pose=leader_pose,
            follower_pose=follower_pose,
            corridor_info=corridor_info,
            obstacles=obstacles,
        )

    def sigma_r_runtime(
        self,
        *,
        leader_pose: VehicleState,
        follower_pose: VehicleState,
        corridor_info: dict[str, Any],
        obstacles: list[Obstacle],
    ) -> tuple[float, dict[str, Any]]:
        runtime = self.executor if self.executor is not None else None
        if runtime is None:
            return 0.0, {}
        sigma_terminal, terminal_meta = self.sigma_t_runtime(
            leader_pose=leader_pose,
            follower_pose=follower_pose,
            corridor_info=corridor_info,
            obstacles=obstacles,
        )
        handoff_ready, handoff_meta = runtime._docking_handoff_ready(
            leader=leader_pose,
            follower=follower_pose,
            obstacles=obstacles,
        )
        witness_gain = float(terminal_meta.get("witness_sigma", sigma_terminal)) - float(sigma_terminal)
        sigma_release = float(
            clamp(
                float(sigma_terminal)
                * (1.0 if handoff_ready else 0.0)
                * float(clamp((witness_gain + 0.08) / 0.16, 0.0, 1.0)),
                0.0,
                1.0,
            )
        )
        return sigma_release, {
            "handoff_ready": bool(handoff_ready),
            "handoff_meta": dict(handoff_meta),
            "witness_gain": float(witness_gain),
            "sigma_terminal": float(sigma_terminal),
            **dict(terminal_meta),
        }

    def unified_staging_score(
        self,
        *,
        leader_pose: VehicleState,
        follower_pose: VehicleState,
        corridor_info: dict[str, Any],
        sigma_progress: float,
        sigma_terminal_nominal: float,
        sigma_terminal_robust: float,
        sigma_corridor_nominal: float,
        sigma_corridor_robust: float,
        sigma_visibility: float = 1.0,
    ) -> float:
        sigma_post = self.sigma_post(leader_pose=leader_pose, corridor_info=corridor_info)
        return float(
            0.18 * float(clamp(sigma_progress, 0.0, 1.0))
            + 0.14 * float(clamp(sigma_terminal_nominal, 0.0, 1.0))
            + 0.12 * float(clamp(sigma_terminal_robust, 0.0, 1.0))
            + 0.18 * float(clamp(sigma_corridor_nominal, 0.0, 1.0))
            + 0.18 * float(clamp(sigma_corridor_robust, 0.0, 1.0))
            + 0.02 * float(clamp(sigma_visibility, 0.0, 1.0))
            + 0.18 * float(clamp(sigma_post, 0.0, 1.0))
        )


class RightBayEscapeLibrary:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg

    def _mirror_sequence(self, names: tuple[str, ...]) -> tuple[str, ...]:
        return tuple(
            str(name).replace("left", "__tmp__").replace("right", "left").replace("__tmp__", "right")
            for name in names
        )

    def build_templates(self, context: RightBayConstraintContext) -> tuple[MacroPrimitiveTemplate, ...]:
        base_back_long = int(clamp(1 + int(context.bay_depth_m > 1.05), 1, 3))
        base_sweep_repeat = int(clamp(1 + int(context.leader_offset_m > 1.10), 1, 2))
        base_settle_short = int(clamp(1 + int(context.leader_offset_m > 1.05 and context.bay_width_m < 1.85), 1, 2))
        back_long_options = tuple(sorted({max(1, base_back_long - 1), base_back_long, min(3, base_back_long + 1)}))
        sweep_options = tuple(sorted({1, base_sweep_repeat}))
        settle_options = tuple(sorted({1, base_settle_short}))

        canonical: list[tuple[str, ...]] = []
        for sweep_repeat in sweep_options:
            for back_long_count in back_long_options:
                for settle_short_count in settle_options:
                    canonical.append(
                        (
                            *("reverse_arc_right_long" for _ in range(sweep_repeat)),
                            *("reverse_straight_long" for _ in range(back_long_count)),
                            "reverse_straight_short",
                            "forward_arc_right",
                            *("reverse_straight_short" for _ in range(settle_short_count)),
                            *("reverse_arc_left" for _ in range(settle_short_count)),
                        )
                    )
                    canonical.append(
                        (
                            "reverse_arc_right_long",
                            "reverse_arc_left_tight",
                            *("reverse_straight_long" for _ in range(back_long_count)),
                            "reverse_straight_short",
                            *("reverse_arc_left_tight" for _ in range(settle_short_count)),
                        )
                    )
                    canonical.append(
                        (
                            "reverse_turn_forward_right",
                            "reverse_arc_right_long",
                            *("reverse_straight_long" for _ in range(max(1, back_long_count - 1))),
                            "forward_arc_right_tight",
                            "reverse_straight_short",
                            "reverse_arc_left",
                        )
                    )
                    canonical.append(
                        (
                            "reverse_arc_right_long",
                            *("reverse_straight_long" for _ in range(back_long_count)),
                            "reverse_straight_short",
                            "forward_arc_right_tight",
                            "forward_straight_short",
                            "reverse_arc_left_tight",
                        )
                    )
        deduped: list[tuple[str, ...]] = []
        seen_sequences: set[tuple[str, ...]] = set()
        for sequence in canonical:
            if sequence in seen_sequences:
                continue
            seen_sequences.add(sequence)
            deduped.append(sequence)
        templates: list[MacroPrimitiveTemplate] = []
        witness_sequence = (
            "reverse_arc_right_long",
            "reverse_straight_long",
            "reverse_straight_long",
            "reverse_straight_short",
            "forward_arc_right",
            "reverse_straight_short",
            "reverse_arc_left",
        )
        witness_sequence_mirror = self._mirror_sequence(witness_sequence)
        for index, names in enumerate(deduped[:10]):
            sequence = tuple(names)
            if float(context.terminal_side) > 0.0:
                sequence = self._mirror_sequence(sequence)
            long_reverse_count = int(sum(1 for primitive_name in sequence if primitive_name.endswith("straight_long")))
            legacy_priority = 0.0
            if sequence == witness_sequence or sequence == witness_sequence_mirror:
                legacy_priority = 1.0
            elif long_reverse_count >= 2 and any("forward_arc_" in primitive_name for primitive_name in sequence):
                legacy_priority = 0.55
            templates.append(
                MacroPrimitiveTemplate(
                    name=f"right_bay_escape_macro_{index}",
                    primitive_names=sequence,
                    metadata={
                        "bay_depth_m": float(context.bay_depth_m),
                        "bay_width_m": float(context.bay_width_m),
                        "leader_offset_m": float(context.leader_offset_m),
                        "terminal_side": float(context.terminal_side),
                        "lane_heading": float(context.lane_heading),
                        "anchor_hint_xy": [float(context.anchor_hint_xy[0]), float(context.anchor_hint_xy[1])],
                        "legacy_priority": float(legacy_priority),
                        "legacy_validated": bool(legacy_priority >= 0.99),
                    },
                )
            )
        return tuple(templates)


class StagingCertificateOptimizer:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.right_bay_library = RightBayEscapeLibrary(cfg)

    def build_context(self, *, leader: VehicleState, corridor_info: dict[str, Any]) -> RightBayConstraintContext:
        lane_heading = float(corridor_info.get("lane_heading", leader.yaw))
        anchor_hint_xy = np.asarray(corridor_info.get("anchor_hint_xy", [leader.x, leader.y]), dtype=float)
        lane_y = float(corridor_info.get("lane_y", leader.y))
        lane_tangent = np.array([math.cos(lane_heading), math.sin(lane_heading)], dtype=float)
        lane_normal = np.array([-math.sin(lane_heading), math.cos(lane_heading)], dtype=float)
        rel_xy = leader.xy() - anchor_hint_xy
        lateral_offset = abs(float(np.dot(rel_xy, lane_normal)))
        lane_center_offset = abs(float(np.dot(leader.xy() - np.array([leader.x, lane_y], dtype=float), lane_normal)))
        bay_depth_m = float(corridor_info.get("bay_depth_m", max(lateral_offset, lane_center_offset, 0.9)))
        bay_width_m = float(corridor_info.get("bay_width_m", corridor_info.get("corridor_width_min_m", 1.75)))
        terminal_side = float(corridor_info.get("terminal_side", 1.0 if lane_y - float(leader.y) >= 0.0 else -1.0))
        leader_offset_m = float(max(lateral_offset, abs(float(leader.y) - float(anchor_hint_xy[1]))))
        return RightBayConstraintContext(
            bay_depth_m=float(bay_depth_m),
            bay_width_m=float(bay_width_m),
            leader_offset_m=float(leader_offset_m),
            terminal_side=float(terminal_side),
            lane_heading=float(lane_heading),
            anchor_hint_xy=(float(anchor_hint_xy[0]), float(anchor_hint_xy[1])),
        )

    def select(self, *, leader: VehicleState, corridor_info: dict[str, Any]) -> tuple[MacroPrimitiveTemplate, ...]:
        context = self.build_context(leader=leader, corridor_info=corridor_info)
        if not (float(context.bay_depth_m) >= 0.95 and float(context.bay_width_m) <= 2.10 and float(context.leader_offset_m) >= 0.70):
            return tuple()
        return self.right_bay_library.build_templates(context)

    def fallback_to_legacy_primitives(self, *, leader: VehicleState, corridor_info: dict[str, Any]) -> tuple[MacroPrimitiveTemplate, ...]:
        templates = list(self.select(leader=leader, corridor_info=corridor_info))
        templates.sort(
            key=lambda template: (
                float(template.metadata.get("legacy_priority", 0.0)),
                len(tuple(template.primitive_names)),
            ),
            reverse=True,
        )
        return tuple(templates)

    def should_prefer_macro_plan(self, *, leader: VehicleState, corridor_info: dict[str, Any]) -> bool:
        context = self.build_context(leader=leader, corridor_info=corridor_info)
        anchor_hint_xy = np.asarray(context.anchor_hint_xy, dtype=float)
        lane_tangent = np.array([math.cos(float(context.lane_heading)), math.sin(float(context.lane_heading))], dtype=float)
        forward_offset_m = float(np.dot(anchor_hint_xy - leader.xy(), lane_tangent))
        return bool(
            float(context.terminal_side) < 0.0
            and float(context.leader_offset_m) >= 0.95
            and float(context.bay_depth_m) >= 1.10
            and float(forward_offset_m) >= 0.30
        )


MacroPrimitiveSelector = StagingCertificateOptimizer


class HybridMotionPrimitiveLibrary:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.dt = float(cfg.control.dt)
        self.model = AckermannModel(cfg.vehicle)
        self.geom = VehicleGeometry(cfg.vehicle)
        self.collision = CollisionEngine(cfg.vehicle, cfg.safety)
        self._primitives = self._build_primitives()

    @property
    def primitives(self) -> tuple[MotionPrimitive, ...]:
        return self._primitives

    def _build_primitives(self) -> tuple[MotionPrimitive, ...]:
        steer_soft = math.radians(16.0)
        steer_mid = math.radians(24.0)
        steer_tight = math.radians(30.0)
        return (
            MotionPrimitive("forward_straight_short", (PrimitivePhase(0.20, 0.0, 0.80),), cost_bias=0.08, gear_switches=0),
            MotionPrimitive("forward_straight_long", (PrimitivePhase(0.22, 0.0, 1.20),), cost_bias=0.16, gear_switches=0),
            MotionPrimitive("reverse_straight_short", (PrimitivePhase(-0.18, 0.0, 0.80),), cost_bias=0.08, gear_switches=0),
            MotionPrimitive("reverse_straight_long", (PrimitivePhase(-0.20, 0.0, 1.20),), cost_bias=0.16, gear_switches=0),
            MotionPrimitive("forward_arc_left", (PrimitivePhase(0.18, steer_mid, 1.00),), cost_bias=0.18, gear_switches=0),
            MotionPrimitive("forward_arc_right", (PrimitivePhase(0.18, -steer_mid, 1.00),), cost_bias=0.18, gear_switches=0),
            MotionPrimitive("forward_arc_left_tight", (PrimitivePhase(0.16, steer_tight, 1.10),), cost_bias=0.24, gear_switches=0),
            MotionPrimitive("forward_arc_right_tight", (PrimitivePhase(0.16, -steer_tight, 1.10),), cost_bias=0.24, gear_switches=0),
            MotionPrimitive("reverse_arc_left", (PrimitivePhase(-0.16, steer_mid, 1.00),), cost_bias=0.18, gear_switches=0),
            MotionPrimitive("reverse_arc_right", (PrimitivePhase(-0.16, -steer_mid, 1.00),), cost_bias=0.18, gear_switches=0),
            MotionPrimitive("reverse_arc_left_tight", (PrimitivePhase(-0.15, steer_tight, 1.10),), cost_bias=0.24, gear_switches=0),
            MotionPrimitive("reverse_arc_right_tight", (PrimitivePhase(-0.15, -steer_tight, 1.10),), cost_bias=0.24, gear_switches=0),
            MotionPrimitive("reverse_arc_left_long", (PrimitivePhase(-0.16, steer_mid, 1.45),), cost_bias=0.26, gear_switches=0),
            MotionPrimitive("reverse_arc_right_long", (PrimitivePhase(-0.16, -steer_mid, 1.45),), cost_bias=0.26, gear_switches=0),
            MotionPrimitive(
                "forward_turn_reverse_left",
                (
                    PrimitivePhase(0.16, steer_soft, 0.65),
                    PrimitivePhase(-0.16, -steer_tight, 0.85),
                ),
                cost_bias=0.36,
                gear_switches=1,
            ),
            MotionPrimitive(
                "forward_turn_reverse_right",
                (
                    PrimitivePhase(0.16, -steer_soft, 0.65),
                    PrimitivePhase(-0.16, steer_tight, 0.85),
                ),
                cost_bias=0.36,
                gear_switches=1,
            ),
            MotionPrimitive(
                "reverse_turn_forward_left",
                (
                    PrimitivePhase(-0.16, steer_soft, 0.70),
                    PrimitivePhase(0.16, -steer_tight, 0.80),
                ),
                cost_bias=0.38,
                gear_switches=1,
            ),
            MotionPrimitive(
                "reverse_turn_forward_right",
                (
                    PrimitivePhase(-0.16, -steer_soft, 0.70),
                    PrimitivePhase(0.16, steer_tight, 0.80),
                ),
                cost_bias=0.38,
                gear_switches=1,
            ),
        )

    def rollout(self, *, start: VehicleState, primitive: MotionPrimitive) -> PrimitiveRollout:
        state = start.copy()
        states: list[tuple[float, float, float]] = []
        commands: list[tuple[float, float, float]] = []
        path_length_m = 0.0
        for phase in primitive.phases:
            steps = max(1, int(math.ceil(float(phase.duration_s) / max(self.dt, 1e-3))))
            for _ in range(steps):
                accel = clamp(2.4 * (float(phase.target_speed) - float(state.v)), -float(self.cfg.vehicle.max_decel), float(self.cfg.vehicle.max_accel))
                steer_rate = clamp(
                    (float(phase.steer_target) - float(state.delta)) / max(self.dt, 1e-3),
                    -self.model.max_steer_rate_rad(),
                    self.model.max_steer_rate_rad(),
                )
                command = ControlCommand(accel=float(accel), steer_rate=float(steer_rate))
                next_state = self.model.step(state, command, self.dt)
                path_length_m += float(np.linalg.norm(next_state.xy() - state.xy()))
                states.append((float(next_state.x), float(next_state.y), float(next_state.yaw)))
                commands.append((float(command.accel), float(command.steer_rate), float(phase.target_speed)))
                state = next_state
        return PrimitiveRollout(
            primitive=primitive,
            end_state=state.copy(),
            states=np.asarray(states, dtype=float),
            commands=np.asarray(commands, dtype=float),
            path_length_m=float(path_length_m),
        )

    def is_safe(
        self,
        *,
        rollout: PrimitiveRollout,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
        other: VehicleState | None,
        min_clearance: float,
        in_bounds,
    ) -> tuple[bool, float, str]:
        min_clearance_m = float("inf")
        for state_row in np.asarray(rollout.states, dtype=float):
            state = VehicleState(
                vehicle_id=-1,
                x=float(state_row[0]),
                y=float(state_row[1]),
                yaw=float(state_row[2]),
                v=0.0,
                delta=0.0,
            )
            if not bool(in_bounds(state, margin=0.42)):
                return False, -1.0, "out_of_bounds"
            if corridor_polygons and not self._pose_in_corridor(state, corridor_polygons):
                return False, -1.0, "outside_corridor"
            if self.collision.in_collision(state, obstacles, [other] if other is not None else []):
                return False, -1.0, "collision"
            obstacle_clearance = float(self.collision.min_clearance_vehicle_obstacles(state, obstacles)) if obstacles else 1e6
            vehicle_clearance = float(self.collision.min_clearance_vehicle_vehicle(state, other)) if other is not None else 1e6
            current_clearance = float(min(obstacle_clearance, vehicle_clearance))
            if current_clearance < float(min_clearance):
                return False, float(current_clearance), "clearance"
            min_clearance_m = min(min_clearance_m, current_clearance)
        if math.isinf(min_clearance_m):
            min_clearance_m = 1e6
        return True, float(min_clearance_m), "ok"

    def _pose_in_corridor(self, state: VehicleState, corridor_polygons: list[np.ndarray]) -> bool:
        support_points = [*self.geom.body_polygon(state), self.geom.front_hitch(state), self.geom.rear_hitch(state)]
        return all(any(_point_in_polygon(point, polygon) for polygon in corridor_polygons) for point in support_points)


def _point_segment_distance(point_xy: np.ndarray, seg_start_xy: np.ndarray, seg_end_xy: np.ndarray) -> float:
    seg_vec = seg_end_xy - seg_start_xy
    denom = float(np.dot(seg_vec, seg_vec))
    if denom <= 1e-12:
        return float(np.linalg.norm(point_xy - seg_start_xy))
    alpha = clamp(float(np.dot(point_xy - seg_start_xy, seg_vec) / denom), 0.0, 1.0)
    proj_xy = seg_start_xy + alpha * seg_vec
    return float(np.linalg.norm(point_xy - proj_xy))


def _point_in_polygon(point_xy: np.ndarray, polygon_xy: np.ndarray, *, edge_tol: float = 1e-6) -> bool:
    polygon = np.asarray(polygon_xy, dtype=float)
    point = np.asarray(point_xy, dtype=float)
    for idx in range(len(polygon)):
        seg_start = polygon[idx]
        seg_end = polygon[(idx + 1) % len(polygon)]
        if _point_segment_distance(point, seg_start, seg_end) <= float(edge_tol):
            return True
    inside = False
    x_coord = float(point[0])
    y_coord = float(point[1])
    for idx in range(len(polygon)):
        x0, y0 = float(polygon[idx][0]), float(polygon[idx][1])
        x1, y1 = float(polygon[(idx + 1) % len(polygon)][0]), float(polygon[(idx + 1) % len(polygon)][1])
        intersects = bool((y0 > y_coord) != (y1 > y_coord))
        if not intersects:
            continue
        x_intersection = float((x1 - x0) * (y_coord - y0) / max(y1 - y0, 1e-12) + x0)
        if x_coord <= x_intersection:
            inside = not inside
    return bool(inside)


class CorridorReciprocityPlanner:
    def __init__(self, cfg: Config, *, seed: int) -> None:
        self.cfg = cfg
        self.seed = int(seed)
        self.geom = VehicleGeometry(cfg.vehicle)
        self.collision = CollisionEngine(cfg.vehicle, cfg.safety)
        self.motion_library = HybridMotionPrimitiveLibrary(cfg)
        self.optimizer = StagingCertificateOptimizer(cfg)
        self.macro_selector = self.optimizer
        self.cert_field = UnifiedCertificateField(cfg, planner=self)
        self.direction_extractor = EnvironmentDominantDirectionExtractor()

    def _lane_y(self, lane: dict[str, Any]) -> float:
        centerline = lane.get("centerline", [])
        if isinstance(centerline, list) and centerline:
            return float(centerline[0][1])
        return float(lane.get("leader_stage_anchor_xy", [0.0, 0.0])[1])

    def _planner(self, obstacles: list[Obstacle]) -> GridAStarPlanner:
        return GridAStarPlanner(
            width=float(self.cfg.environment.width),
            height=float(self.cfg.environment.height),
            obstacles=obstacles,
            cfg=GridPlannerConfig(resolution=0.16, inflation_radius=0.52, boundary_block=1, max_expansions=260_000),
        )

    def _corridor_polygons(self, lane: dict[str, Any]) -> list[np.ndarray]:
        polygons = lane.get("corridor_polygons", [])
        if not isinstance(polygons, list):
            return []
        out: list[np.ndarray] = []
        for polygon in polygons:
            if isinstance(polygon, list) and len(polygon) >= 3:
                out.append(np.asarray(polygon, dtype=float))
        return out

    def _corridor_bounds(self, corridor_polygons: list[np.ndarray]) -> tuple[float, float, float, float]:
        if not corridor_polygons:
            half_w = 0.5 * float(self.cfg.environment.width)
            half_h = 0.5 * float(self.cfg.environment.height)
            return -half_w, half_w, -half_h, half_h
        stacked = np.vstack(corridor_polygons)
        return float(np.min(stacked[:, 0])), float(np.max(stacked[:, 0])), float(np.min(stacked[:, 1])), float(np.max(stacked[:, 1]))

    def _lane_heading(self, lane: dict[str, Any], *, default_yaw: float) -> float:
        centerline = lane.get("centerline", [])
        if isinstance(centerline, list) and len(centerline) >= 2:
            start_xy = np.asarray(centerline[0], dtype=float)
            end_xy = np.asarray(centerline[-1], dtype=float)
            segment_xy = end_xy - start_xy
            if float(np.linalg.norm(segment_xy)) > 1e-9:
                return float(math.atan2(float(segment_xy[1]), float(segment_xy[0])))
        return float(default_yaw)

    def _macro_metadata(self, *, lane: dict[str, Any], leader_ref: VehicleState) -> dict[str, Any]:
        lane_heading = self._lane_heading(lane, default_yaw=float(lane.get("leader_stage_anchor_yaw", leader_ref.yaw)))
        anchor_hint_xy = np.asarray(lane.get("leader_stage_anchor_xy", [leader_ref.x, leader_ref.y]), dtype=float)
        lane_normal = np.array([-math.sin(lane_heading), math.cos(lane_heading)], dtype=float)
        rel_xy = leader_ref.xy() - anchor_hint_xy
        leader_offset_m = abs(float(np.dot(rel_xy, lane_normal)))
        return {
            "bay_depth_m": float(lane.get("bay_depth_m", max(0.9, leader_offset_m))),
            "bay_width_m": float(lane.get("corridor_width_min_m", 1.75)),
            "leader_offset_m": float(leader_offset_m),
            "terminal_side": float(1.0 if self._lane_y(lane) - float(leader_ref.y) >= 0.0 else -1.0),
        }

    def _informed_anchor_sampling_profile(
        self,
        *,
        lane: dict[str, Any],
        leader: VehicleState,
        anchor_hint_xy: np.ndarray,
        anchor_hint_yaw: float,
        corridor_polygons: list[np.ndarray],
    ) -> dict[str, Any]:
        dominant = self.direction_extractor.extract(
            lane=lane,
            leader=leader,
            anchor_hint_xy=np.asarray(anchor_hint_xy, dtype=float),
            corridor_polygons=corridor_polygons,
        )
        tangent_vec = np.array([math.cos(float(dominant.heading)), math.sin(float(dominant.heading))], dtype=float)
        normal_vec = np.array([-math.sin(float(dominant.heading)), math.cos(float(dominant.heading))], dtype=float)
        strong_bias = bool(float(dominant.concentration) >= 0.65 and bool(corridor_polygons))
        medium_bias = bool(float(dominant.concentration) >= 0.35)
        if strong_bias:
            tangent_offsets = (0.0, -0.20, 0.20, -0.45, 0.45, -0.75, 0.75, -1.10, 1.10)
            normal_offsets = (0.0, -0.10, 0.10, -0.20, 0.20, -0.32, 0.32)
            yaw_offsets_deg = (0.0, -6.0, 6.0, -15.0, 15.0, -24.0, 24.0, -30.0, 30.0)
        elif medium_bias:
            tangent_offsets = (0.0, -0.25, 0.25, -0.55, 0.55, -0.85, 0.85, -1.10, 1.10)
            normal_offsets = (0.0, -0.12, 0.12, -0.24, 0.24, -0.36, 0.36)
            yaw_offsets_deg = (0.0, -10.0, 10.0, -18.0, 18.0, -28.0, 28.0)
        else:
            heading_gap_regime = str(lane.get("heading_gap_regime", "")).strip().lower()
            tangent_offsets = (0.0, -0.25, 0.25, -0.55, 0.55)
            normal_offsets = (0.0, -0.16, 0.16, -0.30, 0.30)
            if heading_gap_regime == "large":
                yaw_offsets_deg = (-35.0, -20.0, 0.0, 20.0, 35.0)
            elif heading_gap_regime == "medium":
                yaw_offsets_deg = (-30.0, -15.0, 0.0, 15.0, 30.0)
            else:
                yaw_offsets_deg = (-20.0, -10.0, 0.0, 10.0, 20.0)
        yaw_bases = [float(dominant.heading)]
        if abs(float(angle_diff(float(anchor_hint_yaw), float(dominant.heading)))) >= math.radians(8.0):
            yaw_bases.append(float(anchor_hint_yaw))
        return {
            "dominant_direction": dominant,
            "tangent_vec": tangent_vec,
            "normal_vec": normal_vec,
            "tangent_offsets_m": tuple(float(v) for v in tangent_offsets),
            "normal_offsets_m": tuple(float(v) for v in normal_offsets),
            "yaw_offsets_deg": tuple(float(v) for v in yaw_offsets_deg),
            "yaw_bases": tuple(float(v) for v in yaw_bases),
        }

    def _in_bounds(self, state: VehicleState, margin: float) -> bool:
        half_w = 0.5 * float(self.cfg.environment.width) - float(margin)
        half_h = 0.5 * float(self.cfg.environment.height) - float(margin)
        return (-half_w <= float(state.x) <= half_w) and (-half_h <= float(state.y) <= half_h)

    def _pose_in_corridor(self, state: VehicleState, corridor_polygons: list[np.ndarray]) -> bool:
        if not corridor_polygons:
            return True
        support_points = [*self.geom.body_polygon(state), self.geom.front_hitch(state), self.geom.rear_hitch(state)]
        return all(any(_point_in_polygon(point_xy, polygon_xy) for polygon_xy in corridor_polygons) for point_xy in support_points)

    def _densify_path(self, path_xy: np.ndarray, *, step: float = 0.10) -> np.ndarray:
        if path_xy is None or len(path_xy) < 2:
            return np.asarray(path_xy, dtype=float)
        dense_points = [np.asarray(path_xy[0], dtype=float)]
        for start_xy, end_xy in zip(path_xy[:-1], path_xy[1:]):
            start_arr = np.asarray(start_xy, dtype=float)
            end_arr = np.asarray(end_xy, dtype=float)
            segment_xy = end_arr - start_arr
            segment_len = float(np.linalg.norm(segment_xy))
            if segment_len <= 1e-9:
                continue
            num_points = max(2, int(math.ceil(segment_len / max(step, 1e-3))) + 1)
            dense_points.extend(np.linspace(start_arr, end_arr, num_points, dtype=float)[1:])
        return np.asarray(dense_points, dtype=float)

    def _direct_path_xy(self, *, start_xy: np.ndarray, goal_xy: np.ndarray, step_m: float = 0.10) -> np.ndarray:
        delta_xy = np.asarray(goal_xy, dtype=float) - np.asarray(start_xy, dtype=float)
        distance_m = float(np.linalg.norm(delta_xy))
        if distance_m <= 1e-9:
            return np.asarray([np.asarray(start_xy, dtype=float), np.asarray(goal_xy, dtype=float)], dtype=float)
        num_points = max(2, int(math.ceil(distance_m / max(step_m, 1e-3))) + 1)
        return np.linspace(np.asarray(start_xy, dtype=float), np.asarray(goal_xy, dtype=float), num_points, dtype=float)

    def _path_terminal_yaw(self, path_xy: np.ndarray, *, goal_yaw: float) -> float:
        path = np.asarray(path_xy, dtype=float)
        if len(path) < 2:
            return float(goal_yaw)
        last_xy = path[-1]
        prev_xy = path[-2]
        if float(np.linalg.norm(last_xy - prev_xy)) <= 1e-9:
            return float(goal_yaw)
        return float(math.atan2(float(last_xy[1] - prev_xy[1]), float(last_xy[0] - prev_xy[0])))

    def _path_min_clearance(
        self,
        path_xy: np.ndarray,
        *,
        proto: VehicleState,
        goal_yaw: float,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
        other: VehicleState | None = None,
    ) -> float:
        path = self._densify_path(np.asarray(path_xy, dtype=float), step=0.08)
        if len(path) == 0:
            return -1.0
        min_clearance_m = float("inf")
        for index, point_xy in enumerate(path):
            state = proto.copy()
            state.x = float(point_xy[0])
            state.y = float(point_xy[1])
            if index + 1 < len(path):
                next_xy = path[index + 1]
                state.yaw = float(math.atan2(float(next_xy[1] - point_xy[1]), float(next_xy[0] - point_xy[0])))
            else:
                state.yaw = float(goal_yaw)
            if not self._in_bounds(state, margin=0.40):
                return -1.0
            if corridor_polygons and not self._pose_in_corridor(state, corridor_polygons):
                return -1.0
            if self.collision.in_collision(state, obstacles, [other] if other is not None else []):
                return -1.0
            obstacle_clearance = float(self.collision.min_clearance_vehicle_obstacles(state, obstacles)) if obstacles else 1e6
            vehicle_clearance = float(self.collision.min_clearance_vehicle_vehicle(state, other)) if other is not None else 1e6
            min_clearance_m = min(min_clearance_m, float(min(obstacle_clearance, vehicle_clearance)))
        return float(min_clearance_m if math.isfinite(min_clearance_m) else 1e6)

    def _predock_pose(self, leader_goal: VehicleState, *, follower_id: int, follower_mode: VehicleMode | str) -> VehicleState:
        heading_xy = np.array([math.cos(float(leader_goal.yaw)), math.sin(float(leader_goal.yaw))], dtype=float)
        rear_hitch_xy = self.geom.rear_hitch(leader_goal)
        standoff_m = float(max(0.78, float(self.cfg.docking.stage_switch_distance) - 0.42))
        center_xy = rear_hitch_xy - heading_xy * float(self.geom.front_hitch_x + standoff_m)
        return VehicleState(
            vehicle_id=int(follower_id),
            x=float(center_xy[0]),
            y=float(center_xy[1]),
            yaw=float(leader_goal.yaw),
            v=0.0,
            delta=0.0,
            mode=follower_mode,
        )

    def _dock_zone_clearance(self, leader_goal: VehicleState, obstacles: list[Obstacle]) -> float:
        heading_xy = np.array([math.cos(float(leader_goal.yaw)), math.sin(float(leader_goal.yaw))], dtype=float)
        rear_hitch_xy = self.geom.rear_hitch(leader_goal)
        clearances: list[float] = []
        for standoff_m in (0.30, 0.55, 0.80):
            center_xy = rear_hitch_xy - heading_xy * float(self.geom.front_hitch_x + float(standoff_m))
            probe = VehicleState(vehicle_id=-1, x=float(center_xy[0]), y=float(center_xy[1]), yaw=float(leader_goal.yaw), v=0.0, delta=0.0)
            clearances.append(float(self.collision.min_clearance_vehicle_obstacles(probe, obstacles)))
        return float(min(clearances) if clearances else 1e6)

    def _certificate_from_anchor(
        self,
        *,
        anchor: VehicleState,
        anchor_hint_xy: np.ndarray,
        lane_heading: float,
        follower: VehicleState,
        obstacles: list[Obstacle],
        planner: GridAStarPlanner,
        corridor_polygons: list[np.ndarray],
    ) -> BasinReadyCertificate | None:
        clearance_req = float(max(self.cfg.safety.min_clearance + 0.02, 0.12))
        if not self._pose_in_corridor(anchor, corridor_polygons):
            return None
        if self.collision.in_collision(anchor, obstacles, []):
            return None
        predock = self._predock_pose(anchor, follower_id=int(follower.vehicle_id), follower_mode=follower.mode)
        if not self._pose_in_corridor(predock, corridor_polygons):
            return None
        if self.collision.in_collision(predock, obstacles, [anchor]):
            return None
        follower_path_xy = planner.plan(start_xy=follower.xy(), goal_xy=predock.xy())
        if follower_path_xy is None:
            return None
        follower_path_xy = self._densify_path(np.asarray(follower_path_xy, dtype=float), step=0.10)
        follower_clearance = self._path_min_clearance(
            follower_path_xy,
            proto=follower,
            goal_yaw=float(predock.yaw),
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
            other=anchor,
        )
        if follower_clearance < clearance_req:
            return None
        anchor_clearance = float(self.collision.min_clearance_vehicle_obstacles(anchor, obstacles))
        handoff_clearance = float(
            min(
                self.collision.min_clearance_vehicle_obstacles(predock, obstacles),
                self.collision.min_clearance_vehicle_vehicle(predock, anchor),
            )
        )
        dock_zone_clearance = self._dock_zone_clearance(anchor, obstacles)
        approach_yaw = self._path_terminal_yaw(follower_path_xy, goal_yaw=float(predock.yaw))
        ingress_alignment = float(
            clamp(
                1.0 - abs(float(angle_diff(float(predock.yaw), float(approach_yaw)))) / math.radians(42.0),
                0.0,
                1.0,
            )
        )
        lane_alignment = float(
            max(
                clamp(1.0 - abs(float(angle_diff(float(anchor.yaw), float(lane_heading)))) / math.radians(48.0), 0.0, 1.0),
                clamp(1.0 - abs(float(angle_diff(float(anchor.yaw), float(wrap_angle(float(lane_heading) + math.pi))))) / math.radians(48.0), 0.0, 1.0),
            )
        )
        hint_closeness = float(clamp(1.0 - float(np.linalg.norm(anchor.xy() - anchor_hint_xy)) / 1.6, 0.0, 1.0))
        sigma_geom = (
            0.28 * float(clamp((follower_clearance - clearance_req) / 0.45, 0.0, 1.0))
            + 0.20 * float(clamp((handoff_clearance - clearance_req) / 0.40, 0.0, 1.0))
            + 0.18 * float(clamp((dock_zone_clearance - clearance_req) / 0.50, 0.0, 1.0))
            + 0.14 * float(clamp((anchor_clearance - clearance_req) / 0.55, 0.0, 1.0))
            + 0.12 * float(ingress_alignment)
            + 0.08 * float(max(lane_alignment, hint_closeness))
        )
        return BasinReadyCertificate(
            anchor=anchor.copy(),
            predock=predock.copy(),
            follower_path_xy=follower_path_xy.astype(float),
            sigma_geom=float(clamp(float(sigma_geom), 0.0, 1.0)),
            follower_path_clearance_m=float(follower_clearance),
            anchor_clearance_m=float(anchor_clearance),
            handoff_clearance_m=float(handoff_clearance),
            dock_zone_clearance_m=float(dock_zone_clearance),
            alignment_quality=float(0.55 * ingress_alignment + 0.45 * lane_alignment),
            metadata={
                "ingress_alignment": float(ingress_alignment),
                "lane_alignment": float(lane_alignment),
                "hint_closeness": float(hint_closeness),
            },
        )

    def _candidate_anchor_states(
        self,
        *,
        leader: VehicleState,
        follower: VehicleState,
        lane: dict[str, Any],
        obstacles: list[Obstacle],
        planner: GridAStarPlanner,
        corridor_polygons: list[np.ndarray],
    ) -> list[BasinReadyCertificate]:
        anchor_hint_xy = np.asarray(lane.get("leader_stage_anchor_xy", [leader.x, leader.y]), dtype=float)
        anchor_hint_yaw = float(lane.get("leader_stage_anchor_yaw", 0.0))
        lane_heading = self._lane_heading(lane, default_yaw=anchor_hint_yaw)
        bounds_x_min, bounds_x_max, bounds_y_min, bounds_y_max = self._corridor_bounds(corridor_polygons)
        pocket_polygon = corridor_polygons[1] if len(corridor_polygons) >= 2 else (corridor_polygons[0] if corridor_polygons else None)
        if pocket_polygon is not None:
            pocket_x_min = float(np.min(pocket_polygon[:, 0]))
            pocket_x_max = float(np.max(pocket_polygon[:, 0]))
            pocket_y_min = float(np.min(pocket_polygon[:, 1]))
            pocket_y_max = float(np.max(pocket_polygon[:, 1]))
        else:
            pocket_x_min, pocket_x_max, pocket_y_min, pocket_y_max = bounds_x_min, bounds_x_max, bounds_y_min, bounds_y_max
        sampling_profile = self._informed_anchor_sampling_profile(
            lane=lane,
            leader=leader,
            anchor_hint_xy=anchor_hint_xy,
            anchor_hint_yaw=float(anchor_hint_yaw),
            corridor_polygons=corridor_polygons,
        )
        dominant = sampling_profile["dominant_direction"]
        tangent_vec = np.asarray(sampling_profile["tangent_vec"], dtype=float)
        normal_vec = np.asarray(sampling_profile["normal_vec"], dtype=float)
        tangent_offsets = tuple(float(v) for v in sampling_profile["tangent_offsets_m"])
        normal_offsets = tuple(float(v) for v in sampling_profile["normal_offsets_m"])
        yaw_offsets_deg = tuple(float(v) for v in sampling_profile["yaw_offsets_deg"])
        yaw_bases = tuple(float(v) for v in sampling_profile["yaw_bases"])
        seen: set[tuple[int, int, int]] = set()
        certificates: list[BasinReadyCertificate] = []
        stop_generation = False
        for tangent_offset_m in tangent_offsets:
            for normal_offset_m in normal_offsets:
                candidate_xy = anchor_hint_xy + float(tangent_offset_m) * tangent_vec + float(normal_offset_m) * normal_vec
                anchor_x = float(clamp(float(candidate_xy[0]), pocket_x_min + 0.28, pocket_x_max - 0.28))
                anchor_y = float(clamp(float(candidate_xy[1]), bounds_y_min + 0.28, bounds_y_max - 0.28))
                for yaw_base in yaw_bases:
                    for yaw_offset_deg in yaw_offsets_deg:
                        anchor_yaw = float(wrap_angle(float(yaw_base) + math.radians(float(yaw_offset_deg))))
                        key = (int(round(anchor_x / 0.05)), int(round(anchor_y / 0.05)), int(round(math.degrees(anchor_yaw) / 5.0)))
                        if key in seen:
                            continue
                        seen.add(key)
                        anchor = VehicleState(
                            vehicle_id=int(leader.vehicle_id),
                            x=float(anchor_x),
                            y=float(anchor_y),
                            yaw=float(anchor_yaw),
                            v=0.0,
                            delta=0.0,
                            mode=leader.mode,
                        )
                        certificate = self._certificate_from_anchor(
                            anchor=anchor,
                            anchor_hint_xy=anchor_hint_xy,
                            lane_heading=lane_heading,
                            follower=follower,
                            obstacles=obstacles,
                            planner=planner,
                            corridor_polygons=corridor_polygons,
                        )
                        if certificate is not None:
                            certificates.append(certificate)
                            if len(certificates) >= 32:
                                stop_generation = True
                                break
                    if stop_generation:
                        break
                if stop_generation:
                    break
            if stop_generation:
                break
        heading_gap_regime = str(lane.get("heading_gap_regime", "")).strip().lower()
        if heading_gap_regime == "large":
            legacy_yaw_offsets_deg = (-35.0, -20.0, 0.0, 20.0, 35.0)
        elif heading_gap_regime == "medium":
            legacy_yaw_offsets_deg = (-30.0, -15.0, 0.0, 15.0, 30.0)
        else:
            legacy_yaw_offsets_deg = (-20.0, -10.0, 0.0, 10.0, 20.0)
        legacy_x_offsets = (0.0, -0.25, 0.25, -0.55, 0.55, -0.85, 0.85)
        legacy_y_offsets = (0.0, -0.16, 0.16, -0.30, 0.30)
        for x_offset in legacy_x_offsets:
            anchor_x = float(clamp(float(anchor_hint_xy[0]) + float(x_offset), pocket_x_min + 0.28, pocket_x_max - 0.28))
            for y_offset in legacy_y_offsets:
                anchor_y = float(clamp(float(anchor_hint_xy[1]) + float(y_offset), bounds_y_min + 0.28, bounds_y_max - 0.28))
                for yaw_offset_deg in legacy_yaw_offsets_deg:
                    anchor_yaw = float(wrap_angle(float(anchor_hint_yaw) + math.radians(float(yaw_offset_deg))))
                    key = (int(round(anchor_x / 0.05)), int(round(anchor_y / 0.05)), int(round(math.degrees(anchor_yaw) / 5.0)))
                    if key in seen:
                        continue
                    seen.add(key)
                    anchor = VehicleState(
                        vehicle_id=int(leader.vehicle_id),
                        x=float(anchor_x),
                        y=float(anchor_y),
                        yaw=float(anchor_yaw),
                        v=0.0,
                        delta=0.0,
                        mode=leader.mode,
                    )
                    certificate = self._certificate_from_anchor(
                        anchor=anchor,
                        anchor_hint_xy=anchor_hint_xy,
                        lane_heading=lane_heading,
                        follower=follower,
                        obstacles=obstacles,
                        planner=planner,
                        corridor_polygons=corridor_polygons,
                    )
                    if certificate is not None:
                        certificates.append(certificate)
                        if len(certificates) >= 32:
                            stop_generation = True
                            break
                if stop_generation:
                    break
            if stop_generation:
                break
        certificates.sort(
            key=lambda cert: (
                -float(cert.sigma_geom),
                float(np.linalg.norm(cert.anchor.xy() - anchor_hint_xy)),
                min(
                    abs(float(angle_diff(float(cert.anchor.yaw), float(dominant.heading)))),
                    abs(float(angle_diff(float(cert.anchor.yaw), float(anchor_hint_yaw)))),
                ),
            )
        )
        return certificates[:16]

    def _approx_execution_certificate(
        self,
        *,
        anchor: VehicleState,
        follower: VehicleState,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
        lane_heading: float,
        allow_grid_fallback: bool = True,
    ) -> tuple[float, dict[str, Any]]:
        clearance_req = float(max(self.cfg.safety.min_clearance + 0.02, 0.12))
        if not self._pose_in_corridor(anchor, corridor_polygons):
            return 0.0, {"valid": False, "reason": "anchor_outside_corridor"}
        if self.collision.in_collision(anchor, obstacles, []):
            return 0.0, {"valid": False, "reason": "anchor_collision"}
        predock = self._predock_pose(anchor, follower_id=int(follower.vehicle_id), follower_mode=follower.mode)
        if not self._pose_in_corridor(predock, corridor_polygons):
            return 0.0, {"valid": False, "reason": "predock_outside_corridor"}
        if self.collision.in_collision(predock, obstacles, [anchor]):
            return 0.0, {"valid": False, "reason": "predock_collision"}
        direct_path_xy = self._direct_path_xy(start_xy=follower.xy(), goal_xy=predock.xy(), step_m=0.10)
        follower_path_clearance_m = self._path_min_clearance(
            direct_path_xy,
            proto=follower,
            goal_yaw=float(predock.yaw),
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
            other=anchor,
        )
        if follower_path_clearance_m < clearance_req:
            if not bool(allow_grid_fallback):
                return 0.0, {
                    "valid": False,
                    "reason": "follower_path_clearance",
                    "predock_xy": [float(predock.x), float(predock.y)],
                    "follower_path_clearance_m": float(follower_path_clearance_m),
                }
            return 0.0, {
                "valid": False,
                "reason": "follower_path_clearance",
                "predock_xy": [float(predock.x), float(predock.y)],
                "follower_path_clearance_m": float(follower_path_clearance_m),
            }
        anchor_clearance_m = float(self.collision.min_clearance_vehicle_obstacles(anchor, obstacles))
        handoff_clearance_m = float(
            min(
                self.collision.min_clearance_vehicle_obstacles(predock, obstacles),
                self.collision.min_clearance_vehicle_vehicle(predock, anchor),
            )
        )
        dock_zone_clearance_m = self._dock_zone_clearance(anchor, obstacles)
        heading_to_predock = float(math.atan2(float(predock.y - follower.y), float(predock.x - follower.x)))
        ingress_alignment = float(clamp(1.0 - abs(float(angle_diff(float(predock.yaw), float(heading_to_predock)))) / math.radians(36.0), 0.0, 1.0))
        lane_alignment = float(
            max(
                clamp(1.0 - abs(float(angle_diff(float(anchor.yaw), float(lane_heading)))) / math.radians(42.0), 0.0, 1.0),
                clamp(1.0 - abs(float(angle_diff(float(anchor.yaw), float(wrap_angle(float(lane_heading) + math.pi))))) / math.radians(42.0), 0.0, 1.0),
            )
        )
        sigma_exec = float(
            clamp(
                0.30 * float(clamp((follower_path_clearance_m - clearance_req) / 0.45, 0.0, 1.0))
                + 0.20 * float(clamp((handoff_clearance_m - clearance_req) / 0.35, 0.0, 1.0))
                + 0.18 * float(clamp((dock_zone_clearance_m - clearance_req) / 0.35, 0.0, 1.0))
                + 0.14 * float(clamp((anchor_clearance_m - clearance_req) / 0.40, 0.0, 1.0))
                + 0.10 * float(ingress_alignment)
                + 0.08 * float(lane_alignment),
                0.0,
                1.0,
            )
        )
        return sigma_exec, {
            "valid": True,
            "predock_xy": [float(predock.x), float(predock.y)],
            "predock_yaw": float(predock.yaw),
            "follower_path_xy": np.asarray(direct_path_xy, dtype=float).tolist(),
            "follower_path_clearance_m": float(follower_path_clearance_m),
            "anchor_clearance_m": float(anchor_clearance_m),
            "handoff_clearance_m": float(handoff_clearance_m),
            "dock_zone_clearance_m": float(dock_zone_clearance_m),
            "ingress_alignment": float(ingress_alignment),
            "lane_alignment": float(lane_alignment),
        }

    def _terminal_contact_center(self, *, anchor: VehicleState) -> np.ndarray:
        heading_xy = np.array([math.cos(float(anchor.yaw)), math.sin(float(anchor.yaw))], dtype=float)
        rear_hitch_xy = self.geom.rear_hitch(anchor)
        return rear_hitch_xy - heading_xy * float(self.geom.front_hitch_x + 0.15)

    def _approx_terminal_safety_certificate(
        self,
        *,
        anchor: VehicleState,
        follower: VehicleState,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
    ) -> tuple[float, dict[str, Any]]:
        clearance_req = float(max(self.cfg.safety.min_clearance + 0.02, 0.12))
        predock = self._predock_pose(anchor, follower_id=int(follower.vehicle_id), follower_mode=follower.mode)
        contact_center_xy = self._terminal_contact_center(anchor=anchor)
        terminal_path_xy = self._direct_path_xy(start_xy=predock.xy(), goal_xy=contact_center_xy, step_m=0.08)
        terminal_path_clearance_m = self._path_min_clearance(
            terminal_path_xy,
            proto=follower,
            goal_yaw=float(anchor.yaw),
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
            other=None,
        )
        if terminal_path_clearance_m < clearance_req:
            return 0.0, {
                "valid": False,
                "reason": "terminal_path_clearance",
                "terminal_path_clearance_m": float(terminal_path_clearance_m),
                "predock_xy": [float(predock.x), float(predock.y)],
            }
        dock_zone_clearance_m = float(self._dock_zone_clearance(anchor, obstacles))
        if dock_zone_clearance_m < clearance_req:
            return 0.0, {
                "valid": False,
                "reason": "dock_zone_clearance",
                "dock_zone_clearance_m": float(dock_zone_clearance_m),
                "predock_xy": [float(predock.x), float(predock.y)],
            }
        terminal_safety = float(
            clamp(
                0.58 * float(clamp((terminal_path_clearance_m - clearance_req) / 0.32, 0.0, 1.0))
                + 0.42 * float(clamp((dock_zone_clearance_m - clearance_req) / 0.32, 0.0, 1.0)),
                0.0,
                1.0,
            )
        )
        return terminal_safety, {
            "valid": True,
            "terminal_path_xy": np.asarray(terminal_path_xy, dtype=float).tolist(),
            "terminal_path_clearance_m": float(terminal_path_clearance_m),
            "dock_zone_clearance_m": float(dock_zone_clearance_m),
            "predock_xy": [float(predock.x), float(predock.y)],
        }

    def _robust_execution_certificate(
        self,
        *,
        anchor: VehicleState,
        follower: VehicleState,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
        lane_heading: float,
        allow_grid_fallback: bool = True,
    ) -> tuple[float, float, dict[str, Any]]:
        nominal_sigma, nominal_meta = self._approx_execution_certificate(
            anchor=anchor,
            follower=follower,
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
            lane_heading=lane_heading,
            allow_grid_fallback=bool(allow_grid_fallback),
        )
        if nominal_sigma <= 0.0:
            return 0.0, 0.0, dict(nominal_meta)
        robust_sigma = float(nominal_sigma)
        for dx_body_m, dy_body_m, dyaw_deg in ((0.04, 0.0, 0.0), (0.0, -0.04, 0.0), (0.0, 0.0, -3.0)):
            perturbed_anchor = self._perturb_leader_pose(anchor=anchor, dx_body_m=float(dx_body_m), dy_body_m=float(dy_body_m), dyaw_deg=float(dyaw_deg))
            sigma_value, _ = self._approx_execution_certificate(
                anchor=perturbed_anchor,
                follower=follower,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
                lane_heading=lane_heading,
                allow_grid_fallback=bool(allow_grid_fallback),
            )
            robust_sigma = float(min(robust_sigma, sigma_value))
            if robust_sigma <= 0.0:
                break
        return float(nominal_sigma), float(robust_sigma), dict(nominal_meta)

    def _robust_terminal_safety_certificate(
        self,
        *,
        anchor: VehicleState,
        follower: VehicleState,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
    ) -> tuple[float, float, dict[str, Any]]:
        nominal_sigma, nominal_meta = self._approx_terminal_safety_certificate(
            anchor=anchor,
            follower=follower,
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
        )
        if nominal_sigma <= 0.0:
            return 0.0, 0.0, dict(nominal_meta)
        robust_sigma = float(nominal_sigma)
        for dx_body_m, dy_body_m, dyaw_deg in ((0.04, 0.0, 0.0), (0.0, -0.04, 0.0), (0.0, 0.0, -3.0)):
            perturbed_anchor = self._perturb_leader_pose(anchor=anchor, dx_body_m=float(dx_body_m), dy_body_m=float(dy_body_m), dyaw_deg=float(dyaw_deg))
            sigma_value, _ = self._approx_terminal_safety_certificate(
                anchor=perturbed_anchor,
                follower=follower,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
            )
            robust_sigma = float(min(robust_sigma, sigma_value))
            if robust_sigma <= 0.0:
                break
        return float(nominal_sigma), float(robust_sigma), dict(nominal_meta)

    def _perturb_leader_pose(self, *, anchor: VehicleState, dx_body_m: float, dy_body_m: float, dyaw_deg: float) -> VehicleState:
        world_xy = self.geom.to_world(anchor, float(dx_body_m), float(dy_body_m))
        offset_xy = np.asarray(world_xy, dtype=float) - anchor.xy()
        return VehicleState(
            vehicle_id=int(anchor.vehicle_id),
            x=float(anchor.x + offset_xy[0]),
            y=float(anchor.y + offset_xy[1]),
            yaw=float(wrap_angle(float(anchor.yaw) + math.radians(float(dyaw_deg)))),
            v=0.0,
            delta=0.0,
            mode=anchor.mode,
        )

    def _shape_candidate_primitives(self) -> tuple[MotionPrimitive, ...]:
        primitive_names = {
            "reverse_straight_short",
            "reverse_straight_long",
            "reverse_arc_left",
            "reverse_arc_right",
            "reverse_arc_left_tight",
            "reverse_arc_right_tight",
            "reverse_arc_left_long",
            "reverse_arc_right_long",
            "forward_straight_short",
            "forward_arc_left",
            "forward_arc_right",
            "forward_arc_left_tight",
            "forward_arc_right_tight",
            "reverse_turn_forward_left",
            "reverse_turn_forward_right",
        }
        return tuple(primitive for primitive in self.motion_library.primitives if primitive.name in primitive_names)

    def _escape_shape_primitives(self) -> tuple[MotionPrimitive, ...]:
        escape_names = {
            "reverse_straight_long",
            "reverse_arc_left_long",
            "reverse_arc_right_long",
            "reverse_arc_left_tight",
            "reverse_arc_right_tight",
            "reverse_turn_forward_left",
            "reverse_turn_forward_right",
            "forward_straight_long",
            "forward_arc_left_tight",
            "forward_arc_right_tight",
        }
        return tuple(primitive for primitive in self.motion_library.primitives if primitive.name in escape_names)

    def _small_gap_escape_templates(self) -> tuple[tuple[str, ...], ...]:
        return (
            (
                "reverse_arc_left_tight",
                "reverse_arc_left_tight",
                "reverse_arc_left",
                "reverse_arc_left",
                "reverse_arc_right_tight",
                "reverse_arc_right_tight",
                "reverse_straight_short",
                "forward_straight_short",
            ),
            (
                "reverse_arc_left_tight",
                "reverse_arc_left_tight",
                "reverse_arc_left_long",
                "reverse_arc_left",
                "reverse_arc_right_tight",
                "reverse_straight_short",
            ),
            (
                "reverse_arc_right_tight",
                "reverse_arc_right_tight",
                "reverse_arc_right",
                "reverse_arc_right",
                "reverse_arc_left_tight",
                "reverse_arc_left_tight",
                "reverse_straight_short",
                "forward_straight_short",
            ),
        )

    def _post_feasible_candidate(
        self,
        *,
        exec_robust_sigma: float,
        terminal_nominal_sigma: float,
        combined_robust_sigma: float,
    ) -> bool:
        return bool(
            float(exec_robust_sigma) >= 0.20
            and float(terminal_nominal_sigma) >= 0.25
            and float(combined_robust_sigma) >= 0.18
        )

    def _staging_candidate_score(
        self,
        *,
        leader_pose: VehicleState,
        follower_pose: VehicleState,
        corridor_info: dict[str, Any],
        sigma_progress: float,
        sigma_terminal_nominal: float,
        sigma_terminal_robust: float,
        sigma_corridor_nominal: float,
        sigma_corridor_robust: float,
        sigma_visibility: float = 1.0,
    ) -> tuple[float, float]:
        sigma_post = float(self.cert_field.sigma_post(leader_pose=leader_pose, corridor_info=corridor_info))
        unified_score = float(
            self.cert_field.unified_staging_score(
                leader_pose=leader_pose,
                follower_pose=follower_pose,
                corridor_info=corridor_info,
                sigma_progress=float(sigma_progress),
                sigma_terminal_nominal=float(sigma_terminal_nominal),
                sigma_terminal_robust=float(sigma_terminal_robust),
                sigma_corridor_nominal=float(sigma_corridor_nominal),
                sigma_corridor_robust=float(sigma_corridor_robust),
                sigma_visibility=float(sigma_visibility),
            )
        )
        return unified_score, sigma_post

    def _parallel_yaw_error(self, *, yaw: float, lane_heading: float) -> float:
        return float(
            min(
                abs(float(angle_diff(float(yaw), float(lane_heading)))),
                abs(float(angle_diff(float(yaw), float(wrap_angle(float(lane_heading) + math.pi))))),
            )
        )

    def _critical_clearance_threshold(self) -> float:
        return float(max(float(self.cfg.safety.min_clearance) + 0.03, 0.13))

    def _front_mobility_clearance(
        self,
        *,
        anchor: VehicleState,
        lane_heading: float,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
        horizon_m: float = 5.0,
        step_m: float = 0.25,
    ) -> float:
        heading_candidates = (
            float(lane_heading),
            float(wrap_angle(float(lane_heading) + math.pi)),
        )
        travel_heading = min(
            heading_candidates,
            key=lambda heading: self._parallel_yaw_error(yaw=float(anchor.yaw), lane_heading=float(heading)),
        )
        reachable_distance_m = 0.0
        for distance_m in np.arange(float(step_m), float(horizon_m) + 1e-9, float(step_m)):
            probe_xy = anchor.xy() + np.array([math.cos(float(travel_heading)), math.sin(float(travel_heading))], dtype=float) * float(distance_m)
            probe = anchor.copy()
            probe.x = float(probe_xy[0])
            probe.y = float(probe_xy[1])
            probe.yaw = float(travel_heading)
            if not self._in_bounds(probe, margin=0.40):
                break
            if corridor_polygons and not self._pose_in_corridor(probe, corridor_polygons):
                break
            if self.collision.in_collision(probe, obstacles, []):
                break
            reachable_distance_m = float(distance_m)
        return float(reachable_distance_m)

    def _soft_post_path_cost(
        self,
        *,
        path_xy: np.ndarray,
        min_clearance_m: float,
        goal_yaw: float,
        lane_heading: float,
        anchor_pose: VehicleState | None = None,
        anchor_hint_xy: np.ndarray | None = None,
        obstacles: list[Obstacle] | None = None,
        corridor_polygons: list[np.ndarray] | None = None,
        w_length: float = 0.05,
        w_clearance: float = 0.05,
        w_post: float = 0.85,
        w_front: float = 0.28,
        w_long: float = 0.14,
    ) -> tuple[float, float, float, float]:
        path = np.asarray(path_xy, dtype=float)
        if len(path) < 2:
            path_length_m = 0.0
        else:
            path_length_m = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
        d_min = float(max(min_clearance_m, 0.05))
        yaw_error = float(self._parallel_yaw_error(yaw=float(goal_yaw), lane_heading=float(lane_heading)))
        front_clearance_m = 5.0
        if anchor_pose is not None:
            front_clearance_m = self._front_mobility_clearance(
                anchor=anchor_pose,
                lane_heading=float(lane_heading),
                obstacles=list(obstacles or []),
                corridor_polygons=list(corridor_polygons or []),
            )
            front_clearance_m = float(max(front_clearance_m, 0.10))
        longitudinal_offset_m = 0.0
        if anchor_pose is not None and anchor_hint_xy is not None:
            lane_tangent = np.array([math.cos(float(lane_heading)), math.sin(float(lane_heading))], dtype=float)
            longitudinal_offset_m = abs(float(np.dot(anchor_pose.xy() - np.asarray(anchor_hint_xy, dtype=float), lane_tangent)))
        cost = float(
            float(w_length) * float(path_length_m)
            + float(w_clearance) * (1.0 / float(d_min))
            + float(w_post) * float(yaw_error)
            + float(w_front) * (1.0 / float(front_clearance_m))
            + float(w_long) * float(longitudinal_offset_m)
        )
        return float(cost), float(front_clearance_m), float(longitudinal_offset_m), float(yaw_error)

    def _prefer_cost_modulated_candidate(
        self,
        *,
        best_primary_score: float | None,
        best_soft_cost: float | None,
        candidate_primary_score: float,
        candidate_soft_cost: float,
        primary_margin: float = 0.08,
    ) -> bool:
        if best_primary_score is None or best_soft_cost is None:
            return True
        if float(candidate_primary_score) > float(best_primary_score) + float(primary_margin):
            return True
        if float(candidate_primary_score) < float(best_primary_score) - float(primary_margin):
            return False
        if float(candidate_soft_cost) < float(best_soft_cost) - 1e-9:
            return True
        if abs(float(candidate_soft_cost) - float(best_soft_cost)) <= 1e-9 and float(candidate_primary_score) > float(best_primary_score) + 1e-9:
            return True
        return False

    def _fast_small_gap_escape_plan(
        self,
        *,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
        lane: dict[str, Any],
        planner: GridAStarPlanner,
        corridor_polygons: list[np.ndarray],
        time_budget_s: float = 0.10,
    ) -> CorridorReciprocalPlan | None:
        if str(lane.get("heading_gap_regime", "")).strip().lower() != "small":
            return None
        if float(lane.get("leader_reverse_turn_required", 0.0)) > 0.12:
            return None
        start_time_s = time.perf_counter()
        lane_heading = self._lane_heading(lane, default_yaw=float(lane.get("leader_stage_anchor_yaw", leader.yaw)))
        anchor_hint_xy = np.asarray(lane.get("leader_stage_anchor_xy", [leader.x, leader.y]), dtype=float)
        primitive_map = {primitive.name: primitive for primitive in self.motion_library.primitives}
        best_plan: CorridorReciprocalPlan | None = None
        best_score = -1e9
        terminal_side = 1.0 if float(self._lane_y(lane) - leader.y) >= 0.0 else -1.0
        templates = self._small_gap_escape_templates()
        if terminal_side < 0.0:
            templates = tuple(tuple(name.replace("left", "tmp").replace("right", "left").replace("tmp", "right") for name in template) for template in templates)
        for template in templates[:1]:
            if time.perf_counter() - start_time_s >= float(time_budget_s):
                break
            state = leader.copy()
            state_blocks: list[np.ndarray] = []
            cmd_blocks: list[np.ndarray] = []
            min_clearance_m = 1e6
            feasible = True
            for primitive_name in template:
                primitive = primitive_map.get(str(primitive_name))
                if primitive is None:
                    feasible = False
                    break
                rollout = self.motion_library.rollout(start=state, primitive=primitive)
                rollout_clearance = 1e6
                for sample_idx in (len(rollout.states) - 1,):
                    sample_row = rollout.states[sample_idx]
                    sample_state = VehicleState(
                        vehicle_id=int(leader.vehicle_id),
                        x=float(sample_row[0]),
                        y=float(sample_row[1]),
                        yaw=float(sample_row[2]),
                        v=0.0,
                        delta=0.0,
                        mode=leader.mode,
                    )
                    if not self._in_bounds(sample_state, margin=0.40):
                        feasible = False
                        break
                    if corridor_polygons and not self._pose_in_corridor(sample_state, corridor_polygons):
                        feasible = False
                        break
                    if self.collision.in_collision(sample_state, obstacles, [follower]):
                        feasible = False
                        break
                    sample_clearance = float(
                        min(
                            self.collision.min_clearance_vehicle_obstacles(sample_state, obstacles),
                            self.collision.min_clearance_vehicle_vehicle(sample_state, follower),
                        )
                    )
                    rollout_clearance = float(min(rollout_clearance, sample_clearance))
                if not feasible:
                    feasible = False
                    break
                state = rollout.end_state.copy()
                state_blocks.append(np.asarray(rollout.states, dtype=float))
                cmd_blocks.append(np.asarray(rollout.commands, dtype=float))
                min_clearance_m = float(min(min_clearance_m, rollout_clearance))
            if not feasible:
                continue
            if float(min_clearance_m) < float(self._critical_clearance_threshold()):
                continue
            predock = self._predock_pose(state, follower_id=int(follower.vehicle_id), follower_mode=follower.mode)
            follower_path_xy = self._direct_path_xy(start_xy=follower.xy(), goal_xy=predock.xy(), step_m=0.10)
            score = float(min_clearance_m - 0.01 * len(template))
            if score <= best_score:
                continue
            best_score = float(score)
            leader_path_xy = np.vstack(
                [
                    np.asarray([[float(leader.x), float(leader.y)]], dtype=float),
                    np.vstack(state_blocks)[:, :2],
                ]
            )
            best_plan = CorridorReciprocalPlan(
                leader_goal=state.copy(),
                follower_goal=predock.copy(),
                leader_path_xy=np.asarray(leader_path_xy, dtype=float),
                follower_path_xy=np.asarray(follower_path_xy, dtype=float),
                leader_primitive_states=np.vstack(state_blocks).astype(float),
                leader_primitive_cmd=np.vstack(cmd_blocks).astype(float),
                leader_primitive_steps=int(sum(len(block) for block in cmd_blocks)),
                score=float(1.6 - 0.6 * min_clearance_m),
                reason="lc_corridor_smallgap_escape",
                metadata={
                    "lane_y": float(self._lane_y(lane)),
                    **self._macro_metadata(lane=lane, leader_ref=leader),
                    "anchor_hint_xy": [float(anchor_hint_xy[0]), float(anchor_hint_xy[1])],
                    "anchor_hint_yaw": float(lane.get("leader_stage_anchor_yaw", 0.0)),
                    "lane_heading": float(lane_heading),
                    "corridor_polygons": [np.asarray(poly, dtype=float).tolist() for poly in corridor_polygons],
                    "sigma_c": 0.0,
                    "sigma_geom": 0.0,
                    "sigma_reach": 0.0,
                    "exec_nominal_sigma": 0.0,
                    "exec_robust_sigma": 0.0,
                    "terminal_nominal_sigma": 0.0,
                    "terminal_robust_sigma": 0.0,
                    "combined_nominal_sigma": 0.0,
                    "combined_robust_sigma": 0.0,
                    "release_sigma_threshold": 0.40,
                    "leader_path_clearance_m": float(min_clearance_m),
                    "follower_path_clearance_m": float(min_clearance_m),
                    "handoff_clearance_m": float(min_clearance_m),
                    "dock_zone_clearance_m": float(min_clearance_m),
                    "terminal_path_clearance_m": float(min_clearance_m),
                    "alignment_quality": 0.0,
                    "candidate_rank": 0,
                    "search_expansions": int(sum(len(block) for block in cmd_blocks)),
                    "shape_target_speed": 0.14,
                    "settle_target_speed": 0.12,
                    "follower_target_speed": 0.24,
                    "exec_release_threshold": 0.30,
                    "exec_robust_threshold": 0.20,
                    "terminal_release_threshold": 0.16,
                    "shape_budget_s": 6.0,
                    "settle_budget_s": 3.0,
                    "planner_time_budget_s": float(time_budget_s),
                    "planner_elapsed_s": float(time.perf_counter() - start_time_s),
                    "template": list(template),
                },
            )
        return best_plan

    def _evaluate_macro_template_plan(
        self,
        *,
        macro_template: MacroPrimitiveTemplate,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
        lane: dict[str, Any],
        planner: GridAStarPlanner,
        corridor_polygons: list[np.ndarray],
        lane_heading: float,
        anchor_hint_xy: np.ndarray,
    ) -> MacroTemplateEvaluation | None:
        primitive_map = {primitive.name: primitive for primitive in self.motion_library.primitives}
        state = leader.copy()
        state_blocks: list[np.ndarray] = []
        cmd_blocks: list[np.ndarray] = []
        min_clearance_m = 1e6
        for primitive_name in macro_template.primitive_names:
            primitive = primitive_map.get(primitive_name)
            if primitive is None:
                return None
            rollout = self.motion_library.rollout(start=state, primitive=primitive)
            safe, rollout_clearance, _ = self.motion_library.is_safe(
                rollout=rollout,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
                other=follower,
                min_clearance=float(self.cfg.safety.min_clearance),
                in_bounds=self._in_bounds,
            )
            if not safe:
                return None
            min_clearance_m = float(min(min_clearance_m, rollout_clearance))
            state = rollout.end_state.copy()
            state_blocks.append(np.asarray(rollout.states, dtype=float))
            cmd_blocks.append(np.asarray(rollout.commands, dtype=float))
        if not state_blocks:
            return None
        if float(min_clearance_m) < float(self._critical_clearance_threshold()):
            return None
        exec_nominal, exec_robust, _ = self._robust_execution_certificate(
            anchor=state,
            follower=follower,
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
            lane_heading=lane_heading,
            allow_grid_fallback=False,
        )
        terminal_nominal, terminal_robust, terminal_meta = self._robust_terminal_safety_certificate(
            anchor=state,
            follower=follower,
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
        )
        combined_nominal = float(math.sqrt(max(exec_nominal, 0.0) * max(terminal_nominal, 0.0)))
        combined_robust = float(math.sqrt(max(exec_robust, 0.0) * max(terminal_robust, 0.0)))
        if combined_nominal <= 0.0:
            return None
        full_certificate = self._certificate_from_anchor(
            anchor=state,
            anchor_hint_xy=anchor_hint_xy,
            lane_heading=lane_heading,
            follower=follower,
            obstacles=obstacles,
            planner=planner,
            corridor_polygons=corridor_polygons,
        )
        if full_certificate is None:
            return None
        score_base, sigma_post = self._staging_candidate_score(
            leader_pose=state,
            follower_pose=follower,
            corridor_info={"lane_heading": float(lane_heading)},
            sigma_progress=float(full_certificate.sigma_geom),
            sigma_terminal_nominal=float(terminal_nominal),
            sigma_terminal_robust=float(terminal_robust),
            sigma_corridor_nominal=float(exec_nominal),
            sigma_corridor_robust=float(exec_robust),
            sigma_visibility=1.0,
        )
        plan = CorridorReciprocalPlan(
            leader_goal=state.copy(),
            follower_goal=full_certificate.predock.copy(),
            leader_path_xy=np.asarray(
                np.vstack(
                    [
                        np.asarray([[float(leader.x), float(leader.y)]], dtype=float),
                        np.vstack(state_blocks)[:, :2],
                    ]
                ),
                dtype=float,
            ),
            follower_path_xy=np.asarray(full_certificate.follower_path_xy, dtype=float),
            leader_primitive_states=np.vstack(state_blocks).astype(float),
            leader_primitive_cmd=np.vstack(cmd_blocks).astype(float),
            leader_primitive_steps=int(sum(len(block) for block in cmd_blocks)),
            score=float(
                1.9
                - 0.45 * float(score_base)
                - 0.20 * float(combined_robust)
                - 0.05 * float(exec_robust)
            ),
            reason="lc_corridor_macro_escape",
            metadata={
                "lane_y": float(self._lane_y(lane)),
                **self._macro_metadata(lane=lane, leader_ref=leader),
                "anchor_hint_xy": [float(anchor_hint_xy[0]), float(anchor_hint_xy[1])],
                "anchor_hint_yaw": float(lane.get("leader_stage_anchor_yaw", 0.0)),
                "lane_heading": float(lane_heading),
                "corridor_polygons": [np.asarray(poly, dtype=float).tolist() for poly in corridor_polygons],
                "sigma_c": float(exec_robust),
                "sigma_geom": float(full_certificate.sigma_geom),
                "sigma_reach": float(exec_nominal),
                "sigma_post": float(sigma_post),
                "exec_nominal_sigma": float(exec_nominal),
                "exec_robust_sigma": float(exec_robust),
                "terminal_nominal_sigma": float(terminal_nominal),
                "terminal_robust_sigma": float(terminal_robust),
                "combined_nominal_sigma": float(combined_nominal),
                "combined_robust_sigma": float(combined_robust),
                "release_sigma_threshold": 0.40,
                "leader_path_clearance_m": float(min_clearance_m),
                "follower_path_clearance_m": float(full_certificate.follower_path_clearance_m),
                "handoff_clearance_m": float(full_certificate.handoff_clearance_m),
                "dock_zone_clearance_m": float(full_certificate.dock_zone_clearance_m),
                "terminal_path_clearance_m": float(terminal_meta.get("terminal_path_clearance_m", 0.0)),
                "alignment_quality": float(full_certificate.alignment_quality),
                "candidate_rank": 0,
                "search_expansions": int(sum(len(block) for block in cmd_blocks)),
                "shape_target_speed": 0.14,
                "settle_target_speed": 0.12,
                "follower_target_speed": 0.24,
                "exec_release_threshold": 0.30,
                "exec_robust_threshold": 0.20,
                "terminal_release_threshold": 0.16,
                "macro_template": list(macro_template.primitive_names),
                "macro_template_name": str(macro_template.name),
                "legacy_priority": float(macro_template.metadata.get("legacy_priority", 0.0)),
            },
        )
        soft_path_cost, front_clearance_m, longitudinal_offset_m, yaw_error_rad = self._soft_post_path_cost(
            path_xy=plan.leader_path_xy,
            min_clearance_m=float(min_clearance_m),
            goal_yaw=float(state.yaw),
            lane_heading=float(lane_heading),
            anchor_pose=state,
            anchor_hint_xy=anchor_hint_xy,
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
        )
        plan.metadata["soft_path_cost"] = float(soft_path_cost)
        plan.metadata["path_yaw_error_deg"] = float(math.degrees(yaw_error_rad))
        plan.metadata["front_clearance_m"] = float(front_clearance_m)
        plan.metadata["longitudinal_offset_m"] = float(longitudinal_offset_m)
        return MacroTemplateEvaluation(
            plan=plan,
            sigma_post=float(sigma_post),
            soft_path_cost=float(soft_path_cost),
            front_clearance_m=float(front_clearance_m),
            longitudinal_offset_m=float(longitudinal_offset_m),
            exec_nominal_sigma=float(exec_nominal),
            exec_robust_sigma=float(exec_robust),
            terminal_nominal_sigma=float(terminal_nominal),
            terminal_robust_sigma=float(terminal_robust),
            combined_nominal_sigma=float(combined_nominal),
            combined_robust_sigma=float(combined_robust),
            min_clearance_m=float(min_clearance_m),
            legacy_priority=float(macro_template.metadata.get("legacy_priority", 0.0)),
        )

    def _escape_anchor_hypotheses(
        self,
        *,
        leader: VehicleState,
        lane: dict[str, Any],
        corridor_polygons: list[np.ndarray],
    ) -> list[VehicleState]:
        anchor_hint_xy = np.asarray(lane.get("leader_stage_anchor_xy", [leader.x, leader.y]), dtype=float)
        anchor_hint_yaw = float(lane.get("leader_stage_anchor_yaw", 0.0))
        lane_y = float(self._lane_y(lane))
        bounds_x_min, bounds_x_max, bounds_y_min, bounds_y_max = self._corridor_bounds(corridor_polygons)
        x_offsets = (0.0, -0.25, 0.25, -0.55, 0.55, -0.85, 0.85)
        y_candidates = (
            float(anchor_hint_xy[1]),
            float(lane_y),
            float(clamp(float(anchor_hint_xy[1]) + 0.16, bounds_y_min + 0.28, bounds_y_max - 0.28)),
            float(clamp(float(anchor_hint_xy[1]) - 0.16, bounds_y_min + 0.28, bounds_y_max - 0.28)),
        )
        yaw_offsets_deg = (-20.0, -10.0, 0.0, 10.0, 20.0)
        out: list[VehicleState] = []
        seen: set[tuple[int, int, int]] = set()
        for x_offset in x_offsets:
            anchor_x = float(clamp(float(anchor_hint_xy[0]) + float(x_offset), bounds_x_min + 0.28, bounds_x_max - 0.28))
            for anchor_y in y_candidates:
                for yaw_offset_deg in yaw_offsets_deg:
                    anchor_yaw = float(wrap_angle(float(anchor_hint_yaw) + math.radians(float(yaw_offset_deg))))
                    key = (int(round(anchor_x / 0.05)), int(round(anchor_y / 0.05)), int(round(math.degrees(anchor_yaw) / 5.0)))
                    if key in seen:
                        continue
                    seen.add(key)
                    out.append(
                        VehicleState(
                            vehicle_id=int(leader.vehicle_id),
                            x=float(anchor_x),
                            y=float(anchor_y),
                            yaw=float(anchor_yaw),
                            v=0.0,
                            delta=0.0,
                            mode=leader.mode,
                        )
                    )
        return out

    def _escape_anchor_plan(
        self,
        *,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
        lane: dict[str, Any],
        planner: GridAStarPlanner,
        corridor_polygons: list[np.ndarray],
        time_budget_s: float = 0.10,
    ) -> CorridorReciprocalPlan | None:
        start_time_s = time.perf_counter()
        lane_heading = self._lane_heading(lane, default_yaw=float(lane.get("leader_stage_anchor_yaw", leader.yaw)))
        anchor_hint_xy = np.asarray(lane.get("leader_stage_anchor_xy", [leader.x, leader.y]), dtype=float)
        lane_y = float(self._lane_y(lane))
        terminal_side = 1.0 if float(lane_y - leader.y) >= 0.0 else -1.0
        beam: list[tuple[VehicleState, list[np.ndarray], list[np.ndarray], float, int]] = [(leader.copy(), [], [], 1e6, 0)]
        endpoint_pool: list[tuple[float, VehicleState, list[np.ndarray], list[np.ndarray], float]] = []
        seen: dict[tuple[int, int, int], float] = {}
        for depth in range(1, 5):
            next_beam: list[tuple[VehicleState, list[np.ndarray], list[np.ndarray], float, int, float]] = []
            for state, state_blocks, cmd_blocks, min_clearance_m, gear_switches in beam:
                if time.perf_counter() - start_time_s >= float(time_budget_s):
                    break
                for primitive in self._escape_shape_primitives():
                    rollout = self.motion_library.rollout(start=state, primitive=primitive)
                    safe, rollout_clearance, _ = self.motion_library.is_safe(
                        rollout=rollout,
                        obstacles=obstacles,
                        corridor_polygons=corridor_polygons,
                        other=follower,
                        min_clearance=float(self.cfg.safety.min_clearance),
                        in_bounds=self._in_bounds,
                    )
                    if not safe:
                        continue
                    end_state = rollout.end_state.copy()
                    next_min_clearance_m = float(min(min_clearance_m, rollout_clearance))
                    state_key = (
                        int(round(float(end_state.x) / 0.12)),
                        int(round(float(end_state.y) / 0.12)),
                        int(round(math.degrees(float(end_state.yaw)) / 6.0)),
                    )
                    local_progress = float(terminal_side * (float(end_state.y) - float(leader.y)))
                    lane_proximity = abs(float(end_state.y) - float(lane_y))
                    heading_gap = abs(float(angle_diff(float(end_state.yaw), float(lane_heading))))
                    heuristic_score = float(
                        -0.50 * np.linalg.norm(end_state.xy() - anchor_hint_xy)
                        -0.70 * lane_proximity
                        -0.25 * heading_gap
                        +0.85 * local_progress
                        +0.12 * next_min_clearance_m
                        -0.03 * depth
                        -0.02 * (gear_switches + primitive.gear_switches)
                    )
                    if state_key in seen and seen[state_key] >= heuristic_score:
                        continue
                    seen[state_key] = float(heuristic_score)
                    next_state_blocks = [*state_blocks, np.asarray(rollout.states, dtype=float)]
                    next_cmd_blocks = [*cmd_blocks, np.asarray(rollout.commands, dtype=float)]
                    next_gear_switches = int(gear_switches + primitive.gear_switches)
                    next_beam.append((end_state, next_state_blocks, next_cmd_blocks, next_min_clearance_m, next_gear_switches, float(heuristic_score)))
                    endpoint_pool.append((float(heuristic_score), end_state.copy(), next_state_blocks, next_cmd_blocks, float(next_min_clearance_m)))
            next_beam.sort(key=lambda item: float(item[-1]), reverse=True)
            beam = [(state, state_blocks, cmd_blocks, min_clearance_m, gear_switches) for state, state_blocks, cmd_blocks, min_clearance_m, gear_switches, _score in next_beam[:8]]
            if time.perf_counter() - start_time_s >= float(time_budget_s):
                break
        if not endpoint_pool:
            return None
        endpoint_pool.sort(key=lambda item: float(item[0]), reverse=True)
        best_plan: CorridorReciprocalPlan | None = None
        best_primary_score: float | None = None
        best_soft_cost: float | None = None
        for _heuristic_score, end_state, state_blocks, cmd_blocks, min_clearance_m in endpoint_pool[:3]:
            exec_nominal, exec_robust, _ = self._robust_execution_certificate(
                anchor=end_state,
                follower=follower,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
                lane_heading=lane_heading,
                allow_grid_fallback=False,
            )
            terminal_nominal, terminal_robust, terminal_meta = self._robust_terminal_safety_certificate(
                anchor=end_state,
                follower=follower,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
            )
            combined_robust = float(math.sqrt(max(exec_robust, 0.0) * max(terminal_robust, 0.0)))
            if combined_robust <= 0.0:
                continue
            full_certificate = self._certificate_from_anchor(
                anchor=end_state,
                anchor_hint_xy=anchor_hint_xy,
                lane_heading=lane_heading,
                follower=follower,
                obstacles=obstacles,
                planner=planner,
                corridor_polygons=corridor_polygons,
            )
            if full_certificate is None:
                continue
            primary_score = float(
                1.35 * combined_robust
                + 0.18 * exec_nominal
                + 0.12 * terminal_nominal
                + 0.08 * min_clearance_m
            )
            leader_path_xy = np.vstack(
                [
                    np.asarray([[float(leader.x), float(leader.y)]], dtype=float),
                    np.vstack(state_blocks)[:, :2],
                ]
            )
            soft_cost = self._soft_post_path_cost(
                path_xy=leader_path_xy,
                min_clearance_m=float(min_clearance_m),
                goal_yaw=float(end_state.yaw),
                lane_heading=float(lane_heading),
            )
            if not self._prefer_cost_modulated_candidate(
                best_primary_score=best_primary_score,
                best_soft_cost=best_soft_cost,
                candidate_primary_score=float(primary_score),
                candidate_soft_cost=float(soft_cost),
                primary_margin=0.10,
            ):
                continue
            best_primary_score = float(primary_score)
            best_soft_cost = float(soft_cost)
            best_plan = CorridorReciprocalPlan(
                leader_goal=end_state.copy(),
                follower_goal=full_certificate.predock.copy(),
                leader_path_xy=np.asarray(leader_path_xy, dtype=float),
                follower_path_xy=np.asarray(full_certificate.follower_path_xy, dtype=float),
                leader_primitive_states=np.vstack(state_blocks).astype(float),
                leader_primitive_cmd=np.vstack(cmd_blocks).astype(float),
                leader_primitive_steps=int(sum(len(block) for block in cmd_blocks)),
                score=float(1.9 - 1.4 * combined_robust),
                reason="lc_corridor_escape_shape",
                metadata={
                    "lane_y": float(self._lane_y(lane)),
                    **self._macro_metadata(lane=lane, leader_ref=leader),
                    "anchor_hint_xy": [float(anchor_hint_xy[0]), float(anchor_hint_xy[1])],
                    "anchor_hint_yaw": float(lane.get("leader_stage_anchor_yaw", 0.0)),
                    "lane_heading": float(lane_heading),
                    "corridor_polygons": [np.asarray(poly, dtype=float).tolist() for poly in corridor_polygons],
                    "sigma_c": float(exec_robust),
                    "sigma_geom": float(full_certificate.sigma_geom),
                    "sigma_reach": float(exec_nominal),
                    "exec_nominal_sigma": float(exec_nominal),
                    "exec_robust_sigma": float(exec_robust),
                    "terminal_nominal_sigma": float(terminal_nominal),
                    "terminal_robust_sigma": float(terminal_robust),
                    "combined_nominal_sigma": float(math.sqrt(max(exec_nominal, 0.0) * max(terminal_nominal, 0.0))),
                    "combined_robust_sigma": float(combined_robust),
                    "soft_path_cost": float(soft_cost),
                    "path_yaw_error_deg": float(math.degrees(self._parallel_yaw_error(yaw=float(end_state.yaw), lane_heading=float(lane_heading)))),
                    "release_sigma_threshold": 0.40,
                    "leader_path_clearance_m": float(min_clearance_m),
                    "follower_path_clearance_m": float(full_certificate.follower_path_clearance_m),
                    "handoff_clearance_m": float(full_certificate.handoff_clearance_m),
                    "dock_zone_clearance_m": float(full_certificate.dock_zone_clearance_m),
                    "terminal_path_clearance_m": float(terminal_meta.get("terminal_path_clearance_m", 0.0)),
                    "alignment_quality": float(full_certificate.alignment_quality),
                    "candidate_rank": 0,
                    "search_expansions": int(sum(len(block) for block in cmd_blocks)),
                    "shape_target_speed": 0.14,
                    "settle_target_speed": 0.12,
                    "follower_target_speed": 0.24,
                    "exec_release_threshold": 0.30,
                    "exec_robust_threshold": 0.20,
                    "terminal_release_threshold": 0.16,
                    "shape_budget_s": 6.0,
                    "settle_budget_s": 3.0,
                    "planner_time_budget_s": float(time_budget_s),
                    "planner_elapsed_s": float(time.perf_counter() - start_time_s),
                },
            )
        return best_plan

    def _macro_escape_plan(
        self,
        *,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
        lane: dict[str, Any],
        planner: GridAStarPlanner,
        corridor_polygons: list[np.ndarray],
    ) -> CorridorReciprocalPlan | None:
        start_time_s = time.perf_counter()
        lane_heading = self._lane_heading(lane, default_yaw=float(lane.get("leader_stage_anchor_yaw", leader.yaw)))
        anchor_hint_xy = np.asarray(lane.get("leader_stage_anchor_xy", [leader.x, leader.y]), dtype=float)
        macro_templates = self.macro_selector.fallback_to_legacy_primitives(
            leader=leader,
            corridor_info={
                "lane_y": float(self._lane_y(lane)),
                "lane_heading": float(lane_heading),
                "anchor_hint_xy": [float(anchor_hint_xy[0]), float(anchor_hint_xy[1])],
                **self._macro_metadata(lane=lane, leader_ref=leader),
            },
        )
        if not macro_templates:
            return None
        best_eval: MacroTemplateEvaluation | None = None
        legacy_eval: MacroTemplateEvaluation | None = None
        best_primary_score = -1e9
        best_soft_cost: float | None = None
        macro_time_budget_s = 1.00
        for macro_template in macro_templates:
            if time.perf_counter() - start_time_s >= macro_time_budget_s:
                break
            evaluation = self._evaluate_macro_template_plan(
                macro_template=macro_template,
                obstacles=obstacles,
                leader=leader,
                follower=follower,
                lane=lane,
                planner=planner,
                corridor_polygons=corridor_polygons,
                lane_heading=lane_heading,
                anchor_hint_xy=anchor_hint_xy,
            )
            if evaluation is None:
                continue
            if evaluation.legacy_priority > 0.0 and self._post_feasible_candidate(
                exec_robust_sigma=evaluation.exec_robust_sigma,
                terminal_nominal_sigma=evaluation.terminal_nominal_sigma,
                combined_robust_sigma=evaluation.combined_robust_sigma,
            ):
                if legacy_eval is None or (
                    evaluation.sigma_post > legacy_eval.sigma_post + 1e-6
                    or (
                        evaluation.sigma_post >= legacy_eval.sigma_post - 1e-6
                        and evaluation.combined_robust_sigma > legacy_eval.combined_robust_sigma + 1e-6
                    )
                    or (
                        evaluation.sigma_post >= legacy_eval.sigma_post - 1e-6
                        and evaluation.combined_robust_sigma >= legacy_eval.combined_robust_sigma - 1e-6
                        and evaluation.min_clearance_m > legacy_eval.min_clearance_m + 1e-6
                    )
                ):
                    legacy_eval = evaluation
            primary_score = float(
                0.82 * evaluation.combined_nominal_sigma
                + 1.02 * evaluation.combined_robust_sigma
                + 0.18 * evaluation.exec_robust_sigma
                + 0.14 * evaluation.terminal_robust_sigma
                + 0.05 * evaluation.min_clearance_m
                - 0.02 * np.linalg.norm(evaluation.plan.leader_goal.xy() - anchor_hint_xy)
            )
            soft_cost = float(evaluation.soft_path_cost - 0.05 * evaluation.legacy_priority)
            if self._prefer_cost_modulated_candidate(
                best_primary_score=None if best_eval is None else float(best_primary_score),
                best_soft_cost=best_soft_cost,
                candidate_primary_score=float(primary_score),
                candidate_soft_cost=float(soft_cost),
                primary_margin=0.10,
            ):
                best_primary_score = float(primary_score)
                best_soft_cost = float(soft_cost)
                best_eval = evaluation
        if best_eval is not None:
            return best_eval.plan
        if legacy_eval is not None:
            return legacy_eval.plan
        return None

    def _search_execution_valid_shape_plan(
        self,
        *,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
        lane: dict[str, Any],
        corridor_polygons: list[np.ndarray],
    ) -> CorridorReciprocalPlan | None:
        lane_heading = self._lane_heading(lane, default_yaw=float(lane.get("leader_stage_anchor_yaw", leader.yaw)))
        anchor_hint_xy = np.asarray(lane.get("leader_stage_anchor_xy", [leader.x, leader.y]), dtype=float)
        beam: list[tuple[VehicleState, list[np.ndarray], list[np.ndarray], float, int]] = [(leader.copy(), [], [], 1e6, 0)]
        best: tuple[float, VehicleState, list[np.ndarray], list[np.ndarray], float, dict[str, Any], float, float] | None = None
        best_primary_score: float | None = None
        best_soft_cost: float | None = None
        seen: dict[tuple[int, int, int], float] = {}
        for depth in range(1, 5):
            next_beam: list[tuple[VehicleState, list[np.ndarray], list[np.ndarray], float, int, float]] = []
            for state, state_blocks, cmd_blocks, min_clearance_m, gear_switches in beam:
                for primitive in self._shape_candidate_primitives():
                    rollout = self.motion_library.rollout(start=state, primitive=primitive)
                    safe, rollout_clearance, _ = self.motion_library.is_safe(
                        rollout=rollout,
                        obstacles=obstacles,
                        corridor_polygons=corridor_polygons,
                        other=follower,
                        min_clearance=float(self.cfg.safety.min_clearance),
                        in_bounds=self._in_bounds,
                    )
                    if not safe:
                        continue
                    end_state = rollout.end_state.copy()
                    nominal_sigma, robust_sigma, cert_meta = self._robust_execution_certificate(
                        anchor=end_state,
                        follower=follower,
                        obstacles=obstacles,
                        corridor_polygons=corridor_polygons,
                        lane_heading=lane_heading,
                        allow_grid_fallback=False,
                    )
                    terminal_nominal, terminal_robust, terminal_meta = self._robust_terminal_safety_certificate(
                        anchor=end_state,
                        follower=follower,
                        obstacles=obstacles,
                        corridor_polygons=corridor_polygons,
                    )
                    distance_to_hint_m = float(np.linalg.norm(end_state.xy() - anchor_hint_xy))
                    candidate_min_clearance_m = float(min(min_clearance_m, rollout_clearance))
                    if float(candidate_min_clearance_m) < float(self._critical_clearance_threshold()):
                        continue
                    combined_nominal = float(math.sqrt(max(nominal_sigma, 0.0) * max(terminal_nominal, 0.0)))
                    combined_robust = float(math.sqrt(max(robust_sigma, 0.0) * max(terminal_robust, 0.0)))
                    staging_score, sigma_post = self._staging_candidate_score(
                        leader_pose=end_state,
                        follower_pose=follower,
                        corridor_info={"lane_heading": float(lane_heading)},
                        sigma_progress=float(max(nominal_sigma, combined_nominal)),
                        sigma_terminal_nominal=float(terminal_nominal),
                        sigma_terminal_robust=float(terminal_robust),
                        sigma_corridor_nominal=float(nominal_sigma),
                        sigma_corridor_robust=float(robust_sigma),
                        sigma_visibility=1.0,
                    )
                    score = float(
                        0.95 * staging_score
                        + 0.55 * sigma_post
                        + 0.70 * combined_robust
                        + 0.18 * combined_nominal
                        + 0.12 * robust_sigma
                        + 0.08 * terminal_robust
                        - 0.03 * depth
                        - 0.04 * gear_switches
                        - 0.025 * distance_to_hint_m
                        + 0.015 * candidate_min_clearance_m
                    )
                    state_key = (
                        int(round(float(end_state.x) / 0.08)),
                        int(round(float(end_state.y) / 0.08)),
                        int(round(math.degrees(float(end_state.yaw)) / 4.0)),
                    )
                    if score <= float(seen.get(state_key, -1e9)) + 1e-9:
                        continue
                    seen[state_key] = float(score)
                    next_state_blocks = [*state_blocks, np.asarray(rollout.states, dtype=float)]
                    next_cmd_blocks = [*cmd_blocks, np.asarray(rollout.commands, dtype=float)]
                    next_gear_switches = int(gear_switches + primitive.gear_switches)
                    next_beam.append((end_state, next_state_blocks, next_cmd_blocks, candidate_min_clearance_m, next_gear_switches, score))
                    leader_path_xy_candidate = np.vstack(
                        [
                            np.asarray([[float(leader.x), float(leader.y)]], dtype=float),
                            np.vstack(next_state_blocks)[:, :2],
                        ]
                    )
                    soft_cost, front_clearance_m, longitudinal_offset_m, yaw_error_rad = self._soft_post_path_cost(
                        path_xy=leader_path_xy_candidate,
                        min_clearance_m=float(candidate_min_clearance_m),
                        goal_yaw=float(end_state.yaw),
                        lane_heading=float(lane_heading),
                        anchor_pose=end_state,
                        anchor_hint_xy=anchor_hint_xy,
                        obstacles=obstacles,
                        corridor_polygons=corridor_polygons,
                    )
                    if self._prefer_cost_modulated_candidate(
                        best_primary_score=best_primary_score,
                        best_soft_cost=best_soft_cost,
                        candidate_primary_score=float(score),
                        candidate_soft_cost=float(soft_cost),
                        primary_margin=0.10,
                    ):
                        best_primary_score = float(score)
                        best_soft_cost = float(soft_cost)
                        best = (
                            float(score),
                            end_state.copy(),
                            next_state_blocks,
                            next_cmd_blocks,
                            candidate_min_clearance_m,
                            {
                                **dict(cert_meta),
                                "terminal_nominal_sigma": float(terminal_nominal),
                                "terminal_robust_sigma": float(terminal_robust),
                                "terminal_path_clearance_m": float(terminal_meta.get("terminal_path_clearance_m", 0.0)),
                                "terminal_dock_zone_clearance_m": float(terminal_meta.get("dock_zone_clearance_m", 0.0)),
                                "terminal_path_xy": list(terminal_meta.get("terminal_path_xy", [])),
                                "sigma_post": float(sigma_post),
                                "soft_path_cost": float(soft_cost),
                                "front_clearance_m": float(front_clearance_m),
                                "longitudinal_offset_m": float(longitudinal_offset_m),
                                "path_yaw_error_deg": float(math.degrees(yaw_error_rad)),
                            },
                            float(nominal_sigma),
                            float(robust_sigma),
                            float(combined_nominal),
                            float(combined_robust),
                        )
            next_beam.sort(key=lambda item: float(item[-1]), reverse=True)
            beam = [(state, state_blocks, cmd_blocks, min_clearance_m, gear_switches) for state, state_blocks, cmd_blocks, min_clearance_m, gear_switches, _score in next_beam[:12]]
            if best is not None and float(best[9]) >= 0.16:
                break
        if best is None:
            return None
        _score, best_state, state_blocks, cmd_blocks, min_clearance_m, cert_meta, nominal_sigma, robust_sigma, combined_nominal, combined_robust = best
        if combined_robust <= 0.0:
            return None
        full_planner = self._planner(obstacles)
        full_certificate = self._certificate_from_anchor(
            anchor=best_state,
            anchor_hint_xy=anchor_hint_xy,
            lane_heading=lane_heading,
            follower=follower,
            obstacles=obstacles,
            planner=full_planner,
            corridor_polygons=corridor_polygons,
        )
        if full_certificate is None:
            return None
        leader_path_xy = np.vstack(
            [
                np.asarray([[float(leader.x), float(leader.y)]], dtype=float),
                np.vstack(state_blocks)[:, :2],
            ]
        )
        return CorridorReciprocalPlan(
            leader_goal=best_state.copy(),
            follower_goal=full_certificate.predock.copy(),
            leader_path_xy=np.asarray(leader_path_xy, dtype=float),
            follower_path_xy=np.asarray(full_certificate.follower_path_xy, dtype=float),
            leader_primitive_states=np.vstack(state_blocks).astype(float),
            leader_primitive_cmd=np.vstack(cmd_blocks).astype(float),
            leader_primitive_steps=int(sum(len(block) for block in cmd_blocks)),
            score=float(1.9 - 1.6 * robust_sigma - 0.2 * nominal_sigma),
            reason="lc_corridor_exec_shape",
            metadata={
                "lane_y": float(self._lane_y(lane)),
                **self._macro_metadata(lane=lane, leader_ref=leader),
                "anchor_hint_xy": [float(anchor_hint_xy[0]), float(anchor_hint_xy[1])],
                "anchor_hint_yaw": float(lane.get("leader_stage_anchor_yaw", 0.0)),
                "lane_heading": float(lane_heading),
                "corridor_polygons": [np.asarray(poly, dtype=float).tolist() for poly in corridor_polygons],
                "sigma_c": float(robust_sigma),
                "sigma_geom": float(full_certificate.sigma_geom),
                "sigma_reach": float(nominal_sigma),
                "exec_nominal_sigma": float(nominal_sigma),
                "exec_robust_sigma": float(robust_sigma),
                "sigma_post": float(cert_meta.get("sigma_post", 0.0)),
                "terminal_nominal_sigma": float(cert_meta.get("terminal_nominal_sigma", 0.0)),
                "terminal_robust_sigma": float(cert_meta.get("terminal_robust_sigma", 0.0)),
                "combined_nominal_sigma": float(combined_nominal),
                "combined_robust_sigma": float(combined_robust),
                "soft_path_cost": float(cert_meta.get("soft_path_cost", 0.0)),
                "path_yaw_error_deg": float(cert_meta.get("path_yaw_error_deg", math.degrees(self._parallel_yaw_error(yaw=float(best_state.yaw), lane_heading=float(lane_heading))))),
                "front_clearance_m": float(cert_meta.get("front_clearance_m", 0.0)),
                "longitudinal_offset_m": float(cert_meta.get("longitudinal_offset_m", 0.0)),
                "release_sigma_threshold": 0.40,
                "leader_path_clearance_m": float(min_clearance_m),
                "follower_path_clearance_m": float(full_certificate.follower_path_clearance_m),
                "handoff_clearance_m": float(full_certificate.handoff_clearance_m),
                "dock_zone_clearance_m": float(full_certificate.dock_zone_clearance_m),
                "terminal_path_clearance_m": float(cert_meta.get("terminal_path_clearance_m", 0.0)),
                "terminal_dock_zone_clearance_m": float(cert_meta.get("terminal_dock_zone_clearance_m", 0.0)),
                "alignment_quality": float(full_certificate.alignment_quality),
                "candidate_rank": 0,
                "search_expansions": int(sum(len(block) for block in cmd_blocks)),
                "shape_target_speed": 0.16,
                "settle_target_speed": 0.12,
                "follower_target_speed": 0.24,
                "exec_release_threshold": 0.30,
                "exec_robust_threshold": 0.20,
                "terminal_release_threshold": 0.16,
                "shape_budget_s": float(max(6.0, 1.0 + 1.05 * sum(len(block) for block in cmd_blocks) * float(self.cfg.control.dt))),
                "settle_budget_s": 3.0,
                "follower_path_xy_direct": cert_meta.get("follower_path_xy", []),
                "terminal_path_xy": cert_meta.get("terminal_path_xy", []),
            },
        )

    def _candidate_terminal_safe_plan(
        self,
        *,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
        lane: dict[str, Any],
        planner: GridAStarPlanner,
        corridor_polygons: list[np.ndarray],
    ) -> CorridorReciprocalPlan | None:
        lane_heading = self._lane_heading(lane, default_yaw=float(lane.get("leader_stage_anchor_yaw", leader.yaw)))
        anchor_hint_xy = np.asarray(lane.get("leader_stage_anchor_xy", [leader.x, leader.y]), dtype=float)
        best: dict[str, Any] | None = None
        best_primary_score: float | None = None
        best_soft_cost: float | None = None
        for cert in self._candidate_anchor_states(
            leader=leader,
            follower=follower,
            lane=lane,
            obstacles=obstacles,
            planner=planner,
            corridor_polygons=corridor_polygons,
        ):
            exec_nominal, exec_robust, _ = self._robust_execution_certificate(
                anchor=cert.anchor,
                follower=follower,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
                lane_heading=lane_heading,
                allow_grid_fallback=False,
            )
            terminal_nominal, terminal_robust, terminal_meta = self._robust_terminal_safety_certificate(
                anchor=cert.anchor,
                follower=follower,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
            )
            combined_nominal = float(math.sqrt(max(exec_nominal, 0.0) * max(terminal_nominal, 0.0)))
            combined_robust = float(math.sqrt(max(exec_robust, 0.0) * max(terminal_robust, 0.0)))
            sigma_post_probe = float(self.cert_field.sigma_post(leader_pose=cert.anchor, corridor_info={"lane_heading": float(lane_heading)}))
            path_templates = []
            semantic_seed = lane.get("leader_stage_path_xy", [])
            if isinstance(semantic_seed, list) and len(semantic_seed) >= 2:
                semantic_path_xy = np.asarray(semantic_seed, dtype=float)
                if float(np.linalg.norm(semantic_path_xy[0] - leader.xy())) <= 0.35:
                    candidate_path_xy = np.asarray(semantic_path_xy, dtype=float)
                    if float(np.linalg.norm(candidate_path_xy[-1] - cert.anchor.xy())) > 1e-6:
                        candidate_path_xy = np.vstack([candidate_path_xy, cert.anchor.xy()[None, :]])
                    dense_xy = self._densify_path(candidate_path_xy, step=0.10)
                    clearance_m = self._path_min_clearance(
                        dense_xy,
                        proto=leader,
                        goal_yaw=float(cert.anchor.yaw),
                        obstacles=obstacles,
                        corridor_polygons=corridor_polygons,
                        other=follower,
                    )
                    if clearance_m >= float(self.cfg.safety.min_clearance):
                        path_templates.append((float(clearance_m) + 0.02, dense_xy))
            vertical_first = np.asarray([[leader.x, leader.y], [leader.x, cert.anchor.y], [cert.anchor.x, cert.anchor.y]], dtype=float)
            horizontal_first = np.asarray([[leader.x, leader.y], [cert.anchor.x, leader.y], [cert.anchor.x, cert.anchor.y]], dtype=float)
            diagonal = np.asarray([[leader.x, leader.y], [cert.anchor.x, cert.anchor.y]], dtype=float)
            for path_xy in (vertical_first, horizontal_first, diagonal):
                dense_xy = self._densify_path(path_xy, step=0.10)
                clearance_m = self._path_min_clearance(
                    dense_xy,
                    proto=leader,
                    goal_yaw=float(cert.anchor.yaw),
                    obstacles=obstacles,
                    corridor_polygons=corridor_polygons,
                    other=follower,
                )
                if clearance_m >= float(self.cfg.safety.min_clearance):
                    path_templates.append((float(clearance_m), dense_xy))
            if not path_templates:
                continue
            path_templates.sort(key=lambda item: item[0], reverse=True)
            leader_path_xy = np.asarray(path_templates[0][1], dtype=float)
            leader_path_clearance_m = float(path_templates[0][0])
            if float(leader_path_clearance_m) < float(self._critical_clearance_threshold()):
                continue
            primitive_states: list[tuple[float, float, float]] = []
            primitive_commands: list[tuple[float, float, float]] = []
            previous_yaw = float(leader.yaw)
            previous_xy = leader.xy()
            for point_xy in np.asarray(leader_path_xy, dtype=float)[1:]:
                segment_xy = np.asarray(point_xy, dtype=float) - np.asarray(previous_xy, dtype=float)
                if float(np.linalg.norm(segment_xy)) <= 1e-9:
                    continue
                yaw_ref = float(math.atan2(float(segment_xy[1]), float(segment_xy[0])))
                direction_sign = float(np.dot(segment_xy, np.array([math.cos(previous_yaw), math.sin(previous_yaw)], dtype=float)))
                target_speed = float(0.12 if direction_sign >= 0.0 else -0.12)
                primitive_states.append((float(point_xy[0]), float(point_xy[1]), float(yaw_ref)))
                primitive_commands.append((0.0, 0.0, float(target_speed)))
                previous_xy = np.asarray(point_xy, dtype=float)
                previous_yaw = float(yaw_ref)
            if not primitive_states:
                primitive_states = [(float(cert.anchor.x), float(cert.anchor.y), float(cert.anchor.yaw))]
                primitive_commands = [(0.0, 0.0, 0.12)]
            leader_primitive_states = np.asarray(primitive_states, dtype=float)
            leader_primitive_cmd = np.asarray(primitive_commands, dtype=float)
            leader_primitive_steps = int(len(primitive_commands))
            search_expansions = int(len(leader_path_xy))
            path_quality = float(path_templates[0][0])
            path_mode = "semantic_path"
            staging_score, sigma_post = self._staging_candidate_score(
                leader_pose=cert.anchor,
                follower_pose=follower,
                corridor_info={"lane_heading": float(lane_heading)},
                sigma_progress=float(cert.sigma_geom),
                sigma_terminal_nominal=float(terminal_nominal),
                sigma_terminal_robust=float(terminal_robust),
                sigma_corridor_nominal=float(exec_nominal),
                sigma_corridor_robust=float(exec_robust),
                sigma_visibility=1.0,
            )
            primary_score = float(
                0.92 * staging_score
                + 0.72 * combined_robust
                + 0.16 * combined_nominal
                + 0.08 * exec_robust
                + 0.08 * terminal_robust
                - 0.03 * np.linalg.norm(cert.anchor.xy() - anchor_hint_xy)
                + 0.02 * path_quality
            )
            soft_cost, front_clearance_m, longitudinal_offset_m, yaw_error_rad = self._soft_post_path_cost(
                path_xy=leader_path_xy,
                min_clearance_m=float(leader_path_clearance_m),
                goal_yaw=float(cert.anchor.yaw),
                lane_heading=float(lane_heading),
                anchor_pose=cert.anchor,
                anchor_hint_xy=anchor_hint_xy,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
            )
            if self._prefer_cost_modulated_candidate(
                best_primary_score=best_primary_score,
                best_soft_cost=best_soft_cost,
                candidate_primary_score=float(primary_score),
                candidate_soft_cost=float(soft_cost),
                primary_margin=0.08,
            ):
                best_primary_score = float(primary_score)
                best_soft_cost = float(soft_cost)
                best = {
                    "score": float(primary_score - 0.10 * soft_cost),
                    "primary_score": float(primary_score),
                    "cert": cert,
                    "exec_nominal": float(exec_nominal),
                    "exec_robust": float(exec_robust),
                    "terminal_nominal": float(terminal_nominal),
                    "terminal_robust": float(terminal_robust),
                    "leader_path_xy": np.asarray(leader_path_xy, dtype=float),
                    "leader_path_clearance_m": float(leader_path_clearance_m),
                    "leader_primitive_states": np.asarray(leader_primitive_states, dtype=float),
                    "leader_primitive_cmd": np.asarray(leader_primitive_cmd, dtype=float),
                    "leader_primitive_steps": int(leader_primitive_steps),
                    "search_expansions": int(search_expansions),
                    "sigma_post": float(sigma_post),
                    "soft_path_cost": float(soft_cost),
                    "front_clearance_m": float(front_clearance_m),
                    "longitudinal_offset_m": float(longitudinal_offset_m),
                    "path_yaw_error_deg": float(math.degrees(yaw_error_rad)),
                    "path_mode": str(path_mode),
                }
        if best is None:
            return None
        cert = best["cert"]
        exec_nominal = float(best["exec_nominal"])
        exec_robust = float(best["exec_robust"])
        terminal_nominal = float(best["terminal_nominal"])
        terminal_robust = float(best["terminal_robust"])
        leader_path_xy = np.asarray(best["leader_path_xy"], dtype=float)
        combined_nominal = float(math.sqrt(max(exec_nominal, 0.0) * max(terminal_nominal, 0.0)))
        combined_robust = float(math.sqrt(max(exec_robust, 0.0) * max(terminal_robust, 0.0)))
        if combined_robust <= 0.0:
            return None
        return CorridorReciprocalPlan(
            leader_goal=cert.anchor.copy(),
            follower_goal=cert.predock.copy(),
            leader_path_xy=np.asarray(leader_path_xy, dtype=float),
            follower_path_xy=np.asarray(cert.follower_path_xy, dtype=float),
            leader_primitive_states=np.asarray(best["leader_primitive_states"], dtype=float),
            leader_primitive_cmd=np.asarray(best["leader_primitive_cmd"], dtype=float),
            leader_primitive_steps=int(best["leader_primitive_steps"]),
            score=float(1.8 - 1.5 * combined_robust),
            reason="lc_corridor_semantic_cert_anchor",
            metadata={
                "lane_y": float(self._lane_y(lane)),
                **self._macro_metadata(lane=lane, leader_ref=leader),
                "anchor_hint_xy": [float(anchor_hint_xy[0]), float(anchor_hint_xy[1])],
                "anchor_hint_yaw": float(lane.get("leader_stage_anchor_yaw", 0.0)),
                "lane_heading": float(lane_heading),
                "corridor_polygons": [np.asarray(poly, dtype=float).tolist() for poly in corridor_polygons],
                "sigma_c": float(exec_robust),
                "sigma_geom": float(cert.sigma_geom),
                "sigma_reach": float(exec_nominal),
                "sigma_post": float(self.cert_field.sigma_post(leader_pose=cert.anchor, corridor_info={"lane_heading": float(lane_heading)})),
                "exec_nominal_sigma": float(exec_nominal),
                "exec_robust_sigma": float(exec_robust),
                "terminal_nominal_sigma": float(terminal_nominal),
                "terminal_robust_sigma": float(terminal_robust),
                "combined_nominal_sigma": float(combined_nominal),
                "combined_robust_sigma": float(combined_robust),
                "soft_path_cost": float(best["soft_path_cost"]),
                "path_yaw_error_deg": float(best["path_yaw_error_deg"]),
                "front_clearance_m": float(best["front_clearance_m"]),
                "longitudinal_offset_m": float(best["longitudinal_offset_m"]),
                "release_sigma_threshold": 0.40,
                "leader_path_clearance_m": float(best["leader_path_clearance_m"]),
                "follower_path_clearance_m": float(cert.follower_path_clearance_m),
                "handoff_clearance_m": float(cert.handoff_clearance_m),
                "dock_zone_clearance_m": float(cert.dock_zone_clearance_m),
                "terminal_path_clearance_m": float(self._approx_terminal_safety_certificate(anchor=cert.anchor, follower=follower, obstacles=obstacles, corridor_polygons=corridor_polygons)[1].get("terminal_path_clearance_m", 0.0)),
                "alignment_quality": float(cert.alignment_quality),
                "candidate_rank": 0,
                "search_expansions": int(best["search_expansions"]),
                "shape_target_speed": 0.12,
                "settle_target_speed": 0.12,
                "follower_target_speed": 0.24,
                "exec_release_threshold": 0.30,
                "exec_robust_threshold": 0.20,
                "terminal_release_threshold": 0.16,
                "shape_budget_s": 8.0,
                "settle_budget_s": 3.0,
                "terminal_capture_safe": bool(terminal_robust >= 0.16),
                "path_mode": str(best["path_mode"]),
            },
        )

    def _semantic_projection_plan(
        self,
        *,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
        lane: dict[str, Any],
        planner: GridAStarPlanner,
        corridor_polygons: list[np.ndarray],
    ) -> CorridorReciprocalPlan | None:
        guide_path = lane.get("leader_stage_path_xy", [])
        if not isinstance(guide_path, list) or len(guide_path) < 2:
            return None
        anchor_hint_xy = np.asarray(lane.get("leader_stage_anchor_xy", [leader.x, leader.y]), dtype=float)
        anchor_hint_yaw = float(lane.get("leader_stage_anchor_yaw", 0.0))
        lane_heading = self._lane_heading(lane, default_yaw=anchor_hint_yaw)
        anchor = VehicleState(
            vehicle_id=int(leader.vehicle_id),
            x=float(anchor_hint_xy[0]),
            y=float(anchor_hint_xy[1]),
            yaw=float(anchor_hint_yaw),
            v=0.0,
            delta=0.0,
            mode=leader.mode,
        )
        certificate = self._certificate_from_anchor(
            anchor=anchor,
            anchor_hint_xy=anchor_hint_xy,
            lane_heading=lane_heading,
            follower=follower,
            obstacles=obstacles,
            planner=planner,
            corridor_polygons=corridor_polygons,
        )
        if certificate is None:
            return None
        current_state = leader.copy()
        subgoals = self._guided_subgoals(lane=lane, leader=leader, goal=anchor)
        primitive_states: list[np.ndarray] = []
        primitive_commands: list[np.ndarray] = []
        min_clearance_m = 1e6
        for subgoal in subgoals:
            base_metric = self._heuristic_cost(current_state, subgoal, lane_heading)
            best_choice: tuple[PrimitiveRollout, float, float] | None = None
            for primitive in self._segment_primitives(start=current_state, goal=subgoal):
                rollout = self.motion_library.rollout(start=current_state, primitive=primitive)
                safe, rollout_clearance, _reason = self.motion_library.is_safe(
                    rollout=rollout,
                    obstacles=obstacles,
                    corridor_polygons=corridor_polygons,
                    other=follower,
                    min_clearance=float(max(self.cfg.safety.min_clearance + 0.02, 0.12)),
                    in_bounds=self._in_bounds,
                )
                if not safe:
                    continue
                metric = float(self._heuristic_cost(rollout.end_state, subgoal, lane_heading) + 0.08 * primitive.gear_switches + 0.05 * primitive.cost_bias)
                if metric >= base_metric - 0.02:
                    continue
                if best_choice is None or metric < float(best_choice[1]):
                    best_choice = (rollout, float(metric), float(rollout_clearance))
            if best_choice is None:
                return None
            best_rollout, _metric, rollout_clearance = best_choice
            primitive_states.append(np.asarray(best_rollout.states, dtype=float))
            primitive_commands.append(np.asarray(best_rollout.commands, dtype=float))
            current_state = best_rollout.end_state.copy()
            min_clearance_m = float(min(min_clearance_m, rollout_clearance))
        if not primitive_states or not primitive_commands:
            return None
        reach_position = float(clamp(1.0 - float(np.linalg.norm(current_state.xy() - anchor.xy())) / 0.80, 0.0, 1.0))
        reach_heading = float(clamp(1.0 - abs(float(angle_diff(float(anchor.yaw), float(current_state.yaw)))) / math.radians(28.0), 0.0, 1.0))
        reach_clearance = float(clamp((float(min_clearance_m) - (self.cfg.safety.min_clearance + 0.02)) / 0.40, 0.0, 1.0))
        sigma_reach = 0.42 * reach_position + 0.30 * reach_heading + 0.28 * reach_clearance
        sigma_c = float(clamp(0.60 * float(certificate.sigma_geom) + 0.40 * float(sigma_reach), 0.0, 1.0))
        if sigma_reach <= 0.08:
            return None
        leader_states_xy = np.vstack(primitive_states)[:, :2]
        return CorridorReciprocalPlan(
            leader_goal=anchor.copy(),
            follower_goal=certificate.predock.copy(),
            leader_path_xy=np.vstack(
                [
                    np.asarray([[float(leader.x), float(leader.y)]], dtype=float),
                    np.asarray(leader_states_xy, dtype=float),
                    np.asarray([[float(anchor.x), float(anchor.y)]], dtype=float),
                ]
            ),
            follower_path_xy=np.asarray(certificate.follower_path_xy, dtype=float),
            leader_primitive_states=np.vstack(primitive_states).astype(float),
            leader_primitive_cmd=np.vstack(primitive_commands).astype(float),
            leader_primitive_steps=int(sum(len(block) for block in primitive_commands)),
            score=float(1.8 - 1.6 * sigma_c + 0.06 * sum(len(block) for block in primitive_commands)),
            reason="lc_corridor_semantic_hybrid",
            metadata={
                "lane_y": float(self._lane_y(lane)),
                "anchor_hint_xy": [float(anchor_hint_xy[0]), float(anchor_hint_xy[1])],
                "anchor_hint_yaw": float(anchor_hint_yaw),
                "lane_heading": float(lane_heading),
                "corridor_polygons": [np.asarray(poly, dtype=float).tolist() for poly in corridor_polygons],
                "sigma_c": float(sigma_c),
                "sigma_geom": float(certificate.sigma_geom),
                "sigma_reach": float(sigma_reach),
                "release_sigma_threshold": 0.44,
                "leader_path_clearance_m": float(min_clearance_m),
                "follower_path_clearance_m": float(certificate.follower_path_clearance_m),
                "handoff_clearance_m": float(certificate.handoff_clearance_m),
                "dock_zone_clearance_m": float(certificate.dock_zone_clearance_m),
                "alignment_quality": float(certificate.alignment_quality),
                "candidate_rank": 0,
                "search_expansions": 0,
                "shape_target_speed": 0.18,
                "settle_target_speed": 0.16,
                "follower_target_speed": 0.28,
                "exec_release_threshold": 0.30,
                "exec_robust_threshold": 0.20,
                "shape_budget_s": float(max(6.5, 1.0 + 1.15 * sum(len(block) for block in primitive_commands) * float(self.cfg.control.dt))),
                "settle_budget_s": 4.5,
            },
        )

    def _semantic_reference_plan(
        self,
        *,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
        lane: dict[str, Any],
        planner: GridAStarPlanner,
        corridor_polygons: list[np.ndarray],
    ) -> CorridorReciprocalPlan | None:
        leader_path_seed = lane.get("leader_stage_path_xy", [])
        if not isinstance(leader_path_seed, list) or len(leader_path_seed) < 2:
            return None
        anchor_hint_xy = np.asarray(lane.get("leader_stage_anchor_xy", [leader.x, leader.y]), dtype=float)
        anchor_hint_yaw = float(lane.get("leader_stage_anchor_yaw", 0.0))
        lane_heading = self._lane_heading(lane, default_yaw=anchor_hint_yaw)
        anchor = VehicleState(
            vehicle_id=int(leader.vehicle_id),
            x=float(anchor_hint_xy[0]),
            y=float(anchor_hint_xy[1]),
            yaw=float(anchor_hint_yaw),
            v=0.0,
            delta=0.0,
            mode=leader.mode,
        )
        certificate = self._certificate_from_anchor(
            anchor=anchor,
            anchor_hint_xy=anchor_hint_xy,
            lane_heading=lane_heading,
            follower=follower,
            obstacles=obstacles,
            planner=planner,
            corridor_polygons=corridor_polygons,
        )
        if certificate is None:
            return None
        dense_path_xy = self._densify_path(np.asarray(leader_path_seed, dtype=float), step=0.10)
        if len(dense_path_xy) < 2:
            return None
        yaw_delta = float(angle_diff(float(anchor.yaw), float(leader.yaw)))
        states: list[tuple[float, float, float]] = []
        commands: list[tuple[float, float, float]] = []
        min_clearance_m = 1e6
        previous_speed = -0.12
        for index in range(1, len(dense_path_xy)):
            alpha = float(index / max(len(dense_path_xy) - 1, 1))
            yaw_ref = float(wrap_angle(float(leader.yaw) + alpha * yaw_delta))
            reference_state = VehicleState(
                vehicle_id=int(leader.vehicle_id),
                x=float(dense_path_xy[index, 0]),
                y=float(dense_path_xy[index, 1]),
                yaw=float(yaw_ref),
                v=0.0,
                delta=0.0,
                mode=leader.mode,
            )
            if not self._in_bounds(reference_state, margin=0.40):
                return None
            if corridor_polygons and not self._pose_in_corridor(reference_state, corridor_polygons):
                return None
            if self.collision.in_collision(reference_state, obstacles, [follower]):
                return None
            obstacle_clearance = float(self.collision.min_clearance_vehicle_obstacles(reference_state, obstacles))
            vehicle_clearance = float(self.collision.min_clearance_vehicle_vehicle(reference_state, follower))
            min_clearance_m = float(min(min_clearance_m, obstacle_clearance, vehicle_clearance))
            if min_clearance_m < float(max(self.cfg.safety.min_clearance + 0.02, 0.12)):
                return None
            segment_xy = dense_path_xy[index] - dense_path_xy[index - 1]
            heading_xy = np.array([math.cos(float(yaw_ref)), math.sin(float(yaw_ref))], dtype=float)
            longitudinal_progress = float(np.dot(segment_xy, heading_xy))
            if abs(longitudinal_progress) <= 1e-4:
                target_speed = float(previous_speed)
            else:
                target_speed = float(0.12 if longitudinal_progress >= 0.0 else -0.12)
            previous_speed = float(target_speed)
            states.append((float(reference_state.x), float(reference_state.y), float(reference_state.yaw)))
            commands.append((0.0, 0.0, float(target_speed)))
        follower_path_seed = lane.get("follower_stage_path_xy", [])
        if isinstance(follower_path_seed, list) and len(follower_path_seed) >= 2:
            follower_path_xy = self._densify_path(np.asarray(follower_path_seed, dtype=float), step=0.10)
            follower_clearance = self._path_min_clearance(
                follower_path_xy,
                proto=follower,
                goal_yaw=float(certificate.predock.yaw),
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
                other=anchor,
            )
            if follower_clearance < max(float(self.cfg.safety.min_clearance) + 0.02, 0.12):
                follower_path_xy = np.asarray(certificate.follower_path_xy, dtype=float)
        else:
            follower_path_xy = np.asarray(certificate.follower_path_xy, dtype=float)
        sigma_c = float(clamp(0.68 * float(certificate.sigma_geom) + 0.32 * float(clamp((min_clearance_m - (self.cfg.safety.min_clearance + 0.02)) / 0.40, 0.0, 1.0)), 0.0, 1.0))
        leader_path_length_m = float(np.sum(np.linalg.norm(np.diff(np.asarray(dense_path_xy, dtype=float), axis=0), axis=1)))
        follower_path_length_m = float(np.sum(np.linalg.norm(np.diff(np.asarray(follower_path_xy, dtype=float), axis=0), axis=1)))
        shape_budget_s = float(max(6.0, 1.0 + 1.05 * len(commands) * float(self.cfg.control.dt)))
        settle_budget_s = float(4.0)
        return CorridorReciprocalPlan(
            leader_goal=anchor.copy(),
            follower_goal=certificate.predock.copy(),
            leader_path_xy=np.asarray(dense_path_xy, dtype=float),
            follower_path_xy=np.asarray(follower_path_xy, dtype=float),
            leader_primitive_states=np.asarray(states, dtype=float),
            leader_primitive_cmd=np.asarray(commands, dtype=float),
            leader_primitive_steps=int(len(commands)),
            score=float(1.55 - 1.35 * sigma_c + 0.03 * len(commands)),
            reason="lc_corridor_semantic_reference",
            metadata={
                "lane_y": float(self._lane_y(lane)),
                "anchor_hint_xy": [float(anchor_hint_xy[0]), float(anchor_hint_xy[1])],
                "anchor_hint_yaw": float(anchor_hint_yaw),
                "lane_heading": float(lane_heading),
                "corridor_polygons": [np.asarray(poly, dtype=float).tolist() for poly in corridor_polygons],
                "sigma_c": float(sigma_c),
                "sigma_geom": float(certificate.sigma_geom),
                "sigma_reach": float(clamp((min_clearance_m - (self.cfg.safety.min_clearance + 0.02)) / 0.40, 0.0, 1.0)),
                "release_sigma_threshold": 0.40,
                "leader_path_clearance_m": float(min_clearance_m),
                "follower_path_clearance_m": float(certificate.follower_path_clearance_m),
                "handoff_clearance_m": float(certificate.handoff_clearance_m),
                "dock_zone_clearance_m": float(certificate.dock_zone_clearance_m),
                "alignment_quality": float(certificate.alignment_quality),
                "candidate_rank": 0,
                "search_expansions": 0,
                "shape_target_speed": 0.12,
                "settle_target_speed": 0.14,
                "follower_target_speed": 0.24,
                "exec_release_threshold": 0.30,
                "exec_robust_threshold": 0.20,
                "leader_path_length_m": float(leader_path_length_m),
                "follower_path_length_m": float(follower_path_length_m),
                "shape_budget_s": float(shape_budget_s),
                "settle_budget_s": float(settle_budget_s),
            },
        )

    def _heuristic_cost(self, state: VehicleState, goal: VehicleState, lane_heading: float) -> float:
        position_cost = float(np.linalg.norm(state.xy() - goal.xy()))
        heading_cost = 0.35 * abs(float(angle_diff(float(goal.yaw), float(state.yaw))))
        lane_cost = 0.18 * abs(float(angle_diff(float(state.yaw), float(lane_heading))))
        return float(position_cost + heading_cost + lane_cost)

    def _segment_primitives(self, *, start: VehicleState, goal: VehicleState) -> tuple[MotionPrimitive, ...]:
        primitive_map = {primitive.name: primitive for primitive in self.motion_library.primitives}
        delta_xy = goal.xy() - start.xy()
        longitudinal_gap = float(
            np.dot(
                delta_xy,
                np.array([math.cos(float(start.yaw)), math.sin(float(start.yaw))], dtype=float),
            )
        )
        yaw_gap = abs(float(angle_diff(float(goal.yaw), float(start.yaw))))
        base_names = ["reverse_straight_short", "reverse_arc_left", "reverse_arc_right", "forward_straight_short", "forward_arc_left", "forward_arc_right"]
        if abs(longitudinal_gap) >= 0.55:
            base_names.extend(["reverse_straight_long", "forward_straight_long", "reverse_arc_left_long", "reverse_arc_right_long"])
        if yaw_gap >= math.radians(12.0):
            base_names.extend(["reverse_arc_left_tight", "reverse_arc_right_tight", "forward_arc_left_tight", "forward_arc_right_tight"])
        if yaw_gap >= math.radians(20.0):
            base_names.extend(["forward_turn_reverse_left", "forward_turn_reverse_right", "reverse_turn_forward_left", "reverse_turn_forward_right"])
        if longitudinal_gap >= 0.10:
            ordered_names = [name for name in base_names if name.startswith("forward")] + [name for name in base_names if name.startswith("reverse")]
        else:
            ordered_names = [name for name in base_names if name.startswith("reverse")] + [name for name in base_names if name.startswith("forward")]
        seen_names: set[str] = set()
        ordered_primitives: list[MotionPrimitive] = []
        for name in ordered_names:
            if name in seen_names or name not in primitive_map:
                continue
            seen_names.add(name)
            ordered_primitives.append(primitive_map[name])
        return tuple(ordered_primitives)

    def _guided_subgoals(self, *, lane: dict[str, Any], leader: VehicleState, goal: VehicleState) -> list[VehicleState]:
        guide_path = lane.get("leader_stage_path_xy", [])
        if not isinstance(guide_path, list) or len(guide_path) < 2:
            return [goal.copy()]
        dense_path_xy = self._densify_path(np.asarray(guide_path, dtype=float), step=0.24)
        if len(dense_path_xy) < 2:
            return [goal.copy()]
        subgoal_indices = sorted({max(1, len(dense_path_xy) // 3), max(1, 2 * len(dense_path_xy) // 3)})
        out: list[VehicleState] = []
        for subgoal_index in subgoal_indices:
            subgoal_xy = np.asarray(dense_path_xy[subgoal_index], dtype=float)
            out.append(
                VehicleState(
                    vehicle_id=int(leader.vehicle_id),
                    x=float(subgoal_xy[0]),
                    y=float(subgoal_xy[1]),
                    yaw=float(leader.yaw),
                    v=0.0,
                    delta=0.0,
                    mode=leader.mode,
                )
            )
        out.append(goal.copy())
        return out

    def _search_segment(
        self,
        *,
        start: VehicleState,
        follower: VehicleState,
        goal: VehicleState,
        lane_heading: float,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
        max_depth: int,
        max_expansions: int,
        goal_pos_tol: float,
        goal_yaw_tol_deg: float,
    ) -> tuple[np.ndarray, np.ndarray, float, int] | None:
        clearance_req = float(max(self.cfg.safety.min_clearance + 0.02, 0.12))
        open_heap: list[_SearchNode] = [
            _SearchNode(
                priority=self._heuristic_cost(start, goal, lane_heading),
                counter=0,
                state=start.copy(),
                depth=0,
                g_cost=0.0,
                min_clearance_m=1e6,
                path_states=tuple(),
                path_commands=tuple(),
            )
        ]
        best_cost_for_key: dict[tuple[int, int, int], float] = {}
        best_terminal: _SearchNode | None = None
        counter = 1
        expansions = 0
        primitives = self._segment_primitives(start=start, goal=goal)
        while open_heap and expansions < max_expansions:
            current_node = heapq.heappop(open_heap)
            expansions += 1
            if float(np.linalg.norm(current_node.state.xy() - goal.xy())) <= float(goal_pos_tol) and abs(float(angle_diff(float(goal.yaw), float(current_node.state.yaw)))) <= math.radians(float(goal_yaw_tol_deg)):
                best_terminal = current_node
                break
            if current_node.depth >= max_depth:
                if best_terminal is None or self._heuristic_cost(current_node.state, goal, lane_heading) < self._heuristic_cost(best_terminal.state, goal, lane_heading):
                    best_terminal = current_node
                continue
            for primitive in primitives:
                rollout = self.motion_library.rollout(start=current_node.state, primitive=primitive)
                safe, rollout_clearance, _reason = self.motion_library.is_safe(
                    rollout=rollout,
                    obstacles=obstacles,
                    corridor_polygons=corridor_polygons,
                    other=follower,
                    min_clearance=clearance_req,
                    in_bounds=self._in_bounds,
                )
                if not safe:
                    continue
                next_state = rollout.end_state.copy()
                state_key = (
                    int(round(float(next_state.x) / 0.22)),
                    int(round(float(next_state.y) / 0.22)),
                    int(round(math.degrees(float(next_state.yaw)) / 18.0)),
                )
                new_cost = float(current_node.g_cost + rollout.path_length_m + primitive.cost_bias + 0.24 * primitive.gear_switches)
                if new_cost >= float(best_cost_for_key.get(state_key, float("inf"))) - 1e-9:
                    continue
                best_cost_for_key[state_key] = float(new_cost)
                next_path_states = tuple([*current_node.path_states, *[tuple(state_row) for state_row in np.asarray(rollout.states, dtype=float)]])
                next_path_commands = tuple([*current_node.path_commands, *[tuple(command_row) for command_row in np.asarray(rollout.commands, dtype=float)]])
                next_priority = float(new_cost + self._heuristic_cost(next_state, goal, lane_heading))
                heapq.heappush(
                    open_heap,
                    _SearchNode(
                        priority=next_priority,
                        counter=counter,
                        state=next_state,
                        depth=int(current_node.depth + 1),
                        g_cost=float(new_cost),
                        min_clearance_m=float(min(current_node.min_clearance_m, rollout_clearance)),
                        path_states=next_path_states,
                        path_commands=next_path_commands,
                    ),
                )
                counter += 1
        if best_terminal is None or not best_terminal.path_states or not best_terminal.path_commands:
            return None
        acceptance_radius = 0.95 if float(goal_yaw_tol_deg) >= 24.0 else 0.70
        if float(np.linalg.norm(best_terminal.state.xy() - goal.xy())) > float(acceptance_radius):
            return None
        return (
            np.asarray(best_terminal.path_states, dtype=float),
            np.asarray(best_terminal.path_commands, dtype=float),
            float(best_terminal.min_clearance_m),
            int(expansions),
        )

    def _search_leader_anchor(
        self,
        *,
        leader: VehicleState,
        follower: VehicleState,
        goal: VehicleState,
        lane: dict[str, Any],
        lane_heading: float,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
    ) -> tuple[np.ndarray, np.ndarray, float, int] | None:
        current_state = leader.copy()
        aggregated_states: list[np.ndarray] = []
        aggregated_commands: list[np.ndarray] = []
        min_clearance_m = 1e6
        total_expansions = 0
        subgoals = self._guided_subgoals(lane=lane, leader=leader, goal=goal)
        for segment_index, subgoal in enumerate(subgoals):
            segment_result = self._search_segment(
                start=current_state,
                follower=follower,
                goal=subgoal,
                lane_heading=lane_heading,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
                max_depth=3 if segment_index < len(subgoals) - 1 else 4,
                max_expansions=180 if segment_index < len(subgoals) - 1 else 260,
                goal_pos_tol=0.26 if segment_index < len(subgoals) - 1 else 0.30,
                goal_yaw_tol_deg=28.0 if segment_index < len(subgoals) - 1 else 18.0,
            )
            if segment_result is None:
                return None
            segment_states, segment_commands, segment_clearance, segment_expansions = segment_result
            aggregated_states.append(np.asarray(segment_states, dtype=float))
            aggregated_commands.append(np.asarray(segment_commands, dtype=float))
            current_state = VehicleState(
                vehicle_id=int(leader.vehicle_id),
                x=float(segment_states[-1, 0]),
                y=float(segment_states[-1, 1]),
                yaw=float(segment_states[-1, 2]),
                v=0.0,
                delta=0.0,
                mode=leader.mode,
            )
            min_clearance_m = float(min(min_clearance_m, segment_clearance))
            total_expansions += int(segment_expansions)
        if not aggregated_states or not aggregated_commands:
            return None
        primitive_states = np.vstack(aggregated_states)
        primitive_commands = np.vstack(aggregated_commands)
        if float(np.linalg.norm(current_state.xy() - goal.xy())) > 0.48:
            return None
        return primitive_states, primitive_commands, float(min_clearance_m), int(total_expansions)

    def _legacy_plan(
        self,
        *,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
        lane: dict[str, Any],
    ) -> CorridorReciprocalPlan | None:
        lane_y = self._lane_y(lane)
        branch_sign = 1.0 if float(leader.y - lane_y) >= 0.0 else -1.0
        anchor_xy = np.asarray(lane.get("leader_stage_anchor_xy", [leader.x, lane_y]), dtype=float)
        anchor_yaw = float(lane.get("leader_stage_anchor_yaw", 0.0))
        leader_goal = VehicleState(
            vehicle_id=int(leader.vehicle_id),
            x=float(anchor_xy[0]),
            y=float(lane_y),
            yaw=float(anchor_yaw),
            v=0.0,
            delta=0.0,
            mode=leader.mode,
        )
        predock = self._predock_pose(leader_goal, follower_id=int(follower.vehicle_id), follower_mode=follower.mode)
        if self.collision.in_collision(leader_goal, obstacles, []):
            return None
        if self.collision.in_collision(predock, obstacles, [leader_goal]):
            return None
        reverse_waypoint_xy = np.array([float(anchor_xy[0]), float(lane_y + branch_sign * 0.28)], dtype=float)
        reverse_path_xy = np.asarray([[float(leader.x), float(leader.y)], [float(reverse_waypoint_xy[0]), float(reverse_waypoint_xy[1])]], dtype=float)
        settle_path_xy = np.asarray([[float(reverse_waypoint_xy[0]), float(reverse_waypoint_xy[1])], [float(leader_goal.x), float(leader_goal.y)]], dtype=float)
        leader_path_xy = np.vstack([self._densify_path(reverse_path_xy, step=0.12), self._densify_path(settle_path_xy, step=0.12)[1:]])
        follower_path_xy = self._planner(obstacles).plan(start_xy=follower.xy(), goal_xy=predock.xy())
        if follower_path_xy is None:
            return None
        follower_path_xy = self._densify_path(np.asarray(follower_path_xy, dtype=float), step=0.12)
        follower_clearance = self._path_min_clearance(
            follower_path_xy,
            proto=follower,
            goal_yaw=float(predock.yaw),
            obstacles=obstacles,
            corridor_polygons=self._corridor_polygons(lane),
            other=leader_goal,
        )
        if follower_clearance < max(float(self.cfg.safety.min_clearance) + 0.02, 0.12):
            return None
        score = float(np.linalg.norm(leader.xy() - leader_goal.xy()) + 1.15 * np.linalg.norm(follower.xy() - predock.xy()))
        score += 0.35 * abs(float(angle_diff(float(leader.yaw), float(leader_goal.yaw))))
        return CorridorReciprocalPlan(
            leader_goal=leader_goal,
            follower_goal=predock,
            leader_path_xy=leader_path_xy.astype(float),
            follower_path_xy=follower_path_xy.astype(float),
            leader_primitive_states=None,
            leader_primitive_cmd=None,
            leader_primitive_steps=0,
            score=float(score),
            reason="lc_corridor_legacy",
            metadata={
                "lane_y": float(lane_y),
                "corridor_polygons": [np.asarray(poly, dtype=float).tolist() for poly in self._corridor_polygons(lane)],
                "branch_sign": float(branch_sign),
                "reverse_waypoint_xy": [float(reverse_waypoint_xy[0]), float(reverse_waypoint_xy[1])],
                "reverse_target_speed": -0.20,
                "settle_target_speed": 0.14,
                "follower_target_speed": 0.22,
                "sigma_c": 0.0,
                "sigma_geom": 0.0,
                "sigma_reach": 0.0,
                "release_sigma_threshold": 0.52,
                "exec_release_threshold": 0.30,
                "exec_robust_threshold": 0.20,
            },
        )

    def plan(
        self,
        *,
        obstacles: list[Obstacle],
        leader: VehicleState,
        follower: VehicleState,
        lane: dict[str, Any],
        use_hybrid_search: bool = True,
    ) -> CorridorReciprocalPlan | None:
        if not (bool(lane.get("enabled", False)) and str(lane.get("category", "")).upper() == "LC"):
            return None
        if not bool(use_hybrid_search):
            return self._legacy_plan(obstacles=obstacles, leader=leader, follower=follower, lane=lane)
        corridor_polygons = self._corridor_polygons(lane)
        planner = self._planner(obstacles)
        macro_corridor_info = {
            "lane_y": float(self._lane_y(lane)),
            "lane_heading": float(self._lane_heading(lane, default_yaw=float(lane.get("leader_stage_anchor_yaw", leader.yaw)))),
            "anchor_hint_xy": [float(v) for v in np.asarray(lane.get("leader_stage_anchor_xy", [leader.x, leader.y]), dtype=float)],
            **self._macro_metadata(lane=lane, leader_ref=leader),
        }
        if str(lane.get("heading_gap_regime", "")).strip().lower() == "small":
            smallgap_escape_plan = self._fast_small_gap_escape_plan(
                obstacles=obstacles,
                leader=leader,
                follower=follower,
                lane=lane,
                planner=planner,
                corridor_polygons=corridor_polygons,
                time_budget_s=0.10,
            )
            if smallgap_escape_plan is not None:
                return smallgap_escape_plan
        if self.macro_selector.should_prefer_macro_plan(leader=leader, corridor_info=macro_corridor_info):
            macro_escape_plan = self._macro_escape_plan(
                obstacles=obstacles,
                leader=leader,
                follower=follower,
                lane=lane,
                planner=planner,
                corridor_polygons=corridor_polygons,
            )
            if macro_escape_plan is not None:
                return macro_escape_plan
        cert_anchor_plan = self._candidate_terminal_safe_plan(
            obstacles=obstacles,
            leader=leader,
            follower=follower,
            lane=lane,
            planner=planner,
            corridor_polygons=corridor_polygons,
        )
        if cert_anchor_plan is not None:
            return cert_anchor_plan
        escape_shape_plan = self._escape_anchor_plan(
            obstacles=obstacles,
            leader=leader,
            follower=follower,
            lane=lane,
            planner=planner,
            corridor_polygons=corridor_polygons,
            time_budget_s=0.10,
        )
        if escape_shape_plan is not None:
            return escape_shape_plan
        exec_shape_plan = self._search_execution_valid_shape_plan(
            obstacles=obstacles,
            leader=leader,
            follower=follower,
            lane=lane,
            corridor_polygons=corridor_polygons,
        )
        if exec_shape_plan is not None:
            return exec_shape_plan
        semantic_reference_plan = self._semantic_reference_plan(
            obstacles=obstacles,
            leader=leader,
            follower=follower,
            lane=lane,
            planner=planner,
            corridor_polygons=corridor_polygons,
        )
        if semantic_reference_plan is not None:
            return semantic_reference_plan
        semantic_plan = self._semantic_projection_plan(
            obstacles=obstacles,
            leader=leader,
            follower=follower,
            lane=lane,
            planner=planner,
            corridor_polygons=corridor_polygons,
        )
        if semantic_plan is not None:
            return semantic_plan
        anchor_hint_xy = np.asarray(lane.get("leader_stage_anchor_xy", [leader.x, self._lane_y(lane)]), dtype=float)
        anchor_hint_yaw = float(lane.get("leader_stage_anchor_yaw", 0.0))
        lane_heading = self._lane_heading(lane, default_yaw=anchor_hint_yaw)
        best_plan: CorridorReciprocalPlan | None = None
        for rank, certificate in enumerate(
            self._candidate_anchor_states(
                leader=leader,
                follower=follower,
                lane=lane,
                obstacles=obstacles,
                planner=planner,
                corridor_polygons=corridor_polygons,
            )
        ):
            search_result = self._search_leader_anchor(
                leader=leader,
                follower=follower,
                goal=certificate.anchor,
                lane=lane,
                lane_heading=lane_heading,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
            )
            if search_result is None:
                continue
            primitive_states, primitive_commands, leader_min_clearance, expansions = search_result
            if len(primitive_states) == 0 or len(primitive_commands) == 0:
                continue
            terminal_state = VehicleState(
                vehicle_id=int(leader.vehicle_id),
                x=float(primitive_states[-1, 0]),
                y=float(primitive_states[-1, 1]),
                yaw=float(primitive_states[-1, 2]),
                v=0.0,
                delta=0.0,
                mode=leader.mode,
            )
            reach_position = float(clamp(1.0 - float(np.linalg.norm(terminal_state.xy() - certificate.anchor.xy())) / 0.55, 0.0, 1.0))
            reach_heading = float(clamp(1.0 - abs(float(angle_diff(float(certificate.anchor.yaw), float(terminal_state.yaw)))) / math.radians(24.0), 0.0, 1.0))
            reach_clearance = float(clamp((float(leader_min_clearance) - (self.cfg.safety.min_clearance + 0.02)) / 0.40, 0.0, 1.0))
            reach_efficiency = float(clamp(1.0 - len(primitive_commands) / 64.0, 0.0, 1.0))
            sigma_reach = 0.32 * reach_position + 0.24 * reach_heading + 0.24 * reach_clearance + 0.20 * reach_efficiency
            sigma_c = float(clamp(0.58 * float(certificate.sigma_geom) + 0.42 * float(sigma_reach), 0.0, 1.0))
            leader_path_xy = np.vstack(
                [
                    np.asarray([[float(leader.x), float(leader.y)]], dtype=float),
                    np.asarray(primitive_states[:, :2], dtype=float),
                    np.asarray([[float(certificate.anchor.x), float(certificate.anchor.y)]], dtype=float),
                ]
            )
            score = float(2.0 - 1.7 * sigma_c + 0.08 * len(primitive_commands) + 0.04 * np.sum(np.linalg.norm(np.diff(certificate.follower_path_xy, axis=0), axis=1)))
            candidate_plan = CorridorReciprocalPlan(
                leader_goal=certificate.anchor.copy(),
                follower_goal=certificate.predock.copy(),
                leader_path_xy=leader_path_xy.astype(float),
                follower_path_xy=np.asarray(certificate.follower_path_xy, dtype=float),
                leader_primitive_states=np.asarray(primitive_states, dtype=float),
                leader_primitive_cmd=np.asarray(primitive_commands, dtype=float),
                leader_primitive_steps=int(len(primitive_commands)),
                score=float(score),
                reason="lc_corridor_hybrid_search",
                metadata={
                    "lane_y": float(self._lane_y(lane)),
                    "anchor_hint_xy": [float(anchor_hint_xy[0]), float(anchor_hint_xy[1])],
                    "anchor_hint_yaw": float(anchor_hint_yaw),
                    "lane_heading": float(lane_heading),
                    "corridor_polygons": [np.asarray(poly, dtype=float).tolist() for poly in corridor_polygons],
                    "sigma_c": float(sigma_c),
                    "sigma_geom": float(certificate.sigma_geom),
                    "sigma_reach": float(sigma_reach),
                    "release_sigma_threshold": 0.52,
                    "leader_path_clearance_m": float(leader_min_clearance),
                    "follower_path_clearance_m": float(certificate.follower_path_clearance_m),
                    "handoff_clearance_m": float(certificate.handoff_clearance_m),
                    "dock_zone_clearance_m": float(certificate.dock_zone_clearance_m),
                    "alignment_quality": float(certificate.alignment_quality),
                    "candidate_rank": int(rank),
                    "search_expansions": int(expansions),
                    "shape_target_speed": 0.18,
                    "settle_target_speed": 0.16,
                    "follower_target_speed": 0.28,
                    "exec_release_threshold": 0.30,
                    "exec_robust_threshold": 0.20,
                    "shape_budget_s": float(max(7.5, 1.0 + 1.15 * len(primitive_commands) * float(self.cfg.control.dt))),
                    "settle_budget_s": 5.0,
                },
            )
            if best_plan is None or float(candidate_plan.score) < float(best_plan.score):
                best_plan = candidate_plan
                if sigma_c >= 0.52:
                    break
        if best_plan is not None:
            return best_plan
        return self._legacy_plan(obstacles=obstacles, leader=leader, follower=follower, lane=lane)


class CorridorReciprocityExecutor:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.model = AckermannModel(cfg.vehicle)
        self.geom = VehicleGeometry(cfg.vehicle)
        self.collision = CollisionEngine(cfg.vehicle, cfg.safety)
        self.motion_library = HybridMotionPrimitiveLibrary(cfg)
        self.analysis_helper = CorridorReciprocityPlanner(cfg, seed=0)
        self.optimizer = StagingCertificateOptimizer(cfg)
        self.macro_selector = self.optimizer
        self.local_planner = LocalPlanner(cfg.vehicle, cfg.planner, self.collision, cfg.control.dt)
        self.path_tracker = PathTrackingController(cfg.vehicle, cfg.control, steering_mode="stanley")
        self._bcfd_cfg = BCFDConfig()
        self.cert_field = UnifiedCertificateField(cfg, planner=self.analysis_helper, executor=self)
        self._phase = "LEADER_SHAPE"
        self._shape_time = 0.0
        self._shape_step = 0
        self._settle_time = 0.0
        self._legacy_reverse_time = 0.0
        self._legacy_reverse_best_dist = float("inf")
        self._legacy_reverse_stall_time = 0.0
        self._reshape_states: np.ndarray | None = None
        self._reshape_cmd: np.ndarray | None = None
        self._reshape_step = 0
        self._reshape_searched = False
        self._reshape_attempts = 0
        self._last_exec_sigma = 0.0
        self._settle_candidate_states: np.ndarray | None = None
        self._settle_candidate_cmd: np.ndarray | None = None
        self._settle_candidate_step = 0
        self._settle_candidate_pose: VehicleState | None = None
        self._settle_candidate_sigma = 0.0
        self._settle_candidate_robust_sigma = 0.0
        self._settle_search_attempts = 0
        self._terminal_closure_best_sigma = 0.0
        self._terminal_best_tail = float("inf")
        self._terminal_stall_time = 0.0
        self._shape_origin: VehicleState | None = None
        self._leader_hold_goal_override: VehicleState | None = None

    def _critical_clearance_threshold(self) -> float:
        return float(max(float(self.cfg.safety.min_clearance) + 0.03, 0.13))

    def reset(self) -> None:
        self._phase = "LEADER_SHAPE"
        self._shape_time = 0.0
        self._shape_step = 0
        self._settle_time = 0.0
        self._legacy_reverse_time = 0.0
        self._legacy_reverse_best_dist = float("inf")
        self._legacy_reverse_stall_time = 0.0
        self._reshape_states: np.ndarray | None = None
        self._reshape_cmd: np.ndarray | None = None
        self._reshape_step = 0
        self._reshape_searched = False
        self._reshape_attempts = 0
        self._last_exec_sigma = 0.0
        self._settle_candidate_states = None
        self._settle_candidate_cmd = None
        self._settle_candidate_step = 0
        self._settle_candidate_pose = None
        self._settle_candidate_sigma = 0.0
        self._settle_candidate_robust_sigma = 0.0
        self._settle_search_attempts = 0
        self._terminal_closure_best_sigma = 0.0
        self._terminal_best_tail = float("inf")
        self._terminal_stall_time = 0.0
        self._shape_origin = None
        self._leader_hold_goal_override = None

    def _freeze_vehicle(self, state: VehicleState) -> ControlCommand:
        if abs(float(state.v)) > 0.03:
            return ControlCommand(accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(state.v))), steer_rate=0.0)
        return ControlCommand(accel=0.0, steer_rate=float(-state.delta / max(float(self.cfg.control.dt), 1e-3)))

    def _near_goal(self, state: VehicleState, goal: VehicleState, *, pos_tol: float, yaw_tol_deg: float) -> bool:
        if float(np.linalg.norm(state.xy() - goal.xy())) > float(pos_tol):
            return False
        return abs(float(angle_diff(float(goal.yaw), float(state.yaw)))) <= math.radians(float(yaw_tol_deg))

    def _in_bounds(self, state: VehicleState, *, margin: float) -> bool:
        half_w = 0.5 * float(self.cfg.environment.width) - float(margin)
        half_h = 0.5 * float(self.cfg.environment.height) - float(margin)
        return (-half_w <= float(state.x) <= half_w) and (-half_h <= float(state.y) <= half_h)

    def _plan_step_signed(
        self,
        *,
        state: VehicleState,
        goal: VehicleState,
        target_speed: float,
        obstacles: list[Obstacle],
        other: VehicleState | None,
        allow_reverse: bool,
    ) -> ControlCommand:
        result = self.local_planner.plan_step(
            ego=state,
            goal_xy=goal.xy(),
            goal_yaw=float(goal.yaw),
            obstacles=obstacles,
            dynamic_others=[other] if other is not None else [],
            force_goal_yaw=bool(np.linalg.norm(state.xy() - goal.xy()) <= 0.85),
            target_speed=float(target_speed),
            allow_reverse=bool(allow_reverse),
        )
        command = result.command if result.feasible else self._freeze_vehicle(state)
        next_state = self.model.step(state, command, float(self.cfg.control.dt))
        if self.collision.in_collision(next_state, obstacles, [other] if other is not None else []):
            return self._freeze_vehicle(state)
        obstacle_clearance = float(self.collision.min_clearance_vehicle_obstacles(next_state, obstacles)) if obstacles else 1e6
        vehicle_clearance = float(self.collision.min_clearance_vehicle_vehicle(next_state, other)) if other is not None else 1e6
        if float(min(obstacle_clearance, vehicle_clearance)) < float(self.cfg.safety.min_clearance) + 0.005:
            return self._freeze_vehicle(state)
        return command

    def _guarded_track(
        self,
        *,
        state: VehicleState,
        path_xy: np.ndarray,
        goal: VehicleState,
        target_speed: float,
        obstacles: list[Obstacle],
        other: VehicleState | None,
    ) -> ControlCommand:
        command = self.path_tracker.track_path(state, np.asarray(path_xy, dtype=float), target_speed=float(target_speed))
        next_state = self.model.step(state, command, float(self.cfg.control.dt))
        if self.collision.in_collision(next_state, obstacles, [other] if other is not None else []):
            return self._freeze_vehicle(state)
        obstacle_clearance = float(self.collision.min_clearance_vehicle_obstacles(next_state, obstacles)) if obstacles else 1e6
        vehicle_clearance = float(self.collision.min_clearance_vehicle_vehicle(next_state, other)) if other is not None else 1e6
        if float(min(obstacle_clearance, vehicle_clearance)) < float(self.cfg.safety.min_clearance) + 0.005:
            return self._freeze_vehicle(state)
        return command

    def _reverse_goal(self, plan) -> VehicleState:
        metadata = dict(getattr(plan, "metadata", {}) or {})
        reverse_xy = np.asarray(metadata.get("reverse_waypoint_xy", [plan.leader_goal.x, plan.leader_goal.y]), dtype=float)
        return VehicleState(
            vehicle_id=int(plan.leader_goal.vehicle_id),
            x=float(reverse_xy[0]),
            y=float(reverse_xy[1]),
            yaw=float(plan.leader_goal.yaw),
            v=0.0,
            delta=0.0,
            mode=plan.leader_goal.mode,
        )

    def _dynamic_predock_goal(self, *, leader: VehicleState, follower: VehicleState) -> VehicleState:
        heading_xy = np.array([math.cos(float(leader.yaw)), math.sin(float(leader.yaw))], dtype=float)
        rear_hitch_xy = self.geom.rear_hitch(leader)
        standoff_m = float(max(0.78, float(self.cfg.docking.stage_switch_distance) - 0.42))
        center_xy = rear_hitch_xy - heading_xy * float(self.geom.front_hitch_x + standoff_m)
        return VehicleState(
            vehicle_id=int(follower.vehicle_id),
            x=float(center_xy[0]),
            y=float(center_xy[1]),
            yaw=float(leader.yaw),
            v=0.0,
            delta=0.0,
            mode=follower.mode,
        )

    def _corridor_polygons_from_info(self, corridor_info: dict[str, Any]) -> list[np.ndarray]:
        return [np.asarray(poly, dtype=float) for poly in corridor_info.get("corridor_polygons", []) if isinstance(poly, list)]

    def _leader_frame_relative_state(self, *, leader: VehicleState, follower: VehicleState) -> tuple[float, float, float]:
        rear_hitch_xy = self.geom.rear_hitch(leader)
        front_hitch_xy = self.geom.front_hitch(follower)
        rel_xy = rear_hitch_xy - front_hitch_xy
        leader_cos = math.cos(float(leader.yaw))
        leader_sin = math.sin(float(leader.yaw))
        longitudinal_error_m = float(leader_cos * float(rel_xy[0]) + leader_sin * float(rel_xy[1]))
        lateral_error_m = float(-leader_sin * float(rel_xy[0]) + leader_cos * float(rel_xy[1]))
        yaw_gap_rad = float(angle_diff(float(leader.yaw), float(follower.yaw)))
        return float(longitudinal_error_m), float(lateral_error_m), float(yaw_gap_rad)

    def _terminal_closure_primitives(self) -> tuple[MotionPrimitive, ...]:
        candidate_names = {
            "forward_straight_short",
            "forward_arc_left",
            "forward_arc_right",
            "forward_arc_left_tight",
            "forward_arc_right_tight",
            "reverse_straight_short",
            "reverse_arc_left_tight",
            "reverse_arc_right_tight",
        }
        return tuple(primitive for primitive in self.motion_library.primitives if primitive.name in candidate_names)

    def _direct_path_xy(self, *, start_xy: np.ndarray, goal_xy: np.ndarray, step_m: float = 0.10) -> np.ndarray:
        delta_xy = np.asarray(goal_xy, dtype=float) - np.asarray(start_xy, dtype=float)
        distance_m = float(np.linalg.norm(delta_xy))
        if distance_m <= 1e-9:
            return np.asarray([np.asarray(start_xy, dtype=float), np.asarray(goal_xy, dtype=float)], dtype=float)
        num_points = max(2, int(math.ceil(distance_m / max(step_m, 1e-3))) + 1)
        return np.linspace(np.asarray(start_xy, dtype=float), np.asarray(goal_xy, dtype=float), num_points, dtype=float)

    def _leader_settle_command(self, *, leader: VehicleState, goal: VehicleState, obstacles: list[Obstacle], follower: VehicleState, target_speed: float) -> ControlCommand:
        goal_vec = goal.xy() - leader.xy()
        forward_axis = np.array([math.cos(float(goal.yaw)), math.sin(float(goal.yaw))], dtype=float)
        signed_progress = float(np.dot(goal_vec, forward_axis))
        signed_speed = float(abs(float(target_speed)) if signed_progress >= 0.0 else -abs(float(target_speed)))
        return self._plan_step_signed(
            state=leader,
            goal=goal,
            target_speed=float(signed_speed),
            obstacles=obstacles,
            other=follower,
            allow_reverse=True,
        )

    def _leader_anchor_hold(self, *, leader: VehicleState, goal: VehicleState, obstacles: list[Obstacle], follower: VehicleState) -> ControlCommand:
        if self._near_goal(leader, goal, pos_tol=0.18, yaw_tol_deg=10.0):
            return self._freeze_vehicle(leader)
        return self._leader_settle_command(
            leader=leader,
            goal=goal,
            obstacles=obstacles,
            follower=follower,
            target_speed=0.10,
        )

    def _leader_active_goal(self, plan) -> VehicleState:
        return plan.leader_goal

    def compute_execution_certificate(
        self,
        *,
        leader_actual_pose: VehicleState,
        follower_pose: VehicleState,
        corridor_info: dict[str, Any],
        obstacles: list[Obstacle],
    ) -> tuple[float, dict[str, Any]]:
        corridor_polygons = [np.asarray(poly, dtype=float) for poly in corridor_info.get("corridor_polygons", []) if isinstance(poly, list)]
        predock_goal = self._dynamic_predock_goal(leader=leader_actual_pose, follower=follower_pose)
        if self.collision.in_collision(predock_goal, obstacles, [leader_actual_pose]):
            return 0.0, {
                "valid": False,
                "reason": "predock_collision",
                "predock_xy": [float(predock_goal.x), float(predock_goal.y)],
                "predock_yaw": float(predock_goal.yaw),
            }
        follower_path_xy = self._direct_path_xy(start_xy=follower_pose.xy(), goal_xy=predock_goal.xy(), step_m=0.10)
        follower_path_clearance_m = self.analysis_helper._path_min_clearance(
            follower_path_xy,
            proto=follower_pose,
            goal_yaw=float(predock_goal.yaw),
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
            other=leader_actual_pose,
        )
        path_mode = "direct"
        if follower_path_clearance_m < float(self.cfg.safety.min_clearance):
            planner = GridAStarPlanner(
                width=float(self.cfg.environment.width),
                height=float(self.cfg.environment.height),
                obstacles=obstacles,
                cfg=GridPlannerConfig(resolution=0.16, inflation_radius=0.52, boundary_block=1, max_expansions=50_000),
            )
            follower_path_xy = planner.plan(start_xy=follower_pose.xy(), goal_xy=predock_goal.xy())
            if follower_path_xy is None:
                return 0.0, {
                    "valid": False,
                    "reason": "predock_unreachable",
                    "predock_xy": [float(predock_goal.x), float(predock_goal.y)],
                    "predock_yaw": float(predock_goal.yaw),
                }
            follower_path_xy = self.analysis_helper._densify_path(np.asarray(follower_path_xy, dtype=float), step=0.10)
            follower_path_clearance_m = self.analysis_helper._path_min_clearance(
                follower_path_xy,
                proto=follower_pose,
                goal_yaw=float(predock_goal.yaw),
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
                other=leader_actual_pose,
            )
            path_mode = "grid"
            if follower_path_clearance_m < float(self.cfg.safety.min_clearance):
                return 0.0, {
                    "valid": False,
                    "reason": "path_clearance",
                    "predock_xy": [float(predock_goal.x), float(predock_goal.y)],
                    "predock_yaw": float(predock_goal.yaw),
                    "follower_path_clearance_m": float(follower_path_clearance_m),
                }
        anchor_clearance_m = float(self.collision.min_clearance_vehicle_obstacles(leader_actual_pose, obstacles))
        handoff_clearance_m = float(
            min(
                self.collision.min_clearance_vehicle_obstacles(predock_goal, obstacles),
                self.collision.min_clearance_vehicle_vehicle(predock_goal, leader_actual_pose),
            )
        )
        dock_zone_clearance_m = float(self.analysis_helper._dock_zone_clearance(leader_actual_pose, obstacles))
        robust_clearance_floor_m = float(self.cfg.safety.min_clearance) + 0.02
        if min(anchor_clearance_m, handoff_clearance_m, dock_zone_clearance_m) < robust_clearance_floor_m:
            return 0.0, {
                "valid": False,
                "reason": "basin_clearance_floor",
                "predock_xy": [float(predock_goal.x), float(predock_goal.y)],
                "predock_yaw": float(predock_goal.yaw),
                "anchor_clearance_m": float(anchor_clearance_m),
                "handoff_clearance_m": float(handoff_clearance_m),
                "dock_zone_clearance_m": float(dock_zone_clearance_m),
            }
        rear_hitch_xy = self.geom.rear_hitch(leader_actual_pose)
        front_hitch_xy = self.geom.front_hitch(follower_pose)
        rel_xy = rear_hitch_xy - front_hitch_xy
        leader_cos = math.cos(float(leader_actual_pose.yaw))
        leader_sin = math.sin(float(leader_actual_pose.yaw))
        longitudinal_error_m = float(leader_cos * float(rel_xy[0]) + leader_sin * float(rel_xy[1]))
        lateral_error_m = float(-leader_sin * float(rel_xy[0]) + leader_cos * float(rel_xy[1]))
        yaw_gap_rad = abs(float(angle_diff(float(leader_actual_pose.yaw), float(follower_pose.yaw))))
        path_terminal_yaw = self.analysis_helper._path_terminal_yaw(follower_path_xy, goal_yaw=float(predock_goal.yaw))
        approach_alignment = float(clamp(1.0 - abs(float(angle_diff(float(predock_goal.yaw), float(path_terminal_yaw)))) / math.radians(35.0), 0.0, 1.0))
        terminal_alignment = float(
            clamp(
                1.0
                - (
                    0.52 * abs(float(longitudinal_error_m)) / 1.0
                    + 0.28 * abs(float(lateral_error_m)) / 0.35
                    + 0.20 * yaw_gap_rad / math.radians(20.0)
                ),
                0.0,
                1.0,
            )
        )
        sigma_exec = float(
            clamp(
                0.32 * float(clamp((follower_path_clearance_m - float(self.cfg.safety.min_clearance)) / 0.45, 0.0, 1.0))
                + 0.20 * float(clamp((handoff_clearance_m - float(self.cfg.safety.min_clearance)) / 0.35, 0.0, 1.0))
                + 0.16 * float(clamp((dock_zone_clearance_m - float(self.cfg.safety.min_clearance)) / 0.35, 0.0, 1.0))
                + 0.14 * float(clamp((anchor_clearance_m - float(self.cfg.safety.min_clearance)) / 0.40, 0.0, 1.0))
                + 0.10 * float(approach_alignment)
                + 0.08 * float(terminal_alignment),
                0.0,
                1.0,
            )
        )
        return sigma_exec, {
            "valid": True,
            "path_mode": str(path_mode),
            "predock_xy": [float(predock_goal.x), float(predock_goal.y)],
            "predock_yaw": float(predock_goal.yaw),
            "follower_path_xy": np.asarray(follower_path_xy, dtype=float).tolist(),
            "follower_path_clearance_m": float(follower_path_clearance_m),
            "handoff_clearance_m": float(handoff_clearance_m),
            "dock_zone_clearance_m": float(dock_zone_clearance_m),
            "anchor_clearance_m": float(anchor_clearance_m),
            "longitudinal_error_m": float(longitudinal_error_m),
            "lateral_error_m": float(lateral_error_m),
            "yaw_gap_deg": float(math.degrees(yaw_gap_rad)),
            "approach_alignment": float(approach_alignment),
            "terminal_alignment": float(terminal_alignment),
        }

    def _approx_terminal_safety_certificate(
        self,
        *,
        anchor: VehicleState,
        follower: VehicleState,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
    ) -> tuple[float, dict[str, Any]]:
        return self.analysis_helper._approx_terminal_safety_certificate(
            anchor=anchor,
            follower=follower,
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
        )

    def _robust_terminal_safety_certificate(
        self,
        *,
        anchor: VehicleState,
        follower: VehicleState,
        obstacles: list[Obstacle],
        corridor_polygons: list[np.ndarray],
    ) -> tuple[float, float, dict[str, Any]]:
        return self.analysis_helper._robust_terminal_safety_certificate(
            anchor=anchor,
            follower=follower,
            obstacles=obstacles,
            corridor_polygons=corridor_polygons,
        )

    def _perturb_leader_pose(self, *, leader_pose: VehicleState, dx_body_m: float, dy_body_m: float, dyaw_deg: float) -> VehicleState:
        offset_xy = self.geom.to_world(leader_pose, float(dx_body_m), float(dy_body_m)) - leader_pose.xy()
        return VehicleState(
            vehicle_id=int(leader_pose.vehicle_id),
            x=float(leader_pose.x + offset_xy[0]),
            y=float(leader_pose.y + offset_xy[1]),
            yaw=float(wrap_angle(float(leader_pose.yaw) + math.radians(float(dyaw_deg)))),
            v=float(leader_pose.v),
            delta=float(leader_pose.delta),
            mode=leader_pose.mode,
        )

    def _robust_execution_certificate(
        self,
        *,
        leader_pose: VehicleState,
        follower_pose: VehicleState,
        corridor_info: dict[str, Any],
        obstacles: list[Obstacle],
    ) -> tuple[float, float, dict[str, Any]]:
        perturbations = [
            (0.0, 0.0, 0.0),
            (0.04, 0.0, 0.0),
            (-0.04, 0.0, 0.0),
            (0.0, 0.04, 0.0),
            (0.0, -0.04, 0.0),
            (0.0, 0.0, 3.0),
            (0.0, 0.0, -3.0),
        ]
        nominal_sigma, nominal_meta = self.compute_execution_certificate(
            leader_actual_pose=leader_pose,
            follower_pose=follower_pose,
            corridor_info=corridor_info,
            obstacles=obstacles,
        )
        robust_sigma = float(nominal_sigma)
        for dx_body_m, dy_body_m, dyaw_deg in perturbations[1:]:
            perturbed_pose = self._perturb_leader_pose(
                leader_pose=leader_pose,
                dx_body_m=float(dx_body_m),
                dy_body_m=float(dy_body_m),
                dyaw_deg=float(dyaw_deg),
            )
            sigma_value, _ = self.compute_execution_certificate(
                leader_actual_pose=perturbed_pose,
                follower_pose=follower_pose,
                corridor_info=corridor_info,
                obstacles=obstacles,
            )
            robust_sigma = float(min(robust_sigma, sigma_value))
            if robust_sigma <= 0.0:
                break
        return float(nominal_sigma), float(robust_sigma), dict(nominal_meta)

    def compute_terminal_closure_certificate(
        self,
        *,
        leader_pose: VehicleState,
        follower_pose: VehicleState,
        corridor_info: dict[str, Any],
        obstacles: list[Obstacle],
        estimate_witness: bool = True,
    ) -> tuple[float, dict[str, Any]]:
        corridor_polygons = self._corridor_polygons_from_info(corridor_info)
        contact_center_xy = self.analysis_helper._terminal_contact_center(anchor=leader_pose)
        contact_goal = VehicleState(
            vehicle_id=int(follower_pose.vehicle_id),
            x=float(contact_center_xy[0]),
            y=float(contact_center_xy[1]),
            yaw=float(leader_pose.yaw),
            v=0.0,
            delta=0.0,
            mode=follower_pose.mode,
        )
        x_l_m, y_l_m, yaw_gap_rad = self._leader_frame_relative_state(leader=leader_pose, follower=follower_pose)
        axial_score = float(clamp((2.05 - max(float(x_l_m), 0.0)) / 1.55, 0.0, 1.0))
        lateral_score = float(clamp(1.0 - abs(float(y_l_m)) / 0.34, 0.0, 1.0))
        yaw_score = float(clamp(1.0 - abs(float(yaw_gap_rad)) / math.radians(32.0), 0.0, 1.0))
        obstacle_clearance_m = float(self.collision.min_clearance_vehicle_obstacles(follower_pose, obstacles))
        leader_clearance_m = float(self.collision.min_clearance_vehicle_obstacles(leader_pose, obstacles))
        pair_clearance_m = float(self.collision.min_clearance_vehicle_vehicle(follower_pose, leader_pose))
        terminal_path_clearance_m = float(min(obstacle_clearance_m, leader_clearance_m, pair_clearance_m))
        tube_score = float(clamp((terminal_path_clearance_m - float(self.cfg.safety.min_clearance)) / 0.22, 0.0, 1.0))
        sigma_geom = float(clamp(0.38 * axial_score + 0.34 * lateral_score + 0.28 * yaw_score, 0.0, 1.0))
        sigma_base = float(clamp(0.72 * sigma_geom + 0.28 * tube_score, 0.0, 1.0))
        sigma_terminal = float(sigma_base)
        witness_sigma = float(sigma_base)
        witness_name = "hold"
        if estimate_witness and sigma_terminal < 0.55:
            for primitive in self._terminal_closure_primitives():
                rollout = self.motion_library.rollout(start=follower_pose, primitive=primitive)
                safe, rollout_clearance, _ = self.motion_library.is_safe(
                    rollout=rollout,
                    obstacles=obstacles,
                    corridor_polygons=corridor_polygons,
                    other=leader_pose,
                    min_clearance=float(self.cfg.safety.min_clearance),
                    in_bounds=self._in_bounds,
                )
                if not safe:
                    continue
                sigma_candidate, candidate_meta = self.compute_terminal_closure_certificate(
                    leader_pose=leader_pose,
                    follower_pose=rollout.end_state,
                    corridor_info=corridor_info,
                    obstacles=obstacles,
                    estimate_witness=False,
                )
                candidate_score = float(
                    0.84 * sigma_candidate
                    + 0.16 * clamp(
                        (float(rollout_clearance) - float(self.cfg.safety.min_clearance)) / 0.18,
                        0.0,
                        1.0,
                    )
                )
                if candidate_score > witness_sigma + 1e-6:
                    witness_sigma = float(candidate_score)
                    witness_name = str(primitive.name)
            sigma_terminal = float(clamp(0.62 * sigma_base + 0.38 * witness_sigma, 0.0, 1.0))
        certificate = TerminalClosureCertificate(
            sigma=float(sigma_terminal),
            tube_clearance_m=float(terminal_path_clearance_m),
            axial_score=float(axial_score),
            lateral_score=float(lateral_score),
            yaw_score=float(yaw_score),
            x_l_m=float(x_l_m),
            y_l_m=float(y_l_m),
            yaw_gap_deg=float(math.degrees(float(yaw_gap_rad))),
            witness_sigma=float(witness_sigma),
            witness_name=str(witness_name),
            metadata={
                "contact_goal_xy": [float(contact_goal.x), float(contact_goal.y)],
                "contact_goal_yaw": float(contact_goal.yaw),
                "sigma_geom": float(sigma_geom),
                "tube_score": float(tube_score),
                "sigma_base": float(sigma_base),
            },
        )
        return certificate.sigma, {
            "valid": bool(certificate.sigma > 0.0),
            "sigma_terminal": float(certificate.sigma),
            "tube_clearance_m": float(certificate.tube_clearance_m),
            "axial_score": float(certificate.axial_score),
            "lateral_score": float(certificate.lateral_score),
            "yaw_score": float(certificate.yaw_score),
            "x_l_m": float(certificate.x_l_m),
            "y_l_m": float(certificate.y_l_m),
            "yaw_gap_deg": float(certificate.yaw_gap_deg),
            "witness_sigma": float(certificate.witness_sigma),
            "witness_name": str(certificate.witness_name),
            **dict(certificate.metadata),
        }

    def _terminal_closure_ready(self, *, sigma_terminal: float, terminal_meta: dict[str, Any], threshold: float) -> bool:
        axial_error_m = max(float(terminal_meta.get("x_l_m", 1e6)), 0.0)
        lateral_error_m = abs(float(terminal_meta.get("y_l_m", 1e6)))
        yaw_gap_deg = abs(float(terminal_meta.get("yaw_gap_deg", 1e6)))
        clearance_m = float(terminal_meta.get("tube_clearance_m", -1.0))
        tight_clearance = bool(float(terminal_meta.get("tube_clearance_m", -1.0)) < float(self.cfg.safety.min_clearance) + 0.09)
        if (
            axial_error_m <= float(self.cfg.docking.lock_position_tol) + 0.06
            and lateral_error_m <= 0.04
            and yaw_gap_deg <= 3.0
            and clearance_m >= float(self.cfg.safety.min_clearance) + 0.02
        ):
            return True
        lateral_limit_m = 0.06 if tight_clearance else 0.30
        yaw_limit_deg = 3.0 if tight_clearance else 22.0
        axial_limit_m = 0.12 if tight_clearance else 1.60
        sigma_limit = max(float(threshold), 0.86 if tight_clearance else float(threshold))
        return bool(
            float(sigma_terminal) >= sigma_limit
            and axial_error_m <= axial_limit_m
            and lateral_error_m <= lateral_limit_m
            and yaw_gap_deg <= yaw_limit_deg
            and clearance_m >= float(self.cfg.safety.min_clearance) + 0.01
        )

    def _terminal_lock_ready_state(self, *, leader: VehicleState, follower: VehicleState, obstacles: list[Obstacle]) -> bool:
        axial_error_m, lateral_error_m, yaw_gap_rad = self._leader_frame_relative_state(leader=leader, follower=follower)
        clearance_m = float(
            min(
                self.collision.min_clearance_vehicle_obstacles(follower, obstacles),
                self.collision.min_clearance_vehicle_obstacles(leader, obstacles),
            )
        )
        return bool(
            max(float(axial_error_m), 0.0) <= float(self.cfg.docking.lock_position_tol) + 0.06
            and abs(float(lateral_error_m)) <= 0.04
            and abs(float(math.degrees(yaw_gap_rad))) <= 3.0
            and clearance_m >= float(self.cfg.safety.min_clearance) + 0.02
        )

    def terminal_lock_ready(self, *, leader: VehicleState, follower: VehicleState, obstacles: list[Obstacle]) -> bool:
        return self._terminal_lock_ready_state(leader=leader, follower=follower, obstacles=obstacles)

    def _handoff_release_gain_ok(self, *, sigma_terminal: float, terminal_meta: dict[str, Any]) -> bool:
        witness_sigma = float(terminal_meta.get("witness_sigma", sigma_terminal))
        return bool(witness_sigma - float(sigma_terminal) >= 0.05)

    def _bcfd_handoff_command(self, *, follower: VehicleState, leader: VehicleState) -> ControlCommand:
        cfg = self._bcfd_cfg
        rear_target_xy = self.geom.rear_hitch(leader)
        fh = self.geom.front_hitch(follower)
        rel_world = np.asarray(rear_target_xy, dtype=float) - fh
        lc = math.cos(float(leader.yaw))
        ls = math.sin(float(leader.yaw))
        x_l = float(lc * float(rel_world[0]) + ls * float(rel_world[1]))
        y_l = float(-ls * float(rel_world[0]) + lc * float(rel_world[1]))
        yaw_diff = float(angle_diff(float(leader.yaw), float(follower.yaw)))
        yaw_err = abs(float(yaw_diff))
        dist = float(np.linalg.norm(rel_world))

        if dist > float(cfg.align_enter_dist_m):
            x_ref = float(clamp(0.70 + 0.18 * dist, float(cfg.standoff_approach_min), float(cfg.standoff_approach_max)))
            v_cap = float(cfg.v_cap_approach)
        elif dist > float(cfg.dock_enter_dist_m):
            x_ref = float(cfg.standoff_align)
            v_cap = float(cfg.v_cap_align)
        else:
            x_ref = 0.0
            v_cap = float(cfg.v_cap_dock)

        v_ref = float(leader.v) + float(cfg.k_x) * float(x_l - x_ref)
        v_ref = float(clamp(v_ref, -float(self.cfg.vehicle.max_reverse_speed), float(v_cap)))
        if dist <= float(cfg.gate_contact_dist_m):
            if abs(float(y_l)) > float(cfg.gate_lat_m) or yaw_err > math.radians(float(cfg.gate_yaw_deg)):
                v_ref = float(min(v_ref, float(cfg.gate_backoff_speed)))
        if dist <= 1.0 and yaw_err > math.radians(22.0):
            v_ref = float(min(v_ref, -0.18))
        if dist <= 0.9 and abs(float(y_l)) > 0.20:
            v_ref = float(min(v_ref, -0.14))

        v_term = abs(float(follower.v)) + float(cfg.stanley_softening)
        delta_ref = float(yaw_diff + math.atan2(float(cfg.k_cte) * float(y_l), float(v_term)))
        dmax = math.radians(float(self.cfg.vehicle.max_steer_deg))
        delta_ref = float(clamp(delta_ref, -dmax, dmax))
        max_rate = math.radians(float(self.cfg.vehicle.max_steer_rate_deg_s))
        steer_rate = float(clamp((delta_ref - float(follower.delta)) / max(float(self.cfg.control.dt), 1e-3), -max_rate, max_rate))
        accel = float(
            clamp(
                float(self.cfg.control.speed.kp) * (float(v_ref) - float(follower.v)),
                -float(self.cfg.vehicle.max_decel),
                float(self.cfg.control.speed.max_accel_cmd),
            )
        )
        return ControlCommand(accel=accel, steer_rate=steer_rate)

    def _docking_handoff_ready(
        self,
        *,
        leader: VehicleState,
        follower: VehicleState,
        obstacles: list[Obstacle],
    ) -> tuple[bool, dict[str, Any]]:
        leader_roll = leader.copy()
        follower_roll = follower.copy()
        min_clearance_m = float(
            min(
                self.collision.min_clearance_vehicle_obstacles(follower_roll, obstacles),
                self.collision.min_clearance_vehicle_obstacles(leader_roll, obstacles),
            )
        )
        tail0 = float(np.linalg.norm(self.geom.front_hitch(follower_roll) - self.geom.rear_hitch(leader_roll)))
        tail_best = float(tail0)
        horizon_steps = 24
        safety_margin = float(self.cfg.safety.min_clearance) + 0.01
        for _ in range(horizon_steps):
            cmd = self._bcfd_handoff_command(follower=follower_roll, leader=leader_roll)
            if abs(float(leader_roll.v)) > 0.02:
                leader_cmd = ControlCommand(
                    accel=float(-math.copysign(float(self.cfg.vehicle.max_decel), float(leader_roll.v))),
                    steer_rate=0.0,
                )
            else:
                leader_cmd = ControlCommand(accel=0.0, steer_rate=float(-leader_roll.delta / max(float(self.cfg.control.dt), 1e-3)))
            leader_next = self.model.step(leader_roll, leader_cmd, float(self.cfg.control.dt))
            follower_next = self.model.step(follower_roll, cmd, float(self.cfg.control.dt))
            obstacle_clearance = float(self.collision.min_clearance_vehicle_obstacles(follower_next, obstacles))
            leader_obstacle_clearance = float(self.collision.min_clearance_vehicle_obstacles(leader_next, obstacles))
            vehicle_clearance = float(self.collision.min_clearance_vehicle_vehicle(follower_next, leader_next))
            current_clearance = float(min(obstacle_clearance, vehicle_clearance))
            min_clearance_m = float(min(min_clearance_m, current_clearance, leader_obstacle_clearance))
            d_tail = float(np.linalg.norm(self.geom.front_hitch(follower_next) - self.geom.rear_hitch(leader_next)))
            tail_best = float(min(tail_best, d_tail))
            if self.collision.in_collision(follower_next, obstacles, [leader_next]) or self.collision.in_collision(leader_next, obstacles, []):
                return False, {
                    "reason": "horizon_collision",
                    "predicted_min_clearance_m": float(min_clearance_m),
                    "predicted_best_tail_m": float(tail_best),
                }
            if current_clearance < safety_margin:
                return False, {
                    "reason": "horizon_clearance",
                    "predicted_min_clearance_m": float(min_clearance_m),
                    "predicted_best_tail_m": float(tail_best),
                }
            if leader_obstacle_clearance < safety_margin:
                return False, {
                    "reason": "leader_horizon_clearance",
                    "predicted_min_clearance_m": float(min_clearance_m),
                    "predicted_best_tail_m": float(tail_best),
                }
            leader_roll = leader_next
            follower_roll = follower_next
        return True, {
            "reason": "ok",
            "predicted_min_clearance_m": float(min_clearance_m),
            "predicted_best_tail_m": float(tail_best),
            "predicted_tail_improvement_m": float(tail0 - tail_best),
        }

    def _settle_candidate_primitives(self, *, wide_search: bool = False) -> tuple[MotionPrimitive, ...]:
        candidate_names = {
            "reverse_straight_short",
            "reverse_arc_left",
            "reverse_arc_right",
            "reverse_arc_left_tight",
            "reverse_arc_right_tight",
            "forward_straight_short",
            "forward_arc_left",
            "forward_arc_right",
        }
        if wide_search:
            candidate_names |= {
                "reverse_straight_long",
                "reverse_arc_left_long",
                "reverse_arc_right_long",
                "forward_straight_long",
                "forward_arc_left_tight",
                "forward_arc_right_tight",
                "reverse_turn_forward_left",
                "reverse_turn_forward_right",
            }
        return tuple(primitive for primitive in self.motion_library.primitives if primitive.name in candidate_names)

    def _escape_settle_primitives(self, *, terminal_side: float) -> tuple[MotionPrimitive, ...]:
        if terminal_side < 0.0:
            names = (
                "reverse_arc_right_long",
                "reverse_arc_right_tight",
                "reverse_arc_right",
                "reverse_arc_left_tight",
                "reverse_straight_long",
                "reverse_straight_short",
                "forward_straight_short",
                "reverse_turn_forward_right",
            )
        else:
            names = (
                "reverse_arc_left_long",
                "reverse_arc_left_tight",
                "reverse_arc_left",
                "reverse_arc_right_tight",
                "reverse_straight_long",
                "reverse_straight_short",
                "forward_straight_short",
                "reverse_turn_forward_left",
            )
        ordered = []
        primitive_map = {primitive.name: primitive for primitive in self.motion_library.primitives}
        for name in names:
            primitive = primitive_map.get(name)
            if primitive is not None:
                ordered.append(primitive)
        return tuple(ordered)

    def _escape_macro_templates(self, *, terminal_side: float) -> tuple[tuple[str, ...], ...]:
        if terminal_side < 0.0:
            templates = (
                (
                    "reverse_arc_right_long",
                    "reverse_straight_long",
                    "reverse_straight_long",
                    "reverse_straight_short",
                    "forward_arc_right",
                    "reverse_straight_short",
                    "reverse_arc_left",
                ),
                (
                    "reverse_arc_right_long",
                    "reverse_arc_left_tight",
                    "reverse_straight_long",
                    "reverse_straight_short",
                    "reverse_arc_left_tight",
                ),
            )
        else:
            templates = (
                (
                    "reverse_arc_left_long",
                    "reverse_straight_long",
                    "reverse_straight_long",
                    "reverse_straight_short",
                    "forward_arc_left",
                    "reverse_straight_short",
                    "reverse_arc_right",
                ),
                (
                    "reverse_arc_left_long",
                    "reverse_arc_right_tight",
                    "reverse_straight_long",
                    "reverse_straight_short",
                    "reverse_arc_right_tight",
                ),
            )
        return templates

    def _search_certificate_ascent_anchor(
        self,
        *,
        leader: VehicleState,
        follower: VehicleState,
        plan,
        obstacles: list[Obstacle],
    ) -> bool:
        metadata = dict(getattr(plan, "metadata", {}) or {})
        corridor_polygons = [np.asarray(poly, dtype=float) for poly in metadata.get("corridor_polygons", []) if isinstance(poly, list)]
        nominal_sigma, robust_sigma, _ = self._robust_execution_certificate(
            leader_pose=leader,
            follower_pose=follower,
            corridor_info=metadata,
            obstacles=obstacles,
        )
        wide_search = bool(
            float(robust_sigma) <= 0.05
            or float(np.linalg.norm(leader.xy() - plan.leader_goal.xy())) >= 0.55
            or float(metadata.get("leader_reverse_turn_required", metadata.get("reverse_turn_required", 0.0))) >= 0.25
        )
        best_score = float(1.10 * robust_sigma + 0.20 * nominal_sigma)
        best_states: np.ndarray | None = None
        best_cmd: np.ndarray | None = None
        best_pose: VehicleState | None = None
        best_nominal_sigma = float(nominal_sigma)
        best_robust_sigma = float(robust_sigma)
        start_time_s = time.perf_counter()
        beam: list[tuple[VehicleState, list[np.ndarray], list[np.ndarray], float, int, list[str]]] = [(leader.copy(), [], [], 1e6, 0, [])]
        seen: dict[tuple[int, int, int], float] = {}
        max_depth = 5 if wide_search else 3
        beam_keep = 12 if wide_search else 18
        time_budget_s = 0.30 if wide_search else 0.12
        for depth in range(1, max_depth + 1):
            expanded: list[tuple[VehicleState, list[np.ndarray], list[np.ndarray], float, int, list[str], float]] = []
            for state, state_blocks, cmd_blocks, min_clearance_m, gear_switches, sequence_names in beam:
                if time.perf_counter() - start_time_s >= float(time_budget_s):
                    break
                for primitive in self._settle_candidate_primitives(wide_search=wide_search):
                    if time.perf_counter() - start_time_s >= float(time_budget_s):
                        break
                    rollout = self.motion_library.rollout(start=state, primitive=primitive)
                    safe, rollout_clearance, _ = self.motion_library.is_safe(
                        rollout=rollout,
                        obstacles=obstacles,
                        corridor_polygons=corridor_polygons,
                        other=follower,
                        min_clearance=float(self.cfg.safety.min_clearance),
                        in_bounds=self._in_bounds,
                    )
                    if not safe:
                        continue
                    next_state = rollout.end_state.copy()
                    state_key = (
                        int(round(float(next_state.x) / 0.08)),
                        int(round(float(next_state.y) / 0.08)),
                        int(round(math.degrees(float(next_state.yaw)) / 4.0)),
                    )
                    nominal_value, robust_value, _ = self._robust_execution_certificate(
                        leader_pose=next_state,
                        follower_pose=follower,
                        corridor_info=metadata,
                        obstacles=obstacles,
                    )
                    distance_to_anchor_m = float(np.linalg.norm(next_state.xy() - plan.leader_goal.xy()))
                    candidate_min_clearance_m = float(min(min_clearance_m, rollout_clearance))
                    if float(candidate_min_clearance_m) < float(self._critical_clearance_threshold()):
                        continue
                    score = float(
                        1.20 * robust_value
                        + 0.28 * nominal_value
                        - 0.04 * depth
                        - 0.05 * gear_switches
                        - 0.025 * distance_to_anchor_m
                        + 0.02 * candidate_min_clearance_m
                    )
                    if score <= float(seen.get(state_key, -1e9)) + 1e-9:
                        continue
                    seen[state_key] = float(score)
                    next_state_blocks = [*state_blocks, np.asarray(rollout.states, dtype=float)]
                    next_cmd_blocks = [*cmd_blocks, np.asarray(rollout.commands, dtype=float)]
                    next_names = [*sequence_names, str(primitive.name)]
                    expanded.append((next_state, next_state_blocks, next_cmd_blocks, candidate_min_clearance_m, gear_switches + primitive.gear_switches, next_names, score))
                    if robust_value > best_robust_sigma + 1e-6 or (robust_value >= best_robust_sigma - 1e-6 and nominal_value > best_nominal_sigma + 1e-6):
                        best_score = float(score)
                        best_states = np.vstack(next_state_blocks).astype(float)
                        best_cmd = np.vstack(next_cmd_blocks).astype(float)
                        best_pose = next_state.copy()
                        best_nominal_sigma = float(nominal_value)
                        best_robust_sigma = float(robust_value)
            expanded.sort(key=lambda item: float(item[-1]), reverse=True)
            beam = [(state, state_blocks, cmd_blocks, min_clearance_m, gear_switches, sequence_names) for state, state_blocks, cmd_blocks, min_clearance_m, gear_switches, sequence_names, _score in expanded[:beam_keep]]
            if best_robust_sigma >= float(metadata.get("exec_release_threshold", 0.30)):
                break
            if time.perf_counter() - start_time_s >= float(time_budget_s):
                break
        self._settle_candidate_states = best_states
        self._settle_candidate_cmd = best_cmd
        self._settle_candidate_step = 0
        self._settle_candidate_pose = best_pose
        self._settle_candidate_sigma = float(best_nominal_sigma)
        self._settle_candidate_robust_sigma = float(best_robust_sigma)
        return bool(best_pose is not None and best_states is not None and best_cmd is not None and best_robust_sigma > robust_sigma + 1e-6)

    def _execute_settle_candidate(self, *, leader: VehicleState, follower: VehicleState, obstacles: list[Obstacle]) -> tuple[ControlCommand, bool]:
        if self._settle_candidate_cmd is None or self._settle_candidate_states is None or len(self._settle_candidate_cmd) == 0:
            return self._freeze_vehicle(leader), True
        if self._settle_candidate_step >= len(self._settle_candidate_cmd):
            return self._freeze_vehicle(leader), True
        command_row = self._settle_candidate_cmd[min(self._settle_candidate_step, len(self._settle_candidate_cmd) - 1)]
        command = ControlCommand(accel=float(command_row[0]), steer_rate=float(command_row[1]))
        next_state = self.model.step(leader, command, float(self.cfg.control.dt))
        if self.collision.in_collision(next_state, obstacles, [follower]):
            return self._freeze_vehicle(leader), False
        if float(self.collision.min_clearance_vehicle_obstacles(next_state, obstacles)) < float(self._critical_clearance_threshold()):
            self._settle_candidate_step = len(self._settle_candidate_cmd)
            return self._freeze_vehicle(leader), True
        self._settle_candidate_step += 1
        return command, bool(self._settle_candidate_step >= len(self._settle_candidate_cmd))

    def _exec_terminal_certificate(self, *, leader_candidate: VehicleState, follower: VehicleState, plan, obstacles: list[Obstacle]) -> float:
        metadata = dict(getattr(plan, "metadata", {}) or {})
        sigma_exec, _ = self.compute_execution_certificate(
            leader_actual_pose=leader_candidate,
            follower_pose=follower,
            corridor_info=metadata,
            obstacles=obstacles,
        )
        return float(sigma_exec)

    def _terminal_restage_objective(
        self,
        *,
        sigma_terminal: float,
        sigma_exec: float,
        handoff_ready: bool,
        term_meta: dict[str, Any],
        tail_distance_m: float,
        min_clearance_m: float,
        sequence_length: int,
    ) -> float:
        lateral_error_m = abs(float(term_meta.get("y_l_m", 0.0)))
        yaw_gap_deg = abs(float(term_meta.get("yaw_gap_deg", 0.0)))
        witness_sigma = float(term_meta.get("witness_sigma", sigma_terminal))
        witness_margin = float(max(witness_sigma - float(sigma_terminal), 0.0))
        return float(
            1.05 * float(sigma_terminal)
            + 0.28 * float(sigma_exec)
            + (0.18 if bool(handoff_ready) else 0.0)
            + 0.10 * float(witness_margin)
            - 0.34 * float(lateral_error_m)
            - 0.06 * float(yaw_gap_deg) / 20.0
            - 0.10 * float(tail_distance_m)
            - 0.018 * float(sequence_length)
            + 0.02 * float(min_clearance_m)
        )

    def _legacy_terminal_restage_sequences(self, *, terminal_side: float) -> tuple[tuple[str, ...], ...]:
        if float(terminal_side) < 0.0:
            return (
                ("reverse_arc_right_long",),
                ("reverse_arc_right_tight", "reverse_arc_right"),
                ("reverse_arc_right_long", "reverse_straight_short"),
            )
        return (
            ("reverse_arc_left_long",),
            ("reverse_arc_left_tight", "reverse_arc_left"),
            ("reverse_arc_left_long", "reverse_straight_short"),
        )

    def _search_terminal_restage(self, *, leader: VehicleState, follower: VehicleState, plan, obstacles: list[Obstacle]) -> bool:
        metadata = dict(getattr(plan, "metadata", {}) or {})
        current_sigma, current_meta = self.compute_terminal_closure_certificate(
            leader_pose=leader,
            follower_pose=follower,
            corridor_info=metadata,
            obstacles=obstacles,
        )
        current_exec = self._exec_terminal_certificate(leader_candidate=leader, follower=follower, plan=plan, obstacles=obstacles)
        current_handoff_ready, _ = self._docking_handoff_ready(leader=leader, follower=follower, obstacles=obstacles)
        current_tail_distance_m = float(np.linalg.norm(self.geom.front_hitch(follower) - self.geom.rear_hitch(leader)))
        current_score = self._terminal_restage_objective(
            sigma_terminal=float(current_sigma),
            sigma_exec=float(current_exec),
            handoff_ready=bool(current_handoff_ready),
            term_meta=dict(current_meta),
            tail_distance_m=float(current_tail_distance_m),
            min_clearance_m=float(
                min(
                    self.collision.min_clearance_vehicle_obstacles(leader, obstacles),
                    self.collision.min_clearance_vehicle_obstacles(follower, obstacles),
                    self.collision.min_clearance_vehicle_vehicle(leader, follower),
                )
            ),
            sequence_length=0,
        )
        start_time_s = time.perf_counter()
        macro_time_budget_s = 1.00
        candidate_names = {
            "reverse_straight_short",
            "reverse_straight_long",
            "reverse_arc_left",
            "reverse_arc_right",
            "reverse_arc_left_tight",
            "reverse_arc_right_tight",
            "forward_straight_short",
            "forward_straight_long",
            "reverse_arc_left_long",
            "reverse_arc_right_long",
            "reverse_turn_forward_left",
            "reverse_turn_forward_right",
        }
        primitives = [primitive for primitive in self.motion_library.primitives if primitive.name in candidate_names]
        corridor_polygons = [np.asarray(poly, dtype=float) for poly in metadata.get("corridor_polygons", []) if isinstance(poly, list)]
        best: tuple[float, list[np.ndarray], list[np.ndarray]] | None = None
        for sequence_names in self._legacy_terminal_restage_sequences(terminal_side=float(metadata.get("terminal_side", 1.0))):
            state = leader.copy()
            sequence_states: list[np.ndarray] = []
            sequence_commands: list[np.ndarray] = []
            min_clearance_m = 1e6
            feasible = True
            for primitive_name in sequence_names:
                primitive = next((item for item in self.motion_library.primitives if item.name == primitive_name), None)
                if primitive is None:
                    feasible = False
                    break
                rollout = self.motion_library.rollout(start=state, primitive=primitive)
                safe, rollout_clearance, _ = self.motion_library.is_safe(
                    rollout=rollout,
                    obstacles=obstacles,
                    corridor_polygons=corridor_polygons,
                    other=follower,
                    min_clearance=float(self.cfg.safety.min_clearance),
                    in_bounds=self._in_bounds,
                )
                if not safe:
                    feasible = False
                    break
                min_clearance_m = float(min(min_clearance_m, rollout_clearance))
                sequence_states.append(np.asarray(rollout.states, dtype=float))
                sequence_commands.append(np.asarray(rollout.commands, dtype=float))
                state = rollout.end_state.copy()
            if not feasible or not sequence_states:
                continue
            if float(min_clearance_m) < float(self._critical_clearance_threshold()):
                continue
            sigma_terminal, term_meta = self.compute_terminal_closure_certificate(
                leader_pose=state,
                follower_pose=follower,
                corridor_info=metadata,
                obstacles=obstacles,
            )
            sigma_exec = self._exec_terminal_certificate(leader_candidate=state, follower=follower, plan=plan, obstacles=obstacles)
            handoff_ready, _ = self._docking_handoff_ready(leader=state, follower=follower, obstacles=obstacles)
            tail_distance_m = float(np.linalg.norm(self.geom.front_hitch(follower) - self.geom.rear_hitch(state)))
            candidate_score = self._terminal_restage_objective(
                sigma_terminal=float(sigma_terminal),
                sigma_exec=float(sigma_exec),
                handoff_ready=bool(handoff_ready),
                term_meta=dict(term_meta),
                tail_distance_m=float(tail_distance_m),
                min_clearance_m=float(min_clearance_m),
                sequence_length=int(len(sequence_names)),
            )
            if (
                float(sigma_terminal) >= max(0.58, float(current_sigma) - 0.08)
                and float(sigma_exec) >= max(0.12, float(current_exec) - 0.10)
                and float(candidate_score) > float(current_score) + 0.05
            ):
                self._reshape_states = np.vstack(sequence_states).astype(float)
                self._reshape_cmd = np.vstack(sequence_commands).astype(float)
                self._reshape_step = 0
                return True
        macro_templates = self.macro_selector.fallback_to_legacy_primitives(leader=leader, corridor_info=metadata)
        primitive_map = {primitive.name: primitive for primitive in self.motion_library.primitives}
        for macro_template in macro_templates:
            if time.perf_counter() - start_time_s >= macro_time_budget_s:
                break
            state = leader.copy()
            sequence_states: list[np.ndarray] = []
            sequence_commands: list[np.ndarray] = []
            min_clearance_m = 1e6
            feasible = True
            for primitive_name in macro_template.primitive_names:
                primitive = primitive_map.get(primitive_name)
                if primitive is None:
                    feasible = False
                    break
                rollout = self.motion_library.rollout(start=state, primitive=primitive)
                safe, rollout_clearance, _ = self.motion_library.is_safe(
                    rollout=rollout,
                    obstacles=obstacles,
                    corridor_polygons=corridor_polygons,
                    other=follower,
                    min_clearance=float(self.cfg.safety.min_clearance),
                    in_bounds=self._in_bounds,
                )
                if not safe:
                    feasible = False
                    break
                min_clearance_m = float(min(min_clearance_m, rollout_clearance))
                sequence_states.append(np.asarray(rollout.states, dtype=float))
                sequence_commands.append(np.asarray(rollout.commands, dtype=float))
                state = rollout.end_state.copy()
            if not feasible or not sequence_states:
                continue
            sigma_terminal, term_meta = self.compute_terminal_closure_certificate(
                leader_pose=state,
                follower_pose=follower,
                corridor_info=metadata,
                obstacles=obstacles,
            )
            sigma_exec = self._exec_terminal_certificate(leader_candidate=state, follower=follower, plan=plan, obstacles=obstacles)
            handoff_ready, _ = self._docking_handoff_ready(leader=state, follower=follower, obstacles=obstacles)
            tail_distance_m = float(np.linalg.norm(self.geom.front_hitch(follower) - self.geom.rear_hitch(state)))
            candidate_score = self._terminal_restage_objective(
                sigma_terminal=float(sigma_terminal),
                sigma_exec=float(sigma_exec),
                handoff_ready=bool(handoff_ready),
                term_meta=dict(term_meta),
                tail_distance_m=float(tail_distance_m),
                min_clearance_m=float(min_clearance_m),
                sequence_length=int(len(macro_template.primitive_names)),
            )
            has_structural_gain = bool(
                bool(handoff_ready) and not bool(current_handoff_ready)
                or abs(float(term_meta.get("y_l_m", 0.0))) <= abs(float(current_meta.get("y_l_m", 0.0))) - 0.05
                or float(tail_distance_m) <= float(current_tail_distance_m) - 0.04
            )
            if not (
                float(sigma_terminal) >= max(0.58, float(current_sigma) - 0.08)
                and float(sigma_exec) >= max(0.12, float(current_exec) - 0.10)
                and (float(candidate_score) > float(current_score) + 0.02 or has_structural_gain)
            ):
                continue
            terminal_error = float(np.linalg.norm(state.xy() - plan.leader_goal.xy()))
            score = float(
                float(candidate_score)
                - 0.02 * terminal_error
                + 0.04 * float(macro_template.metadata.get("legacy_priority", 0.0))
            )
            if best is None or score > float(best[0]):
                best = (score, sequence_states, sequence_commands)
        for sequence_length in (1, 2, 3):
            if time.perf_counter() - start_time_s >= 0.35:
                break
            for sequence in itertools.product(primitives, repeat=sequence_length):
                if time.perf_counter() - start_time_s >= macro_time_budget_s:
                    break
                state = leader.copy()
                sequence_states: list[np.ndarray] = []
                sequence_commands: list[np.ndarray] = []
                min_clearance_m = 1e6
                feasible = True
                for primitive in sequence:
                    rollout = self.motion_library.rollout(start=state, primitive=primitive)
                    safe, rollout_clearance, _ = self.motion_library.is_safe(
                        rollout=rollout,
                        obstacles=obstacles,
                        corridor_polygons=corridor_polygons,
                        other=follower,
                        min_clearance=float(self.cfg.safety.min_clearance),
                        in_bounds=self._in_bounds,
                    )
                    if not safe:
                        feasible = False
                        break
                    min_clearance_m = float(min(min_clearance_m, rollout_clearance))
                    sequence_states.append(np.asarray(rollout.states, dtype=float))
                    sequence_commands.append(np.asarray(rollout.commands, dtype=float))
                    state = rollout.end_state.copy()
                if not feasible:
                    continue
                sigma_terminal, term_meta = self.compute_terminal_closure_certificate(
                    leader_pose=state,
                    follower_pose=follower,
                    corridor_info=metadata,
                    obstacles=obstacles,
                )
                sigma_exec = self._exec_terminal_certificate(leader_candidate=state, follower=follower, plan=plan, obstacles=obstacles)
                handoff_ready, _ = self._docking_handoff_ready(leader=state, follower=follower, obstacles=obstacles)
                tail_distance_m = float(np.linalg.norm(self.geom.front_hitch(follower) - self.geom.rear_hitch(state)))
                candidate_score = self._terminal_restage_objective(
                    sigma_terminal=float(sigma_terminal),
                    sigma_exec=float(sigma_exec),
                    handoff_ready=bool(handoff_ready),
                    term_meta=dict(term_meta),
                    tail_distance_m=float(tail_distance_m),
                    min_clearance_m=float(min_clearance_m),
                    sequence_length=int(sequence_length),
                )
                has_structural_gain = bool(
                    bool(handoff_ready) and not bool(current_handoff_ready)
                    or abs(float(term_meta.get("y_l_m", 0.0))) <= abs(float(current_meta.get("y_l_m", 0.0))) - 0.05
                    or float(tail_distance_m) <= float(current_tail_distance_m) - 0.04
                )
                if not (
                    float(sigma_terminal) >= max(0.58, float(current_sigma) - 0.08)
                    and float(sigma_exec) >= max(0.12, float(current_exec) - 0.10)
                    and (float(candidate_score) > float(current_score) + 0.02 or has_structural_gain)
                ):
                    continue
                terminal_error = float(np.linalg.norm(state.xy() - plan.leader_goal.xy()))
                score = float(
                    float(candidate_score)
                    - 0.02 * terminal_error
                )
                if best is None or score > float(best[0]):
                    best = (score, sequence_states, sequence_commands)
        if best is None:
            self._reshape_states = None
            self._reshape_cmd = None
            self._reshape_step = 0
            return False
        self._reshape_states = np.vstack(best[1]).astype(float)
        self._reshape_cmd = np.vstack(best[2]).astype(float)
        self._reshape_step = 0
        return True

    def _search_escape_settle_candidate(
        self,
        *,
        leader: VehicleState,
        follower: VehicleState,
        plan,
        obstacles: list[Obstacle],
    ) -> bool:
        metadata = dict(getattr(plan, "metadata", {}) or {})
        corridor_polygons = [np.asarray(poly, dtype=float) for poly in metadata.get("corridor_polygons", []) if isinstance(poly, list)]
        lane_heading = float(metadata.get("lane_heading", plan.leader_goal.yaw))
        anchor_hint_xy = np.asarray(metadata.get("anchor_hint_xy", [plan.leader_goal.x, plan.leader_goal.y]), dtype=float)
        terminal_side = 1.0 if float(metadata.get("lane_y", leader.y) - float(leader.y)) >= 0.0 else -1.0
        primitives = self._escape_settle_primitives(terminal_side=float(terminal_side))
        primitive_map = {primitive.name: primitive for primitive in primitives}
        start_time_s = time.perf_counter()
        beam: list[tuple[VehicleState, list[np.ndarray], list[np.ndarray], float, int]] = [(leader.copy(), [], [], 1e6, 0)]
        best: tuple[float, np.ndarray, np.ndarray, VehicleState] | None = None
        seen: dict[tuple[int, int, int], float] = {}
        macro_templates = self.macro_selector.select(leader=leader, corridor_info=metadata)
        if not macro_templates:
            macro_templates = tuple(
                MacroPrimitiveTemplate(
                    name=f"legacy_escape_macro_{index}",
                    primitive_names=tuple(template),
                    metadata={},
                )
                for index, template in enumerate(self._escape_macro_templates(terminal_side=float(terminal_side)))
            )
        for macro_template in macro_templates:
            state = leader.copy()
            state_blocks: list[np.ndarray] = []
            cmd_blocks: list[np.ndarray] = []
            min_clearance_m = 1e6
            feasible = True
            for primitive_name in macro_template.primitive_names:
                primitive = primitive_map.get(primitive_name)
                if primitive is None:
                    feasible = False
                    break
                rollout = self.motion_library.rollout(start=state, primitive=primitive)
                safe, rollout_clearance, _ = self.motion_library.is_safe(
                    rollout=rollout,
                    obstacles=obstacles,
                    corridor_polygons=corridor_polygons,
                    other=follower,
                    min_clearance=float(self.cfg.safety.min_clearance),
                    in_bounds=self._in_bounds,
                )
                if not safe:
                    feasible = False
                    break
                min_clearance_m = float(min(min_clearance_m, rollout_clearance))
                state = rollout.end_state.copy()
                state_blocks.append(np.asarray(rollout.states, dtype=float))
                cmd_blocks.append(np.asarray(rollout.commands, dtype=float))
            if not feasible or not state_blocks:
                continue
            approx_exec, _ = self.analysis_helper._approx_execution_certificate(
                anchor=state,
                follower=follower,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
                lane_heading=lane_heading,
                allow_grid_fallback=False,
            )
            approx_terminal, _ = self.analysis_helper._approx_terminal_safety_certificate(
                anchor=state,
                follower=follower,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
            )
            combined_nominal = float(math.sqrt(max(approx_exec, 0.0) * max(approx_terminal, 0.0)))
            score = float(
                1.45 * combined_nominal
                + 0.12 * approx_exec
                + 0.05 * approx_terminal
                + 0.02 * min_clearance_m
                - 0.02 * np.linalg.norm(state.xy() - anchor_hint_xy)
            )
            if best is None or score > float(best[0]):
                best = (
                    float(score),
                    np.vstack(state_blocks).astype(float),
                    np.vstack(cmd_blocks).astype(float),
                    state.copy(),
                )
        for depth in range(1, 6):
            if time.perf_counter() - start_time_s >= 0.60:
                break
            expanded: list[tuple[VehicleState, list[np.ndarray], list[np.ndarray], float, int, float]] = []
            for state, state_blocks, cmd_blocks, min_clearance_m, gear_switches in beam:
                if time.perf_counter() - start_time_s >= 0.60:
                    break
                for primitive in primitives:
                    rollout = self.motion_library.rollout(start=state, primitive=primitive)
                    safe, rollout_clearance, _ = self.motion_library.is_safe(
                        rollout=rollout,
                        obstacles=obstacles,
                        corridor_polygons=corridor_polygons,
                        other=follower,
                        min_clearance=float(self.cfg.safety.min_clearance),
                        in_bounds=self._in_bounds,
                    )
                    if not safe:
                        continue
                    next_state = rollout.end_state.copy()
                    approx_exec, _ = self.analysis_helper._approx_execution_certificate(
                        anchor=next_state,
                        follower=follower,
                        obstacles=obstacles,
                        corridor_polygons=corridor_polygons,
                        lane_heading=lane_heading,
                        allow_grid_fallback=False,
                    )
                    approx_terminal, _ = self.analysis_helper._approx_terminal_safety_certificate(
                        anchor=next_state,
                        follower=follower,
                        obstacles=obstacles,
                        corridor_polygons=corridor_polygons,
                    )
                    combined_nominal = float(math.sqrt(max(approx_exec, 0.0) * max(approx_terminal, 0.0)))
                    candidate_min_clearance_m = float(min(min_clearance_m, rollout_clearance))
                    if float(candidate_min_clearance_m) < float(self._critical_clearance_threshold()):
                        continue
                    distance_to_anchor_m = float(np.linalg.norm(next_state.xy() - anchor_hint_xy))
                    score = float(
                        1.35 * combined_nominal
                        + 0.10 * approx_exec
                        + 0.05 * approx_terminal
                        + 0.02 * candidate_min_clearance_m
                        - 0.03 * depth
                        - 0.02 * gear_switches
                        - 0.015 * distance_to_anchor_m
                    )
                    state_key = (
                        int(round(float(next_state.x) / 0.12)),
                        int(round(float(next_state.y) / 0.12)),
                        int(round(math.degrees(float(next_state.yaw)) / 6.0)),
                    )
                    if score <= float(seen.get(state_key, -1e9)) + 1e-9:
                        continue
                    seen[state_key] = float(score)
                    next_state_blocks = [*state_blocks, np.asarray(rollout.states, dtype=float)]
                    next_cmd_blocks = [*cmd_blocks, np.asarray(rollout.commands, dtype=float)]
                    next_gear_switches = int(gear_switches + primitive.gear_switches)
                    expanded.append((next_state, next_state_blocks, next_cmd_blocks, candidate_min_clearance_m, next_gear_switches, score))
                    if best is None or score > float(best[0]):
                        best = (
                            float(score),
                            np.vstack(next_state_blocks).astype(float),
                            np.vstack(next_cmd_blocks).astype(float),
                            next_state.copy(),
                        )
            expanded.sort(key=lambda item: float(item[-1]), reverse=True)
            beam = [(state, sblocks, cblocks, minc, gears) for state, sblocks, cblocks, minc, gears, _score in expanded[:16]]
        if best is None:
            return False
        _score, best_states, best_cmd, best_pose = best
        if len(best_cmd) == 0:
            return False
        self._settle_candidate_states = best_states
        self._settle_candidate_cmd = best_cmd
        self._settle_candidate_step = 0
        self._settle_candidate_pose = best_pose
        self._settle_candidate_sigma = 0.0
        self._settle_candidate_robust_sigma = 0.0
        return True

    def _replay_sequence_command(self, *, leader: VehicleState, follower: VehicleState, obstacles: list[Obstacle]) -> tuple[ControlCommand, bool]:
        if self._reshape_states is None or self._reshape_cmd is None or len(self._reshape_states) == 0 or len(self._reshape_cmd) == 0:
            return self._freeze_vehicle(leader), True
        if self._reshape_step >= len(self._reshape_cmd):
            return self._freeze_vehicle(leader), True
        command_row = self._reshape_cmd[min(self._reshape_step, len(self._reshape_cmd) - 1)]
        command = ControlCommand(accel=float(command_row[0]), steer_rate=float(command_row[1]))
        next_state = self.model.step(leader, command, float(self.cfg.control.dt))
        if self.collision.in_collision(next_state, obstacles, [follower]):
            return self._freeze_vehicle(leader), False
        self._reshape_step += 1
        return command, False

    def _path_progress_index(self, *, state: VehicleState, path_xy: np.ndarray) -> int:
        path = np.asarray(path_xy, dtype=float)
        if len(path) == 0:
            return 0
        return int(np.argmin(np.linalg.norm(path - state.xy()[None, :], axis=1)))

    def _path_reference(self, *, state: VehicleState, path_xy: np.ndarray, lookahead_m: float) -> tuple[np.ndarray, float, int]:
        path = np.asarray(path_xy, dtype=float)
        if len(path) < 2:
            return state.xy(), float(state.yaw), 0
        nearest_index = self._path_progress_index(state=state, path_xy=path)
        accumulated_length_m = 0.0
        target_index = nearest_index
        target_xy = np.asarray(path[nearest_index], dtype=float)
        for idx in range(nearest_index, len(path) - 1):
            segment_start_xy = np.asarray(path[idx], dtype=float)
            segment_end_xy = np.asarray(path[idx + 1], dtype=float)
            segment_length_m = float(np.linalg.norm(segment_end_xy - segment_start_xy))
            if segment_length_m <= 1e-9:
                continue
            if accumulated_length_m + segment_length_m >= float(lookahead_m):
                ratio = float((lookahead_m - accumulated_length_m) / max(segment_length_m, 1e-9))
                target_xy = (1.0 - ratio) * segment_start_xy + ratio * segment_end_xy
                target_index = idx + 1
                break
            accumulated_length_m += segment_length_m
            target_xy = segment_end_xy
            target_index = idx + 1
        if target_index >= len(path) - 1:
            heading_xy = np.asarray(path[-1], dtype=float) - np.asarray(path[-2], dtype=float)
        else:
            heading_xy = np.asarray(path[target_index], dtype=float) - np.asarray(path[max(0, target_index - 1)], dtype=float)
        heading_yaw = float(math.atan2(float(heading_xy[1]), float(heading_xy[0]))) if float(np.linalg.norm(heading_xy)) > 1e-9 else float(state.yaw)
        return np.asarray(target_xy, dtype=float), float(heading_yaw), int(nearest_index)

    def _primitive_path_command(
        self,
        *,
        state: VehicleState,
        goal: VehicleState,
        path_xy: np.ndarray,
        obstacles: list[Obstacle],
        other: VehicleState | None,
        prefer_reverse: bool,
    ) -> ControlCommand:
        path = np.asarray(path_xy, dtype=float)
        if len(path) < 2:
            return self._plan_step_signed(
                state=state,
                goal=goal,
                target_speed=float(-0.12 if prefer_reverse else 0.12),
                obstacles=obstacles,
                other=other,
                allow_reverse=True,
            )
        reference_xy, reference_yaw, start_index = self._path_reference(
            state=state,
            path_xy=path,
            lookahead_m=float(0.32 if prefer_reverse else 0.42),
        )
        best_command: ControlCommand | None = None
        best_score = float("inf")
        for primitive in self.motion_library.primitives:
            primitive_name = str(primitive.name)
            if prefer_reverse and primitive_name.startswith("forward_turn_reverse"):
                continue
            if (not prefer_reverse) and primitive_name.startswith("reverse_turn"):
                continue
            if prefer_reverse and primitive_name.startswith("forward_") and "turn_reverse" not in primitive_name:
                continue
            if (not prefer_reverse) and primitive_name.startswith("reverse_") and "turn_reverse" not in primitive_name:
                continue
            rollout = self.motion_library.rollout(start=state, primitive=primitive)
            safe, rollout_clearance, _ = self.motion_library.is_safe(
                rollout=rollout,
                obstacles=obstacles,
                corridor_polygons=[],
                other=other,
                min_clearance=float(self.cfg.safety.min_clearance),
                in_bounds=self._in_bounds,
            )
            if not safe:
                continue
            end_state = rollout.end_state
            end_index = self._path_progress_index(state=end_state, path_xy=path)
            progress_score = float(max(0, end_index - start_index))
            reference_error = float(np.linalg.norm(end_state.xy() - reference_xy))
            heading_error = abs(float(angle_diff(float(reference_yaw), float(end_state.yaw))))
            goal_error = float(np.linalg.norm(end_state.xy() - goal.xy()))
            score = (
                2.8 * reference_error
                + 0.9 * heading_error
                + 0.7 * goal_error
                + 0.18 * primitive.gear_switches
                + 0.16 * primitive.cost_bias
                + 0.20 / max(float(rollout_clearance), 0.05)
                - 0.55 * progress_score
            )
            if score < best_score:
                best_score = float(score)
                best_command = ControlCommand(
                    accel=float(rollout.commands[0, 0]),
                    steer_rate=float(rollout.commands[0, 1]),
                )
        if best_command is None:
            return self._guarded_track(
                state=state,
                path_xy=path,
                goal=goal,
                target_speed=float(-0.12 if prefer_reverse else 0.14),
                obstacles=obstacles,
                other=other,
            )
        next_state = self.model.step(state, best_command, float(self.cfg.control.dt))
        if self.collision.in_collision(next_state, obstacles, [other] if other is not None else []):
            return self._freeze_vehicle(state)
        return best_command

    def terminal_capture_command(
        self,
        *,
        leader: VehicleState,
        follower: VehicleState,
        obstacles: list[Obstacle],
        corridor_info: dict[str, Any] | None = None,
    ) -> tuple[ControlCommand, str]:
        corridor_meta = dict(corridor_info or {})
        if self._terminal_lock_ready_state(leader=leader, follower=follower, obstacles=obstacles):
            return self._freeze_vehicle(follower), "LC_TERMINAL_LOCK_HOLD"
        corridor_polygons = self._corridor_polygons_from_info(corridor_meta)
        sigma_terminal, terminal_meta = self.compute_terminal_closure_certificate(
            leader_pose=leader,
            follower_pose=follower,
            corridor_info=corridor_meta,
            obstacles=obstacles,
            estimate_witness=False,
        )
        best_command: ControlCommand | None = None
        best_score = float("inf")
        for primitive in self._terminal_closure_primitives():
            primitive_name = str(primitive.name)
            rollout = self.motion_library.rollout(start=follower, primitive=primitive)
            safe, rollout_clearance, _ = self.motion_library.is_safe(
                rollout=rollout,
                obstacles=obstacles,
                corridor_polygons=corridor_polygons,
                other=leader,
                min_clearance=float(self.cfg.safety.min_clearance),
                in_bounds=self._in_bounds,
            )
            if not safe:
                continue
            if float(rollout_clearance) < float(self.cfg.safety.min_clearance) + 0.025:
                continue
            end_state = rollout.end_state.copy()
            next_sigma, next_meta = self.compute_terminal_closure_certificate(
                leader_pose=leader,
                follower_pose=end_state,
                corridor_info=corridor_meta,
                obstacles=obstacles,
                estimate_witness=False,
            )
            progress_gain = float(next_sigma - sigma_terminal)
            score = (
                -2.8 * float(next_sigma)
                -1.1 * float(progress_gain)
                + 0.32 * abs(float(next_meta.get("y_l_m", 0.0)))
                + 0.22 * abs(float(next_meta.get("yaw_gap_deg", 0.0))) / 20.0
                + 0.12 * primitive.gear_switches
                + 0.08 * primitive.cost_bias
                + 0.30 / max(float(rollout_clearance), 0.05)
            )
            if score < best_score:
                best_score = float(score)
                best_command = ControlCommand(
                    accel=float(rollout.commands[0, 0]),
                    steer_rate=float(rollout.commands[0, 1]),
                )
        if best_command is None:
            contact_goal_xy = np.asarray(terminal_meta.get("contact_goal_xy", follower.xy()), dtype=float)
            contact_goal = VehicleState(
                vehicle_id=int(follower.vehicle_id),
                x=float(contact_goal_xy[0]),
                y=float(contact_goal_xy[1]),
                yaw=float(terminal_meta.get("contact_goal_yaw", leader.yaw)),
                v=0.0,
                delta=0.0,
                mode=follower.mode,
            )
            fallback_command = self._plan_step_signed(
                state=follower,
                goal=contact_goal,
                target_speed=float(0.10 if float(terminal_meta.get("x_l_m", 0.0)) >= 0.0 else -0.10),
                obstacles=obstacles,
                other=leader,
                allow_reverse=True,
            )
            if abs(float(fallback_command.accel)) > 1e-9 or abs(float(fallback_command.steer_rate)) > 1e-9:
                return fallback_command, "LC_TERMINAL_MANIFOLD_FALLBACK"
            return self._freeze_vehicle(follower), "LC_TERMINAL_HOLD"
        return best_command, "LC_TERMINAL_MANIFOLD"

    def _follower_corridor_command(
        self,
        *,
        follower: VehicleState,
        leader: VehicleState,
        goal: VehicleState,
        path_xy: np.ndarray,
        obstacles: list[Obstacle],
        target_speed: float,
    ) -> tuple[ControlCommand, bool, str]:
        rear_hitch_xy = self.geom.rear_hitch(leader)
        front_hitch_xy = self.geom.front_hitch(follower)
        tail_distance_m = float(np.linalg.norm(rear_hitch_xy - front_hitch_xy))
        yaw_gap_deg = float(abs(math.degrees(angle_diff(float(leader.yaw), float(follower.yaw)))))
        if self._near_goal(follower, goal, pos_tol=0.45, yaw_tol_deg=22.0):
            return self._freeze_vehicle(follower), True, "FOLLOWER_READY"
        goal_distance_m = float(np.linalg.norm(follower.xy() - goal.xy()))
        if goal_distance_m <= 0.48:
            return self._freeze_vehicle(follower), True, "FOLLOWER_READY"
        if tail_distance_m <= float(self.cfg.docking.stage_switch_distance) + 0.12 and yaw_gap_deg <= 18.0:
            return self._freeze_vehicle(follower), True, "FOLLOWER_READY"
        rel_goal_xy = goal.xy() - follower.xy()
        follower_heading_xy = np.array([math.cos(float(follower.yaw)), math.sin(float(follower.yaw))], dtype=float)
        follower_lateral_xy = np.array([-math.sin(float(follower.yaw)), math.cos(float(follower.yaw))], dtype=float)
        longitudinal_error_m = float(np.dot(rel_goal_xy, follower_heading_xy))
        lateral_error_m = float(np.dot(rel_goal_xy, follower_lateral_xy))
        allow_reverse = bool(longitudinal_error_m < -0.20 and abs(lateral_error_m) < 0.65)
        signed_target_speed = float(-min(abs(float(target_speed)), 0.16) if allow_reverse else float(target_speed))
        result = self.local_planner.plan_step(
            ego=follower,
            goal_xy=goal.xy(),
            goal_yaw=float(goal.yaw),
            obstacles=obstacles,
            dynamic_others=[leader],
            force_goal_yaw=bool(np.linalg.norm(follower.xy() - goal.xy()) <= 1.1),
            target_speed=float(signed_target_speed),
            allow_reverse=bool(allow_reverse),
        )
        command = result.command if result.feasible else self._guarded_track(
            state=follower,
            path_xy=np.asarray(path_xy, dtype=float),
            goal=goal,
            target_speed=float(signed_target_speed),
            obstacles=obstacles,
            other=leader,
        )
        next_state = self.model.step(follower, command, float(self.cfg.control.dt))
        if self.collision.in_collision(next_state, obstacles, [leader]):
            return self._freeze_vehicle(follower), False, "FOLLOWER_BRAKE"
        return command, False, "FOLLOWER_REVERSE_INGRESS" if allow_reverse else "FOLLOWER_INGRESS"

    def _basin_ready(self, *, leader: VehicleState, follower: VehicleState, plan, obstacles: list[Obstacle]) -> bool:
        metadata = dict(getattr(plan, "metadata", {}) or {})
        sigma_c = float(metadata.get("sigma_c", 0.0))
        threshold = float(metadata.get("release_sigma_threshold", 0.52))
        if sigma_c < threshold:
            return False
        if str(getattr(plan, "reason", "")).startswith("lc_corridor_semantic_"):
            leader_dist = float(np.linalg.norm(leader.xy() - plan.leader_goal.xy()))
            leader_yaw_gap_deg = float(abs(math.degrees(angle_diff(float(plan.leader_goal.yaw), float(leader.yaw)))))
            if not (leader_dist <= 0.95 and leader_yaw_gap_deg <= 15.0):
                return False
        else:
            if not self._near_goal(leader, plan.leader_goal, pos_tol=0.34, yaw_tol_deg=18.0):
                return False
        predock_goal = self._dynamic_predock_goal(leader=leader, follower=follower)
        if self.collision.in_collision(predock_goal, obstacles, [leader]):
            return False
        return float(self.collision.min_clearance_vehicle_obstacles(predock_goal, obstacles)) >= float(self.cfg.safety.min_clearance)

    def _leader_shape_command(self, *, leader: VehicleState, follower: VehicleState, plan, obstacles: list[Obstacle]) -> tuple[ControlCommand, bool]:
        primitive_states_raw = getattr(plan, "leader_primitive_states", None)
        primitive_commands_raw = getattr(plan, "leader_primitive_cmd", None)
        if primitive_states_raw is None or primitive_commands_raw is None:
            return self._freeze_vehicle(leader), True
        primitive_states = np.asarray(primitive_states_raw, dtype=float)
        primitive_commands = np.asarray(primitive_commands_raw, dtype=float)
        primitive_steps = int(getattr(plan, "leader_primitive_steps", 0))
        if primitive_steps <= 0 or len(primitive_states) == 0 or len(primitive_commands) == 0:
            return self._freeze_vehicle(leader), True
        max_step = min(int(primitive_steps), len(primitive_states), len(primitive_commands))
        plan_reason = str(getattr(plan, "reason", ""))
        if plan_reason == "lc_corridor_smallgap_escape":
            if self._shape_step >= max_step:
                return self._freeze_vehicle(leader), True
            command_row = primitive_commands[self._shape_step]
            command = ControlCommand(accel=float(command_row[0]), steer_rate=float(command_row[1]))
            next_state = self.model.step(leader, command, float(self.cfg.control.dt))
            if self.collision.in_collision(next_state, obstacles, [follower]):
                return self._freeze_vehicle(leader), False
            self._shape_step += 1
            return command, bool(self._shape_step >= max_step)
        if plan_reason == "lc_corridor_semantic_cert_anchor":
            path_xy = np.asarray(plan.leader_path_xy, dtype=float)
            if len(path_xy) < 2:
                return self._freeze_vehicle(leader), True
            nearest_idx = self._path_progress_index(state=leader, path_xy=path_xy)
            next_idx = min(len(path_xy) - 1, max(nearest_idx + 1, 1))
            local_segment_xy = np.asarray(path_xy[next_idx], dtype=float) - leader.xy()
            if float(np.linalg.norm(local_segment_xy)) <= 1e-9 and next_idx + 1 < len(path_xy):
                next_idx += 1
                local_segment_xy = np.asarray(path_xy[next_idx], dtype=float) - leader.xy()
            heading_forward_xy = np.array([math.cos(float(leader.yaw)), math.sin(float(leader.yaw))], dtype=float)
            signed_speed = float(0.12 if float(np.dot(local_segment_xy, heading_forward_xy)) >= 0.0 else -0.12)
            path_command = self._guarded_track(
                state=leader,
                path_xy=path_xy,
                goal=plan.leader_goal,
                target_speed=float(signed_speed),
                obstacles=obstacles,
                other=follower,
            )
            shape_done = bool(self._near_goal(leader, plan.leader_goal, pos_tol=0.28, yaw_tol_deg=18.0))
            return path_command, shape_done
        if plan_reason in {"lc_corridor_exec_shape", "lc_corridor_macro_escape"}:
            if self._shape_step >= max_step:
                return self._freeze_vehicle(leader), True
            command_row = primitive_commands[self._shape_step]
            command = ControlCommand(accel=float(command_row[0]), steer_rate=float(command_row[1]))
            next_state = self.model.step(leader, command, float(self.cfg.control.dt))
            if self.collision.in_collision(next_state, obstacles, [follower]):
                return self._freeze_vehicle(leader), False
            self._shape_step += 1
            return command, bool(self._shape_step >= max_step)
        if plan_reason.startswith("lc_corridor_semantic_"):
            nearest_index = int(np.argmin(np.linalg.norm(np.asarray(primitive_states[:, :2], dtype=float) - leader.xy()[None, :], axis=1)))
            self._shape_step = max(int(self._shape_step), int(nearest_index))
        while self._shape_step < max_step:
            target_row = primitive_states[self._shape_step]
            target_state = VehicleState(
                vehicle_id=int(leader.vehicle_id),
                x=float(target_row[0]),
                y=float(target_row[1]),
                yaw=float(target_row[2]),
                v=0.0,
                delta=0.0,
                mode=leader.mode,
            )
            if not self._near_goal(leader, target_state, pos_tol=0.18, yaw_tol_deg=12.0):
                break
            self._shape_step += 1
        if self._shape_step >= max_step:
            return self._freeze_vehicle(leader), True
        target_row = primitive_states[self._shape_step]
        command_row = primitive_commands[min(self._shape_step, len(primitive_commands) - 1)]
        target_state = VehicleState(
            vehicle_id=int(leader.vehicle_id),
            x=float(target_row[0]),
            y=float(target_row[1]),
            yaw=float(target_row[2]),
            v=0.0,
            delta=0.0,
            mode=leader.mode,
        )
        target_speed = float(command_row[2]) if primitive_commands.shape[1] >= 3 else float(-0.18)
        if plan_reason.startswith("lc_corridor_semantic_"):
            command = self._plan_step_signed(
                state=leader,
                goal=target_state,
                target_speed=float(target_speed),
                obstacles=obstacles,
                other=follower,
                allow_reverse=bool(target_speed < 0.0),
            )
            if abs(float(command.accel)) < 1e-9 and abs(float(command.steer_rate)) < 1e-9:
                remaining_path_xy = np.asarray(primitive_states[self._shape_step :, :2], dtype=float)
                if len(remaining_path_xy) < 2:
                    remaining_path_xy = np.vstack([leader.xy(), target_state.xy()])
                command = self._guarded_track(
                    state=leader,
                    path_xy=remaining_path_xy,
                    goal=target_state,
                    target_speed=float(target_speed),
                    obstacles=obstacles,
                    other=follower,
                )
        else:
            command = self._plan_step_signed(
                state=leader,
                goal=target_state,
                target_speed=float(target_speed),
                obstacles=obstacles,
                other=follower,
                allow_reverse=bool(target_speed < 0.0),
            )
        next_state = self.model.step(leader, command, float(self.cfg.control.dt))
        if float(np.linalg.norm(next_state.xy() - target_state.xy())) < float(np.linalg.norm(leader.xy() - target_state.xy())) - 1e-4:
            if self._near_goal(next_state, target_state, pos_tol=0.20, yaw_tol_deg=14.0):
                self._shape_step += 1
        return command, bool(self._shape_step >= max_step)

    def _legacy_command(self, *, leader: VehicleState, follower: VehicleState, plan, obstacles: list[Obstacle]) -> tuple[ControlCommand, ControlCommand, bool, str]:
        metadata = dict(getattr(plan, "metadata", {}) or {})
        reverse_goal = self._reverse_goal(plan)
        reverse_path_xy = np.asarray([[float(leader.x), float(leader.y)], [float(reverse_goal.x), float(reverse_goal.y)]], dtype=float)
        settle_path_xy = np.asarray([[float(reverse_goal.x), float(reverse_goal.y)], [float(plan.leader_goal.x), float(plan.leader_goal.y)]], dtype=float)
        if self._phase == "LEADER_SHAPE":
            self._legacy_reverse_time += float(self.cfg.control.dt)
            leader_command = self._plan_step_signed(
                state=leader,
                goal=reverse_goal,
                target_speed=float(metadata.get("reverse_target_speed", -0.20)),
                obstacles=obstacles,
                other=follower,
                allow_reverse=True,
            )
            if abs(float(leader_command.accel)) < 1e-9 and abs(float(leader_command.steer_rate)) < 1e-9:
                leader_command = self._guarded_track(
                    state=leader,
                    path_xy=reverse_path_xy,
                    goal=reverse_goal,
                    target_speed=float(metadata.get("reverse_target_speed", -0.20)),
                    obstacles=obstacles,
                    other=follower,
                )
            follower_command = self._freeze_vehicle(follower)
            dist_reverse = float(np.linalg.norm(leader.xy() - reverse_goal.xy()))
            if dist_reverse < self._legacy_reverse_best_dist - 0.03:
                self._legacy_reverse_best_dist = dist_reverse
                self._legacy_reverse_stall_time = 0.0
            else:
                self._legacy_reverse_stall_time += float(self.cfg.control.dt)
            reverse_saturated = bool(self._legacy_reverse_time >= 6.0 or self._legacy_reverse_stall_time >= 1.0)
            if self._near_goal(leader, reverse_goal, pos_tol=0.45, yaw_tol_deg=35.0) or reverse_saturated:
                self._phase = "LEADER_SETTLE"
            return leader_command, follower_command, False, "LC_LEADER_REVERSE"
        if self._phase == "LEADER_SETTLE":
            self._settle_time += float(self.cfg.control.dt)
            leader_command = self._plan_step_signed(
                state=leader,
                goal=plan.leader_goal,
                target_speed=float(metadata.get("settle_target_speed", 0.14)),
                obstacles=obstacles,
                other=follower,
                allow_reverse=False,
            )
            if abs(float(leader_command.accel)) < 1e-9 and abs(float(leader_command.steer_rate)) < 1e-9:
                leader_command = self._guarded_track(
                    state=leader,
                    path_xy=settle_path_xy,
                    goal=plan.leader_goal,
                    target_speed=float(metadata.get("settle_target_speed", 0.14)),
                    obstacles=obstacles,
                    other=follower,
                )
            follower_command = self._freeze_vehicle(follower)
            settle_ready = bool(self._near_goal(leader, plan.leader_goal, pos_tol=0.55, yaw_tol_deg=25.0) or self._settle_time >= 8.0)
            if settle_ready:
                self._phase = "FOLLOWER_INGRESS"
            return leader_command, follower_command, False, "LC_LEADER_SETTLE"
        leader_command = self._freeze_vehicle(leader)
        follower_goal = self._dynamic_predock_goal(leader=leader, follower=follower)
        follower_command, done, substage = self._follower_corridor_command(
            follower=follower,
            leader=leader,
            goal=follower_goal,
            path_xy=plan.follower_path_xy,
            obstacles=obstacles,
            target_speed=float(metadata.get("follower_target_speed", 0.22)),
        )
        return leader_command, follower_command, bool(done), str(substage)

    def command(self, *, leader: VehicleState, follower: VehicleState, plan, obstacles: list[Obstacle]) -> tuple[ControlCommand, ControlCommand, bool, str]:
        primitive_states = getattr(plan, "leader_primitive_states", None)
        primitive_steps = int(getattr(plan, "leader_primitive_steps", 0))
        if primitive_states is None or primitive_steps <= 0:
            return self._legacy_command(leader=leader, follower=follower, plan=plan, obstacles=obstacles)
        metadata = dict(getattr(plan, "metadata", {}) or {})
        plan_reason = str(getattr(plan, "reason", ""))
        is_exec_settle_plan = bool(
            plan_reason.startswith("lc_corridor_semantic_")
            or plan_reason in {"lc_corridor_exec_shape", "lc_corridor_smallgap_escape"}
        )
        tau_exec = float(metadata.get("exec_release_threshold", 0.30))
        tau_exec_robust = float(metadata.get("exec_robust_threshold", max(0.18, 0.6 * tau_exec)))
        tau_terminal = float(metadata.get("terminal_release_threshold", 0.16))
        tau_terminal_handoff = float(metadata.get("terminal_handoff_threshold", max(0.34, tau_terminal + 0.18)))
        if self._phase == "LEADER_SHAPE":
            self._shape_time += float(self.cfg.control.dt)
            if self._shape_origin is None:
                self._shape_origin = leader.copy()
            leader_command, shape_done = self._leader_shape_command(leader=leader, follower=follower, plan=plan, obstacles=obstacles)
            follower_command = self._freeze_vehicle(follower)
            shape_budget_s = float(metadata.get("shape_budget_s", 8.0))
            if shape_done or self._shape_time >= shape_budget_s:
                self._phase = "LEADER_SETTLE"
            return leader_command, follower_command, False, "LC_LEADER_SHAPE"
        if self._phase == "LEADER_SETTLE":
            self._settle_time += float(self.cfg.control.dt)
            follower_command = self._freeze_vehicle(follower)
            active_goal = self._leader_active_goal(plan)
            if is_exec_settle_plan:
                sigma_exec, robust_exec, exec_meta = self._robust_execution_certificate(
                    leader_pose=leader,
                    follower_pose=follower,
                    corridor_info=metadata,
                    obstacles=obstacles,
                )
                terminal_nominal, terminal_robust, terminal_meta = self._robust_terminal_safety_certificate(
                    anchor=leader,
                    follower=follower,
                    obstacles=obstacles,
                    corridor_polygons=[np.asarray(poly, dtype=float) for poly in metadata.get("corridor_polygons", []) if isinstance(poly, list)],
                )
                self._last_exec_sigma = float(sigma_exec)
                if sigma_exec > tau_exec and robust_exec > tau_exec_robust and terminal_nominal > tau_terminal and terminal_robust > max(0.5 * tau_terminal, 0.05):
                    self._phase = "FOLLOWER_INGRESS"
                    return self._leader_anchor_hold(leader=leader, goal=leader, obstacles=obstacles, follower=follower), follower_command, False, f"LC_BASIN_READY[e={sigma_exec:.3f}/{robust_exec:.3f},t={terminal_nominal:.3f}/{terminal_robust:.3f}]"
                nominal_escape_release_ok = bool(
                    float(robust_exec) <= 0.05
                    and float(sigma_exec) >= 0.35
                    and float(terminal_nominal) >= 0.22
                    and float(exec_meta.get("follower_path_clearance_m", 0.0)) >= float(self.cfg.safety.min_clearance) + 0.02
                    and float(exec_meta.get("handoff_clearance_m", 0.0)) >= float(self.cfg.safety.min_clearance) + 0.02
                    and float(terminal_meta.get("terminal_path_clearance_m", 0.0)) >= float(self.cfg.safety.min_clearance) + 0.02
                    and self._settle_search_attempts >= 1
                )
                if nominal_escape_release_ok:
                    self._phase = "FOLLOWER_INGRESS"
                    return self._leader_anchor_hold(leader=leader, goal=leader, obstacles=obstacles, follower=follower), follower_command, False, f"LC_BASIN_READY_NOMINAL[e={sigma_exec:.3f}/{robust_exec:.3f},t={terminal_nominal:.3f}/{terminal_robust:.3f}]"
                shape_motion_m = float(np.linalg.norm(leader.xy() - self._shape_origin.xy())) if self._shape_origin is not None else 0.0
                max_settle_attempts = 4 if float(robust_exec) <= 0.05 else 6
                if self._settle_candidate_cmd is None and self._settle_search_attempts < max_settle_attempts:
                    if self._search_certificate_ascent_anchor(leader=leader, follower=follower, plan=plan, obstacles=obstacles):
                        self._settle_search_attempts += 1
                        self._phase = "LEADER_CERT_ASCENT"
                        leader_command, _ = self._execute_settle_candidate(leader=leader, follower=follower, obstacles=obstacles)
                        return leader_command, follower_command, False, f"LC_CERT_ASCENT_PREP[e={sigma_exec:.3f}/{robust_exec:.3f},t={terminal_nominal:.3f}/{terminal_robust:.3f}]"
                    if (
                        float(sigma_exec) <= 1e-6
                        and float(robust_exec) <= 1e-6
                        and self._search_escape_settle_candidate(leader=leader, follower=follower, plan=plan, obstacles=obstacles)
                    ):
                        self._settle_search_attempts += 1
                        self._phase = "LEADER_CERT_ASCENT"
                        leader_command, _ = self._execute_settle_candidate(leader=leader, follower=follower, obstacles=obstacles)
                        return leader_command, follower_command, False, "LC_CERT_ESCAPE_PREP"
                    self._settle_search_attempts += 1
                leader_command = self._leader_anchor_hold(leader=leader, goal=active_goal, obstacles=obstacles, follower=follower)
                return leader_command, follower_command, False, f"LC_LEADER_SETTLE[e={sigma_exec:.3f}/{robust_exec:.3f},t={terminal_nominal:.3f}/{terminal_robust:.3f}]"
            leader_command = self._leader_settle_command(
                leader=leader,
                goal=active_goal,
                obstacles=obstacles,
                follower=follower,
                target_speed=float(metadata.get("settle_target_speed", 0.16)),
            )
            settle_budget_s = float(metadata.get("settle_budget_s", 5.0))
            basin_ready = self._basin_ready(leader=leader, follower=follower, plan=plan, obstacles=obstacles)
            if basin_ready or (self._settle_time >= settle_budget_s and self._near_goal(leader, active_goal, pos_tol=0.44, yaw_tol_deg=22.0)):
                self._phase = "FOLLOWER_INGRESS"
            return leader_command, follower_command, False, "LC_LEADER_SETTLE"
        if self._phase == "LEADER_CERT_ASCENT":
            leader_command, ascent_done = self._execute_settle_candidate(leader=leader, follower=follower, obstacles=obstacles)
            follower_command = self._freeze_vehicle(follower)
            if ascent_done:
                sigma_exec, robust_exec, _ = self._robust_execution_certificate(
                    leader_pose=leader,
                    follower_pose=follower,
                    corridor_info=metadata,
                    obstacles=obstacles,
                )
                terminal_nominal, terminal_robust, _ = self._robust_terminal_safety_certificate(
                    anchor=leader,
                    follower=follower,
                    obstacles=obstacles,
                    corridor_polygons=[np.asarray(poly, dtype=float) for poly in metadata.get("corridor_polygons", []) if isinstance(poly, list)],
                )
                self._last_exec_sigma = float(sigma_exec)
                self._settle_candidate_states = None
                self._settle_candidate_cmd = None
                self._settle_candidate_step = 0
                if sigma_exec > tau_exec and robust_exec > tau_exec_robust and terminal_nominal > tau_terminal and terminal_robust > max(0.5 * tau_terminal, 0.05):
                    self._phase = "FOLLOWER_INGRESS"
                else:
                    self._phase = "LEADER_SETTLE"
            return leader_command, follower_command, False, f"LC_CERT_ASCENT[e={self._settle_candidate_sigma:.3f}/{self._settle_candidate_robust_sigma:.3f}]"
        if self._phase == "LEADER_RESHAPE":
            leader_command, reshape_done = self._replay_sequence_command(leader=leader, follower=follower, obstacles=obstacles)
            follower_command = self._freeze_vehicle(follower)
            if reshape_done:
                sigma_exec, _ = self.compute_execution_certificate(
                    leader_actual_pose=leader,
                    follower_pose=follower,
                    corridor_info=metadata,
                    obstacles=obstacles,
                )
                self._last_exec_sigma = float(sigma_exec)
                self._reshape_states = None
                self._reshape_cmd = None
                if sigma_exec > tau_exec:
                    self._phase = "FOLLOWER_INGRESS"
                else:
                    self._phase = "LEADER_SETTLE"
            return leader_command, follower_command, False, f"LC_LEADER_RESHAPE[sigma={self._last_exec_sigma:.3f}]"
        if self._phase == "FOLLOWER_TERMINAL_CLOSURE":
            active_goal = self._leader_active_goal(plan)
            leader_command = self._leader_anchor_hold(leader=leader, goal=active_goal, obstacles=obstacles, follower=follower)
            tail_distance_m = float(np.linalg.norm(self.geom.front_hitch(follower) - self.geom.rear_hitch(leader)))
            if tail_distance_m < self._terminal_best_tail - 0.02:
                self._terminal_best_tail = float(tail_distance_m)
                self._terminal_stall_time = 0.0
            else:
                self._terminal_stall_time += float(self.cfg.control.dt)
            sigma_terminal, terminal_meta = self.compute_terminal_closure_certificate(
                leader_pose=leader,
                follower_pose=follower,
                corridor_info=metadata,
                obstacles=obstacles,
            )
            self._terminal_closure_best_sigma = float(max(self._terminal_closure_best_sigma, sigma_terminal))
            stall_trigger_s = 0.80 if (float(sigma_terminal) >= 0.70 and float(tail_distance_m) <= 0.80) else 1.50
            macro_escape_needed = bool(
                abs(float(terminal_meta.get("y_l_m", 0.0))) >= 0.85
                and str(terminal_meta.get("witness_name", "")) == "hold"
                and float(sigma_terminal) >= 0.60
            )
            handoff_ready, handoff_meta = self._docking_handoff_ready(leader=leader, follower=follower, obstacles=obstacles)
            lateral_deadlock = bool(
                bool(handoff_ready)
                and float(sigma_terminal) >= 0.70
                and abs(float(terminal_meta.get("y_l_m", 0.0))) >= 0.30
                and abs(float(terminal_meta.get("yaw_gap_deg", 0.0))) <= 6.0
                and float(handoff_meta.get("predicted_tail_improvement_m", 0.0)) <= 0.01
                and float(terminal_meta.get("lateral_score", 1.0)) <= 1e-3
            )
            if (self._terminal_stall_time >= stall_trigger_s or macro_escape_needed or lateral_deadlock) and self._reshape_attempts < 2:
                if self._search_terminal_restage(leader=leader, follower=follower, plan=plan, obstacles=obstacles):
                    self._reshape_attempts += 1
                    self._terminal_stall_time = 0.0
                    self._terminal_best_tail = float("inf")
                    self._phase = "LEADER_RESHAPE"
                    follower_command = self._freeze_vehicle(follower)
                    reshape_command, _ = self._replay_sequence_command(leader=leader, follower=follower, obstacles=obstacles)
                    return reshape_command, follower_command, False, "LC_LEADER_RESHAPE_TERMINAL"
            if (
                self._terminal_lock_ready_state(leader=leader, follower=follower, obstacles=obstacles)
                or (
                    handoff_ready
                    and self._handoff_release_gain_ok(sigma_terminal=sigma_terminal, terminal_meta=terminal_meta)
                    and self._terminal_closure_ready(
                        sigma_terminal=sigma_terminal,
                        terminal_meta=terminal_meta,
                        threshold=tau_terminal_handoff,
                    )
                )
            ):
                return leader_command, self._freeze_vehicle(follower), True, f"FOLLOWER_READY[sigma_m={sigma_terminal:.3f}]"
            if (
                handoff_ready
                and float(tail_distance_m) <= 0.80
                and float(self._terminal_stall_time) >= 0.80
            ):
                return self._freeze_vehicle(leader), self._freeze_vehicle(follower), True, "LEADER_POST_ALIGN_TIMEOUT"
            follower_command, closure_substage = self.terminal_capture_command(
                leader=leader,
                follower=follower,
                obstacles=obstacles,
                corridor_info=metadata,
            )
            follower_next = self.model.step(follower, follower_command, float(self.cfg.control.dt))
            next_sigma_terminal, next_terminal_meta = self.compute_terminal_closure_certificate(
                leader_pose=leader,
                follower_pose=follower_next,
                corridor_info=metadata,
                obstacles=obstacles,
                estimate_witness=False,
            )
            handoff_next_ready, handoff_next_meta = self._docking_handoff_ready(leader=leader, follower=follower_next, obstacles=obstacles)
            if self._terminal_lock_ready_state(leader=leader, follower=follower_next, obstacles=obstacles) or (
                handoff_next_ready
                and self._handoff_release_gain_ok(sigma_terminal=next_sigma_terminal, terminal_meta=next_terminal_meta)
                and self._terminal_closure_ready(
                    sigma_terminal=next_sigma_terminal,
                    terminal_meta=next_terminal_meta,
                    threshold=tau_terminal_handoff,
                )
            ):
                return self._freeze_vehicle(leader), self._freeze_vehicle(follower), True, f"FOLLOWER_READY[sigma_m={next_sigma_terminal:.3f}]"
            handoff_reason = handoff_next_meta.get("reason") if isinstance(handoff_next_meta, dict) else None
            if handoff_reason and closure_substage.startswith("LC_TERMINAL_MANIFOLD"):
                closure_substage = f"{closure_substage}:{handoff_reason}"
            return leader_command, follower_command, False, f"{closure_substage}[sigma_m={sigma_terminal:.3f},best={float(terminal_meta.get('witness_sigma', sigma_terminal)):.3f}]"
        active_goal = self._leader_active_goal(plan)
        leader_command = self._leader_anchor_hold(leader=leader, goal=active_goal, obstacles=obstacles, follower=follower)
        follower_goal = self._dynamic_predock_goal(leader=leader, follower=follower)
        follower_command, done, substage = self._follower_corridor_command(
            follower=follower,
            leader=leader,
            goal=follower_goal,
            path_xy=np.asarray(plan.follower_path_xy, dtype=float),
            obstacles=obstacles,
            target_speed=float(metadata.get("follower_target_speed", 0.28)),
        )
        if not done:
            return leader_command, follower_command, False, str(substage)
        sigma_terminal, terminal_meta = self.compute_terminal_closure_certificate(
            leader_pose=leader,
            follower_pose=follower,
            corridor_info=metadata,
            obstacles=obstacles,
        )
        self._terminal_closure_best_sigma = float(max(self._terminal_closure_best_sigma, sigma_terminal))
        handoff_ready, handoff_meta = self._docking_handoff_ready(leader=leader, follower=follower, obstacles=obstacles)
        if (
            self._terminal_lock_ready_state(leader=leader, follower=follower, obstacles=obstacles)
            or (
                handoff_ready
                and self._handoff_release_gain_ok(sigma_terminal=sigma_terminal, terminal_meta=terminal_meta)
                and self._terminal_closure_ready(
                    sigma_terminal=sigma_terminal,
                    terminal_meta=terminal_meta,
                    threshold=tau_terminal_handoff,
                )
            )
        ):
            return leader_command, self._freeze_vehicle(follower), True, f"FOLLOWER_READY[sigma_m={sigma_terminal:.3f}]"
        self._phase = "FOLLOWER_TERMINAL_CLOSURE"
        self._terminal_best_tail = float("inf")
        self._terminal_stall_time = 0.0
        self._reshape_attempts = 0
        closure_command, closure_substage = self.terminal_capture_command(
            leader=leader,
            follower=follower,
            obstacles=obstacles,
            corridor_info=metadata,
        )
        follower_next = self.model.step(follower, closure_command, float(self.cfg.control.dt))
        next_sigma_terminal, next_terminal_meta = self.compute_terminal_closure_certificate(
            leader_pose=leader,
            follower_pose=follower_next,
            corridor_info=metadata,
            obstacles=obstacles,
            estimate_witness=False,
        )
        handoff_next_ready, handoff_next_meta = self._docking_handoff_ready(leader=leader, follower=follower_next, obstacles=obstacles)
        if self._terminal_lock_ready_state(leader=leader, follower=follower_next, obstacles=obstacles) or (
            handoff_next_ready
            and self._handoff_release_gain_ok(sigma_terminal=next_sigma_terminal, terminal_meta=next_terminal_meta)
            and self._terminal_closure_ready(
                sigma_terminal=next_sigma_terminal,
                terminal_meta=next_terminal_meta,
                threshold=tau_terminal_handoff,
            )
        ):
            return self._freeze_vehicle(leader), self._freeze_vehicle(follower), True, f"FOLLOWER_READY[sigma_m={next_sigma_terminal:.3f}]"
        handoff_reason = handoff_next_meta.get("reason") if isinstance(handoff_next_meta, dict) else None
        if handoff_reason and closure_substage.startswith("LC_TERMINAL_MANIFOLD"):
            closure_substage = f"{closure_substage}:{handoff_reason}"
        return leader_command, closure_command, False, f"{closure_substage}[sigma_m={sigma_terminal:.3f},best={float(terminal_meta.get('witness_sigma', sigma_terminal)):.3f}]"
