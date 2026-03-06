from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

import numpy as np

from .collision import CollisionEngine
from .config import Config
from .controllers import PathTrackingController
from .kinematics import VehicleGeometry
from .math_utils import angle_diff, clamp, wrap_angle
from .sensors import GlobalPoseSensor, VisionSensor
from .types import ControlCommand, Obstacle, VehicleState


def _sigmoid(x: float) -> float:
    xx = float(x)
    if xx >= 30.0:
        return 1.0
    if xx <= -30.0:
        return 0.0
    return 1.0 / (1.0 + math.exp(-xx))


DockingMode = Literal["APPROACH", "ALIGN", "DOCK"]


@dataclass(frozen=True)
class BCFDConfig:
    """
    Belief-Consistent Funnel Docking (BCFD) — deterministic closed-loop core.

    The "innovation hook" for P-1.1 is the *belief-consistent* GNSS↔Vision fusion plus the
    *funnel-gated* standoff (APPROACH/ALIGN/DOCK) that prevents close-range misalignment
    deadlocks for Ackermann vehicles.
    """

    # Fusion gating
    nis_threshold: float = 9.21
    k_dist: float = 8.0
    k_nis: float = 2.2

    # Mode thresholds (leader-frame, using fused yaw)
    align_enter_dist_m: float = 1.4
    dock_enter_dist_m: float = 0.95
    align_lat_tol_m: float = 0.10
    align_yaw_tol_deg: float = 10.0
    dock_abort_lat_m: float = 0.18
    dock_abort_yaw_deg: float = 18.0

    # Standoff targets (leader-frame x, desired rel_x_l for follower front-hitch)
    standoff_approach_min: float = 0.90
    standoff_approach_max: float = 1.60
    standoff_align: float = 0.90

    # Speed shaping: v_ref = v_leader + k_x * (x_l - x_ref)
    k_x: float = 0.95
    v_cap_approach: float = 1.1
    v_cap_align: float = 0.28
    v_cap_dock: float = 0.14

    # Stanley-like lateral/yaw regulation in leader frame
    k_cte: float = 2.2
    stanley_softening: float = 0.25

    # Near-field gate: don't enter contact band if misaligned; back off instead
    gate_contact_dist_m: float = 0.55
    gate_lat_m: float = 0.05
    gate_yaw_deg: float = 5.0
    gate_backoff_speed: float = -0.12


@dataclass
class DockingDebugBCFD:
    stage: str
    mode: str
    distance: float
    w_vis: float
    visual_valid: bool
    fusion_gamma: float
    nis: float
    fallback_global: bool
    fallback_reason: str = ""
    visual_lost_time: float = 0.0
    step_cost: float = 0.0
    min_clearance: float = 1e6


class BeliefConsistentFunnelDockingController:
    def __init__(
        self,
        cfg: Config,
        global_sensor: GlobalPoseSensor,
        vision_sensor: VisionSensor,
        tracker: PathTrackingController,
        collision: CollisionEngine,
        *,
        bcfd_cfg: BCFDConfig | None = None,
    ) -> None:
        self.cfg = cfg
        self.global_sensor = global_sensor
        self.vision_sensor = vision_sensor
        self.tracker = tracker
        self.collision = collision
        self.bcfd = bcfd_cfg or BCFDConfig()

        self.geom = VehicleGeometry(cfg.vehicle)
        self._x_front_hitch = float(self.geom.front_hitch_x)

        self._visual_lost_time = 0.0
        self._last_global = None
        self._last_vision = None
        self._last_cmd: ControlCommand | None = None
        self._mode: DockingMode = "APPROACH"

    def reset(self) -> None:
        self._visual_lost_time = 0.0
        self._last_global = None
        self._last_vision = None
        self._last_cmd = None
        self._mode = "APPROACH"

    def _rear_hitch_from_global_meas(self, meas) -> tuple[np.ndarray, float]:
        x = float(meas.x)
        y = float(meas.y)
        yaw = float(meas.yaw)
        rear_x = float(self.geom.rear_x - self.cfg.vehicle.hitch_length)
        rear = np.array([x + math.cos(yaw) * rear_x, y + math.sin(yaw) * rear_x], dtype=float)
        return rear, yaw

    def _rear_hitch_from_vision(self, follower: VehicleState, vis_meas) -> tuple[np.ndarray, float]:
        cam = self.geom.front_hitch(follower)
        c = math.cos(float(follower.yaw))
        s = math.sin(float(follower.yaw))
        rel = np.array([float(vis_meas.rel_x), float(vis_meas.rel_y)], dtype=float)
        rel_world = np.array([c * rel[0] - s * rel[1], s * rel[0] + c * rel[1]], dtype=float)
        rear = cam + rel_world
        yaw = wrap_angle(float(follower.yaw) + float(vis_meas.rel_yaw))
        return rear, yaw

    def _blend_schedule(self, d: float) -> float:
        d_min = float(self.cfg.docking.blend_distance_min)
        d_max = float(self.cfg.docking.blend_distance_max)
        if d <= d_min:
            return 1.0
        if d >= d_max:
            return 0.0
        return float((d_max - d) / max(d_max - d_min, 1e-9))

    def _robust_fusion(
        self,
        *,
        follower: VehicleState,
        leader_true: VehicleState,
        d_true: float,
        timestamp: float,
        obstacles: list[Obstacle],
    ) -> tuple[np.ndarray, float, float, float, float, bool]:
        g = self.global_sensor.observe(leader_true, timestamp)
        if g is not None:
            self._last_global = g
        v = self.vision_sensor.observe_rear_hitch(follower, leader_true, obstacles, timestamp)
        if v is not None:
            self._last_vision = v

        if self._last_global is None:
            return (
                np.array([float(leader_true.x), float(leader_true.y)], dtype=float),
                float(leader_true.yaw),
                0.0,
                0.0,
                float("inf"),
                False,
            )

        rear_g, yaw_g = self._rear_hitch_from_global_meas(self._last_global)

        visual_valid = bool(self._last_vision and bool(self._last_vision.valid))
        if not visual_valid:
            self._visual_lost_time += float(self.cfg.control.dt)
            return rear_g, yaw_g, 0.0, 0.0, 0.0, True

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

        s_dist = self._blend_schedule(float(d_true))
        gamma_dist = _sigmoid(float(self.bcfd.k_dist) * (float(s_dist) - 0.4))
        gamma_nis = _sigmoid(float(self.bcfd.k_nis) * (float(self.bcfd.nis_threshold) - float(nis)))
        gamma = float(clamp(gamma_dist * gamma_nis, 0.0, 1.0))

        info_w = float(np.trace(rg) / max(np.trace(rg) + np.trace(rv), 1e-9))
        w_vis = float(clamp(gamma * info_w, 0.0, 1.0))

        rear = (1.0 - w_vis) * rear_g + w_vis * rear_v
        yaw_err = angle_diff(yaw_v, yaw_g)
        yaw = wrap_angle(float(yaw_g) + float(w_vis) * float(yaw_err))
        return rear, yaw, w_vis, float(gamma), float(nis), True

    def _smooth_command(self, cmd: ControlCommand) -> ControlCommand:
        if self._last_cmd is None:
            self._last_cmd = cmd
            return cmd
        max_da = max(0.0, float(self.cfg.docking.cmd_smooth_max_accel_step))
        max_ds = max(0.0, float(self.cfg.docking.cmd_smooth_max_steer_rate_step))
        accel = float(self._last_cmd.accel) + clamp(float(cmd.accel) - float(self._last_cmd.accel), -max_da, max_da)
        steer_rate = float(self._last_cmd.steer_rate) + clamp(
            float(cmd.steer_rate) - float(self._last_cmd.steer_rate), -max_ds, max_ds
        )
        out = ControlCommand(accel=float(accel), steer_rate=float(steer_rate))
        self._last_cmd = out
        return out

    def compute_command(
        self,
        follower: VehicleState,
        leader_true: VehicleState,
        timestamp: float,
        obstacles: list[Obstacle],
        dynamic_others: list[VehicleState],
    ) -> tuple[ControlCommand, DockingDebugBCFD]:
        _ = dynamic_others
        d_true = float(np.linalg.norm(self.geom.front_hitch(follower) - self.geom.rear_hitch(leader_true)))

        rear, yaw_target, w_vis, gamma, nis, have_global = self._robust_fusion(
            follower=follower,
            leader_true=leader_true,
            d_true=d_true,
            timestamp=timestamp,
            obstacles=obstacles,
        )

        if not have_global:
            cmd = self._smooth_command(ControlCommand(accel=0.0, steer_rate=0.0))
            return cmd, DockingDebugBCFD(
                stage="WAIT_GLOBAL",
                mode=str(self._mode),
                distance=float(d_true),
                w_vis=0.0,
                visual_valid=False,
                fusion_gamma=0.0,
                nis=float("inf"),
                fallback_global=True,
                fallback_reason="missing_global",
                visual_lost_time=float(self._visual_lost_time),
            )

        visual_valid = bool(self._last_vision and bool(getattr(self._last_vision, "valid", False)))
        stage = "VISUAL_FUSED" if (d_true <= float(self.cfg.docking.stage_switch_distance)) else "GLOBAL_FUSED"

        # Relative pose in leader frame (front-hitch as control point).
        fh = self.geom.front_hitch(follower)
        rel_world = rear - fh
        lc = math.cos(float(yaw_target))
        ls = math.sin(float(yaw_target))
        x_l = float(lc * float(rel_world[0]) + ls * float(rel_world[1]))
        y_l = float(-ls * float(rel_world[0]) + lc * float(rel_world[1]))
        yaw_diff = float(angle_diff(float(yaw_target), float(follower.yaw)))
        yaw_err = abs(float(yaw_diff))
        dist = float(np.linalg.norm(rel_world))

        # Mode transitions.
        if self._mode == "APPROACH" and dist <= float(self.bcfd.align_enter_dist_m):
            self._mode = "ALIGN"
        if (
            self._mode == "ALIGN"
            and dist <= float(self.bcfd.dock_enter_dist_m)
            and abs(y_l) <= float(self.bcfd.align_lat_tol_m)
            and yaw_err <= math.radians(float(self.bcfd.align_yaw_tol_deg))
        ):
            self._mode = "DOCK"
        if (
            self._mode == "DOCK"
            and dist <= float(self.bcfd.dock_enter_dist_m)
            and (abs(y_l) > float(self.bcfd.dock_abort_lat_m) or yaw_err > math.radians(float(self.bcfd.dock_abort_yaw_deg)))
        ):
            self._mode = "ALIGN"

        # Standoff reference in leader frame.
        if self._mode == "APPROACH":
            x_ref = float(
                clamp(
                    0.70 + 0.18 * float(dist),
                    float(self.bcfd.standoff_approach_min),
                    float(self.bcfd.standoff_approach_max),
                )
            )
            v_cap = float(self.bcfd.v_cap_approach)
        elif self._mode == "ALIGN":
            x_ref = float(self.bcfd.standoff_align)
            v_cap = float(self.bcfd.v_cap_align)
        else:
            x_ref = 0.0
            v_cap = float(self.bcfd.v_cap_dock)

        ex = float(x_l - x_ref)
        v_ref = float(leader_true.v) + float(self.bcfd.k_x) * float(ex)
        v_ref = float(clamp(v_ref, -float(self.cfg.vehicle.max_reverse_speed), float(v_cap)))

        # Gate: if close but misaligned, back off to create room.
        if dist <= float(self.bcfd.gate_contact_dist_m):
            if abs(y_l) > float(self.bcfd.gate_lat_m) or yaw_err > math.radians(float(self.bcfd.gate_yaw_deg)):
                v_ref = float(self.bcfd.gate_backoff_speed)

        # Stanley-like steering in leader frame (use speed magnitude for robustness).
        v_term = abs(float(follower.v)) + float(self.bcfd.stanley_softening)
        delta_ref = float(yaw_diff + math.atan2(float(self.bcfd.k_cte) * float(y_l), float(v_term)))
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

        # Near-field fallback: if vision is expected but currently lost, slow down.
        fallback_global = False
        fallback_reason = ""
        if (d_true <= float(self.cfg.docking.stage_switch_distance)) and (not visual_valid) and (self._visual_lost_time > 0.0):
            fallback_global = True
            fallback_reason = "vision_lost"
            accel = min(float(accel), 0.0)

        cmd = ControlCommand(accel=float(accel), steer_rate=float(steer_rate))

        # Lock assist: stop + straighten to satisfy hold condition.
        if float(d_true) <= 0.08 and float(yaw_err) <= math.radians(5.0):
            steer_rate = float(clamp(-float(follower.delta) / float(self.cfg.control.dt), -max_rate, max_rate))
            accel = float(
                clamp(
                    float(self.cfg.control.speed.kp) * (float(leader_true.v) - float(follower.v)),
                    -float(self.cfg.vehicle.max_decel),
                    float(self.cfg.control.speed.max_accel_cmd),
                )
            )
            cmd = ControlCommand(accel=float(accel), steer_rate=float(steer_rate))

        cmd = self._smooth_command(cmd)
        return cmd, DockingDebugBCFD(
            stage=str(stage),
            mode=str(self._mode),
            distance=float(d_true),
            w_vis=float(w_vis),
            visual_valid=bool(visual_valid),
            fusion_gamma=float(gamma),
            nis=float(nis),
            fallback_global=bool(fallback_global),
            fallback_reason=str(fallback_reason),
            visual_lost_time=float(self._visual_lost_time),
            step_cost=0.0,
            min_clearance=float(self.collision.min_clearance_vehicle_obstacles(follower, obstacles)),
        )


class BaselineDockingController:
    """Simple baseline family for P-1.2 comparisons."""

    def __init__(
        self,
        cfg: Config,
        global_sensor: GlobalPoseSensor,
        vision_sensor: VisionSensor,
        tracker: PathTrackingController,
        collision: CollisionEngine,
        *,
        mode: str,
    ) -> None:
        if mode not in {"global_only", "hard_switch", "pure_pursuit", "dist_blend"}:
            raise ValueError(f"Unsupported baseline mode: {mode}")
        self.cfg = cfg
        self.mode = str(mode)
        self.global_sensor = global_sensor
        self.vision_sensor = vision_sensor
        self.tracker = tracker
        self.collision = collision
        self.geom = VehicleGeometry(cfg.vehicle)

        self._last_global = None
        self._last_vision = None
        self._visual_lost_time = 0.0
        self._last_cmd: ControlCommand | None = None

    def reset(self) -> None:
        self._last_global = None
        self._last_vision = None
        self._visual_lost_time = 0.0
        self._last_cmd = None

    def _rear_hitch_from_global_meas(self, meas) -> tuple[np.ndarray, float]:
        rear_x = float((VehicleGeometry(self.cfg.vehicle).rear_x) - self.cfg.vehicle.hitch_length)
        yaw = float(meas.yaw)
        rear = np.array([float(meas.x) + math.cos(yaw) * rear_x, float(meas.y) + math.sin(yaw) * rear_x], dtype=float)
        return rear, yaw

    def _rear_hitch_from_vision(self, follower: VehicleState, vis_meas) -> tuple[np.ndarray, float]:
        geom = VehicleGeometry(self.cfg.vehicle)
        cam = geom.front_hitch(follower)
        c = math.cos(float(follower.yaw))
        s = math.sin(float(follower.yaw))
        rel = np.array([float(vis_meas.rel_x), float(vis_meas.rel_y)], dtype=float)
        rel_world = np.array([c * rel[0] - s * rel[1], s * rel[0] + c * rel[1]], dtype=float)
        rear = cam + rel_world
        yaw = wrap_angle(float(follower.yaw) + float(vis_meas.rel_yaw))
        return rear, yaw

    def _smooth_command(self, cmd: ControlCommand) -> ControlCommand:
        if self._last_cmd is None:
            self._last_cmd = cmd
            return cmd
        max_da = max(0.0, float(self.cfg.docking.cmd_smooth_max_accel_step))
        max_ds = max(0.0, float(self.cfg.docking.cmd_smooth_max_steer_rate_step))
        accel = float(self._last_cmd.accel) + clamp(float(cmd.accel) - float(self._last_cmd.accel), -max_da, max_da)
        steer_rate = float(self._last_cmd.steer_rate) + clamp(
            float(cmd.steer_rate) - float(self._last_cmd.steer_rate), -max_ds, max_ds
        )
        out = ControlCommand(accel=float(accel), steer_rate=float(steer_rate))
        self._last_cmd = out
        return out

    def compute_command(
        self,
        follower: VehicleState,
        leader_true: VehicleState,
        timestamp: float,
        obstacles: list[Obstacle],
        dynamic_others: list[VehicleState],
    ) -> tuple[ControlCommand, DockingDebugBCFD]:
        _ = dynamic_others
        d_true = float(np.linalg.norm(self.geom.front_hitch(follower) - self.geom.rear_hitch(leader_true)))

        g = self.global_sensor.observe(leader_true, timestamp)
        if g is not None:
            self._last_global = g
        v = self.vision_sensor.observe_rear_hitch(follower, leader_true, obstacles, timestamp)
        if v is not None:
            self._last_vision = v

        if self._last_global is None:
            cmd = ControlCommand(accel=0.0, steer_rate=0.0)
            return self._smooth_command(cmd), DockingDebugBCFD(
                stage="WAIT_GLOBAL",
                mode="baseline",
                distance=d_true,
                w_vis=0.0,
                visual_valid=False,
                fusion_gamma=0.0,
                nis=float("inf"),
                fallback_global=True,
                fallback_reason="missing_global",
                visual_lost_time=float(self._visual_lost_time),
                min_clearance=float(self.collision.min_clearance_vehicle_obstacles(follower, obstacles)),
            )

        rear_g, yaw_g = self._rear_hitch_from_global_meas(self._last_global)
        visual_valid = bool(self._last_vision and bool(self._last_vision.valid))
        if not visual_valid:
            self._visual_lost_time += float(self.cfg.control.dt)
        else:
            self._visual_lost_time = 0.0

        rear = rear_g
        yaw = yaw_g
        w_vis = 0.0

        if self.mode == "global_only":
            pass
        elif self.mode == "hard_switch":
            if d_true <= float(self.cfg.docking.stage_switch_distance) and visual_valid:
                rear, yaw = self._rear_hitch_from_vision(follower, self._last_vision)
                w_vis = 1.0
        elif self.mode == "dist_blend":
            if visual_valid:
                rear_v, yaw_v = self._rear_hitch_from_vision(follower, self._last_vision)
                d_min = float(self.cfg.docking.blend_distance_min)
                d_max = float(self.cfg.docking.blend_distance_max)
                if d_true <= d_min:
                    alpha = 1.0
                elif d_true >= d_max:
                    alpha = 0.0
                else:
                    alpha = float((d_max - d_true) / max(d_max - d_min, 1e-9))
                w_vis = float(clamp(alpha, 0.0, 1.0))
                rear = (1.0 - w_vis) * rear_g + w_vis * rear_v
                yaw = wrap_angle(float(yaw_g) + float(w_vis) * float(angle_diff(float(yaw_v), float(yaw_g))))

        heading = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
        standoff = float(clamp(0.70 + 0.18 * d_true, 0.60, 1.6))
        approach = rear - heading * standoff

        target_speed = float(clamp(0.75 + 0.30 * d_true, 0.18, 1.3))
        if self.mode == "pure_pursuit":
            target_speed = float(clamp(0.6 + 0.25 * d_true, 0.15, 1.2))

        cmd = self.tracker.track_point(follower, float(approach[0]), float(approach[1]), float(yaw), target_speed)
        return self._smooth_command(cmd), DockingDebugBCFD(
            stage=f"baseline_{self.mode}",
            mode="baseline",
            distance=float(d_true),
            w_vis=float(w_vis),
            visual_valid=bool(visual_valid),
            fusion_gamma=float(w_vis),
            nis=0.0,
            fallback_global=bool((d_true <= float(self.cfg.docking.stage_switch_distance)) and (not visual_valid)),
            fallback_reason="vision_lost"
            if ((d_true <= float(self.cfg.docking.stage_switch_distance)) and (not visual_valid))
            else "",
            visual_lost_time=float(self._visual_lost_time),
            step_cost=0.0,
            min_clearance=float(self.collision.min_clearance_vehicle_obstacles(follower, obstacles)),
        )
