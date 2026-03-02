from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import Config, DockingConfig
from .controllers import PathTrackingController
from .kinematics import VehicleGeometry
from .math_utils import angle_diff, clamp
from .planner import LocalPlanner
from .sensors import GlobalPoseSensor, VisionSensor
from .types import ControlCommand, DockingCondition, Obstacle, VehicleState


@dataclass
class DockingDebug:
    stage: str
    distance: float
    w_vis: float
    visual_valid: bool
    fallback_global: bool
    fallback_reason: str = ""
    visual_lost_time: float = 0.0


class DockingLockEvaluator:
    def __init__(self, cfg: DockingConfig, vehicle_geom: VehicleGeometry):
        self.cfg = cfg
        self.geom = vehicle_geom
        self.hold_timer = 0.0

    def reset(self) -> None:
        self.hold_timer = 0.0

    def update(self, follower: VehicleState, leader: VehicleState, dt: float) -> DockingCondition:
        f_h = self.geom.front_hitch(follower)
        l_h = self.geom.rear_hitch(leader)
        pos_err = float(np.linalg.norm(f_h - l_h))
        yaw_err = abs(angle_diff(leader.yaw, follower.yaw))
        speed_err = abs(leader.v - follower.v)

        pos_ok = pos_err < self.cfg.lock_position_tol
        # Enforce final docking body-angle rule: must be < 10 deg.
        yaw_tol_deg = min(self.cfg.lock_heading_tol_deg, 10.0)
        yaw_ok = yaw_err < math.radians(yaw_tol_deg)
        speed_ok = speed_err < self.cfg.lock_speed_tol
        if pos_ok and yaw_ok and speed_ok:
            self.hold_timer += dt
        else:
            self.hold_timer = 0.0

        return DockingCondition(
            pos_error=pos_err,
            yaw_error=yaw_err,
            speed_error=speed_err,
            hold_time=self.hold_timer,
            locked=self.hold_timer >= self.cfg.lock_hold_s,
        )


class TwoStageDockingController:
    def __init__(
        self,
        cfg: Config,
        global_sensor: GlobalPoseSensor,
        vision_sensor: VisionSensor,
        tracker: PathTrackingController,
        planner: LocalPlanner,
    ):
        self.cfg = cfg
        self.global_sensor = global_sensor
        self.vision_sensor = vision_sensor
        self.tracker = tracker
        self.planner = planner
        self.geom = VehicleGeometry(cfg.vehicle)
        self._visual_lost_time = 0.0
        self._last_global = None
        self._last_vision = None
        self._last_cmd: ControlCommand | None = None

    def _blend_weight(self, distance: float) -> float:
        d_min = self.cfg.docking.blend_distance_min
        d_max = self.cfg.docking.blend_distance_max
        if distance <= d_min:
            return 1.0
        if distance >= d_max:
            return 0.0
        return (d_max - distance) / (d_max - d_min)

    def _rear_hitch_from_global_meas(self, meas) -> tuple[np.ndarray, float]:
        x = meas.x
        y = meas.y
        yaw = meas.yaw
        rear_x = self.geom.rear_x - self.cfg.vehicle.hitch_length
        rear = np.array([x + math.cos(yaw) * rear_x, y + math.sin(yaw) * rear_x], dtype=float)
        return rear, yaw

    def _rear_hitch_from_vision(self, follower: VehicleState, vis_meas) -> tuple[np.ndarray, float]:
        cam = self.geom.front_hitch(follower)
        c = math.cos(follower.yaw)
        s = math.sin(follower.yaw)
        rel = np.array([vis_meas.rel_x, vis_meas.rel_y], dtype=float)
        rel_world = np.array([c * rel[0] - s * rel[1], s * rel[0] + c * rel[1]], dtype=float)
        rear = cam + rel_world
        yaw = follower.yaw + vis_meas.rel_yaw
        return rear, yaw

    def _smooth_command(self, cmd: ControlCommand) -> ControlCommand:
        if self._last_cmd is None:
            self._last_cmd = cmd
            return cmd
        max_da = max(0.0, self.cfg.docking.cmd_smooth_max_accel_step)
        max_ds = max(0.0, self.cfg.docking.cmd_smooth_max_steer_rate_step)
        accel = self._last_cmd.accel + clamp(cmd.accel - self._last_cmd.accel, -max_da, max_da)
        steer_rate = self._last_cmd.steer_rate + clamp(cmd.steer_rate - self._last_cmd.steer_rate, -max_ds, max_ds)
        out = ControlCommand(accel=accel, steer_rate=steer_rate)
        self._last_cmd = out
        return out

    def compute_command(
        self,
        follower: VehicleState,
        leader_true: VehicleState,
        timestamp: float,
        obstacles: list[Obstacle],
        dynamic_others: list[VehicleState],
    ) -> tuple[ControlCommand, DockingDebug]:
        # True distance only for stage scheduling/fusion window.
        d_true = float(np.linalg.norm(self.geom.front_hitch(follower) - self.geom.rear_hitch(leader_true)))

        g = self.global_sensor.observe(leader_true, timestamp)
        if g is not None:
            self._last_global = g
        v = self.vision_sensor.observe_rear_hitch(follower, leader_true, obstacles, timestamp)
        if v is not None:
            self._last_vision = v

        if self._last_global is None:
            return self._smooth_command(ControlCommand(accel=0.0, steer_rate=0.0)), DockingDebug(
                stage="WAIT_GLOBAL",
                distance=d_true,
                w_vis=0.0,
                visual_valid=False,
                fallback_global=True,
                fallback_reason="missing_global",
                visual_lost_time=self._visual_lost_time,
            )

        rear_global, yaw_global = self._rear_hitch_from_global_meas(self._last_global)
        rear_target = rear_global
        yaw_target = yaw_global

        visual_valid = bool(self._last_vision and self._last_vision.valid)
        w_vis = self._blend_weight(d_true)
        fallback = False

        if visual_valid:
            rear_vis, yaw_vis = self._rear_hitch_from_vision(follower, self._last_vision)
            rear_target = (1.0 - w_vis) * rear_global + w_vis * rear_vis
            yaw_target = float((1.0 - w_vis) * yaw_global + w_vis * yaw_vis)
            self._visual_lost_time = 0.0
        else:
            self._visual_lost_time += self.cfg.control.dt
            fallback = d_true <= self.cfg.docking.stage_switch_distance
            w_vis = 0.0

        # Predictive intercept in global stage: aim slightly ahead on target trajectory.
        if d_true > self.cfg.docking.stage_switch_distance:
            t_pred = clamp(d_true / max(follower.v + 0.4, 0.4), 0.4, 2.0)
            rear_target = rear_target + np.array(
                [math.cos(leader_true.yaw), math.sin(leader_true.yaw)], dtype=float
            ) * (leader_true.v * t_pred)

        yaw_err_abs = abs(angle_diff(yaw_target, follower.yaw))
        rel_to_target = rear_target - self.geom.front_hitch(follower)
        lat_err = abs(
            -math.sin(yaw_target) * rel_to_target[0] + math.cos(yaw_target) * rel_to_target[1]
        )
        align_limit = math.radians(self.cfg.docking.max_alignment_angle_deg)
        can_enter_visual = (
            d_true <= self.cfg.docking.stage_switch_distance
            and yaw_err_abs < math.radians(35.0)
            and lat_err < 0.65
        )

        heading_vec = np.array([math.cos(yaw_target), math.sin(yaw_target)], dtype=float)
        fallback_reason = ""

        angle_recovery_distance = min(self.cfg.docking.stage_switch_distance, self.cfg.docking.soft_capture_distance + 0.2)
        if d_true <= angle_recovery_distance and yaw_err_abs >= align_limit:
            fallback = True
            fallback_reason = "angle_exceeded"

        if (
            d_true <= self.cfg.docking.stage_switch_distance
            and not visual_valid
            and self._visual_lost_time >= self.cfg.docking.recovery_visual_loss_timeout_s
        ):
            fallback = True
            if not fallback_reason:
                fallback_reason = "vision_lost"

        # Desired speed schedule.
        if not can_enter_visual:
            stage = "GLOBAL_APPROACH"
            target_speed = clamp(0.6 + 0.38 * d_true, 0.25, 1.5)
            approach_offset = clamp(0.55 + 0.25 * d_true, 0.55, 1.7)
            approach_target = rear_target - heading_vec * approach_offset

            plan = self.planner.plan_step(
                ego=follower,
                goal_xy=approach_target,
                goal_yaw=yaw_target,
                obstacles=obstacles,
                dynamic_others=dynamic_others,
            )
            if plan.feasible:
                cmd = plan.command
            else:
                fallback_reason = "global_plan_infeasible"
                cmd = self.tracker.track_point(
                    follower,
                    approach_target[0],
                    approach_target[1],
                    yaw_target,
                    target_speed,
                )
        else:
            stage = "VISUAL_SERVO"
            rel_world = rear_target - self.geom.front_hitch(follower)
            c = math.cos(-follower.yaw)
            s = math.sin(-follower.yaw)
            rel_x = c * rel_world[0] - s * rel_world[1]
            rel_y = s * rel_world[0] + c * rel_world[1]
            dist = float(np.hypot(rel_x, rel_y))

            delta_ff = math.atan2(2.0 * self.cfg.vehicle.wheelbase * rel_y, rel_x * rel_x + rel_y * rel_y + 1e-4)
            delta_ref = delta_ff + 0.55 * angle_diff(yaw_target, follower.yaw)
            dmax = math.radians(self.cfg.vehicle.max_steer_deg)
            delta_ref = clamp(delta_ref, -dmax, dmax)
            steer_rate = clamp(
                (delta_ref - follower.delta) / self.cfg.control.dt,
                -math.radians(self.cfg.vehicle.max_steer_rate_deg_s),
                math.radians(self.cfg.vehicle.max_steer_rate_deg_s),
            )

            target_speed = clamp(leader_true.v + 0.9 * rel_x, -0.25, 0.9)
            if abs(rel_y) > 0.35 or yaw_err_abs > math.radians(10.0):
                target_speed = min(target_speed, 0.12)
                target_speed = max(target_speed, -0.08)
            kp = self.cfg.control.speed.kp
            accel = clamp(
                kp * (target_speed - follower.v),
                -self.cfg.vehicle.max_decel,
                self.cfg.control.speed.max_accel_cmd,
            )
            cmd = ControlCommand(accel=accel, steer_rate=steer_rate)

        return self._smooth_command(cmd), DockingDebug(
            stage=stage,
            distance=d_true,
            w_vis=w_vis,
            visual_valid=visual_valid,
            fallback_global=fallback,
            fallback_reason=fallback_reason,
            visual_lost_time=self._visual_lost_time,
        )
