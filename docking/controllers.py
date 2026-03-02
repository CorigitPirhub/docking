from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import ControlConfig, VehicleConfig
from .math_utils import angle_diff, clamp
from .types import ControlCommand, VehicleState


@dataclass
class PathTrackingController:
    vehicle_cfg: VehicleConfig
    control_cfg: ControlConfig
    steering_mode: str = "pure_pursuit"  # pure_pursuit | stanley

    def _nearest_index(self, state: VehicleState, path_xy: np.ndarray) -> int:
        d = np.linalg.norm(path_xy - state.xy(), axis=1)
        return int(np.argmin(d))

    def _path_heading(self, path_xy: np.ndarray, idx: int) -> float:
        j = min(idx + 1, len(path_xy) - 1)
        if j == idx:
            i = max(0, idx - 1)
            vec = path_xy[idx] - path_xy[i]
        else:
            vec = path_xy[j] - path_xy[idx]
        return math.atan2(vec[1], vec[0]) if np.linalg.norm(vec) > 1e-9 else 0.0

    def _pure_pursuit_delta(self, state: VehicleState, path_xy: np.ndarray) -> float:
        nearest = self._nearest_index(state, path_xy)
        lookahead = self.control_cfg.pure_pursuit.lookahead_min + self.control_cfg.pure_pursuit.lookahead_gain * state.v
        acc = 0.0
        target_idx = nearest
        while target_idx + 1 < len(path_xy) and acc < lookahead:
            seg = np.linalg.norm(path_xy[target_idx + 1] - path_xy[target_idx])
            acc += float(seg)
            target_idx += 1

        tgt = path_xy[target_idx]
        vec = tgt - state.xy()
        alpha = angle_diff(math.atan2(vec[1], vec[0]), state.yaw)
        ld = max(np.linalg.norm(vec), 1e-3)
        delta = math.atan2(2.0 * self.vehicle_cfg.wheelbase * math.sin(alpha), ld)
        dmax = math.radians(self.vehicle_cfg.max_steer_deg)
        return clamp(delta, -dmax, dmax)

    def _stanley_delta(self, state: VehicleState, path_xy: np.ndarray) -> float:
        # Stanley uses the front-axle projection as the tracking point.
        front = state.xy() + self.vehicle_cfg.wheelbase * np.array([math.cos(state.yaw), math.sin(state.yaw)])
        d = np.linalg.norm(path_xy - front, axis=1)
        idx = int(np.argmin(d))

        path_yaw = self._path_heading(path_xy, idx)
        heading_error = angle_diff(path_yaw, state.yaw)

        if idx + 1 < len(path_xy):
            seg = path_xy[idx + 1] - path_xy[idx]
        else:
            seg = path_xy[idx] - path_xy[max(0, idx - 1)]
        seg_norm = np.linalg.norm(seg)
        if seg_norm < 1e-9:
            cte = 0.0
        else:
            n = np.array([-seg[1], seg[0]], dtype=float) / seg_norm
            # Positive CTE should steer the vehicle toward the path normal direction.
            cte = float(np.dot(path_xy[idx] - front, n))

        v_term = abs(state.v) + self.control_cfg.stanley.softening
        cte_term = math.atan2(self.control_cfg.stanley.k * cte, v_term)
        delta = heading_error + cte_term
        dmax = math.radians(self.vehicle_cfg.max_steer_deg)
        return clamp(delta, -dmax, dmax)

    def track_path(self, state: VehicleState, path_xy: np.ndarray, target_speed: float) -> ControlCommand:
        if len(path_xy) < 2:
            return self.track_point(state, state.x + 1.0, state.y, state.yaw, target_speed)

        if self.steering_mode == "stanley":
            delta_ref = self._stanley_delta(state, path_xy)
        else:
            delta_ref = self._pure_pursuit_delta(state, path_xy)

        max_rate = math.radians(self.vehicle_cfg.max_steer_rate_deg_s)
        steer_rate = clamp((delta_ref - state.delta) / self.control_cfg.dt, -max_rate, max_rate)
        accel = self._speed_accel(state.v, target_speed)
        return ControlCommand(accel=accel, steer_rate=steer_rate)

    def track_point(
        self,
        state: VehicleState,
        target_x: float,
        target_y: float,
        target_yaw: float,
        target_speed: float,
    ) -> ControlCommand:
        vec = np.array([target_x - state.x, target_y - state.y], dtype=float)
        dist = float(np.linalg.norm(vec))
        desired_yaw = math.atan2(vec[1], vec[0]) if dist > 1e-6 else target_yaw
        yaw_err = angle_diff(desired_yaw, state.yaw)

        # Point attractor with damping on heading.
        k_yaw = 1.3
        k_dist = 0.4
        delta_ref = yaw_err + math.atan2(k_dist * dist, max(state.v, 0.3))
        dmax = math.radians(self.vehicle_cfg.max_steer_deg)
        delta_ref = clamp(delta_ref, -dmax, dmax)

        max_rate = math.radians(self.vehicle_cfg.max_steer_rate_deg_s)
        steer_rate = clamp((delta_ref - state.delta) / self.control_cfg.dt, -max_rate, max_rate)
        accel = self._speed_accel(state.v, target_speed)
        return ControlCommand(accel=accel, steer_rate=steer_rate)

    def _speed_accel(self, v: float, v_ref: float) -> float:
        v_ref = clamp(v_ref, 0.0, self.vehicle_cfg.max_speed)
        accel = self.control_cfg.speed.kp * (v_ref - v)
        accel = clamp(accel, -self.vehicle_cfg.max_decel, self.control_cfg.speed.max_accel_cmd)
        return accel
