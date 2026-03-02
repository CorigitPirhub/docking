from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import VehicleConfig
from .math_utils import clamp, wrap_angle
from .types import ControlCommand, VehicleState


@dataclass
class AckermannModel:
    cfg: VehicleConfig

    def max_steer_rad(self) -> float:
        return math.radians(self.cfg.max_steer_deg)

    def max_steer_rate_rad(self) -> float:
        return math.radians(self.cfg.max_steer_rate_deg_s)

    def min_turning_steer_bound(self) -> float:
        # Enforce turning radius lower-bound with a steer upper-bound.
        return math.atan(self.cfg.wheelbase / self.cfg.min_turn_radius_single)

    def clamp_delta(self, delta: float) -> float:
        dmax = min(self.max_steer_rad(), self.min_turning_steer_bound())
        return clamp(delta, -dmax, dmax)

    def step(self, state: VehicleState, cmd: ControlCommand, dt: float) -> VehicleState:
        accel = clamp(cmd.accel, -self.cfg.max_decel, self.cfg.max_accel)
        steer_rate = clamp(cmd.steer_rate, -self.max_steer_rate_rad(), self.max_steer_rate_rad())

        v = clamp(state.v + accel * dt, -self.cfg.max_reverse_speed, self.cfg.max_speed)
        delta = self.clamp_delta(state.delta + steer_rate * dt)

        yaw_rate = 0.0
        if abs(self.cfg.wheelbase) > 1e-9:
            yaw_rate = v * math.tan(delta) / self.cfg.wheelbase
            yaw_rate = clamp(yaw_rate, -self.cfg.max_yaw_rate, self.cfg.max_yaw_rate)

        yaw_mid = state.yaw + 0.5 * yaw_rate * dt
        x = state.x + v * math.cos(yaw_mid) * dt
        y = state.y + v * math.sin(yaw_mid) * dt
        yaw = wrap_angle(state.yaw + yaw_rate * dt)

        return VehicleState(
            vehicle_id=state.vehicle_id,
            x=x,
            y=y,
            yaw=yaw,
            v=v,
            delta=delta,
            mode=state.mode,
        )

    def turning_radius(self, state: VehicleState) -> float:
        if abs(math.tan(state.delta)) < 1e-9:
            return math.inf
        return abs(self.cfg.wheelbase / math.tan(state.delta))


@dataclass(frozen=True)
class VehicleGeometry:
    cfg: VehicleConfig

    @property
    def front_x(self) -> float:
        return self.cfg.wheelbase + self.cfg.front_overhang

    @property
    def rear_x(self) -> float:
        return -self.cfg.rear_overhang

    @property
    def front_hitch_x(self) -> float:
        return self.front_x + self.cfg.hitch_length

    @property
    def rear_hitch_x(self) -> float:
        return self.rear_x - self.cfg.hitch_length

    def to_world(self, state: VehicleState, local_x: float, local_y: float = 0.0) -> np.ndarray:
        c = math.cos(state.yaw)
        s = math.sin(state.yaw)
        return np.array([state.x + c * local_x - s * local_y, state.y + s * local_x + c * local_y], dtype=float)

    def front_hitch(self, state: VehicleState) -> np.ndarray:
        return self.to_world(state, self.front_hitch_x)

    def rear_hitch(self, state: VehicleState) -> np.ndarray:
        return self.to_world(state, self.rear_hitch_x)

    def body_polygon(self, state: VehicleState) -> np.ndarray:
        half_w = 0.5 * self.cfg.car_width
        corners_local = np.array(
            [
                [self.front_x, half_w],
                [self.front_x, -half_w],
                [self.rear_x, -half_w],
                [self.rear_x, half_w],
            ],
            dtype=float,
        )
        c = math.cos(state.yaw)
        s = math.sin(state.yaw)
        rot = np.array([[c, -s], [s, c]], dtype=float)
        return (rot @ corners_local.T).T + np.array([state.x, state.y], dtype=float)

    def hitch_points(self, state: VehicleState) -> np.ndarray:
        return np.array([self.front_hitch(state), self.rear_hitch(state)], dtype=float)
