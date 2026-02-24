from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from core.utils import clip, heading, wrap_angle


@dataclass
class VehicleSpec:
    vehicle_id: str
    length: float
    width: float
    wheelbase: float
    max_steer: float
    steer_mode: str  # "front" or "rear"
    front_port_offset: float
    rear_port_offset: float

    @property
    def steer_sign(self) -> float:
        return 1.0 if self.steer_mode == "front" else -1.0


@dataclass
class VehicleState:
    x: float
    y: float
    theta: float


class AckermannVehicle:
    def __init__(self, spec: VehicleSpec, state: VehicleState):
        self.spec = spec
        self.state = state
        self.last_control = (0.0, 0.0)
        self.attached = False

    def pose(self) -> Tuple[float, float, float]:
        return (self.state.x, self.state.y, self.state.theta)

    def set_pose(self, x: float, y: float, theta: float) -> None:
        self.state.x = float(x)
        self.state.y = float(y)
        self.state.theta = wrap_angle(float(theta))

    def center(self) -> np.ndarray:
        return np.array([self.state.x, self.state.y], dtype=float)

    def heading_vec(self) -> np.ndarray:
        return heading(self.state.theta)

    def front_port(self) -> np.ndarray:
        return self.center() + self.spec.front_port_offset * self.heading_vec()

    def rear_port(self) -> np.ndarray:
        return self.center() - self.spec.rear_port_offset * self.heading_vec()

    def footprint_circles(self) -> Tuple[np.ndarray, float]:
        h = self.heading_vec()
        c = self.center()
        l = self.spec.length
        r = 0.5 * self.spec.width
        c1 = c + 0.25 * l * h
        c2 = c
        c3 = c - 0.25 * l * h
        return np.vstack([c1, c2, c3]), r

    def step(self, v: float, steer: float, dt: float) -> None:
        steer = clip(steer, -self.spec.max_steer, self.spec.max_steer)
        x, y, th = self.state.x, self.state.y, self.state.theta

        yaw_rate = self.spec.steer_sign * v * np.tan(steer) / max(self.spec.wheelbase, 1e-6)
        x += v * np.cos(th) * dt
        y += v * np.sin(th) * dt
        th = wrap_angle(th + yaw_rate * dt)

        self.state.x, self.state.y, self.state.theta = float(x), float(y), float(th)
        self.last_control = (float(v), float(steer))
