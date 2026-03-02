from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum

import numpy as np


class VehicleMode(str, Enum):
    FREE = "FREE"
    DOCKING = "DOCKING"
    TRAIN_FOLLOW = "TRAIN_FOLLOW"
    WAIT = "WAIT"
    SPLIT = "SPLIT"


@dataclass
class VehicleState:
    vehicle_id: int
    x: float
    y: float
    yaw: float
    v: float
    delta: float
    mode: VehicleMode = VehicleMode.FREE

    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=float)

    def copy(self) -> "VehicleState":
        return replace(self)


@dataclass(frozen=True)
class ControlCommand:
    accel: float
    steer_rate: float


@dataclass(frozen=True)
class ReferencePoint:
    x: float
    y: float
    yaw: float
    speed: float


@dataclass
class DockingCondition:
    pos_error: float
    yaw_error: float
    speed_error: float
    hold_time: float = 0.0
    locked: bool = False


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    width: float
    height: float
    yaw: float = 0.0
