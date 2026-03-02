from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .collision import obstacle_polygon
from .config import SensorGlobalConfig, SensorVisionConfig, VehicleConfig
from .kinematics import VehicleGeometry
from .math_utils import angle_diff, wrap_angle
from .types import Obstacle, VehicleState


@dataclass(frozen=True)
class GlobalMeasurement:
    x: float
    y: float
    yaw: float
    v: float
    delta: float
    timestamp: float


@dataclass(frozen=True)
class VisionMeasurement:
    rel_x: float
    rel_y: float
    rel_yaw: float
    distance: float
    valid: bool
    timestamp: float


def _line_intersect(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray) -> bool:
    def ccw(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> bool:
        return (p3[1] - p1[1]) * (p2[0] - p1[0]) > (p2[1] - p1[1]) * (p3[0] - p1[0])

    return ccw(a0, b0, b1) != ccw(a1, b0, b1) and ccw(a0, a1, b0) != ccw(a0, a1, b1)


def line_blocked_by_obstacles(p0: np.ndarray, p1: np.ndarray, obstacles: list[Obstacle]) -> bool:
    for obs in obstacles:
        poly = obstacle_polygon(obs)
        for i in range(len(poly)):
            q0 = poly[i]
            q1 = poly[(i + 1) % len(poly)]
            if _line_intersect(p0, p1, q0, q1):
                return True
    return False


class GlobalPoseSensor:
    def __init__(self, cfg: SensorGlobalConfig, seed: int = 0):
        self.cfg = cfg
        self.rng = np.random.default_rng(seed)
        self.period = 1.0 / max(cfg.rate_hz, 1e-9)
        self.last_t = -1e9
        self.last_meas: GlobalMeasurement | None = None

    def observe(self, true_state: VehicleState, timestamp: float) -> GlobalMeasurement | None:
        if timestamp - self.last_t + 1e-12 < self.period:
            return None
        self.last_t = timestamp
        x = true_state.x + float(self.rng.normal(0.0, self.cfg.sigma_pos))
        y = true_state.y + float(self.rng.normal(0.0, self.cfg.sigma_pos))
        yaw = wrap_angle(true_state.yaw + float(self.rng.normal(0.0, math.radians(self.cfg.sigma_yaw_deg))))
        meas = GlobalMeasurement(x=x, y=y, yaw=yaw, v=true_state.v, delta=true_state.delta, timestamp=timestamp)
        self.last_meas = meas
        return meas


class VisionSensor:
    def __init__(self, sensor_cfg: SensorVisionConfig, vehicle_cfg: VehicleConfig, seed: int = 0):
        self.cfg = sensor_cfg
        self.vehicle_cfg = vehicle_cfg
        self.geom = VehicleGeometry(vehicle_cfg)
        self.rng = np.random.default_rng(seed)
        self.period = 1.0 / max(sensor_cfg.rate_hz, 1e-9)
        self.last_t = -1e9
        self.last_meas: VisionMeasurement | None = None

    def observe_rear_hitch(
        self,
        observer: VehicleState,
        target: VehicleState,
        obstacles: list[Obstacle],
        timestamp: float,
    ) -> VisionMeasurement | None:
        if timestamp - self.last_t + 1e-12 < self.period:
            return None
        self.last_t = timestamp

        cam = self.geom.front_hitch(observer)
        tgt = self.geom.rear_hitch(target)
        rel_world = tgt - cam
        dist = float(np.linalg.norm(rel_world))

        if dist > self.cfg.max_distance:
            meas = VisionMeasurement(0.0, 0.0, 0.0, dist, False, timestamp)
            self.last_meas = meas
            return meas

        bearing = angle_diff(math.atan2(rel_world[1], rel_world[0]), observer.yaw)
        fov_half = math.radians(self.cfg.fov_deg) * 0.5
        if abs(bearing) > fov_half:
            meas = VisionMeasurement(0.0, 0.0, 0.0, dist, False, timestamp)
            self.last_meas = meas
            return meas

        if line_blocked_by_obstacles(cam, tgt, obstacles):
            meas = VisionMeasurement(0.0, 0.0, 0.0, dist, False, timestamp)
            self.last_meas = meas
            return meas

        c = math.cos(-observer.yaw)
        s = math.sin(-observer.yaw)
        rel_x = c * rel_world[0] - s * rel_world[1]
        rel_y = s * rel_world[0] + c * rel_world[1]
        rel_yaw = angle_diff(target.yaw, observer.yaw)

        rel_x += float(self.rng.normal(0.0, self.cfg.sigma_pos))
        rel_y += float(self.rng.normal(0.0, self.cfg.sigma_pos))
        rel_yaw = wrap_angle(rel_yaw + float(self.rng.normal(0.0, math.radians(self.cfg.sigma_yaw_deg))))

        meas = VisionMeasurement(
            rel_x=float(rel_x),
            rel_y=float(rel_y),
            rel_yaw=float(rel_yaw),
            distance=dist,
            valid=True,
            timestamp=timestamp,
        )
        self.last_meas = meas
        return meas
