from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from core.utils import clip, cumulative_lengths, interpolate_along_polyline, nearest_point_index, wrap_angle
from core.vehicle import AckermannVehicle, VehicleState


@dataclass
class TrackerConfig:
    base_speed: float = 2.5
    lookahead: float = 2.6
    goal_stop_dist: float = 0.8


class PurePursuitTracker:
    def __init__(self, config: TrackerConfig | None = None):
        self.config = config if config is not None else TrackerConfig()

    def control(self, vehicle: AckermannVehicle, path: Sequence[VehicleState]) -> Tuple[float, float, bool]:
        if not path:
            return 0.0, 0.0, True

        pts = [(s.x, s.y) for s in path]
        pos = (vehicle.state.x, vehicle.state.y)
        idx = nearest_point_index(pts, pos)
        s = cumulative_lengths(pts)
        s_cur = s[idx]
        s_target = s_cur + self.config.lookahead
        tx, ty = interpolate_along_polyline(pts, s_target)

        dx = tx - vehicle.state.x
        dy = ty - vehicle.state.y
        ld = max(math.hypot(dx, dy), 1e-6)

        alpha = wrap_angle(math.atan2(dy, dx) - vehicle.state.theta)
        delta = math.atan2(2.0 * vehicle.spec.wheelbase * math.sin(alpha), ld)
        delta = clip(delta, -vehicle.spec.max_steer, vehicle.spec.max_steer)

        goal = path[-1]
        d_goal = math.hypot(vehicle.state.x - goal.x, vehicle.state.y - goal.y)
        heading_err = abs(wrap_angle(vehicle.state.theta - goal.theta))

        speed_scale = clip(1.0 - 0.5 * abs(alpha), 0.35, 1.0)
        v = self.config.base_speed * speed_scale

        if d_goal < 4.0:
            v = min(v, 1.8)
        if d_goal < self.config.goal_stop_dist and heading_err < 0.45:
            return 0.0, 0.0, True

        return v, delta, False


@dataclass
class DockingConfig:
    max_speed: float = 1.2
    k_long: float = 0.8
    k_lat: float = 1.6
    k_heading: float = 1.4
    done_long_tol: float = 0.18
    done_lat_tol: float = 0.10
    done_heading_tol: float = 0.12


class DockingController:
    def __init__(self, config: DockingConfig | None = None):
        self.config = config if config is not None else DockingConfig()

    def control(
        self,
        active_vehicle: AckermannVehicle,
        socket_xy: np.ndarray,
        socket_heading: float,
    ) -> Tuple[float, float, bool]:
        dirv = np.array([math.cos(socket_heading), math.sin(socket_heading)], dtype=float)
        front = active_vehicle.front_port()
        err = socket_xy - front

        long_err = float(np.dot(err, dirv))
        lat_err = float(dirv[0] * err[1] - dirv[1] * err[0])
        heading_err = wrap_angle(socket_heading - active_vehicle.state.theta)

        v = clip(self.config.k_long * long_err, 0.0, self.config.max_speed)
        steer = self.config.k_lat * math.atan2(lat_err, 1.0) + self.config.k_heading * heading_err
        steer = clip(steer, -active_vehicle.spec.max_steer, active_vehicle.spec.max_steer)

        done = (
            abs(long_err) < self.config.done_long_tol
            and abs(lat_err) < self.config.done_lat_tol
            and abs(heading_err) < self.config.done_heading_tol
        )
        if done:
            v, steer = 0.0, 0.0

        return float(v), float(steer), bool(done)
