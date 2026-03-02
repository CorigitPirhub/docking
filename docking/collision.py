from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .config import SafetyConfig, VehicleConfig
from .kinematics import VehicleGeometry
from .math_utils import unit
from .types import Obstacle, VehicleState


def _axes_from_polygon(poly: np.ndarray) -> list[np.ndarray]:
    axes: list[np.ndarray] = []
    for i in range(len(poly)):
        p0 = poly[i]
        p1 = poly[(i + 1) % len(poly)]
        edge = p1 - p0
        normal = np.array([-edge[1], edge[0]], dtype=float)
        n = unit(normal)
        if np.linalg.norm(n) > 1e-9:
            axes.append(n)
    return axes


def _project(poly: np.ndarray, axis: np.ndarray) -> tuple[float, float]:
    values = poly @ axis
    return float(np.min(values)), float(np.max(values))


def polygon_bbox(poly: np.ndarray) -> tuple[float, float, float, float]:
    xs = poly[:, 0]
    ys = poly[:, 1]
    return float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())


def bbox_distance(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    dx = max(0.0, bx0 - ax1, ax0 - bx1)
    dy = max(0.0, by0 - ay1, ay0 - by1)
    return math.hypot(dx, dy)


def polygons_intersect(poly_a: np.ndarray, poly_b: np.ndarray) -> bool:
    for axis in _axes_from_polygon(poly_a) + _axes_from_polygon(poly_b):
        a0, a1 = _project(poly_a, axis)
        b0, b1 = _project(poly_b, axis)
        if a1 < b0 or b1 < a0:
            return False
    return True


def _point_seg_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-12:
        return float(np.linalg.norm(p - a))
    t = float(np.dot(p - a, ab) / denom)
    t = min(1.0, max(0.0, t))
    proj = a + t * ab
    return float(np.linalg.norm(p - proj))


def polygon_distance(poly_a: np.ndarray, poly_b: np.ndarray) -> float:
    if polygons_intersect(poly_a, poly_b):
        return 0.0

    d_min = math.inf
    for p in poly_a:
        for i in range(len(poly_b)):
            d_min = min(d_min, _point_seg_distance(p, poly_b[i], poly_b[(i + 1) % len(poly_b)]))
    for p in poly_b:
        for i in range(len(poly_a)):
            d_min = min(d_min, _point_seg_distance(p, poly_a[i], poly_a[(i + 1) % len(poly_a)]))
    return float(d_min)


def obstacle_polygon(obs: Obstacle) -> np.ndarray:
    hw = 0.5 * obs.width
    hh = 0.5 * obs.height
    local = np.array([[hw, hh], [hw, -hh], [-hw, -hh], [-hw, hh]], dtype=float)
    c = math.cos(obs.yaw)
    s = math.sin(obs.yaw)
    rot = np.array([[c, -s], [s, c]], dtype=float)
    return (rot @ local.T).T + np.array([obs.x, obs.y], dtype=float)


def _disk_poly(center: np.ndarray, radius: float, num_sides: int = 10) -> np.ndarray:
    pts = []
    for i in range(num_sides):
        th = 2.0 * math.pi * i / num_sides
        pts.append([center[0] + radius * math.cos(th), center[1] + radius * math.sin(th)])
    return np.array(pts, dtype=float)


@dataclass
class CollisionEngine:
    vehicle_cfg: VehicleConfig
    safety_cfg: SafetyConfig

    def __post_init__(self) -> None:
        self.geom = VehicleGeometry(self.vehicle_cfg)

    def vehicle_polygons(self, state: VehicleState) -> list[np.ndarray]:
        body = self.geom.body_polygon(state)
        front_hitch = _disk_poly(self.geom.front_hitch(state), radius=0.06)
        rear_hitch = _disk_poly(self.geom.rear_hitch(state), radius=0.06)
        return [body, front_hitch, rear_hitch]

    def collide_vehicle_obstacle(self, state: VehicleState, obs: Obstacle, include_clearance: bool = True) -> bool:
        obs_poly = obstacle_polygon(obs)
        obs_bbox = polygon_bbox(obs_poly)
        for poly in self.vehicle_polygons(state):
            poly_bbox = polygon_bbox(poly)
            needed = self.safety_cfg.min_clearance if include_clearance else 0.0
            if bbox_distance(poly_bbox, obs_bbox) > needed:
                continue
            if polygons_intersect(poly, obs_poly):
                return True
            clearance = polygon_distance(poly, obs_poly)
            if clearance < needed:
                return True
        return False

    def collide_vehicle_vehicle(self, a: VehicleState, b: VehicleState, include_clearance: bool = True) -> bool:
        polys_a = self.vehicle_polygons(a)
        polys_b = self.vehicle_polygons(b)
        bboxes_a = [polygon_bbox(p) for p in polys_a]
        bboxes_b = [polygon_bbox(p) for p in polys_b]
        needed = self.safety_cfg.min_clearance if include_clearance else 0.0
        for poly_a, box_a in zip(polys_a, bboxes_a):
            for poly_b, box_b in zip(polys_b, bboxes_b):
                if bbox_distance(box_a, box_b) > needed:
                    continue
                if polygons_intersect(poly_a, poly_b):
                    return True
                clearance = polygon_distance(poly_a, poly_b)
                if clearance < needed:
                    return True
        return False

    def collide_vehicle_train(
        self,
        vehicle: VehicleState,
        train_states: Iterable[VehicleState],
        include_clearance: bool = True,
        skip_vehicle_ids: set[int] | None = None,
    ) -> bool:
        skip_ids = set() if skip_vehicle_ids is None else set(skip_vehicle_ids)
        for unit in train_states:
            if unit.vehicle_id in skip_ids:
                continue
            if self.collide_vehicle_vehicle(vehicle, unit, include_clearance=include_clearance):
                return True
        return False

    def collide_train_obstacles(
        self,
        train_states: Iterable[VehicleState],
        obstacles: Iterable[Obstacle],
        include_clearance: bool = True,
    ) -> bool:
        for state in train_states:
            for obs in obstacles:
                if self.collide_vehicle_obstacle(state, obs, include_clearance=include_clearance):
                    return True
        return False

    def collide_train_self(
        self,
        train_states: list[VehicleState],
        include_clearance: bool = False,
        non_adjacent_only: bool = True,
    ) -> bool:
        for i in range(len(train_states)):
            for j in range(i + 1, len(train_states)):
                if non_adjacent_only and abs(i - j) <= 1:
                    continue
                if self.collide_vehicle_vehicle(train_states[i], train_states[j], include_clearance=include_clearance):
                    return True
        return False

    def collide_train_any(
        self,
        train_states: list[VehicleState],
        obstacles: Iterable[Obstacle],
        include_clearance: bool = True,
        non_adjacent_self_only: bool = True,
    ) -> bool:
        if self.collide_train_obstacles(train_states, obstacles, include_clearance=include_clearance):
            return True
        if self.collide_train_self(train_states, include_clearance=False, non_adjacent_only=non_adjacent_self_only):
            return True
        return False

    def min_clearance_vehicle_obstacles(self, state: VehicleState, obstacles: Iterable[Obstacle]) -> float:
        d_min = math.inf
        polys = self.vehicle_polygons(state)
        poly_bboxes = [polygon_bbox(p) for p in polys]
        for obs in obstacles:
            obs_poly = obstacle_polygon(obs)
            obs_bbox = polygon_bbox(obs_poly)
            for poly, box in zip(polys, poly_bboxes):
                box_d = bbox_distance(box, obs_bbox)
                if box_d > d_min:
                    continue
                d_min = min(d_min, polygon_distance(poly, obs_poly))
        if math.isinf(d_min):
            return 1e6
        return float(d_min)

    def min_clearance_train_obstacles(self, train_states: Iterable[VehicleState], obstacles: Iterable[Obstacle]) -> float:
        d_min = math.inf
        for s in train_states:
            d_min = min(d_min, self.min_clearance_vehicle_obstacles(s, obstacles))
        if math.isinf(d_min):
            return 1e6
        return float(d_min)

    def in_collision(self, state: VehicleState, obstacles: Iterable[Obstacle], others: Iterable[VehicleState]) -> bool:
        for obs in obstacles:
            if self.collide_vehicle_obstacle(state, obs):
                return True
        for o in others:
            if o.vehicle_id == state.vehicle_id:
                continue
            if self.collide_vehicle_vehicle(state, o):
                return True
        return False
