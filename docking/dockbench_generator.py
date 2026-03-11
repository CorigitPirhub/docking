from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from .collision import CollisionEngine, obstacle_polygon, polygon_distance, polygons_intersect
from .config import Config
from .math_utils import clamp
from .kinematics import VehicleGeometry
from .dockbench import (
    DIFFICULTIES,
    FAMILIES,
    SPLIT_COUNTS,
    DockBenchSceneSpec,
    build_scene_id,
    build_scene_payload,
    dataset_root,
    descriptor_distance,
    family_label,
    quality_record,
    compute_descriptors,
    family_thresholds,
    save_manifest_files,
    style_for_family,
)
from .types import Obstacle, VehicleMode, VehicleState


@dataclass(frozen=True)
class RoleObstacle:
    obstacle: Obstacle
    role: str


@dataclass(frozen=True)
class CandidateScene:
    family: str
    difficulty: str
    seed: int
    max_time_s: float
    leader: VehicleState
    follower: VehicleState
    obstacles: tuple[Obstacle, ...]
    roles: tuple[str, ...]
    descriptors: dict[str, float]
    quality: dict[str, Any]
    aux: dict[str, Any]


_SPLIT_SEQUENCE: tuple[str, ...] = tuple(["tuning", *(["test"] * SPLIT_COUNTS["test"]), "challenge"])


def _in_bounds(cfg: Config, state: VehicleState, margin: float) -> bool:
    half_w = 0.5 * float(cfg.environment.width) - float(margin)
    half_h = 0.5 * float(cfg.environment.height) - float(margin)
    return (-half_w <= float(state.x) <= half_w) and (-half_h <= float(state.y) <= half_h)


def _obstacle_in_bounds(cfg: Config, obstacle: Obstacle, margin: float = 0.2) -> bool:
    poly = obstacle_polygon(obstacle)
    half_w = 0.5 * float(cfg.environment.width) - float(margin)
    half_h = 0.5 * float(cfg.environment.height) - float(margin)
    return bool(np.all(poly[:, 0] >= -half_w) and np.all(poly[:, 0] <= half_w) and np.all(poly[:, 1] >= -half_h) and np.all(poly[:, 1] <= half_h))


def _obstacle_clear_of_vehicles(collision: CollisionEngine, obstacle: Obstacle, leader: VehicleState, follower: VehicleState) -> bool:
    return (not collision.collide_vehicle_obstacle(leader, obstacle, include_clearance=True)) and (not collision.collide_vehicle_obstacle(follower, obstacle, include_clearance=True))


def _obstacles_separated(obstacle: Obstacle, existing: Sequence[Obstacle], min_gap: float = 0.12) -> bool:
    poly = obstacle_polygon(obstacle)
    for other in existing:
        other_poly = obstacle_polygon(other)
        if polygons_intersect(poly, other_poly):
            return False
        if polygon_distance(poly, other_poly) < float(min_gap):
            return False
    return True


def _quantile_range(lo: float, hi: float, split: str, *, family: str, quantity: str) -> tuple[float, float]:
    span = max(float(hi) - float(lo), 1e-6)
    split_kind = str(split).lower()
    table: dict[str, tuple[float, float]]
    if quantity == "d0":
        table = {"test": (0.06, 0.42), "tuning": (0.14, 0.56), "challenge": (0.48, 0.94)}
        if str(family).upper() == "EC" and split_kind != "challenge":
            table = {**table, split_kind: (table[split_kind][0], min(table[split_kind][1], 0.36))}
        if str(family).upper() == "FC" and split_kind == "test":
            table = {**table, "test": (0.08, 0.34)}
    elif quantity == "heading":
        table = {"test": (0.08, 0.38), "tuning": (0.16, 0.52), "challenge": (0.50, 0.96)}
        fam = str(family).upper()
        if fam == "SC" and split_kind == "tuning":
            table = {**table, "tuning": (0.06, 0.24)}
        elif fam == "FC" and split_kind == "test":
            table = {**table, "test": (0.10, 0.32)}
        elif fam == "EC" and split_kind == "test":
            table = {**table, "test": (0.00, 0.24)}
        elif fam == "EC" and split_kind == "tuning":
            table = {**table, "tuning": (0.04, 0.28)}
    else:
        table = {"test": (0.30, 0.60), "tuning": (0.42, 0.78), "challenge": (0.70, 1.00)}
        fam = str(family).upper()
        if fam == "EC" and split_kind != "challenge":
            table = {**table, split_kind: (0.24, 0.50) if split_kind == "test" else (0.30, 0.62)}
        elif fam == "FC" and split_kind == "test":
            table = {**table, "test": (0.26, 0.52)}
    q0, q1 = table[split_kind]
    lo_q = float(lo + q0 * span)
    hi_q = float(lo + q1 * span)
    if hi_q <= lo_q:
        hi_q = float(lo_q + 1e-3)
    return lo_q, hi_q


def _make_vehicle_pair(cfg: Config, rng: np.random.Generator, *, family: str, difficulty: str, split: str) -> tuple[VehicleState, VehicleState]:
    fam = str(family).upper()
    lvl = str(difficulty).upper()
    leader_x_ranges = {
        "CF": (6.0, 11.5),
        "SC": (5.0, 11.0),
        "FC": (4.0, 9.5),
        "EC": (-8.5, -2.5),
    }
    leader_y_ranges = {
        "CF": (-3.2, 3.2),
        "SC": (-3.0, 3.0),
        "FC": (-2.8, 2.8),
        "EC": (-3.0, 3.0),
    }
    yaw_span_deg = {
        "CF": 18.0,
        "SC": 24.0,
        "FC": 20.0,
        "EC": 18.0,
    }
    d0_ranges = {
        "CF": {"L1": (7.1, 8.2), "L2": (8.1, 9.4), "L3": (9.3, 11.0)},
        "SC": {"L1": (7.3, 8.5), "L2": (8.3, 9.8), "L3": (9.5, 11.3)},
        "FC": {"L1": (6.5, 7.5), "L2": (7.3, 8.4), "L3": (8.1, 9.3)},
        "EC": {"L1": (8.3, 10.1), "L2": (9.7, 11.3), "L3": (10.5, 12.5)},
    }
    bearing_spans = {
        "CF": 12.0,
        "SC": 14.0,
        "FC": 18.0,
        "EC": 16.0,
    }
    leader = VehicleState(
        vehicle_id=2,
        x=float(rng.uniform(*leader_x_ranges[fam])),
        y=float(rng.uniform(*leader_y_ranges[fam])),
        yaw=float(rng.uniform(-math.radians(yaw_span_deg[fam]), math.radians(yaw_span_deg[fam]))),
        v=0.0,
        delta=0.0,
        mode=VehicleMode.FREE,
    )
    d0_lo, d0_hi = _quantile_range(*d0_ranges[fam][lvl], split, family=fam, quantity="d0")
    d0 = float(rng.uniform(d0_lo, d0_hi))
    bearing_lo, bearing_hi = _quantile_range(-math.radians(bearing_spans[fam]), math.radians(bearing_spans[fam]), split, family=fam, quantity="bearing")
    bearing = math.pi + float(rng.uniform(bearing_lo, bearing_hi))
    dx = d0 * math.cos(float(leader.yaw) + bearing)
    dy = d0 * math.sin(float(leader.yaw) + bearing)
    head_lo, head_hi = family_thresholds(fam, lvl)["heading_diff_deg"]
    if head_hi <= head_lo + 1e-6:
        heading_mag = float(head_hi)
    else:
        q_lo, q_hi = _quantile_range(head_lo, head_hi, split, family=fam, quantity="heading")
        heading_mag = float(rng.uniform(q_lo, q_hi))
    heading_sign = float(rng.choice([-1.0, 1.0]))
    follower = VehicleState(
        vehicle_id=1,
        x=float(leader.x + dx),
        y=float(leader.y + dy),
        yaw=float(leader.yaw + heading_sign * math.radians(heading_mag)),
        v=0.0,
        delta=0.0,
        mode=VehicleMode.DOCKING,
    )
    return leader, follower


def _role_obstacle(x: float, y: float, width: float, height: float, yaw: float, role: str) -> RoleObstacle:
    return RoleObstacle(Obstacle(x=float(x), y=float(y), width=float(width), height=float(height), yaw=float(yaw)), str(role))


def _append_if_valid(
    cfg: Config,
    collision: CollisionEngine,
    items: list[RoleObstacle],
    candidate: RoleObstacle,
    leader: VehicleState,
    follower: VehicleState,
) -> bool:
    if not _obstacle_in_bounds(cfg, candidate.obstacle):
        return False
    if not _obstacle_clear_of_vehicles(collision, candidate.obstacle, leader, follower):
        return False
    if not _obstacles_separated(candidate.obstacle, [item.obstacle for item in items]):
        return False
    items.append(candidate)
    return True


def _distance_point_to_segment(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-9:
        return float(np.linalg.norm(point - a))
    t = float(np.dot(point - a, ab) / denom)
    t = float(min(1.0, max(0.0, t)))
    proj = a + t * ab
    return float(np.linalg.norm(point - proj))


def _sample_background_obstacles(
    cfg: Config,
    rng: np.random.Generator,
    collision: CollisionEngine,
    leader: VehicleState,
    follower: VehicleState,
    items: list[RoleObstacle],
    *,
    count: int,
    size_range: tuple[float, float],
    corridor_margin: float,
) -> None:
    a = follower.xy()
    b = leader.xy()
    for _ in range(int(count) * 40):
        if len([item for item in items if item.role == "background"]) >= int(count):
            break
        w = float(rng.uniform(size_range[0], size_range[1]))
        h = float(rng.uniform(size_range[0], size_range[1]))
        x = float(rng.uniform(-17.0, 17.0))
        y = float(rng.uniform(-8.0, 8.0))
        yaw = float(rng.uniform(-0.6, 0.6))
        center = np.array([x, y], dtype=float)
        if _distance_point_to_segment(center, a, b) < float(corridor_margin):
            continue
        if np.linalg.norm(center - leader.xy()) < 2.4:
            continue
        if np.linalg.norm(center - follower.xy()) < 2.2:
            continue
        _append_if_valid(cfg, collision, items, _role_obstacle(x, y, w, h, yaw, "background"), leader, follower)


def _rect_polygon(x0: float, x1: float, y0: float, y1: float) -> list[list[float]]:
    return [
        [float(x1), float(y1)],
        [float(x1), float(y0)],
        [float(x0), float(y0)],
        [float(x0), float(y1)],
    ]


def _point_in_poly(point: np.ndarray, poly: list[list[float]]) -> bool:
    x = float(point[0])
    y = float(point[1])
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = float(poly[i][0]), float(poly[i][1])
        xj, yj = float(poly[j][0]), float(poly[j][1])
        cond = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / max(yj - yi, 1e-9) + xi)
        if cond:
            inside = not inside
        j = i
    return bool(inside)


def _corridor_sample_inside(point: np.ndarray, corridor_polygons: list[list[list[float]]]) -> bool:
    return any(_point_in_poly(point, poly) for poly in corridor_polygons)


def _segment_outside_ratio(p0: np.ndarray, p1: np.ndarray, corridor_polygons: list[list[list[float]]], *, n: int = 61) -> float:
    alphas = np.linspace(0.0, 1.0, int(max(n, 2)), dtype=float)
    outside = 0
    for alpha in alphas:
        pt = (1.0 - alpha) * np.asarray(p0, dtype=float) + alpha * np.asarray(p1, dtype=float)
        if not _corridor_sample_inside(pt, corridor_polygons):
            outside += 1
    return float(outside / max(len(alphas), 1))


def _lane_fidelity_audit(
    follower_start: VehicleState,
    follower_goal: VehicleState,
    corridor_polygons: list[list[list[float]]],
    *,
    legal_path_len_m: float,
) -> tuple[bool, float]:
    direct = float(np.linalg.norm(follower_goal.xy() - follower_start.xy()))
    shortcut_gap = float(max(0.0, legal_path_len_m - direct))
    outside_ratio = _segment_outside_ratio(follower_start.xy(), follower_goal.xy(), corridor_polygons)
    geodesic_ratio = float(legal_path_len_m / max(direct, 1e-6))
    lane_fidelity = bool(shortcut_gap >= 0.75 and (outside_ratio >= 0.18 or geodesic_ratio >= 1.12))
    return lane_fidelity, float(shortcut_gap)


def _generate_cf(cfg: Config, rng: np.random.Generator, *, difficulty: str, seed: int, split: str) -> tuple[VehicleState, VehicleState, list[RoleObstacle], float]:
    split_kind = str(split).lower()
    leader, follower = _make_vehicle_pair(cfg, rng, family="CF", difficulty=difficulty, split=split)
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    items: list[RoleObstacle] = []
    bg_counts = {"L1": 3, "L2": 4, "L3": 5}
    size_ranges = {"L1": (0.6, 1.1), "L2": (0.7, 1.3), "L3": (0.8, 1.5)}
    _sample_background_obstacles(
        cfg,
        rng,
        collision,
        leader,
        follower,
        items,
        count=bg_counts[str(difficulty).upper()],
        size_range=size_ranges[str(difficulty).upper()],
        corridor_margin=1.9,
    )
    base_time = {"L1": 16.0, "L2": 18.0, "L3": 20.0}[str(difficulty).upper()]
    if split_kind == "challenge":
        base_time += 2.0
    return leader, follower, items, base_time


def _generate_sc(cfg: Config, rng: np.random.Generator, *, difficulty: str, seed: int, split: str) -> tuple[VehicleState, VehicleState, list[RoleObstacle], float]:
    split_kind = str(split).lower()
    leader, follower = _make_vehicle_pair(cfg, rng, family="SC", difficulty=difficulty, split=split)
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    items: list[RoleObstacle] = []
    cam = follower.xy()
    rear = leader.xy()
    los = rear - cam
    los_yaw = float(math.atan2(float(los[1]), float(los[0])))
    n = np.array([-math.sin(los_yaw), math.cos(los_yaw)], dtype=float)
    mid = 0.48 * cam + 0.52 * rear
    lvl = str(difficulty).upper()
    offset_mag = {"L1": 0.60, "L2": 0.45, "L3": 0.30}[lvl]
    width = {"L1": 0.60, "L2": 0.70, "L3": 0.80}[lvl]
    height = {"L1": 1.4, "L2": 1.6, "L3": 1.8}[lvl]
    bg_count = {"L1": 2, "L2": 3, "L3": 4}[lvl]
    bg_size = {"L1": (0.7, 1.3), "L2": (0.8, 1.5), "L3": (0.9, 1.7)}[lvl]
    corridor_margin = 2.0
    if split_kind == "test":
        offset_mag += {"L1": 0.12, "L2": 0.10, "L3": 0.08}[lvl]
        width = max(0.46, width - 0.08)
        height = max(1.05, height - 0.18)
        bg_count = max(1, bg_count - 1)
        bg_size = (max(0.60, bg_size[0] - 0.08), max(bg_size[0] + 0.18, bg_size[1] - 0.12))
        corridor_margin = 2.25
        if lvl == "L3":
            offset_mag += 0.12
            width = max(0.42, width - 0.08)
            height = max(0.95, height - 0.28)
            bg_count = max(1, bg_count - 1)
            corridor_margin = 2.45
    elif split_kind == "tuning":
        offset_mag += {"L1": 0.34, "L2": 0.22, "L3": 0.12}[lvl]
        width = max(0.42, width - 0.14)
        height = max(0.95, height - 0.42)
        bg_count = max(1, bg_count - 1)
        bg_size = (max(0.60, bg_size[0] - 0.10), max(bg_size[0] + 0.16, bg_size[1] - 0.18))
        corridor_margin = 2.35
        if lvl == "L3":
            offset_mag += 0.08
            height = max(0.92, height - 0.12)
            bg_count = max(1, bg_count - 1)
            corridor_margin = 2.45
    elif split_kind == "challenge":
        offset_mag = max(0.15, offset_mag - 0.08)
        width += 0.08
        height += 0.25
        bg_count += 1
    offset = offset_mag * float(rng.choice([-1.0, 1.0]))
    screen = _role_obstacle(mid[0] + offset * n[0], mid[1] + offset * n[1], width, height, los_yaw, "screen")
    if not _append_if_valid(cfg, collision, items, screen, leader, follower):
        raise RuntimeError("failed to place SC screen obstacle")
    _sample_background_obstacles(
        cfg,
        rng,
        collision,
        leader,
        follower,
        items,
        count=bg_count,
        size_range=bg_size,
        corridor_margin=corridor_margin,
    )
    base_time = {"L1": 20.0, "L2": 26.0, "L3": 30.0}[lvl]
    if split_kind == "test":
        base_time += 2.0
    elif split_kind == "tuning":
        base_time += 8.0
    elif split_kind == "challenge":
        base_time += 3.0
    return leader, follower, items, base_time


def _generate_fc(cfg: Config, rng: np.random.Generator, *, difficulty: str, seed: int, split: str) -> tuple[VehicleState, VehicleState, list[RoleObstacle], float]:
    split_kind = str(split).lower()
    leader, follower = _make_vehicle_pair(cfg, rng, family="FC", difficulty=difficulty, split=split)
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    items: list[RoleObstacle] = []
    yaw = float(leader.yaw)
    h = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
    n = np.array([-math.sin(yaw), math.cos(yaw)], dtype=float)
    rear = leader.xy() - h * 0.8
    lvl = str(difficulty).upper()
    gap = {"L1": 1.70, "L2": 1.45, "L3": 1.20}[lvl]
    wall_len = {"L1": 1.4, "L2": 1.65, "L3": 1.9}[lvl]
    wall_thickness = {"L1": 0.30, "L2": 0.34, "L3": 0.38}[lvl]
    wall_angle = math.radians({"L1": 6.0, "L2": 9.0, "L3": 12.0}[lvl])
    back_offset = {"L1": 1.35, "L2": 1.20, "L3": 1.05}[lvl]
    bg_count = {"L1": 1, "L2": 2, "L3": 3}[lvl]
    bg_size = {"L1": (0.8, 1.2), "L2": (0.8, 1.4), "L3": (0.9, 1.5)}[lvl]
    if split_kind == "test":
        gap += {"L1": 0.14, "L2": 0.24, "L3": 0.30}[lvl]
        wall_len = max(1.05, wall_len - 0.12)
        wall_thickness = max(0.24, wall_thickness - 0.02)
        wall_angle = max(math.radians(3.0), wall_angle - math.radians(2.5))
        back_offset += 0.12
        bg_count = max(0, bg_count - 1)
        bg_size = (max(0.70, bg_size[0] - 0.08), max(bg_size[0] + 0.12, bg_size[1] - 0.12))
    elif split_kind == "tuning":
        gap += 0.34
        wall_len = max(1.0, wall_len - 0.18)
        wall_angle = max(math.radians(3.0), wall_angle - math.radians(3.0))
        wall_thickness = max(0.24, wall_thickness - 0.02)
        back_offset += 0.18
        bg_count = max(0, bg_count - 1)
    elif split_kind == "challenge":
        gap = max(0.95, gap - 0.10)
        wall_len += 0.15
        wall_angle += math.radians(2.0)
    left_center = rear - h * back_offset + n * (0.5 * gap + 0.30)
    right_center = rear - h * back_offset - n * (0.5 * gap + 0.30)
    left = _role_obstacle(left_center[0], left_center[1], wall_len, wall_thickness, yaw + wall_angle, "dock_zone")
    right = _role_obstacle(right_center[0], right_center[1], wall_len, wall_thickness, yaw - wall_angle, "dock_zone")
    if not _append_if_valid(cfg, collision, items, left, leader, follower):
        raise RuntimeError("failed to place FC left funnel wall")
    if not _append_if_valid(cfg, collision, items, right, leader, follower):
        raise RuntimeError("failed to place FC right funnel wall")
    if lvl != "L1" and split_kind != "test":
        bg = _role_obstacle(
            float(leader.x + rng.uniform(2.6, 4.6)),
            float(leader.y + rng.uniform(-1.8, 1.8)),
            float(rng.uniform(0.8, 1.4)),
            float(rng.uniform(0.7, 1.3)),
            float(rng.uniform(-0.4, 0.4)),
            "channel",
        )
        _append_if_valid(cfg, collision, items, bg, leader, follower)
    _sample_background_obstacles(
        cfg,
        rng,
        collision,
        leader,
        follower,
        items,
        count=bg_count,
        size_range=bg_size,
        corridor_margin=2.15 if split_kind == "challenge" else 2.30,
    )
    base_time = {"L1": 22.0, "L2": 28.0, "L3": 34.0}[lvl]
    if split_kind == "test":
        base_time += 2.0
    elif split_kind == "tuning":
        base_time += 6.0
    elif split_kind == "challenge":
        base_time += 2.0
    return leader, follower, items, base_time


def _generate_ec(cfg: Config, rng: np.random.Generator, *, difficulty: str, seed: int, split: str) -> tuple[VehicleState, VehicleState, list[RoleObstacle], float]:
    split_kind = str(split).lower()
    leader, follower = _make_vehicle_pair(cfg, rng, family="EC", difficulty=difficulty, split=split)
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    items: list[RoleObstacle] = []
    yaw = float(leader.yaw)
    h = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
    n = np.array([-math.sin(yaw), math.cos(yaw)], dtype=float)
    rear = leader.xy() - h * 0.8
    lvl = str(difficulty).upper()
    screen_back = {"L1": 2.40, "L2": 2.05, "L3": 1.80}[lvl]
    screen_width = {"L1": 1.4, "L2": 1.6, "L3": 1.8}[lvl]
    screen_height = {"L1": 2.4, "L2": 2.8, "L3": 3.0}[lvl]
    side_offset = {"L1": 1.70, "L2": 1.50, "L3": 1.30}[lvl]
    pocket_front = {"L1": 0.90, "L2": 0.70, "L3": 0.50}[lvl]
    left_size = ({"L1": 1.0, "L2": 1.2, "L3": 1.4}[lvl], {"L1": 1.3, "L2": 1.5, "L3": 1.8}[lvl])
    right_size = None if lvl == "L1" else ({"L2": 1.0, "L3": 1.2}[lvl], {"L2": 1.3, "L3": 1.6}[lvl])
    bg_count = {"L1": 2, "L2": 3, "L3": 4}[lvl]
    bg_size = {"L1": (1.0, 1.7), "L2": (1.1, 1.9), "L3": (1.2, 2.1)}[lvl]
    if split_kind == "test":
        screen_back += {"L1": 0.55, "L2": 0.60, "L3": 0.70}[lvl]
        screen_width = max(1.0, screen_width - 0.22)
        screen_height = max(1.7, screen_height - 0.40)
        side_offset += {"L1": 0.45, "L2": 0.55, "L3": 0.70}[lvl]
        pocket_front += {"L1": 0.24, "L2": 0.30, "L3": 0.38}[lvl]
        left_size = (max(0.85, left_size[0] - 0.12), max(1.05, left_size[1] - 0.22))
        if right_size is not None:
            right_size = (max(0.82, right_size[0] - 0.12), max(1.00, right_size[1] - 0.22))
        bg_count = max(1, bg_count - 1)
        bg_size = (max(0.9, bg_size[0] - 0.12), max(bg_size[0] + 0.20, bg_size[1] - 0.25))
        if lvl == "L1":
            screen_back += 0.50
            screen_width = max(0.92, screen_width - 0.14)
            screen_height = max(1.45, screen_height - 0.28)
            side_offset += 0.35
            pocket_front += 0.18
            left_size = (max(0.72, left_size[0] - 0.10), max(0.90, left_size[1] - 0.16))
            bg_count = 0
            bg_size = (0.8, 1.2)
    elif split_kind == "tuning":
        screen_back += {"L1": 0.80, "L2": 0.85, "L3": 0.95}[lvl]
        screen_width = max(1.0, screen_width - 0.28)
        screen_height = max(1.6, screen_height - 0.48)
        side_offset += {"L1": 0.46, "L2": 0.58, "L3": 0.76}[lvl]
        pocket_front += {"L1": 0.28, "L2": 0.34, "L3": 0.42}[lvl]
        left_size = (max(0.82, left_size[0] - 0.14), max(1.00, left_size[1] - 0.25))
        if right_size is not None:
            right_size = (max(0.80, right_size[0] - 0.14), max(0.96, right_size[1] - 0.24))
        bg_count = max(1, bg_count - 1)
        bg_size = (max(0.9, bg_size[0] - 0.12), max(bg_size[0] + 0.18, bg_size[1] - 0.28))
        if lvl == "L1":
            screen_back += 0.45
            screen_width = max(0.90, screen_width - 0.12)
            screen_height = max(1.40, screen_height - 0.24)
            side_offset += 0.32
            pocket_front += 0.15
            left_size = (max(0.70, left_size[0] - 0.08), max(0.88, left_size[1] - 0.14))
            bg_count = 0
            bg_size = (0.8, 1.2)
    elif split_kind == "challenge":
        screen_back = max(1.25, screen_back - 0.15)
        screen_width += 0.10
        screen_height += 0.15
        side_offset = max(1.05, side_offset - 0.10)
    screen = _role_obstacle(
        rear[0] - screen_back * h[0],
        rear[1] - screen_back * h[1],
        screen_width,
        screen_height,
        yaw,
        "screen",
    )
    if not _append_if_valid(cfg, collision, items, screen, leader, follower):
        raise RuntimeError("failed to place EC screen")
    left_added = False
    left_extras = (0.60, 0.90, 0.30, -0.20, -0.50) if lvl == "L1" and split_kind != "challenge" else (0.0, 0.30, -0.30, 0.60, -0.55)
    for extra in left_extras:
        left = _role_obstacle(
            leader.x - (pocket_front + 0.10 * extra) * h[0] + (side_offset + extra) * n[0],
            leader.y - (pocket_front + 0.10 * extra) * h[1] + (side_offset + extra) * n[1],
            left_size[0],
            left_size[1],
            yaw + math.radians({"L1": 4.0, "L2": 7.0, "L3": 9.0}[lvl]),
            "hybrid",
        )
        if _append_if_valid(cfg, collision, items, left, leader, follower):
            left_added = True
            break
    if right_size is not None and split_kind != "test":
        for extra in (0.0, 0.28, -0.28, 0.56):
            right = _role_obstacle(
                leader.x + (0.35 + 0.10 * extra) * h[0] - (side_offset + 0.08 + extra) * n[0],
                leader.y + (0.35 + 0.10 * extra) * h[1] - (side_offset + 0.08 + extra) * n[1],
                right_size[0],
                right_size[1],
                yaw - math.radians({"L2": 4.0, "L3": 7.0}[lvl]),
                "channel",
            )
            if _append_if_valid(cfg, collision, items, right, leader, follower):
                break
    if not left_added:
        fallback = _role_obstacle(
            leader.x + 1.8 * h[0] + 1.6 * n[0],
            leader.y + 1.8 * h[1] + 1.6 * n[1],
            max(0.9, left_size[0]),
            max(1.1, left_size[1]),
            yaw,
            "hybrid",
        )
        _append_if_valid(cfg, collision, items, fallback, leader, follower)
    _sample_background_obstacles(
        cfg,
        rng,
        collision,
        leader,
        follower,
        items,
        count=bg_count,
        size_range=bg_size,
        corridor_margin=2.45 if split_kind == "challenge" else 2.65,
    )
    base_time = {"L1": 30.0, "L2": 36.0, "L3": 42.0}[lvl]
    if split_kind == "test":
        base_time += 2.0
    elif split_kind == "tuning":
        base_time += 4.0
    elif split_kind == "challenge":
        base_time += 2.0
    return leader, follower, items, base_time


def _generate_lc(cfg: Config, rng: np.random.Generator, *, difficulty: str, seed: int, split: str) -> tuple[VehicleState, VehicleState, list[RoleObstacle], float, dict[str, Any]]:
    lvl = str(difficulty).upper()
    split_kind = str(split).lower()
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    items: list[RoleObstacle] = []

    lane_y = float(rng.uniform(-1.8, 1.8))
    x_left = -16.5
    x_right = 8.5
    wall_t = 0.34
    lane_width = {"L1": 2.15, "L2": 1.75, "L3": 1.45}[lvl]
    bay_extra = {"L1": 1.95, "L2": 2.35, "L3": 3.20}[lvl]
    pocket_len = {"L1": 2.8, "L2": 3.3, "L3": 4.4}[lvl]
    heading_ranges = {"L1": (4.0, 14.0), "L2": (18.0, 34.0), "L3": (42.0, 68.0)}
    d0_ranges = {"L1": (6.2, 7.8), "L2": (6.8, 8.6), "L3": (7.2, 9.2)}
    if split_kind == 'challenge':
        lane_width = max(0.95, lane_width - 0.08)
        bay_extra += 0.16
    elif split_kind == 'tuning':
        lane_width += 0.08
    branch_sign = float(rng.choice([-1.0, 1.0]))
    pocket_mid_x = float(rng.uniform(3.5, 6.0))
    pocket_x0 = float(pocket_mid_x - 0.5 * pocket_len)
    pocket_x1 = float(pocket_mid_x + 0.5 * pocket_len)
    bay_width = float(lane_width + bay_extra)

    lower_y = float(lane_y - 0.5 * lane_width)
    upper_y = float(lane_y + 0.5 * lane_width)
    bay_upper = float(lane_y + 0.5 * bay_width)
    bay_lower = float(lane_y - 0.5 * bay_width)

    if branch_sign > 0.0:
        walls = [
            _role_obstacle(0.5 * (x_left + x_right), lane_y - 0.5 * lane_width - 0.5 * wall_t, x_right - x_left, wall_t, 0.0, 'channel'),
            _role_obstacle(0.5 * (x_left + pocket_x0), lane_y + 0.5 * lane_width + 0.5 * wall_t, pocket_x0 - x_left, wall_t, 0.0, 'channel'),
            _role_obstacle(0.5 * (pocket_x1 + x_right), lane_y + 0.5 * lane_width + 0.5 * wall_t, x_right - pocket_x1, wall_t, 0.0, 'channel'),
            _role_obstacle(pocket_mid_x, bay_upper + 0.5 * wall_t, pocket_len, wall_t, 0.0, 'hybrid'),
        ]
        bay_y_lo = float(upper_y + 0.34)
        bay_y_hi = float(bay_upper - 0.34)
        corridor_polygons = [
            _rect_polygon(x_left, x_right, lower_y, upper_y),
            _rect_polygon(pocket_x0, pocket_x1, upper_y, bay_upper),
        ]
    else:
        walls = [
            _role_obstacle(0.5 * (x_left + x_right), lane_y + 0.5 * lane_width + 0.5 * wall_t, x_right - x_left, wall_t, 0.0, 'channel'),
            _role_obstacle(0.5 * (x_left + pocket_x0), lane_y - 0.5 * lane_width - 0.5 * wall_t, pocket_x0 - x_left, wall_t, 0.0, 'channel'),
            _role_obstacle(0.5 * (pocket_x1 + x_right), lane_y - 0.5 * lane_width - 0.5 * wall_t, x_right - pocket_x1, wall_t, 0.0, 'channel'),
            _role_obstacle(pocket_mid_x, bay_lower - 0.5 * wall_t, pocket_len, wall_t, 0.0, 'hybrid'),
        ]
        bay_y_lo = float(bay_lower + 0.34)
        bay_y_hi = float(lower_y - 0.34)
        corridor_polygons = [
            _rect_polygon(x_left, x_right, lower_y, upper_y),
            _rect_polygon(pocket_x0, pocket_x1, bay_lower, lower_y),
        ]
    for obs in walls:
        if not _obstacle_in_bounds(cfg, obs.obstacle):
            raise RuntimeError('LC lane wall out of bounds')
        items.append(obs)

    corridor_centerline = [[float(x_left), float(lane_y)], [float(pocket_mid_x), float(lane_y)], [float(x_right), float(lane_y)]]

    heading_gap = float(rng.uniform(*heading_ranges[lvl]))
    d0 = float(rng.uniform(*d0_ranges[lvl]))
    follower = VehicleState(
        vehicle_id=1,
        x=float(pocket_mid_x - d0),
        y=float(lane_y + rng.uniform(-0.08, 0.08)),
        yaw=0.0,
        v=0.0,
        delta=0.0,
        mode=VehicleMode.DOCKING,
    )
    leader_x_pad = 0.72
    depth_ratio = float(clamp(0.52 + 0.30 * ((heading_gap - heading_ranges[lvl][0]) / max(heading_ranges[lvl][1] - heading_ranges[lvl][0], 1e-6)), 0.48, 0.88))
    target_y = float((bay_y_lo + bay_y_hi) * 0.5 + (depth_ratio - 0.5) * (bay_y_hi - bay_y_lo))
    leader = None
    leader_clear_req = {"L1": 0.22, "L2": 0.20, "L3": 0.18}[lvl]
    wall_obstacles = [item.obstacle for item in items]
    for _ in range(220):
        cand = VehicleState(
            vehicle_id=2,
            x=float(rng.uniform(pocket_x0 + leader_x_pad, pocket_x1 - leader_x_pad)),
            y=float(clamp(target_y + rng.normal(0.0, 0.10), min(bay_y_lo, bay_y_hi), max(bay_y_lo, bay_y_hi))),
            yaw=float(branch_sign * math.radians(heading_gap)),
            v=0.0,
            delta=0.0,
            mode=VehicleMode.FREE,
        )
        if collision.in_collision(cand, wall_obstacles, []):
            continue
        if collision.min_clearance_vehicle_obstacles(cand, wall_obstacles) < float(leader_clear_req):
            continue
        leader = cand
        break
    if leader is None:
        raise RuntimeError('LC leader collides with lane walls')

    if not (_in_bounds(cfg, leader, 0.8) and _in_bounds(cfg, follower, 0.8)):
        raise RuntimeError('LC vehicles out of bounds')
    if collision.in_collision(follower, [item.obstacle for item in items], []):
        raise RuntimeError('LC follower collides with lane walls')

    reverse_turn_required = float(clamp(0.65 * (heading_gap - 8.0) / 34.0 + 0.35 * max(0.0, 1.80 - lane_width) / 0.80, 0.0, 1.0))
    stage_anchor_xy = [
        float(pocket_mid_x),
        float(lane_y + branch_sign * min(0.18 * lane_width, 0.30)),
    ]
    leader_seg1 = np.linspace(0.0, 1.0, 7, dtype=float)
    leader_seg2 = np.linspace(0.0, 1.0, 5, dtype=float)
    leader_stage_path_xy = [[float(leader.x), float(leader.y)]]
    leader_stage_path_xy.extend(
        [[float(leader.x), float((1.0 - a) * leader.y + a * stage_anchor_xy[1])] for a in leader_seg1[1:]]
    )
    leader_stage_path_xy.extend(
        [[float((1.0 - a) * leader.x + a * stage_anchor_xy[0]), float(stage_anchor_xy[1])] for a in leader_seg2[1:]]
    )
    geom = VehicleGeometry(cfg.vehicle)
    predock_standoff = float(max(0.70, float(cfg.docking.stage_switch_distance) - 0.55))
    anchor_state = VehicleState(
        vehicle_id=2,
        x=float(stage_anchor_xy[0]),
        y=float(stage_anchor_xy[1]),
        yaw=0.0,
        v=0.0,
        delta=0.0,
        mode=VehicleMode.FREE,
    )
    heading = np.array([1.0, 0.0], dtype=float)
    predock_center = geom.rear_hitch(anchor_state) - heading * float(geom.front_hitch_x + predock_standoff)
    follower_seg1 = np.linspace(0.0, 1.0, 9, dtype=float)
    follower_seg2 = np.linspace(0.0, 1.0, 5, dtype=float)
    follower_stage_path_xy = [[float(follower.x), float(follower.y)]]
    follower_stage_path_xy.extend(
        [[float((1.0 - a) * follower.x + a * predock_center[0]), float((1.0 - a) * follower.y + a * lane_y)] for a in follower_seg1[1:]]
    )
    follower_stage_path_xy.extend(
        [[float(predock_center[0]), float((1.0 - a) * lane_y + a * stage_anchor_xy[1])] for a in follower_seg2[1:]]
    )
    lane_meta = {
        'enabled': True,
        'category': 'LC',
        'corridor_polygons': corridor_polygons,
        'centerline': corridor_centerline,
        'width_profile_m': [float(lane_width), float(bay_width), float(lane_width)],
        'branch_count': 1,
        'bottleneck_ratio': float(lane_width / max(bay_width, 1e-6)),
        'corridor_width_min_m': float(lane_width),
        'corridor_turn_deg_max': float(min(65.0, 14.0 + 0.72 * heading_gap)),
        'drivable_area_ratio': float(sum(abs((poly[0][0]-poly[2][0]) * (poly[0][1]-poly[2][1])) for poly in corridor_polygons) / (float(cfg.environment.width) * float(cfg.environment.height))),
        'leader_reverse_turn_required': reverse_turn_required,
        'leader_micro_adjust_budget_m': float(0.34 + 0.022 * heading_gap + 0.48 * max(0.0, 1.50 - lane_width)),
        'heading_gap_regime': ('small' if heading_gap < 15.0 else ('medium' if heading_gap < 38.0 else 'large')),
        'bay_depth_m': float(0.5 * bay_extra),
        'leader_stage_anchor_xy': stage_anchor_xy,
        'leader_stage_anchor_yaw': 0.0,
        'leader_stage_path_xy': leader_stage_path_xy,
        'follower_stage_path_xy': follower_stage_path_xy,
        'corridor_entry_xy': [float(pocket_mid_x), float(lane_y)],
    }
    base_time = {"L1": 24.0, "L2": 30.0, "L3": 36.0}[lvl]
    if split_kind == 'test':
        base_time += 2.0
    elif split_kind == 'challenge':
        base_time += 3.0
    return leader, follower, items, base_time, lane_meta


def _relaxed_tuning_label(family: str, descriptors: dict[str, float], direct_los_blocked: bool) -> bool:
    fam = str(family).upper()
    if fam == "SC":
        return bool((direct_los_blocked or descriptors["occlusion_index"] >= 0.20) and descriptors["d0_m"] >= 5.5 and descriptors["detour_factor"] <= 1.20 and descriptors["staging_shift_required_m"] <= 0.45)
    if fam == "FC":
        return bool(descriptors["d0_m"] >= 5.0 and descriptors["dock_zone_clearance_m"] <= 1.20 and descriptors["detour_factor"] <= 1.25 and descriptors["staging_shift_required_m"] <= 0.50)
    if fam == "EC":
        return bool(direct_los_blocked and descriptors["staging_shift_required_m"] >= 0.80 and descriptors["detour_factor"] >= 1.18)
    if fam == "LC":
        return bool(
            descriptors.get('corridor_width_min_m', 0.0) > 0.0
            and descriptors.get('off_lane_shortcut_gap_m', 0.0) >= 0.75
            and descriptors.get('leader_micro_adjust_budget_m', 0.0) >= 0.35
            and descriptors.get('leader_reverse_turn_required', 0.0) >= 0.05
        )
    return False


def generate_candidate_scene(cfg: Config, *, family: str, difficulty: str, split: str, seed: int, existing_cell_descriptors: Sequence[dict[str, float]], cell_fill_ratio: float) -> CandidateScene:
    fam = str(family).upper()
    lvl = str(difficulty).upper()
    family_generators = {
        "CF": _generate_cf,
        "SC": _generate_sc,
        "FC": _generate_fc,
        "EC": _generate_ec,
        "LC": _generate_lc,
    }
    if fam not in family_generators:
        raise KeyError(f"unsupported family {family}")
    for local_attempt in range(300):
        rng = np.random.default_rng(int(seed) + 97 * local_attempt)
        try:
            generated = family_generators[fam](cfg, rng, difficulty=lvl, seed=int(seed) + local_attempt, split=str(split))
            if len(generated) == 5:
                leader, follower, role_obstacles, max_time_s, generator_aux = generated
            else:
                leader, follower, role_obstacles, max_time_s = generated
                generator_aux = {}
        except RuntimeError:
            continue
        collision = CollisionEngine(cfg.vehicle, cfg.safety)
        if not (_in_bounds(cfg, leader, 0.8) and _in_bounds(cfg, follower, 0.8)):
            continue
        obstacles = [item.obstacle for item in role_obstacles]
        roles = [item.role for item in role_obstacles]
        if collision.in_collision(leader, obstacles, []):
            continue
        if collision.in_collision(follower, obstacles, []):
            continue
        if np.linalg.norm(leader.xy() - follower.xy()) < 4.5:
            continue
        clearance_req = 0.12 if fam == "LC" else (0.15 if fam == "EC" else (0.24 if fam == "FC" else 0.35))
        if collision.min_clearance_vehicle_obstacles(leader, obstacles) < clearance_req:
            continue
        if collision.min_clearance_vehicle_obstacles(follower, obstacles) < max(0.24, clearance_req):
            continue
        descriptors, aux = compute_descriptors(
            cfg,
            seed=int(seed),
            leader=leader,
            follower=follower,
            obstacles=obstacles,
            obstacle_roles=roles,
            family=fam,
            difficulty=lvl,
            lane_meta=dict(generator_aux),
        )
        if fam == 'LC':
            lane_payload = dict(aux.get('lane', {}))
            entry_xy = np.asarray(lane_payload.get('corridor_entry_xy', [leader.x, leader.y]), dtype=float)
            anchor_xy = np.asarray(lane_payload.get('leader_stage_anchor_xy', [leader.x, leader.y]), dtype=float)
            legal_len = float(
                np.linalg.norm(follower.xy() - entry_xy)
                + np.linalg.norm(entry_xy - anchor_xy)
                + np.linalg.norm(anchor_xy - leader.xy())
            )
            nominal_goal = VehicleState(
                vehicle_id=int(follower.vehicle_id),
                x=float(leader.x),
                y=float(leader.y),
                yaw=0.0,
                v=0.0,
                delta=0.0,
                mode=follower.mode,
            )
            lane_fidelity, shortcut_gap = _lane_fidelity_audit(
                follower_start=follower,
                follower_goal=nominal_goal,
                corridor_polygons=list(lane_payload.get('corridor_polygons', [])),
                legal_path_len_m=float(legal_len),
            )
            stage_shift = float(np.linalg.norm(anchor_xy - leader.xy()))
            lane_payload['off_lane_shortcut_gap_m'] = float(shortcut_gap)
            lane_payload['lane_fidelity'] = bool(lane_fidelity)
            lane_payload['leader_micro_adjust_budget_m'] = float(max(lane_payload.get('leader_micro_adjust_budget_m', 0.0), stage_shift + np.linalg.norm(anchor_xy - entry_xy)))
            aux = dict(aux)
            aux['lane'] = lane_payload
            aux['lane_fidelity'] = bool(lane_fidelity)
            descriptors['off_lane_shortcut_gap_m'] = float(shortcut_gap)
            descriptors['leader_micro_adjust_budget_m'] = float(lane_payload['leader_micro_adjust_budget_m'])
            descriptors['staging_shift_required_m'] = float(stage_shift)
        quality = quality_record(
            family=fam,
            difficulty=lvl,
            descriptors=descriptors,
            direct_los_blocked=bool(aux["direct_los_blocked"]),
            existing_cell_descriptors=existing_cell_descriptors,
            cell_fill_ratio=float(cell_fill_ratio),
            lane_fidelity=bool(aux.get('lane_fidelity', True)),
        )
        relaxed_ok = bool(str(split).lower() == "tuning" and _relaxed_tuning_label(fam, descriptors, bool(aux["direct_los_blocked"])))
        if not bool(quality["valid"]):
            continue
        if not bool(quality["label_fidelity"]) and not relaxed_ok:
            continue
        if relaxed_ok and not bool(quality["label_fidelity"]):
            quality = dict(quality)
            quality["label_fidelity"] = True
            quality["scene_quality_score"] = max(float(quality["scene_quality_score"]), 0.72)
        min_diversity = 0.015 if float(cell_fill_ratio) < 0.67 else 0.008
        if fam in {"SC", "EC", "LC"}:
            min_diversity = min(min_diversity, 0.006)
        if existing_cell_descriptors and float(quality["raw_diversity_distance"]) < min_diversity:
            continue
        return CandidateScene(
            family=fam,
            difficulty=lvl,
            seed=int(seed),
            max_time_s=float(max_time_s),
            leader=leader,
            follower=follower,
            obstacles=tuple(obstacles),
            roles=tuple(roles),
            descriptors=descriptors,
            quality=quality,
            aux=aux,
        )
    raise RuntimeError(f"failed to generate DockBench scene for {fam}-{lvl} seed={seed}")


def build_dockbench_v1(cfg: Config, *, root: str | Path | None = None, base_seed: int = 20260306) -> dict[str, Any]:
    ds_root = dataset_root(root)
    ds_root.mkdir(parents=True, exist_ok=True)
    scene_dir = ds_root / "scenes"
    if scene_dir.exists():
        for scene_path in scene_dir.glob('*.json'):
            scene_path.unlink()
    scene_dir.mkdir(parents=True, exist_ok=True)
    for stale_name in (
        'dockbench_v1_manifest.json',
        'dockbench_v1_split_tuning.json',
        'dockbench_v1_split_test.json',
        'dockbench_v1_split_challenge.json',
        'dockbench_v1_representatives.json',
        'dockbench_v1_quality_report.json',
        'dockbench_v1_generation_summary.json',
        'dockbench_v1_split_freeze_summary.json',
        'dockbench_v1_stage0_audit.json',
        'DOCKBENCH_V1_STAGE0_AUDIT.md',
    ):
        stale_path = ds_root / stale_name
        if stale_path.exists():
            stale_path.unlink()
    records: list[DockBenchSceneSpec] = []
    representatives: dict[str, DockBenchSceneSpec] = {}
    quality_summary: dict[str, Any] = {
        "schema_version": "dockbench_v1_quality_report.1",
        "dataset_name": ds_root.name,
        "scene_count_target": int(len(FAMILIES) * len(DIFFICULTIES) * len(_SPLIT_SEQUENCE)),
        "cells": {},
    }

    ordinal = 1
    scene_counter = 0
    for fam_idx, family in enumerate(FAMILIES):
        for diff_idx, difficulty in enumerate(DIFFICULTIES):
            cell_key = f"{family}-{difficulty}"
            cell_records: list[DockBenchSceneSpec] = []
            cell_descriptors: list[dict[str, float]] = []
            cell_quality: list[dict[str, Any]] = []
            attempt = 0
            while len(cell_records) < len(_SPLIT_SEQUENCE):
                candidate_seed = int(base_seed + fam_idx * 100000 + diff_idx * 10000 + attempt * 131 + 17)
                split = _SPLIT_SEQUENCE[len(cell_records)]
                try:
                    candidate = generate_candidate_scene(
                        cfg,
                        family=family,
                        difficulty=difficulty,
                        split=split,
                        seed=candidate_seed,
                        existing_cell_descriptors=cell_descriptors,
                        cell_fill_ratio=float(len(cell_records) / len(_SPLIT_SEQUENCE)),
                    )
                except RuntimeError:
                    if split == "challenge":
                        try:
                            candidate = generate_candidate_scene(
                                cfg,
                                family=family,
                                difficulty=difficulty,
                                split="test",
                                seed=candidate_seed + 19,
                                existing_cell_descriptors=cell_descriptors,
                                cell_fill_ratio=float(len(cell_records) / len(_SPLIT_SEQUENCE)),
                            )
                        except RuntimeError:
                            attempt += 1
                            if attempt > 5000:
                                raise RuntimeError(f"unable to populate DockBench cell {family}-{difficulty}")
                            continue
                    else:
                        attempt += 1
                        if attempt > 5000:
                            raise RuntimeError(f"unable to populate DockBench cell {family}-{difficulty}")
                        continue
                scene_id = build_scene_id(family=family, difficulty=difficulty, ordinal=ordinal)
                ordinal += 1
                payload = build_scene_payload(
                    cfg,
                    scene_id=scene_id,
                    split=split,
                    family=family,
                    difficulty=difficulty,
                    seed=candidate.seed,
                    max_time_s=candidate.max_time_s,
                    leader=candidate.leader,
                    follower=candidate.follower,
                    obstacles=candidate.obstacles,
                    obstacle_roles=candidate.roles,
                    descriptors=candidate.descriptors,
                    quality=candidate.quality,
                    aux=candidate.aux,
                )
                scene_path = scene_dir / f"{scene_id}.json"
                scene_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
                spec = DockBenchSceneSpec(
                    scene_id=scene_id,
                    split=split,
                    family=family,
                    difficulty=difficulty,
                    seed=int(candidate.seed),
                    style=style_for_family(family),
                    subset_tag=family_label(family),
                    max_time_s=float(candidate.max_time_s),
                    scenario_json=str(scene_path.resolve()),
                )
                records.append(spec)
                cell_records.append(spec)
                cell_descriptors.append(candidate.descriptors)
                cell_quality.append(candidate.quality)
                attempt += 1
                scene_counter += 1
            quality_summary["cells"][cell_key] = {
                "count": len(cell_records),
                "splits": {split: sum(1 for record in cell_records if record.split == split) for split in SPLIT_COUNTS},
                "quality_mean": float(np.mean([float(item["scene_quality_score"]) for item in cell_quality])) if cell_quality else 0.0,
                "min_diversity": float(min(float(item["raw_diversity_distance"]) for item in cell_quality[1:])) if len(cell_quality) > 1 else 1.0,
                "descriptor_centroid": {
                    key: float(np.mean([float(desc[key]) for desc in cell_descriptors])) for key in cell_descriptors[0]
                },
            }
            rep_key = f"{family}_{difficulty}"
            test_l2 = next((record for record in cell_records if record.split == "test"), cell_records[0])
            if rep_key in {"CF_L2", "SC_L2", "FC_L2", "EC_L2", "LC_L2"}:
                representatives[rep_key] = test_l2

    save_manifest_files(ds_root, records=records, representatives=representatives, quality_report=quality_summary)
    summary = {
        "dataset_root": str(ds_root),
        "scene_count": int(scene_counter),
        "manifest": str((ds_root / "dockbench_v1_manifest.json").resolve()),
        "quality_report": str((ds_root / "dockbench_v1_quality_report.json").resolve()),
        "representatives": {key: value.to_dict() for key, value in representatives.items()},
    }
    (ds_root / "dockbench_v1_generation_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary
