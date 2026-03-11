from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

from .collision import CollisionEngine, obstacle_polygon
from .config import Config
from .kinematics import VehicleGeometry
from .math_utils import angle_diff, clamp
from .sensors import line_blocked_by_obstacles
from .types import Obstacle, VehicleState

SCHEMA_VERSION = "dockbench-v1.0"
GENERATOR_VERSION = "dock_sgen_1.1"
DATASET_NAME = "dockbench_v1"
FAMILIES: tuple[str, ...] = ("CF", "SC", "FC", "EC", "LC")
DIFFICULTIES: tuple[str, ...] = ("L1", "L2", "L3")
SPLIT_COUNTS: dict[str, int] = {"tuning": 1, "test": 4, "challenge": 1}
FAMILY_LABELS: dict[str, str] = {
    "CF": "Common-Feasible",
    "SC": "Switching-Critical",
    "FC": "Funnel-Critical",
    "EC": "Extension-Critical",
    "LC": "Lane-Constrained",
}
STYLE_BY_FAMILY: dict[str, str] = {
    "CF": "common_feasible",
    "SC": "switching_critical",
    "FC": "funnel_critical",
    "EC": "extension_critical",
    "LC": "lane_constrained",
}
REPRESENTATIVE_KEYS: tuple[str, ...] = ("CF_L2", "SC_L2", "FC_L2", "EC_L2", "LC_L2")

_DESCRIPTOR_KEYS: tuple[str, ...] = (
    "d0_m",
    "heading_diff_deg",
    "bearing_deg",
    "occlusion_index",
    "detour_factor",
    "staging_shift_required_m",
    "capture_constraint_index",
    "dock_zone_clearance_m",
    "homotopy_count",
    "corridor_width_min_m",
    "bottleneck_ratio",
    "corridor_turn_deg_max",
    "branch_count",
    "drivable_area_ratio",
    "off_lane_shortcut_gap_m",
    "leader_reverse_turn_required",
    "leader_micro_adjust_budget_m",
)


@dataclass(frozen=True)
class DockBenchSceneSpec:
    scene_id: str
    split: str
    family: str
    difficulty: str
    seed: int
    style: str
    subset_tag: str
    max_time_s: float
    scenario_json: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def family_label(family: str) -> str:
    key = str(family).upper()
    if key not in FAMILY_LABELS:
        raise KeyError(f"Unknown DockBench family: {family}")
    return FAMILY_LABELS[key]


def style_for_family(family: str) -> str:
    key = str(family).upper()
    if key not in STYLE_BY_FAMILY:
        raise KeyError(f"Unknown DockBench family: {family}")
    return STYLE_BY_FAMILY[key]


def dataset_root(root: str | Path | None = None) -> Path:
    if root is None:
        return Path(__file__).resolve().parents[1] / "data" / DATASET_NAME
    return Path(root).expanduser().resolve()


def build_scene_id(*, family: str, difficulty: str, ordinal: int) -> str:
    return f"DBv1-{str(family).upper()}-{str(difficulty).upper()}-{int(ordinal):03d}"


def _point_in_poly(point: np.ndarray, poly: np.ndarray) -> bool:
    x = float(point[0])
    y = float(point[1])
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = float(poly[i, 0]), float(poly[i, 1])
        xj, yj = float(poly[j, 0]), float(poly[j, 1])
        cond = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / max(yj - yi, 1e-9) + xi)
        if cond:
            inside = not inside
        j = i
    return bool(inside)


def _segment_samples(p0: np.ndarray, p1: np.ndarray, n: int = 41) -> np.ndarray:
    alphas = np.linspace(0.0, 1.0, int(max(n, 2)), dtype=float)
    return (1.0 - alphas)[:, None] * p0[None, :] + alphas[:, None] * p1[None, :]


def _obstacle_coverage_ratio(p0: np.ndarray, p1: np.ndarray, obstacles: Sequence[Obstacle]) -> float:
    if not obstacles:
        return 0.0
    samples = _segment_samples(np.asarray(p0, dtype=float), np.asarray(p1, dtype=float), n=51)
    covered = 0
    for pt in samples:
        if any(_point_in_poly(pt, obstacle_polygon(obs)) for obs in obstacles):
            covered += 1
    return float(covered / max(len(samples), 1))


def _path_length(path_xy: np.ndarray | None) -> float:
    if path_xy is None or len(path_xy) < 2:
        return math.inf
    return float(np.sum(np.linalg.norm(np.diff(path_xy, axis=0), axis=1)))


def _boundary_margin(cfg: Config, state: VehicleState) -> float:
    half_w = 0.5 * float(cfg.environment.width)
    half_h = 0.5 * float(cfg.environment.height)
    return float(min(half_w - abs(float(state.x)), half_h - abs(float(state.y))))


def _dock_zone_clearance(cfg: Config, leader: VehicleState, obstacles: Sequence[Obstacle]) -> float:
    geom = VehicleGeometry(cfg.vehicle)
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    heading = np.array([math.cos(float(leader.yaw)), math.sin(float(leader.yaw))], dtype=float)
    rear = geom.rear_hitch(leader)
    offsets = (0.30, 0.55, 0.80)
    clearances: list[float] = []
    for standoff in offsets:
        center = rear - heading * float(geom.front_hitch_x + standoff)
        probe = VehicleState(vehicle_id=-1, x=float(center[0]), y=float(center[1]), yaw=float(leader.yaw), v=0.0, delta=0.0)
        clearances.append(float(collision.min_clearance_vehicle_obstacles(probe, obstacles)))
    return float(min(clearances) if clearances else 1e6)


def _estimate_homotopy_count(obstacle_roles: Sequence[str], *, blocked: bool, clutter_ratio: float) -> int:
    roles = {str(role) for role in obstacle_roles}
    score = 1
    if blocked or "screen" in roles or "channel" in roles:
        score += 1
    if clutter_ratio >= 0.10 or len(roles & {"background", "hybrid"}) >= 2:
        score += 1
    return int(max(1, min(score, 3)))


def descriptor_vector(descriptors: dict[str, float]) -> np.ndarray:
    return np.asarray([float(descriptors[k]) for k in _DESCRIPTOR_KEYS], dtype=float)


def descriptor_distance(a: dict[str, float], b: dict[str, float]) -> float:
    wa = descriptor_vector(a)
    wb = descriptor_vector(b)
    scale = np.asarray([
        12.0,
        35.0,
        180.0,
        1.0,
        0.6,
        2.4,
        1.0,
        1.5,
        3.0,
        1.6,
        1.0,
        90.0,
        3.0,
        0.20,
        6.0,
        1.0,
        3.0,
    ], dtype=float)
    return float(np.linalg.norm((wa - wb) / scale))


def family_thresholds(family: str, difficulty: str) -> dict[str, tuple[float, float]]:
    fam = str(family).upper()
    lvl = str(difficulty).upper()
    thresholds: dict[str, dict[str, tuple[float, float]]] = {
        "CF": {
            "d0_m": (5.5, 10.5),
            "heading_diff_deg": (0.0, {"L1": 12.0, "L2": 18.0, "L3": 26.0}[lvl]),
            "occlusion_index": (0.0, 0.24),
            "detour_factor": (1.0, {"L1": 1.05, "L2": 1.10, "L3": 1.15}[lvl]),
            "staging_shift_required_m": (0.0, {"L1": 0.22, "L2": 0.34, "L3": 0.48}[lvl]),
            "dock_zone_clearance_m": ({"L1": 0.85, "L2": 0.72, "L3": 0.60}[lvl], 1e6),
            "capture_constraint_index": (0.0, {"L1": 0.42, "L2": 0.54, "L3": 0.64}[lvl]),
        },
        "SC": {
            "d0_m": (5.8, 10.5),
            "heading_diff_deg": (0.0, {"L1": 18.0, "L2": 24.0, "L3": 30.0}[lvl]),
            "occlusion_index": ({"L1": 0.12, "L2": 0.22, "L3": 0.36}[lvl], 1.0),
            "detour_factor": (1.0, {"L1": 1.14, "L2": 1.20, "L3": 1.30}[lvl]),
            "staging_shift_required_m": (0.0, {"L1": 0.30, "L2": 0.44, "L3": 0.60}[lvl]),
            "dock_zone_clearance_m": (0.35, 1e6),
            "capture_constraint_index": ({"L1": 0.10, "L2": 0.08, "L3": 0.08}[lvl], {"L1": 0.66, "L2": 0.74, "L3": 0.84}[lvl]),
        },
        "FC": {
            "d0_m": (5.0, 9.0),
            "heading_diff_deg": ({"L1": 6.0, "L2": 10.0, "L3": 14.0}[lvl], {"L1": 18.0, "L2": 26.0, "L3": 34.0}[lvl]),
            "occlusion_index": (0.0, 1.0),
            "detour_factor": (1.0, {"L1": 1.20, "L2": 1.28, "L3": 1.36}[lvl]),
            "staging_shift_required_m": (0.0, {"L1": 0.38, "L2": 0.54, "L3": 0.70}[lvl]),
            "dock_zone_clearance_m": ({"L1": 0.45, "L2": 0.30, "L3": 0.10}[lvl], {"L1": 1.12, "L2": 0.92, "L3": 0.72}[lvl]),
            "capture_constraint_index": ({"L1": 0.15, "L2": 0.24, "L3": 0.30}[lvl], 1.0),
        },
        "EC": {
            "d0_m": (6.0, 12.5),
            "heading_diff_deg": (0.0, {"L1": 22.0, "L2": 30.0, "L3": 38.0}[lvl]),
            "occlusion_index": ({"L1": 0.16, "L2": 0.28, "L3": 0.38}[lvl], 1.0),
            "detour_factor": ({"L1": 1.10, "L2": 1.18, "L3": 1.28}[lvl], 3.0),
            "staging_shift_required_m": ({"L1": 0.72, "L2": 1.02, "L3": 1.35}[lvl], 6.0),
            "dock_zone_clearance_m": (0.0, {"L1": 0.90, "L2": 0.76, "L3": 0.64}[lvl]),
            "capture_constraint_index": ({"L1": 0.16, "L2": 0.36, "L3": 0.44}[lvl], 1.0),
        },
        "LC": {
            "d0_m": ({"L1": 5.0, "L2": 5.4, "L3": 4.8}[lvl], {"L1": 9.5, "L2": 10.0, "L3": 10.5}[lvl]),
            "heading_diff_deg": ({"L1": 0.0, "L2": 12.0, "L3": 28.0}[lvl], {"L1": 16.0, "L2": 38.0, "L3": 75.0}[lvl]),
            "detour_factor": ({"L1": 1.08, "L2": 1.12, "L3": 1.18}[lvl], 2.6),
            "staging_shift_required_m": ({"L1": 0.20, "L2": 0.55, "L3": 0.95}[lvl], 4.0),
            "corridor_width_min_m": ({"L1": 1.45, "L2": 1.20, "L3": 0.95}[lvl], {"L1": 2.20, "L2": 1.90, "L3": 1.60}[lvl]),
            "bottleneck_ratio": (0.25, 1.0),
            "corridor_turn_deg_max": (0.0, {"L1": 25.0, "L2": 45.0, "L3": 65.0}[lvl]),
            "branch_count": (1.0, 3.0),
            "drivable_area_ratio": (0.04, 0.22),
            "off_lane_shortcut_gap_m": ({"L1": 0.75, "L2": 1.00, "L3": 1.25}[lvl], 8.0),
            "leader_reverse_turn_required": ({"L1": 0.0, "L2": 0.30, "L3": 0.65}[lvl], 1.0),
            "leader_micro_adjust_budget_m": ({"L1": 0.25, "L2": 0.70, "L3": 1.15}[lvl], 4.0),
            "capture_constraint_index": ({"L1": 0.16, "L2": 0.28, "L3": 0.40}[lvl], 1.0),
        },
    }
    return thresholds[fam]


def label_fidelity(*, family: str, difficulty: str, descriptors: dict[str, float], direct_los_blocked: bool) -> bool:
    thresholds = family_thresholds(family, difficulty)
    for key, (lo, hi) in thresholds.items():
        value = float(descriptors[key])
        if value < lo - 1e-9 or value > hi + 1e-9:
            return False
    fam = str(family).upper()
    if fam == "CF" and direct_los_blocked:
        return False
    if fam in {"SC", "EC"} and not direct_los_blocked:
        return False
    if fam == "FC" and float(descriptors["dock_zone_clearance_m"]) > 1.05:
        return False
    if fam == "LC":
        if float(descriptors.get("corridor_width_min_m", 0.0)) <= 0.0:
            return False
        if float(descriptors.get("off_lane_shortcut_gap_m", 0.0)) < 0.75:
            return False
        if float(descriptors.get("leader_micro_adjust_budget_m", 0.0)) < 0.20:
            return False
    return True


def compute_descriptors(
    cfg: Config,
    *,
    seed: int,
    leader: VehicleState,
    follower: VehicleState,
    obstacles: Sequence[Obstacle],
    obstacle_roles: Sequence[str],
    family: str | None = None,
    difficulty: str | None = None,
    lane_meta: dict[str, Any] | None = None,
) -> tuple[dict[str, float], dict[str, Any]]:
    geom = VehicleGeometry(cfg.vehicle)
    rear = geom.rear_hitch(leader)
    cam = geom.front_hitch(follower)
    d0 = float(np.linalg.norm(cam - rear))
    heading_diff = float(abs(math.degrees(angle_diff(float(leader.yaw), float(follower.yaw)))))
    bearing = float(math.degrees(angle_diff(math.atan2(float(follower.y - leader.y), float(follower.x - leader.x)), float(leader.yaw))))
    clutter_ratio = float(sum(float(obs.width) * float(obs.height) for obs in obstacles) / (float(cfg.environment.width) * float(cfg.environment.height)))
    blocked = bool(line_blocked_by_obstacles(cam, rear, list(obstacles)))
    coverage = _obstacle_coverage_ratio(cam, rear, obstacles)
    occlusion_index = float(clamp((1.8 * coverage) + (0.30 if blocked else 0.0), 0.0, 1.0))
    dock_clearance = _dock_zone_clearance(cfg, leader, obstacles)
    fam = str(family).upper() if family is not None else None
    lvl = str(difficulty).upper() if difficulty is not None else None
    level_bias = {None: 0.0, "L1": 0.02, "L2": 0.08, "L3": 0.16}[lvl]
    screen_count = float(sum(1 for role in obstacle_roles if role == "screen"))
    channel_count = float(sum(1 for role in obstacle_roles if role in {"channel", "hybrid"}))
    dock_role_count = float(sum(1 for role in obstacle_roles if role == "dock_zone"))
    background_count = float(sum(1 for role in obstacle_roles if role == "background"))

    lane = dict(lane_meta or {})
    corridor_width_min = float(lane.get("corridor_width_min_m", 0.0))
    bottleneck_ratio = float(lane.get("bottleneck_ratio", 0.0))
    corridor_turn_deg = float(lane.get("corridor_turn_deg_max", 0.0))
    branch_count = float(lane.get("branch_count", 0.0))
    drivable_area_ratio = float(lane.get("drivable_area_ratio", 1.0 if fam != "LC" else 0.0))
    off_lane_shortcut_gap = float(lane.get("off_lane_shortcut_gap_m", 0.0))
    leader_reverse_turn_required = float(lane.get("leader_reverse_turn_required", 0.0))
    leader_micro_adjust_budget = float(lane.get("leader_micro_adjust_budget_m", 0.0))
    lane_fidelity = bool(lane.get("lane_fidelity", False if fam == "LC" else True))

    if fam == "CF":
        stage_shift = 0.03 + 0.05 * level_bias + 0.03 * channel_count + 0.06 * max(0.65 - dock_clearance, 0.0)
        detour_factor = 1.00 + 0.01 * background_count + 0.02 * channel_count + 0.02 * level_bias
    elif fam == "SC":
        stage_shift = 0.08 + 0.12 * screen_count + 0.03 * background_count + 0.05 * level_bias
        detour_factor = 1.03 + 0.04 * screen_count + 0.02 * background_count + 0.06 * level_bias
    elif fam == "FC":
        stage_shift = 0.10 + 0.08 * dock_role_count + 0.05 * channel_count + 0.05 * level_bias
        detour_factor = 1.05 + 0.04 * dock_role_count + 0.02 * background_count + 0.05 * level_bias
    elif fam == "EC":
        stage_shift = 0.74 + 0.35 * screen_count + 0.18 * channel_count + 0.22 * max(0.75 - dock_clearance, 0.0) + 0.10 * level_bias
        detour_factor = 1.10 + 0.06 * screen_count + 0.04 * channel_count + 0.03 * background_count + 0.07 * level_bias
    elif fam == "LC":
        stage_shift = float(max(0.20, leader_micro_adjust_budget))
        detour_factor = float(max(1.05, 1.02 + 0.08 * branch_count + 0.12 * max(0.0, 1.0 - bottleneck_ratio)))
    else:
        stage_shift = 0.06 + 0.08 * blocked + 0.06 * channel_count + 0.04 * max(0.75 - dock_clearance, 0.0)
        detour_factor = 1.00 + 0.04 * blocked + 0.01 * background_count + 0.02 * channel_count

    stage_shift = float(clamp(stage_shift, 0.0, 2.4))
    detour_factor = float(clamp(detour_factor, 1.0, 3.0))
    heading_term = float(clamp(heading_diff / 35.0, 0.0, 1.0))
    clearance_term = float(clamp((0.95 - dock_clearance) / 0.95, 0.0, 1.0))
    detour_term = float(clamp((detour_factor - 1.0) / 0.55, 0.0, 1.0))
    stage_term = float(clamp(stage_shift / 2.0, 0.0, 1.0))
    lane_width_term = float(clamp((1.8 - corridor_width_min) / 1.2, 0.0, 1.0)) if corridor_width_min > 0.0 else 0.0
    reverse_term = float(clamp(leader_reverse_turn_required, 0.0, 1.0))
    capture_constraint = float(clamp(0.32 * heading_term + 0.26 * clearance_term + 0.14 * detour_term + 0.10 * stage_term + 0.10 * lane_width_term + 0.08 * reverse_term, 0.0, 1.0))
    boundary_margin = min(_boundary_margin(cfg, leader), _boundary_margin(cfg, follower))
    homotopy_count = _estimate_homotopy_count(obstacle_roles, blocked=blocked, clutter_ratio=clutter_ratio)

    descriptors = {
        "d0_m": d0,
        "heading_diff_deg": heading_diff,
        "bearing_deg": bearing,
        "boundary_margin_m": float(boundary_margin),
        "clutter_ratio": clutter_ratio,
        "occlusion_index": occlusion_index,
        "detour_factor": detour_factor,
        "homotopy_count": float(homotopy_count),
        "dock_zone_clearance_m": dock_clearance,
        "staging_shift_required_m": stage_shift,
        "capture_constraint_index": capture_constraint,
        "corridor_width_min_m": float(corridor_width_min),
        "bottleneck_ratio": float(bottleneck_ratio),
        "corridor_turn_deg_max": float(corridor_turn_deg),
        "branch_count": float(branch_count),
        "drivable_area_ratio": float(drivable_area_ratio),
        "off_lane_shortcut_gap_m": float(off_lane_shortcut_gap),
        "leader_reverse_turn_required": float(leader_reverse_turn_required),
        "leader_micro_adjust_budget_m": float(leader_micro_adjust_budget),
    }
    leader_goal = leader.copy()
    leader_goal.x = float(leader.x + stage_shift * math.cos(float(leader.yaw)))
    leader_goal.y = float(leader.y + stage_shift * math.sin(float(leader.yaw)))
    predock_center = rear - np.array([math.cos(float(leader.yaw)), math.sin(float(leader.yaw))], dtype=float) * float(geom.front_hitch_x + 0.85)
    follower_goal = follower.copy()
    follower_goal.x = float(predock_center[0])
    follower_goal.y = float(predock_center[1])
    follower_goal.yaw = float(leader.yaw)
    aux = {
        "stage_plan": {
            "leader_goal": asdict(leader_goal),
            "follower_goal": asdict(follower_goal),
            "leader_path_xy": [[float(leader.x), float(leader.y)], [float(leader_goal.x), float(leader_goal.y)]],
            "follower_path_xy": [[float(follower.x), float(follower.y)], [float(follower_goal.x), float(follower_goal.y)]],
            "score": float(stage_shift + detour_factor),
            "reason": "heuristic_descriptor_estimate",
        },
        "direct_los_blocked": blocked,
        "large_obstacle_present": bool(any(max(float(obs.width), float(obs.height)) >= 2.8 for obs in obstacles)),
        "lane": lane,
        "lane_fidelity": bool(lane_fidelity),
    }
    return descriptors, aux


def quality_record(
    *,
    family: str,
    difficulty: str,
    descriptors: dict[str, float],
    direct_los_blocked: bool,
    existing_cell_descriptors: Sequence[dict[str, float]],
    cell_fill_ratio: float,
    lane_fidelity: bool = True,
) -> dict[str, Any]:
    valid = bool(
        descriptors["boundary_margin_m"] >= 0.65
        and descriptors["dock_zone_clearance_m"] >= 0.0
        and math.isfinite(descriptors["detour_factor"])
        and descriptors["d0_m"] >= 4.5
        and bool(lane_fidelity)
    )
    label_ok = bool(label_fidelity(family=family, difficulty=difficulty, descriptors=descriptors, direct_los_blocked=direct_los_blocked) and bool(lane_fidelity))
    if existing_cell_descriptors:
        diversity = min(descriptor_distance(descriptors, other) for other in existing_cell_descriptors)
        diversity_score = float(clamp(diversity / 0.18, 0.0, 1.0))
    else:
        diversity = math.inf
        diversity_score = 1.0
    balance_score = float(clamp(1.0 - float(cell_fill_ratio), 0.0, 1.0))
    base = float(0.7 * diversity_score + 0.3 * balance_score)
    scene_quality = float(base if valid and label_ok else 0.0)
    return {
        "valid": valid,
        "label_fidelity": label_ok,
        "lane_fidelity": bool(lane_fidelity),
        "diversity_score": diversity_score,
        "balance_score": balance_score,
        "scene_quality_score": scene_quality,
        "raw_diversity_distance": float(diversity if math.isfinite(diversity) else 9.0),
    }


def build_scene_payload(
    cfg: Config,
    *,
    scene_id: str,
    split: str,
    family: str,
    difficulty: str,
    seed: int,
    max_time_s: float,
    leader: VehicleState,
    follower: VehicleState,
    obstacles: Sequence[Obstacle],
    obstacle_roles: Sequence[str],
    descriptors: dict[str, float],
    quality: dict[str, Any],
    aux: dict[str, Any],
) -> dict[str, Any]:
    family_code = str(family).upper()
    lane_payload = dict(aux.get("lane", {})) if isinstance(aux.get("lane", {}), dict) else {}
    scenario = {
        "seed": int(seed),
        "style": style_for_family(family_code),
        "planner_seed": int(seed) + 97,
        "obstacles": [asdict(obs) for obs in obstacles],
        "leader": asdict(leader),
        "follower": asdict(follower),
        "leader_relocation_m": float(descriptors["staging_shift_required_m"]),
        "follower_detour_ratio": float(descriptors["detour_factor"]),
        "direct_los_blocked": bool(aux["direct_los_blocked"]),
        "large_obstacle_present": bool(aux["large_obstacle_present"]),
        "subset_tag": family_label(family_code),
        "lane_constrained": bool(lane_payload.get("enabled", False)),
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "scene_id": str(scene_id),
        "split": str(split),
        "family": family_code,
        "difficulty": str(difficulty).upper(),
        "generator_version": GENERATOR_VERSION,
        "max_time_s": float(max_time_s),
        "map": {
            "width_m": float(cfg.environment.width),
            "height_m": float(cfg.environment.height),
            "resolution_m": float(cfg.environment.resolution),
        },
        "leader": {"x": float(leader.x), "y": float(leader.y), "yaw": float(leader.yaw)},
        "follower": {"x": float(follower.x), "y": float(follower.y), "yaw": float(follower.yaw)},
        "obstacles": [
            {
                "x": float(obs.x),
                "y": float(obs.y),
                "width": float(obs.width),
                "height": float(obs.height),
                "yaw": float(obs.yaw),
                "role": str(role),
            }
            for obs, role in zip(obstacles, obstacle_roles)
        ],
        "descriptors": {k: float(v) for k, v in descriptors.items()},
        "quality": {k: (bool(v) if isinstance(v, bool) else float(v)) for k, v in quality.items()},
        "lane": lane_payload,
        "audit": {
            "direct_los_blocked": bool(aux["direct_los_blocked"]),
            "large_obstacle_present": bool(aux["large_obstacle_present"]),
            "stage_plan": aux["stage_plan"],
            "lane_fidelity": bool(aux.get("lane_fidelity", True)),
            "off_lane_shortcut_gap_m": float(descriptors.get("off_lane_shortcut_gap_m", 0.0)),
            "leader_reverse_turn_required": float(descriptors.get("leader_reverse_turn_required", 0.0)),
        },
        "scenario": scenario,
    }


def load_scene(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).expanduser().read_text(encoding="utf-8"))


def load_manifest(root: str | Path | None = None, *, split: str | None = None) -> list[DockBenchSceneSpec]:
    ds_root = dataset_root(root)
    if split is None:
        manifest_path = ds_root / "dockbench_v1_manifest.json"
    else:
        manifest_path = ds_root / f"dockbench_v1_split_{str(split)}.json"
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    return [DockBenchSceneSpec(**item) for item in payload]


def load_representatives(root: str | Path | None = None) -> dict[str, DockBenchSceneSpec]:
    ds_root = dataset_root(root)
    payload = json.loads((ds_root / "dockbench_v1_representatives.json").read_text(encoding="utf-8"))
    return {key: DockBenchSceneSpec(**value) for key, value in payload.items()}


def save_manifest_files(root: str | Path, *, records: Sequence[DockBenchSceneSpec], representatives: dict[str, DockBenchSceneSpec], quality_report: dict[str, Any]) -> None:
    ds_root = dataset_root(root)
    ds_root.mkdir(parents=True, exist_ok=True)
    manifest = [record.to_dict() for record in records]
    (ds_root / "dockbench_v1_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    for split in SPLIT_COUNTS:
        subset = [record.to_dict() for record in records if record.split == split]
        (ds_root / f"dockbench_v1_split_{split}.json").write_text(json.dumps(subset, ensure_ascii=False, indent=2), encoding="utf-8")
    (ds_root / "dockbench_v1_representatives.json").write_text(
        json.dumps({key: value.to_dict() for key, value in representatives.items()}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    (ds_root / "dockbench_v1_quality_report.json").write_text(json.dumps(quality_report, ensure_ascii=False, indent=2), encoding="utf-8")
