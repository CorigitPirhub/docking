from __future__ import annotations

import math
import heapq
from collections import deque
from dataclasses import asdict, dataclass
from typing import Any

import numpy as np

from .collision import CollisionEngine, obstacle_polygon, polygon_bbox
from .config import Config
from .sensors import line_blocked_by_obstacles
from .train import TrainKinematics
from .types import Obstacle, VehicleState


TYPE_TO_SUBTYPES = {
    "A": ("A1", "A2", "A3"),
    "B": ("B1", "B2", "B3"),
    "C": ("C1", "C2", "C3"),
}


@dataclass
class ScenarioInstance:
    scenario_id: str
    expected_type: str
    subtype: str
    start_xy: np.ndarray
    goal_xy: np.ndarray
    path_xy: np.ndarray
    obstacles: list[Obstacle]
    vehicles_init: list[VehicleState]
    params: dict[str, Any]
    labels: "ScenarioLabels | None" = None


@dataclass
class ScenarioLabels:
    scenario_id: str
    expected_type: str
    predicted_type: str
    subtype: str
    path_length: float
    direct_length: float
    detour_factor: float
    bottleneck_count: int
    n_max_pass_global: int
    n_max_pass_profile: list[dict[str, float | int]]
    open_area_ratio: float
    dock_friendly_zones: list[dict[str, float]]
    split_mandatory_zones: list[dict[str, float]]
    occlusion_level: str
    reconfig_window: float
    decision_horizon: str
    coordination_intensity: str
    initial_dispersion_mode: str
    initial_state_seed: int
    path_dependent: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ValidationResult:
    scenario_id: str
    subtype: str
    expected_type: str
    predicted_type: str
    passed: bool
    checks: dict[str, bool]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _polyline_s(path_xy: np.ndarray) -> np.ndarray:
    if len(path_xy) <= 1:
        return np.zeros(len(path_xy), dtype=float)
    ds = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(ds)])


def _polyline_curvature(path_xy: np.ndarray) -> np.ndarray:
    n = len(path_xy)
    kappa = np.zeros(n, dtype=float)
    if n < 3:
        return kappa
    for i in range(1, n - 1):
        p0 = path_xy[i - 1]
        p1 = path_xy[i]
        p2 = path_xy[i + 1]
        a = float(np.linalg.norm(p1 - p0))
        b = float(np.linalg.norm(p2 - p1))
        c = float(np.linalg.norm(p2 - p0))
        if min(a, b, c) < 1e-9:
            continue
        area2 = float((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]))
        if abs(area2) < 1e-9:
            continue
        radius = a * b * c / max(2.0 * abs(area2), 1e-9)
        kappa[i] = (1.0 / max(radius, 1e-9)) * (1.0 if area2 > 0.0 else -1.0)
    if n >= 2:
        kappa[0] = kappa[1]
        kappa[-1] = kappa[-2]
    return kappa


def _nearest_path_idx(path_xy: np.ndarray, p: np.ndarray) -> int:
    d = np.linalg.norm(path_xy - p, axis=1)
    return int(np.argmin(d))


def _segmentize_mask(s: np.ndarray, mask: np.ndarray, path_xy: np.ndarray) -> list[dict[str, float]]:
    out: list[dict[str, float]] = []
    n = len(mask)
    i = 0
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and mask[j + 1]:
            j += 1
        out.append(
            {
                "s0": float(s[i]),
                "s1": float(s[j]),
                "x0": float(path_xy[i, 0]),
                "y0": float(path_xy[i, 1]),
                "x1": float(path_xy[j, 0]),
                "y1": float(path_xy[j, 1]),
            }
        )
        i = j + 1
    return out


def _compress_profile(s: np.ndarray, nmax: np.ndarray) -> list[dict[str, float | int]]:
    out: list[dict[str, float | int]] = []
    if len(nmax) == 0:
        return out
    i = 0
    while i < len(nmax):
        j = i
        while j + 1 < len(nmax) and int(nmax[j + 1]) == int(nmax[i]):
            j += 1
        out.append({"s0": float(s[i]), "s1": float(s[j]), "n_max": int(nmax[i])})
        i = j + 1
    return out


def _ray_segment_distance(origin: np.ndarray, direction: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    v = b - a
    denom = direction[0] * v[1] - direction[1] * v[0]
    if abs(denom) < 1e-9:
        return math.inf
    diff = a - origin
    t = (diff[0] * v[1] - diff[1] * v[0]) / denom
    u = (diff[0] * direction[1] - diff[1] * direction[0]) / denom
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return float(t)
    return math.inf


def _ray_hit_boundary(origin: np.ndarray, direction: np.ndarray, width: float, height: float) -> float:
    half_w = 0.5 * width
    half_h = 0.5 * height
    out = math.inf
    if abs(direction[0]) > 1e-9:
        for x_lim in (-half_w, half_w):
            t = (x_lim - origin[0]) / direction[0]
            if t <= 0.0:
                continue
            y = origin[1] + t * direction[1]
            if -half_h <= y <= half_h:
                out = min(out, float(t))
    if abs(direction[1]) > 1e-9:
        for y_lim in (-half_h, half_h):
            t = (y_lim - origin[1]) / direction[1]
            if t <= 0.0:
                continue
            x = origin[0] + t * direction[0]
            if -half_w <= x <= half_w:
                out = min(out, float(t))
    return out


class ScenarioLabelCalculator:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.train_kin = TrainKinematics(cfg.vehicle)
        self._lookup_cache: dict[int, tuple[np.ndarray, np.ndarray]] = {}

    def _width_req(self, n: int, kappa_abs: float) -> float:
        scfg = self.cfg.scenario
        k_ref = 1.0 / max(self.cfg.vehicle.min_turn_radius_single, 1e-6)
        k_scale = min(2.0, kappa_abs / max(k_ref, 1e-6))
        return (
            self.cfg.vehicle.car_width
            + scfg.width_req_base_margin
            + scfg.width_req_per_trailer * max(0, n - 1)
            + scfg.sweep_gain_curvature * k_scale * max(0, n - 1) * self.cfg.vehicle.car_width
        )

    def _build_lookup(self, n_total: int) -> tuple[np.ndarray, np.ndarray]:
        if n_total in self._lookup_cache:
            return self._lookup_cache[n_total]
        scfg = self.cfg.scenario
        kappas = np.linspace(0.0, scfg.kappa_lookup_max, scfg.kappa_lookup_bins)
        width_tbl = np.zeros((n_total, len(kappas)), dtype=float)
        for ni in range(1, n_total + 1):
            for j, k in enumerate(kappas):
                width_tbl[ni - 1, j] = self._width_req(ni, float(k))
        self._lookup_cache[n_total] = (kappas, width_tbl)
        return kappas, width_tbl

    def _width_env_samples(self, path_xy: np.ndarray, obstacles: list[Obstacle]) -> np.ndarray:
        s_cfg = self.cfg.scenario
        curv = _polyline_curvature(path_xy)
        widths = np.zeros(len(path_xy), dtype=float)
        for i, p in enumerate(path_xy):
            if i < len(path_xy) - 1:
                d = path_xy[i + 1] - p
            else:
                d = p - path_xy[i - 1]
            nrm = np.linalg.norm(d)
            if nrm < 1e-9:
                d = np.array([1.0, 0.0], dtype=float)
            else:
                d = d / nrm
            normal = np.array([-d[1], d[0]], dtype=float)

            d_plus = _ray_hit_boundary(p, normal, self.cfg.environment.width, self.cfg.environment.height)
            d_minus = _ray_hit_boundary(p, -normal, self.cfg.environment.width, self.cfg.environment.height)

            for obs in obstacles:
                poly = obstacle_polygon(obs)
                for k in range(len(poly)):
                    a = poly[k]
                    b = poly[(k + 1) % len(poly)]
                    d_plus = min(d_plus, _ray_segment_distance(p, normal, a, b))
                    d_minus = min(d_minus, _ray_segment_distance(p, -normal, a, b))
            widths[i] = max(0.0, d_plus) + max(0.0, d_minus)
            if abs(curv[i]) > s_cfg.kappa_lookup_max:
                widths[i] *= 0.98
        return widths

    def _obstacle_density_samples(self, path_xy: np.ndarray, obstacles: list[Obstacle]) -> np.ndarray:
        s_cfg = self.cfg.scenario
        out = np.zeros(len(path_xy), dtype=float)
        for i, p in enumerate(path_xy):
            xmin = p[0] - 0.5 * s_cfg.density_window_length
            xmax = p[0] + 0.5 * s_cfg.density_window_length
            ymin = p[1] - 0.5 * s_cfg.density_window_width
            ymax = p[1] + 0.5 * s_cfg.density_window_width
            area_win = max(1e-6, (xmax - xmin) * (ymax - ymin))
            area_obs = 0.0
            for obs in obstacles:
                bx0, by0, bx1, by1 = polygon_bbox(obstacle_polygon(obs))
                ox0 = max(xmin, bx0)
                ox1 = min(xmax, bx1)
                oy0 = max(ymin, by0)
                oy1 = min(ymax, by1)
                if ox1 > ox0 and oy1 > oy0:
                    area_obs += (ox1 - ox0) * (oy1 - oy0)
            out[i] = min(1.0, area_obs / area_win)
        return out

    def _los_samples(self, path_xy: np.ndarray, obstacles: list[Obstacle]) -> np.ndarray:
        out = np.zeros(len(path_xy), dtype=float)
        d_check = self.cfg.scenario.los_check_distance
        for i, p in enumerate(path_xy):
            if i < len(path_xy) - 1:
                d = path_xy[i + 1] - p
            else:
                d = p - path_xy[i - 1]
            nrm = np.linalg.norm(d)
            if nrm < 1e-9:
                d = np.array([1.0, 0.0], dtype=float)
            else:
                d = d / nrm
            p1 = p + d * d_check
            blocked = line_blocked_by_obstacles(p, p1, obstacles)
            out[i] = 0.0 if blocked else 1.0
        return out

    def _nmax_samples(self, path_xy: np.ndarray, widths: np.ndarray, n_total: int) -> np.ndarray:
        kappas = np.abs(_polyline_curvature(path_xy))
        if len(kappas) >= 5:
            kern = np.array([0.1, 0.2, 0.4, 0.2, 0.1], dtype=float)
            kappas = np.convolve(kappas, kern, mode="same")
        k_grid, width_tbl = self._build_lookup(n_total)
        nmax = np.zeros(len(path_xy), dtype=int)
        for i, kappa in enumerate(kappas):
            kappa = float(kappa)
            k_idx = int(np.argmin(np.abs(k_grid - kappa)))
            n_k = 0
            for n in range(1, n_total + 1):
                rmin_n = self.train_kin.min_turn_radius_train(n)
                if kappa <= (1.0 / max(rmin_n, 1e-9)) + 1e-9:
                    n_k = n
            n_w = 0
            for n in range(1, n_total + 1):
                req = width_tbl[n - 1, k_idx] + 2.0 * self.cfg.safety.min_clearance
                if widths[i] >= req:
                    n_w = n
            nmax[i] = int(min(n_w, n_k))
        if len(nmax) >= 7:
            edge = min(3, len(nmax) // 6)
            nmax[:edge] = nmax[edge]
            nmax[-edge:] = nmax[-edge - 1]
        return nmax

    def _occlusion_level(self, los: np.ndarray) -> str:
        clear_ratio = float(np.mean(los)) if len(los) > 0 else 1.0
        if clear_ratio < 0.7:
            return "high"
        if clear_ratio < 0.9:
            return "medium"
        return "low"

    def _decision_horizon(self, split_zones: list[dict[str, float]], path_len: float, reconfig_window: float) -> str:
        if not split_zones:
            return "low"
        first_split = split_zones[0]["s0"]
        score = 0
        if first_split > 0.25 * path_len:
            score += 1
        if first_split > 0.45 * path_len:
            score += 1
        if reconfig_window < 0.18 * path_len:
            score += 1
        if len(split_zones) > 1:
            score += 1
        if score >= 3:
            return "high"
        if score >= 2:
            return "medium"
        return "low"

    def _coordination_intensity(
        self,
        split_zones: list[dict[str, float]],
        dock_zones: list[dict[str, float]],
        transitions: int,
        initial_mode: str,
    ) -> str:
        score = len(split_zones) + max(0, len(dock_zones) - 1)
        if transitions >= 2:
            score += 1
        if initial_mode != "Clustered_At_A":
            score += 1
        if score >= 4:
            return "high"
        if score >= 2:
            return "medium"
        return "low"

    def classify_type(
        self,
        n_max_profile: list[dict[str, float | int]],
        n_total: int,
        bottleneck_count: int,
        *,
        occlusion_level: str,
        kappa_p90: float,
        reconfig_window: float,
        path_len: float,
    ) -> str:
        if not n_max_profile:
            return "C"
        values = [int(seg["n_max"]) for seg in n_max_profile]
        transitions = sum(1 for i in range(1, len(values)) if values[i] != values[i - 1])
        nmax_global = min(values)
        if nmax_global >= n_total and bottleneck_count == 0:
            return "A"
        if (
            bottleneck_count >= 1
            and transitions <= 5
            and occlusion_level != "high"
            and kappa_p90 <= self.cfg.scenario.b_type_kappa_max
            and bottleneck_count <= 3
        ):
            if reconfig_window > self.cfg.scenario.c_reconfig_window_ratio * max(path_len, 1e-6):
                return "C"
            return "B"
        return "C"

    def compute(self, instance: ScenarioInstance) -> ScenarioLabels:
        path = instance.path_xy
        s = _polyline_s(path)
        path_len = float(s[-1]) if len(s) else 0.0
        direct = float(np.linalg.norm(instance.goal_xy - instance.start_xy))
        detour = path_len / max(direct, 1e-6)

        widths = self._width_env_samples(path, instance.obstacles)
        dens = self._obstacle_density_samples(path, instance.obstacles)
        los = self._los_samples(path, instance.obstacles)
        kappas = np.abs(_polyline_curvature(path))
        n_total = len(instance.vehicles_init)
        nmax = self._nmax_samples(path, widths, n_total)

        n_profile = _compress_profile(s, nmax)
        nmax_global = int(np.min(nmax)) if len(nmax) else 0

        split_mask = nmax < n_total
        split_zones = _segmentize_mask(s, split_mask, path)
        bottleneck_count = len(split_zones)

        req_two = self._width_req(2, 0.0) + 2.0 * self.cfg.safety.min_clearance
        dock_mask = (
            (widths >= max(self.cfg.scenario.dock_clearance_min, req_two))
            & (dens <= self.cfg.scenario.dock_obs_density_max)
            & (los >= self.cfg.scenario.dock_los_min)
            & (kappas <= self.cfg.scenario.dock_kappa_max)
            & (nmax >= 2)
        )
        dock_zones = _segmentize_mask(s, dock_mask, path)

        reconfig_window = 0.0
        if split_zones:
            windows: list[float] = []
            for z in split_zones:
                z_end = z["s1"]
                next_dock = None
                for d in dock_zones:
                    if d["s0"] > z_end:
                        next_dock = d["s0"]
                        break
                if next_dock is None:
                    windows.append(path_len - z_end)
                else:
                    windows.append(next_dock - z_end)
            reconfig_window = float(min(windows)) if windows else 0.0

        transitions = sum(1 for i in range(1, len(nmax)) if int(nmax[i]) != int(nmax[i - 1]))
        occlusion = self._occlusion_level(los)
        decision_horizon = self._decision_horizon(split_zones, path_len, reconfig_window)
        coordination = self._coordination_intensity(
            split_zones,
            dock_zones,
            transitions=transitions,
            initial_mode=str(instance.params["initial_dispersion_mode"]),
        )
        kappa_p90 = float(np.percentile(kappas, 90)) if len(kappas) else 0.0
        pred_type = self.classify_type(
            n_profile,
            n_total,
            bottleneck_count,
            occlusion_level=occlusion,
            kappa_p90=kappa_p90,
            reconfig_window=reconfig_window,
            path_len=path_len,
        )
        area_total = self.cfg.environment.width * self.cfg.environment.height
        area_obs = sum(max(0.0, o.width * o.height) for o in instance.obstacles)
        open_ratio = max(0.0, min(1.0, (area_total - area_obs) / max(area_total, 1e-9)))

        return ScenarioLabels(
            scenario_id=instance.scenario_id,
            expected_type=instance.expected_type,
            predicted_type=pred_type,
            subtype=instance.subtype,
            path_length=path_len,
            direct_length=direct,
            detour_factor=detour,
            bottleneck_count=bottleneck_count,
            n_max_pass_global=nmax_global,
            n_max_pass_profile=n_profile,
            open_area_ratio=open_ratio,
            dock_friendly_zones=dock_zones,
            split_mandatory_zones=split_zones,
            occlusion_level=occlusion,
            reconfig_window=reconfig_window,
            decision_horizon=decision_horizon,
            coordination_intensity=coordination,
            initial_dispersion_mode=str(instance.params["initial_dispersion_mode"]),
            initial_state_seed=int(instance.params["initial_state_seed"]),
            path_dependent=False,
        )


class ScenarioGenerator:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._label_calc = ScenarioLabelCalculator(cfg)
        self._collision = CollisionEngine(cfg.vehicle, cfg.safety)

    def _smooth_path(self, path: np.ndarray, win: int = 9, passes: int = 2) -> np.ndarray:
        if len(path) < win + 2 or win < 3:
            return path
        out = path.copy()
        x_ref = out[:, 0].copy()
        kernel = np.ones(win, dtype=float) / float(win)
        for _ in range(passes):
            y = np.convolve(out[:, 1], kernel, mode="same")
            out[:, 0] = x_ref
            out[:, 1] = y
            out[0] = path[0]
            out[-1] = path[-1]
        return out

    def _polyline_from_waypoints(self, waypoints: list[tuple[float, float]], n: int) -> np.ndarray:
        wp = np.asarray(waypoints, dtype=float)
        if len(wp) < 2:
            raise ValueError("waypoints must include at least 2 points")
        ds = np.linalg.norm(np.diff(wp, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(ds)])
        total = float(s[-1])
        if total < 1e-9:
            return np.repeat(wp[:1], n, axis=0)
        qs = np.linspace(0.0, total, n)
        out = np.zeros((n, 2), dtype=float)
        j = 0
        for i, q in enumerate(qs):
            while j + 1 < len(s) and q > s[j + 1]:
                j += 1
            if j + 1 >= len(wp):
                out[i] = wp[-1]
                continue
            seg = max(1e-9, s[j + 1] - s[j])
            t = (q - s[j]) / seg
            out[i] = (1.0 - t) * wp[j] + t * wp[j + 1]
        return self._smooth_path(out, win=15, passes=3)

    def _path(self, subtype: str, params: dict[str, Any]) -> np.ndarray:
        n = int(params["path_samples"])
        x0, x1 = float(params["start_x"]), float(params["goal_x"])
        xs = np.linspace(x0, x1, n)
        if subtype == "A1":
            ys = np.zeros_like(xs)
            return np.stack([xs, ys], axis=1)
        if subtype == "A2":
            ys = 0.8 * np.sin((xs - x0) / 5.5)
            return np.stack([xs, ys], axis=1)
        if subtype == "A3":
            amp = float(params.get("A3_amp", 1.5))
            ys = amp * np.sin((xs - x0) / float(params.get("A3_period", 4.8)))
            return np.stack([xs, ys], axis=1)

        yoff = float(self.cfg.scenario.gate_turn_y_offset)
        # B/C types include turning bottlenecks by design.
        if subtype in {"B1", "B2"}:
            yb = yoff
            if subtype == "B2":
                k_target = int(params.get("B2_K", 2))
                if k_target >= 4:
                    yb = 0.65 * yoff
                elif k_target >= 3:
                    yb = 0.8 * yoff
            return self._polyline_from_waypoints(
                [(-16.0, 0.0), (-8.0, 0.0), (-2.5, -yb), (1.8, yb), (8.0, yb), (16.0, 0.0)],
                n=n,
            )
        if subtype == "B3":
            return self._polyline_from_waypoints(
                [
                    (-16.0, 0.0),
                    (-10.0, -0.9 * yoff),
                    (-5.0, 0.9 * yoff),
                    (-1.0, 0.9 * yoff),
                    (3.5, yoff),
                    (8.5, -yoff),
                    (13.0, -yoff),
                    (16.0, 0.0),
                ],
                n=n,
            )
        if subtype == "C1":
            return self._polyline_from_waypoints(
                [
                    (-16.0, 0.0),
                    (-12.0, 0.0),
                    (-10.0, -0.95 * yoff),
                    (-6.5, 0.95 * yoff),
                    (-1.0, 0.8 * yoff),
                    (4.8, -0.8 * yoff),
                    (10.0, -0.6 * yoff),
                    (16.0, 0.0),
                ],
                n=n,
            )
        if subtype == "C2":
            return self._polyline_from_waypoints(
                [
                    (-16.0, 0.0),
                    (-10.0, -0.9 * yoff),
                    (-5.0, 0.9 * yoff),
                    (-1.0, yoff),
                    (3.0, -yoff),
                    (7.5, 0.8 * yoff),
                    (12.0, -0.75 * yoff),
                    (16.0, 0.0),
                ],
                n=n,
            )
        # C3
        return self._polyline_from_waypoints(
            [
                (-16.0, 0.0),
                (-12.0, 0.0),
                (-10.0, -0.85 * yoff),
                (-6.0, 0.85 * yoff),
                (-1.0, 0.75 * yoff),
                (3.5, -0.75 * yoff),
                (9.0, -0.6 * yoff),
                (16.0, 0.0),
            ],
            n=n,
        )

    def _add_sparse_obstacles(
        self,
        out: list[Obstacle],
        rng: np.random.Generator,
        count: int,
        path: np.ndarray,
        min_path_dist: float,
    ) -> None:
        for _ in range(count * 8):
            if len(out) >= count:
                break
            x = float(rng.uniform(-16.0, 16.0))
            y = float(rng.uniform(-8.0, 8.0))
            p = np.array([x, y], dtype=float)
            d = float(np.min(np.linalg.norm(path - p, axis=1)))
            if d < min_path_dist:
                continue
            w = float(rng.uniform(0.9, 1.8))
            h = float(rng.uniform(0.9, 1.8))
            out.append(Obstacle(x=x, y=y, width=w, height=h, yaw=0.0))

    def _add_cross_wall_with_gap(
        self,
        out: list[Obstacle],
        x_center: float,
        gap: float,
        gap_center_y: float,
        thickness: float,
    ) -> dict[str, float]:
        half_h = 0.5 * self.cfg.environment.height
        y_top_start = gap_center_y + 0.5 * gap
        y_bot_end = gap_center_y - 0.5 * gap
        top_h = max(0.0, half_h - y_top_start)
        bot_h = max(0.0, y_bot_end + half_h)
        if top_h > 1e-6:
            out.append(
                Obstacle(
                    x=float(x_center),
                    y=float(0.5 * (y_top_start + half_h)),
                    width=float(thickness),
                    height=float(top_h),
                    yaw=0.0,
                )
            )
        if bot_h > 1e-6:
            out.append(
                Obstacle(
                    x=float(x_center),
                    y=float(0.5 * (-half_h + y_bot_end)),
                    width=float(thickness),
                    height=float(bot_h),
                    yaw=0.0,
                )
            )
        return {
            "x0": float(x_center - 0.5 * thickness),
            "x1": float(x_center + 0.5 * thickness),
            "y0": float(y_bot_end),
            "y1": float(y_top_start),
            "cx": float(x_center),
            "cy": float(gap_center_y),
            "gap": float(gap),
        }

    def _add_chicane(
        self,
        out: list[Obstacle],
        *,
        x0: float,
        x1: float,
        gap: float,
        yoff: float,
        thickness: float,
    ) -> list[dict[str, float]]:
        gates: list[dict[str, float]] = []
        gates.append(self._add_cross_wall_with_gap(out, x_center=x0, gap=gap, gap_center_y=-yoff, thickness=thickness))
        gates.append(self._add_cross_wall_with_gap(out, x_center=x1, gap=gap, gap_center_y=yoff, thickness=thickness))
        return gates

    def _build_obstacles(
        self,
        subtype: str,
        params: dict[str, Any],
        path: np.ndarray,
        n_total: int,
    ) -> tuple[list[Obstacle], list[dict[str, float]]]:
        rng = np.random.default_rng(int(params["seed"]) + 19)
        obs: list[Obstacle] = []
        mandatory_gates: list[dict[str, float]] = []
        thickness = float(self.cfg.scenario.gate_wall_thickness)
        yoff = float(self.cfg.scenario.gate_turn_y_offset)
        if subtype == "A1":
            self._add_sparse_obstacles(obs, rng, count=2, path=path, min_path_dist=4.0)
        elif subtype == "A2":
            self._add_sparse_obstacles(obs, rng, count=5, path=path, min_path_dist=2.6)
        elif subtype == "A3":
            self._add_sparse_obstacles(obs, rng, count=4, path=path, min_path_dist=3.0)
        elif subtype == "B1":
            mandatory_gates += self._add_chicane(
                obs,
                x0=-2.5,
                x1=1.8,
                gap=float(params.get("B_gap", 0.74)),
                yoff=yoff,
                thickness=thickness,
            )
        elif subtype == "B2":
            k = int(params.get("B2_K", 2))
            req_k = self._label_calc._width_req(k, 0.0) + 2.0 * self.cfg.safety.min_clearance
            if k < n_total:
                req_next = self._label_calc._width_req(k + 1, 0.0) + 2.0 * self.cfg.safety.min_clearance
                gap = min(req_k + 0.03, req_next - 0.03)
            else:
                gap = req_k + 0.05
            yoff_b2 = yoff
            if k >= 4:
                yoff_b2 = 0.65 * yoff
            elif k >= 3:
                yoff_b2 = 0.8 * yoff
            mandatory_gates += self._add_chicane(
                obs,
                x0=-2.5,
                x1=1.8,
                gap=float(gap),
                yoff=yoff_b2,
                thickness=thickness,
            )
        elif subtype == "B3":
            mandatory_gates += self._add_chicane(obs, x0=-10.0, x1=-5.0, gap=0.95, yoff=0.9 * yoff, thickness=thickness)
            mandatory_gates += self._add_chicane(obs, x0=3.5, x1=8.5, gap=0.74, yoff=yoff, thickness=thickness)
        elif subtype == "C1":
            mandatory_gates += self._add_chicane(obs, x0=-10.0, x1=-6.5, gap=0.74, yoff=0.95 * yoff, thickness=thickness)
            mandatory_gates += self._add_chicane(obs, x0=-1.0, x1=4.8, gap=0.95, yoff=0.8 * yoff, thickness=thickness)
            self._add_sparse_obstacles(obs, rng, count=3, path=path, min_path_dist=2.3)
        elif subtype == "C2":
            mandatory_gates += self._add_chicane(obs, x0=-10.0, x1=-5.0, gap=1.08, yoff=0.9 * yoff, thickness=thickness)
            mandatory_gates += self._add_chicane(obs, x0=-1.0, x1=3.0, gap=0.74, yoff=yoff, thickness=thickness)
            mandatory_gates += self._add_chicane(obs, x0=7.5, x1=12.0, gap=0.95, yoff=0.8 * yoff, thickness=thickness)
            self._add_sparse_obstacles(obs, rng, count=4, path=path, min_path_dist=1.8)
        else:  # C3
            mandatory_gates += self._add_chicane(obs, x0=-10.0, x1=-6.0, gap=0.90, yoff=0.85 * yoff, thickness=thickness)
            mandatory_gates += self._add_chicane(obs, x0=-1.0, x1=3.5, gap=1.05, yoff=0.75 * yoff, thickness=thickness)
            obs.append(Obstacle(x=2.5, y=1.8, width=1.2, height=1.0, yaw=0.0))
            obs.append(Obstacle(x=8.0, y=-1.8, width=1.4, height=1.1, yaw=0.0))
            self._add_sparse_obstacles(obs, rng, count=2, path=path, min_path_dist=2.2)

        if subtype in ("B1", "B2", "B3", "C1", "C2", "C3"):
            # keep side clutter low for deterministic corridor interpretation
            side_rng = np.random.default_rng(int(params["seed"]) + 29)
            self._add_sparse_obstacles(obs, side_rng, count=2, path=path, min_path_dist=3.4)

        return obs, mandatory_gates

    def _spawn_vehicles(
        self,
        path: np.ndarray,
        obstacles: list[Obstacle],
        n_total: int,
        seed: int,
        mode: str,
        pose_jitter: float,
        spawn_clearance: float,
    ) -> list[VehicleState]:
        rng = np.random.default_rng(seed)
        vehicles: list[VehicleState] = []
        start = path[0]
        head_vec = path[1] - path[0]
        yaw0 = math.atan2(float(head_vec[1]), float(head_vec[0]))

        def collision_free(candidate: VehicleState) -> bool:
            for obs in obstacles:
                if self._collision.collide_vehicle_obstacle(candidate, obs, include_clearance=False):
                    if self._collision.min_clearance_vehicle_obstacles(candidate, [obs]) < spawn_clearance:
                        return False
            for other in vehicles:
                if self._collision.collide_vehicle_vehicle(candidate, other, include_clearance=False):
                    return False
            return True

        def make_state(vid: int, x: float, y: float) -> VehicleState:
            idx = _nearest_path_idx(path, np.array([x, y], dtype=float))
            if idx < len(path) - 1:
                d = path[idx + 1] - path[idx]
            else:
                d = path[idx] - path[max(0, idx - 1)]
            yaw = math.atan2(float(d[1]), float(d[0])) + float(rng.uniform(-pose_jitter, pose_jitter))
            return VehicleState(vehicle_id=vid, x=x, y=y, yaw=yaw, v=0.0, delta=0.0)

        half_w = 0.5 * self.cfg.environment.width - 1.0
        half_h = 0.5 * self.cfg.environment.height - 1.0

        if mode == "Clustered_At_A":
            placed = 0
            for _ in range(3000):
                if placed >= n_total:
                    break
                ang = float(rng.uniform(-math.pi, math.pi))
                rad = float(rng.uniform(0.1, self.cfg.scenario.cluster_radius))
                x = float(start[0] + rad * math.cos(ang))
                y = float(start[1] + rad * math.sin(ang))
                if not (-half_w <= x <= half_w and -half_h <= y <= half_h):
                    continue
                c = make_state(placed + 1, x, y)
                if collision_free(c):
                    vehicles.append(c)
                    placed += 1
        elif mode == "Uniform_Spread":
            spacing = self.cfg.scenario.uniform_spacing
            xs = np.linspace(start[0] - 4.0, start[0] + 4.0, max(2, int(math.ceil(n_total / 2))))
            ys = np.linspace(start[1] - 2.5, start[1] + 2.5, 3)
            placed = 0
            for x in xs:
                for y in ys:
                    if placed >= n_total:
                        break
                    c = make_state(placed + 1, float(x), float(y))
                    if collision_free(c):
                        vehicles.append(c)
                        placed += 1
                if placed >= n_total:
                    break
            # fallback fill
            for _ in range(2000):
                if len(vehicles) >= n_total:
                    break
                x = float(rng.uniform(start[0] - spacing, start[0] + spacing))
                y = float(rng.uniform(start[1] - spacing * 0.7, start[1] + spacing * 0.7))
                c = make_state(len(vehicles) + 1, x, y)
                if collision_free(c):
                    vehicles.append(c)
        else:  # Random_Scattered
            for _ in range(6000):
                if len(vehicles) >= n_total:
                    break
                x = float(rng.uniform(-half_w, half_w))
                y = float(rng.uniform(-half_h, half_h))
                c = make_state(len(vehicles) + 1, x, y)
                if collision_free(c):
                    vehicles.append(c)

        if len(vehicles) < n_total:
            # deterministic fallback near path start.
            spacing = 1.2
            for i in range(len(vehicles), n_total):
                x = float(start[0] - spacing * i)
                y = float(start[1] + 0.2 * ((i % 2) * 2 - 1))
                vehicles.append(VehicleState(vehicle_id=i + 1, x=x, y=y, yaw=yaw0, v=0.0, delta=0.0))

        return vehicles[:n_total]

    def generate(
        self,
        subtype: str,
        *,
        n_vehicles: int | None = None,
        seed: int | None = None,
        initial_dispersion_mode: str = "Clustered_At_A",
        overrides: dict[str, Any] | None = None,
    ) -> ScenarioInstance:
        if subtype not in {"A1", "A2", "A3", "B1", "B2", "B3", "C1", "C2", "C3"}:
            raise ValueError(f"Unsupported subtype: {subtype}")
        if initial_dispersion_mode not in {"Clustered_At_A", "Random_Scattered", "Uniform_Spread"}:
            raise ValueError(f"Unsupported initial_dispersion_mode: {initial_dispersion_mode}")

        n_total = self.cfg.scenario.representative_vehicle_count if n_vehicles is None else int(n_vehicles)
        seed = self.cfg.testing.random_seed if seed is None else int(seed)
        expected_type = subtype[0]
        params: dict[str, Any] = {
            "seed": seed,
            "path_samples": self.cfg.scenario.path_samples,
            "start_x": -16.0,
            "goal_x": 16.0,
            "initial_dispersion_mode": initial_dispersion_mode,
            "initial_state_seed": seed + 101,
        }
        if overrides:
            params.update(overrides)

        path = self._path(subtype, params)
        start_xy = path[0].copy()
        goal_xy = path[-1].copy()
        obstacles, mandatory_gates = self._build_obstacles(subtype, params, path, n_total=n_total)
        params["mandatory_gates"] = mandatory_gates
        vehicles = self._spawn_vehicles(
            path,
            obstacles,
            n_total=n_total,
            seed=int(params["initial_state_seed"]),
            mode=initial_dispersion_mode,
            pose_jitter=self.cfg.scenario.initial_pose_jitter,
            spawn_clearance=self.cfg.scenario.spawn_clearance_min,
        )

        scenario_id = f"{subtype}_seed{seed}_n{n_total}_{initial_dispersion_mode}"
        out = ScenarioInstance(
            scenario_id=scenario_id,
            expected_type=expected_type,
            subtype=subtype,
            start_xy=start_xy,
            goal_xy=goal_xy,
            path_xy=path,
            obstacles=obstacles,
            vehicles_init=vehicles,
            params=params,
        )
        out.labels = self._label_calc.compute(out)
        return out


class ScenarioValidator:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.generator = ScenarioGenerator(cfg)
        self.collision = CollisionEngine(cfg.vehicle, cfg.safety)

    def _world_to_grid(self, x: float, y: float, res: float, nx: int, ny: int) -> tuple[int, int]:
        half_w = 0.5 * self.cfg.environment.width
        half_h = 0.5 * self.cfg.environment.height
        ix = int(round((x + half_w) / res))
        iy = int(round((y + half_h) / res))
        ix = min(max(ix, 0), nx - 1)
        iy = min(max(iy, 0), ny - 1)
        return ix, iy

    def _nearest_free(self, occ: np.ndarray, start: tuple[int, int], max_r: int = 40) -> tuple[int, int] | None:
        if not occ[start[1], start[0]]:
            return start
        q: deque[tuple[int, int]] = deque([start])
        seen = np.zeros_like(occ, dtype=bool)
        seen[start[1], start[0]] = True
        w, h = occ.shape[1], occ.shape[0]
        while q:
            x, y = q.popleft()
            if abs(x - start[0]) + abs(y - start[1]) > max_r:
                continue
            if not occ[y, x]:
                return (x, y)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and not seen[ny, nx]:
                    seen[ny, nx] = True
                    q.append((nx, ny))
        return None

    def _build_occupancy(
        self,
        instance: ScenarioInstance,
        forbidden_regions: list[dict[str, float]] | None = None,
    ) -> tuple[np.ndarray, float]:
        res = float(self.cfg.scenario.validation_grid_resolution)
        width = float(self.cfg.environment.width)
        height = float(self.cfg.environment.height)
        half_w = 0.5 * width
        half_h = 0.5 * height
        nx = int(math.ceil(width / res)) + 1
        ny = int(math.ceil(height / res)) + 1
        occ = np.zeros((ny, nx), dtype=bool)

        # Topology-only reachability check for "must-pass" validation, use geometric free-space connectivity.
        inflate = 0.0
        bnd = 1
        occ[:bnd, :] = True
        occ[-bnd:, :] = True
        occ[:, :bnd] = True
        occ[:, -bnd:] = True

        def block_rect(x0: float, y0: float, x1: float, y1: float) -> None:
            ix0 = int(math.floor((x0 + half_w) / res))
            ix1 = int(math.ceil((x1 + half_w) / res))
            iy0 = int(math.floor((y0 + half_h) / res))
            iy1 = int(math.ceil((y1 + half_h) / res))
            ix0 = min(max(ix0, 0), nx - 1)
            ix1 = min(max(ix1, 0), nx - 1)
            iy0 = min(max(iy0, 0), ny - 1)
            iy1 = min(max(iy1, 0), ny - 1)
            if ix1 >= ix0 and iy1 >= iy0:
                occ[iy0 : iy1 + 1, ix0 : ix1 + 1] = True

        for obs in instance.obstacles:
            ox0 = obs.x - 0.5 * obs.width - inflate
            ox1 = obs.x + 0.5 * obs.width + inflate
            oy0 = obs.y - 0.5 * obs.height - inflate
            oy1 = obs.y + 0.5 * obs.height + inflate
            block_rect(ox0, oy0, ox1, oy1)

        if forbidden_regions:
            margin = float(self.cfg.scenario.validation_gate_block_margin)
            for z in forbidden_regions:
                x0 = float(z["x0"]) - margin - inflate
                x1 = float(z["x1"]) + margin + inflate
                y0 = float(z["y0"]) - margin - inflate
                y1 = float(z["y1"]) + margin + inflate
                block_rect(x0, y0, x1, y1)

        return occ, res

    def _path_exists_grid(
        self,
        instance: ScenarioInstance,
        forbidden_regions: list[dict[str, float]] | None = None,
    ) -> bool:
        occ, res = self._build_occupancy(instance, forbidden_regions=forbidden_regions)
        ny, nx = occ.shape
        sx, sy = self._world_to_grid(float(instance.start_xy[0]), float(instance.start_xy[1]), res, nx, ny)
        gx, gy = self._world_to_grid(float(instance.goal_xy[0]), float(instance.goal_xy[1]), res, nx, ny)

        s = self._nearest_free(occ, (sx, sy))
        g = self._nearest_free(occ, (gx, gy))
        if s is None or g is None:
            return False
        if s == g:
            return True

        open_heap: list[tuple[float, int, int]] = []
        heapq.heappush(open_heap, (0.0, s[0], s[1]))
        visited = np.zeros_like(occ, dtype=bool)
        g_cost = np.full_like(occ, fill_value=np.inf, dtype=float)
        g_cost[s[1], s[0]] = 0.0

        neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0))]
        while open_heap:
            _, x, y = heapq.heappop(open_heap)
            if visited[y, x]:
                continue
            visited[y, x] = True
            if (x, y) == g:
                return True
            for dx, dy, dc in neighbors:
                nx2, ny2 = x + dx, y + dy
                if not (0 <= nx2 < nx and 0 <= ny2 < ny):
                    continue
                if occ[ny2, nx2] or visited[ny2, nx2]:
                    continue
                new_g = g_cost[y, x] + dc
                if new_g >= g_cost[ny2, nx2]:
                    continue
                g_cost[ny2, nx2] = new_g
                h = math.hypot(g[0] - nx2, g[1] - ny2)
                heapq.heappush(open_heap, (new_g + h, nx2, ny2))
        return False

    def _bottleneck_turning_ok(self, instance: ScenarioInstance, gates: list[dict[str, float]]) -> bool:
        if not gates:
            return False
        x_min = min(float(g["x0"]) for g in gates) - 0.8
        x_max = max(float(g["x1"]) for g in gates) + 0.8
        mask = (instance.path_xy[:, 0] >= x_min) & (instance.path_xy[:, 0] <= x_max)
        if int(np.sum(mask)) < 4:
            return False
        y_span = float(np.max(instance.path_xy[mask, 1]) - np.min(instance.path_xy[mask, 1]))
        kappa = np.abs(_polyline_curvature(instance.path_xy))
        kappa_max = float(np.max(kappa[mask]))
        return y_span >= self.cfg.scenario.turning_gate_y_span_min and kappa_max >= self.cfg.scenario.turning_gate_kappa_min

    def validate_instance(self, instance: ScenarioInstance) -> ValidationResult:
        assert instance.labels is not None
        labels = instance.labels
        n_total = len(instance.vehicles_init)
        checks: dict[str, bool] = {}

        checks["type_match"] = labels.predicted_type == labels.expected_type
        checks["single_pass_feasible"] = labels.n_max_pass_global >= 1
        checks["profile_non_empty"] = len(labels.n_max_pass_profile) > 0
        checks["metadata_complete"] = (
            labels.decision_horizon in {"low", "medium", "high"}
            and labels.coordination_intensity in {"low", "medium", "high"}
            and labels.initial_dispersion_mode in {"Clustered_At_A", "Random_Scattered", "Uniform_Spread"}
        )

        checks["vehicles_spawn_safe"] = True
        for i, s in enumerate(instance.vehicles_init):
            for obs in instance.obstacles:
                if self.collision.collide_vehicle_obstacle(s, obs, include_clearance=False):
                    checks["vehicles_spawn_safe"] = False
                    break
            if not checks["vehicles_spawn_safe"]:
                break
            for j in range(i + 1, len(instance.vehicles_init)):
                if self.collision.collide_vehicle_vehicle(s, instance.vehicles_init[j], include_clearance=False):
                    checks["vehicles_spawn_safe"] = False
                    break
            if not checks["vehicles_spawn_safe"]:
                break

        if instance.subtype in {"B1", "B2", "B3", "C1", "C2", "C3"}:
            gates = list(instance.params.get("mandatory_gates", []))
            checks["baseline_route_exists"] = self._path_exists_grid(instance)
            checks["bottleneck_has_turn"] = self._bottleneck_turning_ok(instance, gates)
            unavoidable = len(gates) > 0 and checks["baseline_route_exists"]
            if unavoidable:
                for gate in gates:
                    if self._path_exists_grid(instance, forbidden_regions=[gate]):
                        unavoidable = False
                        break
            checks["bottleneck_unavoidable"] = unavoidable

        if instance.subtype == "A3":
            checks["a3_full_pass"] = labels.n_max_pass_global >= n_total
        if instance.subtype == "B1":
            checks["b1_single_only"] = labels.n_max_pass_global == 1
        if instance.subtype == "B2":
            k_target = int(instance.params.get("B2_K", 2))
            checks["b2_kcap_match"] = labels.n_max_pass_global == k_target
        if instance.subtype == "B3":
            checks["b3_has_multi_bottleneck"] = labels.bottleneck_count >= 2
        if instance.subtype in {"C1", "C2", "C3"}:
            values = [int(seg["n_max"]) for seg in labels.n_max_pass_profile]
            transitions = sum(1 for i in range(1, len(values)) if values[i] != values[i - 1])
            checks["c_requires_reconfig"] = transitions >= 1

        passed = all(checks.values())
        return ValidationResult(
            scenario_id=instance.scenario_id,
            subtype=instance.subtype,
            expected_type=instance.expected_type,
            predicted_type=labels.predicted_type,
            passed=passed,
            checks=checks,
        )

    def validate_batch(self, seeds_per_subtype: int | None = None) -> dict[str, Any]:
        n_seed = self.cfg.scenario.validation_seeds_per_subtype if seeds_per_subtype is None else int(seeds_per_subtype)
        subtypes = ["A1", "A2", "A3", "B1", "B2", "B3", "C1", "C2", "C3"]
        modes = ["Clustered_At_A", "Random_Scattered", "Uniform_Spread"]
        results: list[ValidationResult] = []
        cases: list[ScenarioInstance] = []
        base_seed = self.cfg.testing.random_seed

        for sub in subtypes:
            for i in range(n_seed):
                mode = modes[i % len(modes)]
                seed = base_seed + 1000 * subtypes.index(sub) + i
                override = None
                if sub == "B2":
                    k = 2 + (i % 3)  # 2/3/4
                    override = {"B2_K": k}
                inst = self.generator.generate(
                    sub,
                    seed=seed,
                    n_vehicles=self.cfg.scenario.representative_vehicle_count,
                    initial_dispersion_mode=mode,
                    overrides=override,
                )
                cases.append(inst)
                results.append(self.validate_instance(inst))

        by_subtype: dict[str, dict[str, Any]] = {}
        for sub in subtypes:
            rs = [r for r in results if r.subtype == sub]
            pass_rate = sum(1 for r in rs if r.passed) / max(len(rs), 1)
            by_subtype[sub] = {
                "total": len(rs),
                "pass_rate": float(pass_rate),
                "passed": int(sum(1 for r in rs if r.passed)),
                "min_required": float(self.cfg.scenario.validation_min_pass_rate),
                "ok": bool(pass_rate >= self.cfg.scenario.validation_min_pass_rate),
            }

        overall = {
            "total": len(results),
            "passed": int(sum(1 for r in results if r.passed)),
            "pass_rate": float(sum(1 for r in results if r.passed) / max(len(results), 1)),
            "ok": bool(all(v["ok"] for v in by_subtype.values())),
        }
        return {
            "overall": overall,
            "by_subtype": by_subtype,
            "sample_labels": [c.labels.to_dict() for c in cases[:12] if c.labels is not None],
            "failed_examples": [r.to_dict() for r in results if not r.passed][:30],
        }
