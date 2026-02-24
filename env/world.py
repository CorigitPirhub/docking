from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from core.vehicle import AckermannVehicle, VehicleSpec, VehicleState


@dataclass
class RectObstacle:
    cx: float
    cy: float
    w: float
    h: float

    @property
    def xmin(self) -> float:
        return self.cx - self.w / 2.0

    @property
    def xmax(self) -> float:
        return self.cx + self.w / 2.0

    @property
    def ymin(self) -> float:
        return self.cy - self.h / 2.0

    @property
    def ymax(self) -> float:
        return self.cy + self.h / 2.0


@dataclass
class KeepoutZone:
    x: float
    y: float
    r: float


class WorldMap:
    def __init__(self, size_m: float = 100.0, resolution: float = 0.5):
        self.size_m = float(size_m)
        self.resolution = float(resolution)
        self.width = int(round(self.size_m / self.resolution))
        self.height = int(round(self.size_m / self.resolution))
        self.obstacles: List[RectObstacle] = []

        self.occ = np.zeros((self.height, self.width), dtype=np.uint8)
        self.inflated_occ = np.zeros_like(self.occ)

    def clone_empty(self) -> "WorldMap":
        w = WorldMap(size_m=self.size_m, resolution=self.resolution)
        return w

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(np.clip(np.floor(x / self.resolution), 0, self.width - 1))
        gy = int(np.clip(np.floor(y / self.resolution), 0, self.height - 1))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = (gx + 0.5) * self.resolution
        y = (gy + 0.5) * self.resolution
        return float(x), float(y)

    def in_bounds(self, x: float, y: float) -> bool:
        return 0.0 <= x <= self.size_m and 0.0 <= y <= self.size_m

    def generate_random_obstacles(
        self,
        n_obs: int,
        min_size: float,
        max_size: float,
        seed: int,
        keepouts: Optional[Sequence[KeepoutZone]] = None,
    ) -> None:
        rng = np.random.default_rng(seed)
        keepouts = list(keepouts) if keepouts is not None else []
        self.obstacles = []

        attempts = 0
        while len(self.obstacles) < n_obs and attempts < n_obs * 80:
            attempts += 1
            w = float(rng.uniform(min_size, max_size))
            h = float(rng.uniform(min_size, max_size))
            cx = float(rng.uniform(6.0, self.size_m - 6.0))
            cy = float(rng.uniform(6.0, self.size_m - 6.0))
            obs = RectObstacle(cx=cx, cy=cy, w=w, h=h)

            if any(self._overlaps_keepout(obs, kz) for kz in keepouts):
                continue
            if any(self._overlaps(obs, old, margin=1.5) for old in self.obstacles):
                continue
            self.obstacles.append(obs)

        self._rasterize_obstacles()

    def _overlaps(self, a: RectObstacle, b: RectObstacle, margin: float = 0.0) -> bool:
        return not (
            a.xmax + margin < b.xmin
            or b.xmax + margin < a.xmin
            or a.ymax + margin < b.ymin
            or b.ymax + margin < a.ymin
        )

    def _overlaps_keepout(self, obs: RectObstacle, kz: KeepoutZone) -> bool:
        cx = np.clip(kz.x, obs.xmin, obs.xmax)
        cy = np.clip(kz.y, obs.ymin, obs.ymax)
        dist = float(np.hypot(kz.x - cx, kz.y - cy))
        return dist < kz.r

    def _rasterize_obstacles(self) -> None:
        self.occ.fill(0)
        for obs in self.obstacles:
            gx0, gy0 = self.world_to_grid(obs.xmin, obs.ymin)
            gx1, gy1 = self.world_to_grid(obs.xmax, obs.ymax)
            self.occ[gy0 : gy1 + 1, gx0 : gx1 + 1] = 1

    def set_obstacles(self, obstacles: Sequence[RectObstacle]) -> None:
        self.obstacles = list(obstacles)
        self._rasterize_obstacles()

    def inflate(self, radius_m: float) -> None:
        rad_cells = int(np.ceil(radius_m / self.resolution))
        if rad_cells <= 0:
            self.inflated_occ = self.occ.copy()
            return

        self.inflated_occ = self.occ.copy()
        ys, xs = np.where(self.occ > 0)
        for gy, gx in zip(ys, xs):
            x0 = max(0, gx - rad_cells)
            x1 = min(self.width - 1, gx + rad_cells)
            y0 = max(0, gy - rad_cells)
            y1 = min(self.height - 1, gy + rad_cells)
            for ny in range(y0, y1 + 1):
                dy = ny - gy
                for nx in range(x0, x1 + 1):
                    dx = nx - gx
                    if dx * dx + dy * dy <= rad_cells * rad_cells:
                        self.inflated_occ[ny, nx] = 1

    def is_free_point(self, x: float, y: float, use_inflated: bool = True) -> bool:
        if not self.in_bounds(x, y):
            return False
        gx, gy = self.world_to_grid(x, y)
        grid = self.inflated_occ if use_inflated else self.occ
        return bool(grid[gy, gx] == 0)

    def collision_for_pose(
        self,
        x: float,
        y: float,
        theta: float,
        spec: VehicleSpec,
        static_vehicles: Optional[Iterable[AckermannVehicle]] = None,
    ) -> bool:
        dummy = AckermannVehicle(spec=spec, state=VehicleState(x=x, y=y, theta=theta))
        circles, r = dummy.footprint_circles()

        for c in circles:
            if not self.is_free_point(float(c[0]), float(c[1]), use_inflated=True):
                return True

        if static_vehicles is not None:
            for other in static_vehicles:
                ocircles, orad = other.footprint_circles()
                for c in circles:
                    d = np.linalg.norm(ocircles - c.reshape(1, 2), axis=1)
                    if np.any(d < (r + orad) * 0.95):
                        return True

        return False

    def sample_free_point(self, rng: np.random.Generator) -> Tuple[float, float]:
        for _ in range(2000):
            x = float(rng.uniform(1.0, self.size_m - 1.0))
            y = float(rng.uniform(1.0, self.size_m - 1.0))
            if self.is_free_point(x, y, use_inflated=True):
                return x, y
        return 0.5 * self.size_m, 0.5 * self.size_m
