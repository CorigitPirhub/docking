from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from core.utils import clip
from env.world import WorldMap


@dataclass
class DockingHub:
    x: float
    y: float
    heading: float
    openness: float
    corridor_clearance: float
    score: float


class CostMap:
    def __init__(self, world: WorldMap):
        self.world = world
        self.dist = self._compute_distance_transform(world.inflated_occ, world.resolution)

    def _compute_distance_transform(self, occ: np.ndarray, res: float) -> np.ndarray:
        h, w = occ.shape
        inf = 1e9
        dist = np.full((h, w), inf, dtype=float)
        pq: List[Tuple[float, int, int]] = []

        ys, xs = np.where(occ > 0)
        for gy, gx in zip(ys, xs):
            dist[gy, gx] = 0.0
            heapq.heappush(pq, (0.0, int(gx), int(gy)))

        nbrs = [
            (1, 0, res),
            (-1, 0, res),
            (0, 1, res),
            (0, -1, res),
            (1, 1, math.sqrt(2.0) * res),
            (1, -1, math.sqrt(2.0) * res),
            (-1, 1, math.sqrt(2.0) * res),
            (-1, -1, math.sqrt(2.0) * res),
        ]

        while pq:
            cd, gx, gy = heapq.heappop(pq)
            if cd > dist[gy, gx] + 1e-9:
                continue
            for dx, dy, wgt in nbrs:
                nx, ny = gx + dx, gy + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                nd = cd + wgt
                if nd + 1e-9 < dist[ny, nx]:
                    dist[ny, nx] = nd
                    heapq.heappush(pq, (nd, nx, ny))

        # If there is no obstacle at all, set high clearance everywhere.
        if len(xs) == 0:
            dist.fill(10.0)
        return dist

    def clearance(self, x: float, y: float) -> float:
        gx, gy = self.world.world_to_grid(x, y)
        return float(self.dist[gy, gx])

    def obstacle_cost(self, x: float, y: float, sigma: float = 2.2) -> float:
        d = self.clearance(x, y)
        if d <= 0.0:
            return 1.0
        return float(math.exp(-d / max(sigma, 1e-3)))

    def openness(self, x: float, y: float, radius_m: float = 6.0) -> float:
        gx, gy = self.world.world_to_grid(x, y)
        rad = int(max(1, round(radius_m / self.world.resolution)))
        y0 = max(0, gy - rad)
        y1 = min(self.world.height - 1, gy + rad)
        x0 = max(0, gx - rad)
        x1 = min(self.world.width - 1, gx + rad)
        patch = self.dist[y0 : y1 + 1, x0 : x1 + 1]
        if patch.size == 0:
            return 0.0
        avg = float(np.mean(np.clip(patch, 0.0, radius_m)))
        return clip(avg / max(radius_m, 1e-6), 0.0, 1.0)

    def line_risk(self, p0: Sequence[float], p1: Sequence[float], samples: int = 30) -> float:
        p0 = np.array(p0, dtype=float)
        p1 = np.array(p1, dtype=float)
        risks = []
        for i in range(samples + 1):
            t = i / max(samples, 1)
            p = (1.0 - t) * p0 + t * p1
            risks.append(self.obstacle_cost(float(p[0]), float(p[1])))
        return float(np.mean(risks)) if risks else 0.0

    def corridor_clearance(self, x: float, y: float, heading: float, length: float, step: float = 1.0) -> float:
        dirv = np.array([math.cos(heading), math.sin(heading)], dtype=float)
        clear = []
        s = 0.0
        while s <= length + 1e-6:
            p = np.array([x, y], dtype=float) - s * dirv
            clear.append(self.clearance(float(p[0]), float(p[1])))
            s += step
        if not clear:
            return 0.0
        return float(min(clear))

    def sample_docking_hubs(
        self,
        n_point_samples: int,
        max_hubs: int,
        required_corridor_len: float,
        rng: np.random.Generator,
        min_openness: float = 0.35,
    ) -> List[DockingHub]:
        candidates: List[DockingHub] = []
        headings = [k * (math.pi / 4.0) for k in range(8)]
        boundary_margin = 16.0

        for _ in range(n_point_samples):
            x, y = self.world.sample_free_point(rng)
            if (
                x < boundary_margin
                or x > self.world.size_m - boundary_margin
                or y < boundary_margin
                or y > self.world.size_m - boundary_margin
            ):
                continue
            local_open = self.openness(x, y)
            if local_open < min_openness:
                continue

            for hdg in headings:
                corridor = self.corridor_clearance(x, y, hdg, required_corridor_len, step=1.0)
                if corridor < 1.0:
                    continue
                end_x = x - required_corridor_len * math.cos(hdg)
                end_y = y - required_corridor_len * math.sin(hdg)
                if not self.world.in_bounds(end_x, end_y):
                    continue
                center_bias = 1.0 - min(
                    1.0,
                    math.hypot(x - 50.0, y - 50.0) / 55.0,
                )
                score = (
                    1.35 * local_open
                    + 0.5 * clip(corridor / 6.0, 0.0, 1.0)
                    + 0.35 * center_bias
                )
                candidates.append(
                    DockingHub(
                        x=float(x),
                        y=float(y),
                        heading=float(hdg),
                        openness=float(local_open),
                        corridor_clearance=float(corridor),
                        score=float(score),
                    )
                )

        candidates.sort(key=lambda c: c.score, reverse=True)
        return candidates[:max_hubs]
