from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np

from .types import Obstacle


@dataclass(frozen=True)
class GridPlannerConfig:
    resolution: float = 0.15
    inflation_radius: float = 0.30
    boundary_block: int = 1
    max_expansions: int = 400_000


class GridAStarPlanner:
    """
    Lightweight grid-based A* planner for static 2D maps.

    Notes:
    - The planner is used as a *foundation* capability for joining the shared corridor path
      from arbitrary scattered starts (P6 robustness bottleneck).
    - Obstacles are conservatively inflated by `inflation_radius` in axis-aligned bounding-box
      space. This is sufficient for the project's rectangular static obstacles and chicane walls.
    """

    def __init__(self, *, width: float, height: float, obstacles: list[Obstacle], cfg: GridPlannerConfig):
        self.width = float(width)
        self.height = float(height)
        self.cfg = cfg
        self.half_w = 0.5 * self.width
        self.half_h = 0.5 * self.height

        res = float(max(1e-6, cfg.resolution))
        self.nx = int(math.ceil(self.width / res)) + 1
        self.ny = int(math.ceil(self.height / res)) + 1
        self.resolution = res

        self.occ = np.zeros((self.ny, self.nx), dtype=bool)
        bnd = int(max(0, cfg.boundary_block))
        if bnd > 0:
            self.occ[:bnd, :] = True
            self.occ[-bnd:, :] = True
            self.occ[:, :bnd] = True
            self.occ[:, -bnd:] = True

        inflate = float(max(0.0, cfg.inflation_radius))
        for obs in obstacles:
            # Conservative inflation in AABB space (yaw is ignored by design).
            x0 = float(obs.x - 0.5 * obs.width - inflate)
            x1 = float(obs.x + 0.5 * obs.width + inflate)
            y0 = float(obs.y - 0.5 * obs.height - inflate)
            y1 = float(obs.y + 0.5 * obs.height + inflate)
            ix0, iy0 = self._world_to_grid(x0, y0)
            ix1, iy1 = self._world_to_grid(x1, y1)
            ix_lo, ix_hi = (min(ix0, ix1), max(ix0, ix1))
            iy_lo, iy_hi = (min(iy0, iy1), max(iy0, iy1))
            self.occ[iy_lo : iy_hi + 1, ix_lo : ix_hi + 1] = True

        self._neighbors: list[tuple[int, int, float]] = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ]

    def _world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        ix = int(round((float(x) + self.half_w) / self.resolution))
        iy = int(round((float(y) + self.half_h) / self.resolution))
        ix = min(max(ix, 0), self.nx - 1)
        iy = min(max(iy, 0), self.ny - 1)
        return ix, iy

    def _grid_to_world(self, ix: int, iy: int) -> np.ndarray:
        x = float(ix) * self.resolution - self.half_w
        y = float(iy) * self.resolution - self.half_h
        return np.array([x, y], dtype=float)

    def _nearest_free(self, start: tuple[int, int]) -> tuple[int, int] | None:
        sx, sy = int(start[0]), int(start[1])
        if not (0 <= sx < self.nx and 0 <= sy < self.ny):
            return None
        if not bool(self.occ[sy, sx]):
            return (sx, sy)
        q: list[tuple[int, int]] = [(sx, sy)]
        visited = np.zeros_like(self.occ, dtype=bool)
        visited[sy, sx] = True
        head = 0
        while head < len(q):
            x, y = q[head]
            head += 1
            for dx, dy, _ in self._neighbors[:4]:
                x2, y2 = x + dx, y + dy
                if not (0 <= x2 < self.nx and 0 <= y2 < self.ny):
                    continue
                if visited[y2, x2]:
                    continue
                visited[y2, x2] = True
                if not bool(self.occ[y2, x2]):
                    return (x2, y2)
                q.append((x2, y2))
        return None

    def _segment_free(self, p0: np.ndarray, p1: np.ndarray) -> bool:
        dist = float(np.linalg.norm(p1 - p0))
        if dist <= 1e-9:
            return True
        step = max(self.resolution * 0.45, 0.05)
        n = int(math.ceil(dist / step))
        for i in range(n + 1):
            a = i / max(1, n)
            p = (1.0 - a) * p0 + a * p1
            ix, iy = self._world_to_grid(float(p[0]), float(p[1]))
            if bool(self.occ[iy, ix]):
                return False
        return True

    def plan(self, *, start_xy: np.ndarray, goal_xy: np.ndarray) -> np.ndarray | None:
        sx, sy = self._world_to_grid(float(start_xy[0]), float(start_xy[1]))
        gx, gy = self._world_to_grid(float(goal_xy[0]), float(goal_xy[1]))
        s = self._nearest_free((sx, sy))
        g = self._nearest_free((gx, gy))
        if s is None or g is None:
            return None
        if s == g:
            return np.stack([start_xy.astype(float).copy(), goal_xy.astype(float).copy()], axis=0)

        g_cost = np.full((self.ny, self.nx), fill_value=np.inf, dtype=float)
        visited = np.zeros((self.ny, self.nx), dtype=bool)
        parent_x = np.full((self.ny, self.nx), fill_value=-1, dtype=np.int32)
        parent_y = np.full((self.ny, self.nx), fill_value=-1, dtype=np.int32)

        g_cost[s[1], s[0]] = 0.0
        open_heap: list[tuple[float, float, int, int]] = []
        heapq.heappush(open_heap, (0.0, 0.0, int(s[0]), int(s[1])))

        expansions = 0
        while open_heap:
            _, cur_g, x, y = heapq.heappop(open_heap)
            if visited[y, x]:
                continue
            visited[y, x] = True
            if (x, y) == g:
                break
            expansions += 1
            if expansions >= int(self.cfg.max_expansions):
                return None

            for dx, dy, dc in self._neighbors:
                x2, y2 = x + dx, y + dy
                if not (0 <= x2 < self.nx and 0 <= y2 < self.ny):
                    continue
                if visited[y2, x2] or bool(self.occ[y2, x2]):
                    continue
                new_g = float(cur_g + dc)
                if new_g >= float(g_cost[y2, x2]):
                    continue
                g_cost[y2, x2] = new_g
                parent_x[y2, x2] = int(x)
                parent_y[y2, x2] = int(y)
                h = math.hypot(float(g[0] - x2), float(g[1] - y2))
                heapq.heappush(open_heap, (new_g + h, new_g, int(x2), int(y2)))

        if not visited[g[1], g[0]]:
            return None

        cells: list[tuple[int, int]] = []
        cx, cy = int(g[0]), int(g[1])
        while True:
            cells.append((cx, cy))
            px = int(parent_x[cy, cx])
            py = int(parent_y[cy, cx])
            if px < 0 or py < 0:
                break
            cx, cy = px, py
        cells.reverse()

        pts = [self._grid_to_world(ix, iy) for ix, iy in cells]
        if len(pts) <= 1:
            return np.stack([start_xy.astype(float).copy(), goal_xy.astype(float).copy()], axis=0)

        # Direction compression.
        compressed: list[np.ndarray] = [pts[0]]
        last_dir: tuple[int, int] | None = None
        for a, b in zip(pts[:-1], pts[1:]):
            dx = float(b[0] - a[0])
            dy = float(b[1] - a[1])
            step_dir = (int(math.copysign(1, dx)) if abs(dx) > 1e-9 else 0, int(math.copysign(1, dy)) if abs(dy) > 1e-9 else 0)
            if last_dir is None or step_dir != last_dir:
                compressed.append(b)
                last_dir = step_dir
            else:
                compressed[-1] = b

        # Shortcut smoothing in free space (string pulling).
        shortcut: list[np.ndarray] = [compressed[0]]
        i = 0
        while i < len(compressed) - 1:
            j = len(compressed) - 1
            while j > i + 1 and not self._segment_free(compressed[i], compressed[j]):
                j -= 1
            shortcut.append(compressed[j])
            i = j

        # Replace start/end with exact query points for better tracking.
        shortcut[0] = start_xy.astype(float).copy()
        shortcut[-1] = goal_xy.astype(float).copy()
        return np.stack(shortcut, axis=0)
