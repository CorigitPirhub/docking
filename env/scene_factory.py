from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from env.world import KeepoutZone, RectObstacle, WorldMap


@dataclass
class SceneBuildResult:
    mode: str
    difficulty_level: int
    obstacle_count: int
    inflation_radius: float


class SceneFactory:
    MODES = ("simple", "mode_a", "mode_b", "mode_c")

    def __init__(self, world: WorldMap, rng: np.random.Generator):
        self.world = world
        self.rng = rng

    def generate(
        self,
        mode: str,
        difficulty_level: int,
        keepouts: Sequence[KeepoutZone],
        obstacle_count_hint: int = 9,
    ) -> SceneBuildResult:
        mode = mode.lower()
        diff = int(np.clip(difficulty_level, 1, 3))

        if mode == "simple":
            n_obs = int(obstacle_count_hint + (diff - 1) * 2)
            self.world.generate_random_obstacles(
                n_obs=n_obs,
                min_size=3.8,
                max_size=9.0,
                seed=int(self.rng.integers(1, 2_000_000_000)),
                keepouts=keepouts,
            )
            inflation = 1.1 if diff <= 1 else 1.0
            self.world.inflate(inflation)
            return SceneBuildResult(
                mode="simple",
                difficulty_level=diff,
                obstacle_count=len(self.world.obstacles),
                inflation_radius=inflation,
            )

        if mode == "mode_a":
            obstacles = self._build_mode_a(diff, keepouts)
            inflation = 0.95 if diff == 1 else (0.9 if diff == 2 else 0.85)
        elif mode == "mode_b":
            obstacles = self._build_mode_b(diff, keepouts)
            inflation = 0.72 if diff == 1 else (0.66 if diff == 2 else 0.62)
        elif mode == "mode_c":
            obstacles = self._build_mode_c(diff, keepouts)
            inflation = 0.88 if diff == 1 else (0.82 if diff == 2 else 0.78)
        else:
            raise ValueError(f"Unknown scene mode: {mode}")

        self.world.set_obstacles(obstacles)
        self.world.inflate(inflation)
        return SceneBuildResult(
            mode=mode,
            difficulty_level=diff,
            obstacle_count=len(obstacles),
            inflation_radius=inflation,
        )

    def _build_mode_a(self, difficulty: int, keepouts: Sequence[KeepoutZone]) -> List[RectObstacle]:
        target = {1: 24, 2: 30, 3: 36}[difficulty]
        min_size = {1: 2.3, 2: 2.1, 3: 1.8}[difficulty]
        max_size = {1: 6.0, 2: 5.6, 3: 5.2}[difficulty]

        obstacles: List[RectObstacle] = []
        attempts = 0
        while len(obstacles) < target and attempts < target * 120:
            attempts += 1
            w = float(self.rng.uniform(min_size, max_size))
            h = float(self.rng.uniform(min_size, max_size))
            cx = float(self.rng.uniform(6.0, self.world.size_m - 6.0))
            cy = float(self.rng.uniform(6.0, self.world.size_m - 6.0))
            obs = RectObstacle(cx=cx, cy=cy, w=w, h=h)
            if not self._valid(obs, obstacles, keepouts, margin=0.8):
                continue
            obstacles.append(obs)
        return obstacles

    def _build_mode_b(self, difficulty: int, keepouts: Sequence[KeepoutZone]) -> List[RectObstacle]:
        obstacles: List[RectObstacle] = []

        wall_count = {1: 6, 2: 7, 3: 7}[difficulty]
        wall_thickness = {1: 1.5, 2: 1.6, 3: 1.7}[difficulty]
        gap_size = {1: 5.4, 2: 4.7, 3: 4.2}[difficulty]

        x_min = 22.0
        x_max = 78.0
        spacing = (x_max - x_min) / max(wall_count - 1, 1)

        for i in range(wall_count):
            x = x_min + i * spacing
            gap_center = 20.0 if i % 2 == 0 else 80.0
            gap_center += float(self.rng.uniform(-4.0, 4.0))
            gap0 = np.clip(gap_center - 0.5 * gap_size, 8.0, 92.0)
            gap1 = np.clip(gap_center + 0.5 * gap_size, 8.0, 92.0)

            lower_h = max(0.0, gap0 - 6.0)
            if lower_h > 1.2:
                obs = RectObstacle(cx=x, cy=6.0 + 0.5 * lower_h, w=wall_thickness, h=lower_h)
                if self._valid(obs, obstacles, keepouts, margin=0.3):
                    obstacles.append(obs)

            upper_h = max(0.0, 94.0 - gap1)
            if upper_h > 1.2:
                obs = RectObstacle(cx=x, cy=gap1 + 0.5 * upper_h, w=wall_thickness, h=upper_h)
                if self._valid(obs, obstacles, keepouts, margin=0.3):
                    obstacles.append(obs)

        # Add short blockers to form warehouse-style choke points.
        blocker_count = {1: 2, 2: 3, 3: 4}[difficulty]
        for _ in range(blocker_count):
            cx = float(self.rng.uniform(28.0, 72.0))
            cy = float(self.rng.uniform(24.0, 76.0))
            w = float(self.rng.uniform(3.0, 6.0))
            h = float(self.rng.uniform(1.2, 2.0))
            if self.rng.uniform() < 0.5:
                w, h = h, w
            obs = RectObstacle(cx=cx, cy=cy, w=w, h=h)
            if self._valid(obs, obstacles, keepouts, margin=0.45):
                obstacles.append(obs)

        return obstacles

    def _build_mode_c(self, difficulty: int, keepouts: Sequence[KeepoutZone]) -> List[RectObstacle]:
        obstacles: List[RectObstacle] = []

        # U-shapes and C-rings induce dead-ends and non-convex trap-like regions.
        u_count = {1: 3, 2: 4, 3: 5}[difficulty]
        ring_count = {1: 1, 2: 2, 3: 2}[difficulty]
        thick = {1: 1.8, 2: 2.0, 3: 2.2}[difficulty]

        dirs = ["up", "down", "left", "right"]

        for _ in range(u_count):
            cx = float(self.rng.uniform(20.0, 80.0))
            cy = float(self.rng.uniform(20.0, 80.0))
            span = float(self.rng.uniform(10.0, 16.0))
            depth = float(self.rng.uniform(9.0, 15.0))
            d = dirs[int(self.rng.integers(0, len(dirs)))]
            for obs in self._u_shape(cx, cy, span, depth, thick, d):
                if self._valid(obs, obstacles, keepouts, margin=0.45):
                    obstacles.append(obs)

        for _ in range(ring_count):
            cx = float(self.rng.uniform(30.0, 70.0))
            cy = float(self.rng.uniform(30.0, 70.0))
            outer = float(self.rng.uniform(14.0, 20.0))
            gap = float(self.rng.uniform(4.2, 6.0))
            open_dir = dirs[int(self.rng.integers(0, len(dirs)))]
            for obs in self._c_ring(cx, cy, outer, thick, gap, open_dir):
                if self._valid(obs, obstacles, keepouts, margin=0.45):
                    obstacles.append(obs)

        # Background clutter increases local ambiguity.
        clutter = {1: 6, 2: 8, 3: 10}[difficulty]
        for _ in range(clutter):
            w = float(self.rng.uniform(2.0, 4.5))
            h = float(self.rng.uniform(2.0, 4.5))
            cx = float(self.rng.uniform(8.0, 92.0))
            cy = float(self.rng.uniform(8.0, 92.0))
            obs = RectObstacle(cx=cx, cy=cy, w=w, h=h)
            if self._valid(obs, obstacles, keepouts, margin=0.35):
                obstacles.append(obs)

        return obstacles

    def _u_shape(
        self,
        cx: float,
        cy: float,
        span: float,
        depth: float,
        thick: float,
        open_dir: str,
    ) -> List[RectObstacle]:
        half_span = 0.5 * span
        half_depth = 0.5 * depth
        out: List[RectObstacle] = []

        if open_dir in {"up", "down"}:
            # Two vertical legs.
            out.append(RectObstacle(cx=cx - half_span + 0.5 * thick, cy=cy, w=thick, h=depth))
            out.append(RectObstacle(cx=cx + half_span - 0.5 * thick, cy=cy, w=thick, h=depth))
            # Bottom or top base.
            if open_dir == "up":
                out.append(RectObstacle(cx=cx, cy=cy - half_depth + 0.5 * thick, w=span, h=thick))
            else:
                out.append(RectObstacle(cx=cx, cy=cy + half_depth - 0.5 * thick, w=span, h=thick))
        else:
            # Two horizontal legs.
            out.append(RectObstacle(cx=cx, cy=cy - half_span + 0.5 * thick, w=depth, h=thick))
            out.append(RectObstacle(cx=cx, cy=cy + half_span - 0.5 * thick, w=depth, h=thick))
            # Left or right base.
            if open_dir == "right":
                out.append(RectObstacle(cx=cx - half_depth + 0.5 * thick, cy=cy, w=thick, h=span))
            else:
                out.append(RectObstacle(cx=cx + half_depth - 0.5 * thick, cy=cy, w=thick, h=span))
        return out

    def _c_ring(
        self,
        cx: float,
        cy: float,
        outer: float,
        thick: float,
        gap: float,
        open_dir: str,
    ) -> List[RectObstacle]:
        hs = 0.5 * outer
        out: List[RectObstacle] = []

        # Four ring walls with one opening segment removed.
        # Top
        if open_dir != "up":
            out.append(RectObstacle(cx=cx, cy=cy + hs - 0.5 * thick, w=outer, h=thick))
        else:
            out.append(RectObstacle(cx=cx - 0.25 * outer, cy=cy + hs - 0.5 * thick, w=0.5 * outer - 0.5 * gap, h=thick))
            out.append(RectObstacle(cx=cx + 0.25 * outer, cy=cy + hs - 0.5 * thick, w=0.5 * outer - 0.5 * gap, h=thick))

        # Bottom
        if open_dir != "down":
            out.append(RectObstacle(cx=cx, cy=cy - hs + 0.5 * thick, w=outer, h=thick))
        else:
            out.append(RectObstacle(cx=cx - 0.25 * outer, cy=cy - hs + 0.5 * thick, w=0.5 * outer - 0.5 * gap, h=thick))
            out.append(RectObstacle(cx=cx + 0.25 * outer, cy=cy - hs + 0.5 * thick, w=0.5 * outer - 0.5 * gap, h=thick))

        # Left
        if open_dir != "left":
            out.append(RectObstacle(cx=cx - hs + 0.5 * thick, cy=cy, w=thick, h=outer))
        else:
            out.append(RectObstacle(cx=cx - hs + 0.5 * thick, cy=cy - 0.25 * outer, w=thick, h=0.5 * outer - 0.5 * gap))
            out.append(RectObstacle(cx=cx - hs + 0.5 * thick, cy=cy + 0.25 * outer, w=thick, h=0.5 * outer - 0.5 * gap))

        # Right
        if open_dir != "right":
            out.append(RectObstacle(cx=cx + hs - 0.5 * thick, cy=cy, w=thick, h=outer))
        else:
            out.append(RectObstacle(cx=cx + hs - 0.5 * thick, cy=cy - 0.25 * outer, w=thick, h=0.5 * outer - 0.5 * gap))
            out.append(RectObstacle(cx=cx + hs - 0.5 * thick, cy=cy + 0.25 * outer, w=thick, h=0.5 * outer - 0.5 * gap))

        return out

    def _valid(
        self,
        obs: RectObstacle,
        existing: Sequence[RectObstacle],
        keepouts: Sequence[KeepoutZone],
        margin: float,
    ) -> bool:
        if obs.xmin < 1.0 or obs.ymin < 1.0 or obs.xmax > self.world.size_m - 1.0 or obs.ymax > self.world.size_m - 1.0:
            return False

        for kz in keepouts:
            cx = np.clip(kz.x, obs.xmin, obs.xmax)
            cy = np.clip(kz.y, obs.ymin, obs.ymax)
            if float(np.hypot(kz.x - cx, kz.y - cy)) < kz.r:
                return False

        for old in existing:
            if not (
                obs.xmax + margin < old.xmin
                or old.xmax + margin < obs.xmin
                or obs.ymax + margin < old.ymin
                or old.ymax + margin < obs.ymin
            ):
                return False

        return True
