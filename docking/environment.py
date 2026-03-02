from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import EnvironmentConfig
from .types import Obstacle, VehicleState


@dataclass
class Environment:
    cfg: EnvironmentConfig
    obstacles: list[Obstacle]

    def in_bounds(self, state: VehicleState, margin: float = 0.0) -> bool:
        half_w = self.cfg.width * 0.5 - margin
        half_h = self.cfg.height * 0.5 - margin
        return -half_w <= state.x <= half_w and -half_h <= state.y <= half_h


def random_obstacles(
    env_cfg: EnvironmentConfig,
    rng: np.random.Generator,
    num: int,
    min_size: float = 0.5,
    max_size: float = 2.0,
    margin: float = 2.0,
) -> list[Obstacle]:
    obs: list[Obstacle] = []
    for _ in range(num):
        w = float(rng.uniform(min_size, max_size))
        h = float(rng.uniform(min_size, max_size))
        x = float(rng.uniform(-env_cfg.width * 0.5 + margin, env_cfg.width * 0.5 - margin))
        y = float(rng.uniform(-env_cfg.height * 0.5 + margin, env_cfg.height * 0.5 - margin))
        yaw = float(rng.uniform(-0.6, 0.6))
        obs.append(Obstacle(x=x, y=y, width=w, height=h, yaw=yaw))
    return obs
