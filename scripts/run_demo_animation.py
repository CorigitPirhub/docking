#!/usr/bin/env python3
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.environment import random_obstacles
from docking.simulation import run_docking_case
from docking.visualization import save_docking_animation


def main() -> None:
    cfg = load_config()
    seed = cfg.testing.random_seed
    rng_seed = seed + 999

    # Re-generate obstacles with same logic used in simulation for visual replay.
    import numpy as np

    rng = np.random.default_rng(rng_seed)
    obstacles = random_obstacles(cfg.environment, rng, 10)
    result = run_docking_case(cfg, leader_train_size=2, seed=seed, with_obstacles=True, moving_leader=True)
    save_docking_animation(cfg, result.history, obstacles, output_path="docking_demo.gif", fps=25)
    print("saved docking_demo.gif", "success=", result.success, "reason=", result.reason)


if __name__ == "__main__":
    main()
