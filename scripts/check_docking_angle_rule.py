#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.simulation import run_docking_case


def main() -> None:
    cfg = load_config()
    violate = 0
    succ = 0
    total = 60
    for i in range(total):
        r = run_docking_case(
            cfg,
            leader_train_size=1 + (i % 3),
            seed=500 + i,
            with_obstacles=(i % 2 == 0),
            moving_leader=True,
        )
        if r.success:
            succ += 1
            if r.final_yaw_error_deg >= 10.0:
                violate += 1

    print("docking_success_count", succ)
    print("docking_angle_violation_count", violate)
    print("docking_angle_rule_ok", violate == 0)


if __name__ == "__main__":
    main()
