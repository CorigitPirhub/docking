#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.simulation import run_docking_case


def scan_history(history: dict[str, list[float]], dt: float, vmax: float) -> dict[str, float]:
    x = np.array(history["follower_x"], dtype=float)
    y = np.array(history["follower_y"], dtype=float)
    yaw = np.unwrap(np.array(history["follower_yaw"], dtype=float))

    if len(x) < 2:
        return {
            "max_step_speed": 0.0,
            "max_yaw_step_deg": 0.0,
            "max_lateral_speed": 0.0,
            "inplace_rotate_events": 0.0,
            "jump_events": 0.0,
        }

    dx = np.diff(x)
    dy = np.diff(y)
    ds = np.hypot(dx, dy)
    step_speed = ds / dt
    dyaw = np.abs(np.diff(yaw))

    max_lateral = 0.0
    inplace = 0
    jumps = 0
    for i in range(len(dx)):
        c = math.cos(-yaw[i])
        s = math.sin(-yaw[i])
        bx = c * dx[i] - s * dy[i]
        by = s * dx[i] + c * dy[i]
        lat_speed = abs(by) / dt
        max_lateral = max(max_lateral, lat_speed)

        if abs(bx) < 1e-4 and dyaw[i] > math.radians(5.0):
            inplace += 1
        if step_speed[i] > vmax * 1.2 or math.degrees(dyaw[i]) > 8.0:
            jumps += 1

    return {
        "max_step_speed": float(np.max(step_speed)),
        "max_yaw_step_deg": float(np.degrees(np.max(dyaw))),
        "max_lateral_speed": float(max_lateral),
        "inplace_rotate_events": float(inplace),
        "jump_events": float(jumps),
    }


def main() -> None:
    cfg = load_config()
    stats = []
    for seed in range(40, 70):
        r = run_docking_case(cfg, leader_train_size=2, seed=seed, with_obstacles=(seed % 2 == 0), moving_leader=True)
        s = scan_history(r.history, cfg.control.dt, cfg.vehicle.max_speed)
        stats.append(s)

    agg = {
        "max_step_speed": max(v["max_step_speed"] for v in stats),
        "max_yaw_step_deg": max(v["max_yaw_step_deg"] for v in stats),
        "max_lateral_speed": max(v["max_lateral_speed"] for v in stats),
        "inplace_rotate_events_total": int(sum(v["inplace_rotate_events"] for v in stats)),
        "jump_events_total": int(sum(v["jump_events"] for v in stats)),
    }
    print("dynamics_aggregate", agg)


if __name__ == "__main__":
    main()
