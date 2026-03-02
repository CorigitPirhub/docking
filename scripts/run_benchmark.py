#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.simulation import DockingRunResult, run_docking_case


@dataclass
class GroupSummary:
    name: str
    total: int
    success_rate: float
    avg_time: float
    p95_time: float
    avg_pos_error: float
    avg_yaw_error_deg: float
    avg_speed_error: float
    min_clearance: float
    emergency_events: int


def summarize(name: str, runs: list[DockingRunResult]) -> GroupSummary:
    ok = [r for r in runs if r.success]
    success_rate = len(ok) / len(runs) if runs else 0.0
    times = np.array([r.sim_time for r in ok], dtype=float) if ok else np.array([np.nan])
    pos = np.array([r.final_pos_error for r in ok], dtype=float) if ok else np.array([np.nan])
    yaw = np.array([r.final_yaw_error_deg for r in ok], dtype=float) if ok else np.array([np.nan])
    spd = np.array([r.final_speed_error for r in ok], dtype=float) if ok else np.array([np.nan])
    min_clearance = min(r.min_clearance for r in runs) if runs else float("nan")
    emergencies = sum(r.emergency_events for r in runs)
    return GroupSummary(
        name=name,
        total=len(runs),
        success_rate=float(success_rate),
        avg_time=float(np.nanmean(times)),
        p95_time=float(np.nanpercentile(times, 95)),
        avg_pos_error=float(np.nanmean(pos)),
        avg_yaw_error_deg=float(np.nanmean(yaw)),
        avg_speed_error=float(np.nanmean(spd)),
        min_clearance=float(min_clearance),
        emergency_events=int(emergencies),
    )


def main() -> None:
    cfg = load_config()
    n = cfg.testing.num_random_scenarios

    groups = {
        "dock_1_to_1": [],
        "dock_1_to_2train": [],
        "dock_1_to_3train": [],
    }

    for i in range(n):
        seed = cfg.testing.random_seed + i
        groups["dock_1_to_1"].append(
            run_docking_case(cfg, leader_train_size=1, seed=seed, with_obstacles=(i % 2 == 0), moving_leader=(i % 3 != 0))
        )
        groups["dock_1_to_2train"].append(
            run_docking_case(cfg, leader_train_size=2, seed=1000 + seed, with_obstacles=(i % 2 == 1), moving_leader=True)
        )
        groups["dock_1_to_3train"].append(
            run_docking_case(cfg, leader_train_size=3, seed=2000 + seed, with_obstacles=(i % 2 == 0), moving_leader=True)
        )

    summaries = [summarize(name, runs) for name, runs in groups.items()]
    out = {
        "config": {
            "num_random_scenarios": n,
            "seed_base": cfg.testing.random_seed,
            "max_sim_time": cfg.testing.max_sim_time,
        },
        "summaries": [asdict(s) for s in summaries],
        "targets": {
            "task_success_rate_min": 0.95,
            "single_pos_error_max": 0.02,
            "single_yaw_error_deg_max": 5.0,
            "single_dock_time_max": 15.0,
            "min_clearance_min": 0.1,
        },
    }

    print(json.dumps(out, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
