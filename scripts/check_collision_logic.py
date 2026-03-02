#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.collision import CollisionEngine
from docking.config import load_config
from docking.types import Obstacle, VehicleState


def main() -> None:
    cfg = load_config()
    ce = CollisionEngine(cfg.vehicle, cfg.safety)

    # 1) single vs single
    a = VehicleState(vehicle_id=1, x=0.0, y=0.0, yaw=0.0, v=0.0, delta=0.0)
    b_hit = VehicleState(vehicle_id=2, x=0.35, y=0.0, yaw=0.0, v=0.0, delta=0.0)
    b_free = VehicleState(vehicle_id=3, x=4.0, y=0.0, yaw=0.0, v=0.0, delta=0.0)

    # 2) single vs train (non-adjacent check)
    t1 = VehicleState(vehicle_id=10, x=2.0, y=0.0, yaw=0.0, v=0.0, delta=0.0)
    t2 = VehicleState(vehicle_id=11, x=0.4, y=0.0, yaw=0.0, v=0.0, delta=0.0)

    # 3) vehicle vs obstacle
    obs_hit = Obstacle(x=0.1, y=0.0, width=1.0, height=1.0)
    obs_free = Obstacle(x=6.0, y=0.0, width=1.0, height=1.0)

    checks = {
        "single_single_hit": ce.collide_vehicle_vehicle(a, b_hit, include_clearance=False),
        "single_single_free": not ce.collide_vehicle_vehicle(a, b_free, include_clearance=False),
        "single_train_tail_hit": ce.collide_vehicle_vehicle(a, t2, include_clearance=False),
        "single_train_head_free": not ce.collide_vehicle_vehicle(a, t1, include_clearance=False),
        "single_vs_train_api_hit": ce.collide_vehicle_train(a, [t1, t2], include_clearance=False),
        "vehicle_obstacle_hit": ce.collide_vehicle_obstacle(a, obs_hit, include_clearance=False),
        "vehicle_obstacle_free": not ce.collide_vehicle_obstacle(a, obs_free, include_clearance=False),
        "symmetry_vehicle_collision": ce.collide_vehicle_vehicle(a, b_hit, include_clearance=False)
        == ce.collide_vehicle_vehicle(b_hit, a, include_clearance=False),
    }

    all_ok = all(checks.values())
    print("collision_checks", checks)
    print("collision_all_ok", all_ok)


if __name__ == "__main__":
    main()
