#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.collision import CollisionEngine
from docking.config import load_config
from docking.kinematics import VehicleGeometry
from docking.types import Obstacle, VehicleState


def make_train(n: int, x0: float, y0: float, geom: VehicleGeometry) -> list[VehicleState]:
    spacing = geom.front_hitch_x - geom.rear_hitch_x
    return [VehicleState(vehicle_id=100 + i, x=x0 - i * spacing, y=y0, yaw=0.0, v=0.0, delta=0.0) for i in range(n)]


def main() -> None:
    cfg = load_config()
    geom = VehicleGeometry(cfg.vehicle)
    ce = CollisionEngine(cfg.vehicle, cfg.safety)

    train = make_train(4, x0=0.0, y0=0.0, geom=geom)
    # obstacle only hits tail section, not head
    obs_tail = [Obstacle(x=train[-1].x, y=train[-1].y, width=1.0, height=1.0)]
    # obstacle far away from whole train
    obs_far = [Obstacle(x=20.0, y=20.0, width=1.0, height=1.0)]

    # non-adjacent self-collision mock
    train_self = make_train(4, x0=0.0, y0=0.0, geom=geom)
    train_self[-1].x = train_self[0].x
    train_self[-1].y = train_self[0].y

    checks = {
        "whole_train_obstacle_tail_detected": ce.collide_train_any(train, obs_tail, include_clearance=False),
        "whole_train_obstacle_far_free": not ce.collide_train_any(train, obs_far, include_clearance=False),
        "whole_train_nonadjacent_self_detected": ce.collide_train_self(train_self, include_clearance=False, non_adjacent_only=True),
    }
    print("train_collision_checks", checks)
    print("train_collision_all_ok", all(checks.values()))


if __name__ == "__main__":
    main()
