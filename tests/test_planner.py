import numpy as np

from docking.collision import CollisionEngine
from docking.config import load_config
from docking.kinematics import AckermannModel
from docking.planner import LocalPlanner
from docking.types import Obstacle, VehicleState


def test_local_planner_respects_nonholonomic_and_collision():
    cfg = load_config()
    ce = CollisionEngine(cfg.vehicle, cfg.safety)
    planner = LocalPlanner(cfg.vehicle, cfg.planner, ce, cfg.control.dt)
    model = AckermannModel(cfg.vehicle)

    state = VehicleState(vehicle_id=1, x=0.0, y=-1.0, yaw=0.0, v=0.2, delta=0.0)
    goal = np.array([8.0, 1.5])
    goal_yaw = 0.0
    obstacles = [Obstacle(x=3.0, y=-0.6, width=1.4, height=1.2)]

    for _ in range(120):
        res = planner.plan_step(state, goal, goal_yaw, obstacles, [])
        assert res.feasible
        state = model.step(state, res.command, cfg.control.dt)
        assert not ce.in_collision(state, obstacles, [])

    assert np.linalg.norm(state.xy() - goal) < 2.5
