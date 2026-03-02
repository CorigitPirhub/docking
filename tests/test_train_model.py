import math

import numpy as np

from docking.config import load_config
from docking.train import TrainController, TrainKinematics
from docking.kinematics import VehicleGeometry
from docking.types import ControlCommand, VehicleState


def test_train_kinematics_hitch_consistency_and_articulation():
    cfg = load_config()
    geom = VehicleGeometry(cfg.vehicle)
    train = TrainKinematics(cfg.vehicle)

    states = [
        VehicleState(vehicle_id=100, x=0.0, y=0.0, yaw=0.0, v=1.0, delta=0.0),
        VehicleState(vehicle_id=101, x=-1.6, y=0.0, yaw=0.0, v=1.0, delta=0.0),
        VehicleState(vehicle_id=102, x=-3.2, y=0.0, yaw=0.0, v=1.0, delta=0.0),
    ]

    for _ in range(120):
        out = train.update(states, ControlCommand(accel=0.2, steer_rate=math.radians(8.0)), cfg.control.dt)
        states = out.states

        for i in range(1, len(states)):
            lead_rear = geom.rear_hitch(states[i - 1])
            fol_front = geom.front_hitch(states[i])
            assert abs(float(((lead_rear - fol_front) ** 2).sum())) < 5e-4

        for phi in out.articulation_angles:
            assert abs(phi) < math.radians(50.0)


def test_train_turn_radius_formula():
    cfg = load_config()
    train = TrainKinematics(cfg.vehicle)
    assert abs(train.min_turn_radius_train(1) - 1.0) < 1e-9
    assert abs(train.min_turn_radius_train(3) - 2.0) < 1e-9


def test_train_controller_rejects_tight_path():
    cfg = load_config()
    controller = TrainController(cfg.vehicle, cfg.control)
    geom = VehicleGeometry(cfg.vehicle)
    states = [
        VehicleState(vehicle_id=100 + i, x=-i * (geom.front_hitch_x - geom.rear_hitch_x), y=0.0, yaw=0.0, v=0.0, delta=0.0)
        for i in range(4)
    ]

    xs = np.linspace(-4.0, 4.0, 120)
    ys = 2.0 * np.sin(xs / 1.1)  # intentionally tight
    path = np.stack([xs, ys], axis=1)
    cmd = controller.track_head_path(states, path, target_speed=0.5, dt=cfg.control.dt)
    assert cmd.accel <= -0.8 * cfg.vehicle.max_decel
    assert not controller.last_path_feasibility.feasible
