import math

from docking.config import load_config
from docking.kinematics import AckermannModel
from docking.types import ControlCommand, VehicleState


def test_ackermann_turning_radius_and_constraints():
    cfg = load_config()
    model = AckermannModel(cfg.vehicle)

    state = VehicleState(vehicle_id=1, x=0.0, y=0.0, yaw=0.0, v=1.0, delta=math.radians(20.0))
    radius_theory = model.turning_radius(state)

    for _ in range(60):
        state = model.step(state, ControlCommand(accel=0.0, steer_rate=0.0), cfg.control.dt)

    radius_end = model.turning_radius(state)
    assert abs(radius_end - radius_theory) < 1e-4
    assert 0.0 <= state.v <= cfg.vehicle.max_speed
    assert abs(state.delta) <= math.radians(cfg.vehicle.max_steer_deg) + 1e-9


def test_ackermann_steer_and_speed_clamping():
    cfg = load_config()
    model = AckermannModel(cfg.vehicle)
    state = VehicleState(vehicle_id=1, x=0.0, y=0.0, yaw=0.0, v=0.0, delta=0.0)

    for _ in range(80):
        state = model.step(state, ControlCommand(accel=10.0, steer_rate=10.0), cfg.control.dt)

    assert state.v <= cfg.vehicle.max_speed + 1e-9
    assert abs(state.delta) <= math.radians(cfg.vehicle.max_steer_deg) + 1e-9
