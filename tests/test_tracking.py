import numpy as np

from docking.config import load_config
from docking.controllers import PathTrackingController
from docking.kinematics import AckermannModel
from docking.types import VehicleState


def run_tracking(mode: str):
    cfg = load_config()
    model = AckermannModel(cfg.vehicle)
    ctrl = PathTrackingController(cfg.vehicle, cfg.control, steering_mode=mode)

    xs = np.linspace(0.0, 22.0, 400)
    ys = 1.2 * np.sin(xs / 5.0)
    path = np.stack([xs, ys], axis=1)

    state = VehicleState(vehicle_id=1, x=-0.5, y=-2.5, yaw=0.3, v=0.0, delta=0.0)
    errors = []

    for _ in range(420):
        cmd = ctrl.track_path(state, path, target_speed=1.0)
        state = model.step(state, cmd, cfg.control.dt)
        d = np.linalg.norm(path - state.xy(), axis=1)
        errors.append(float(np.min(d)))

    return float(np.mean(errors[-80:])), float(np.max(errors))


def test_pure_pursuit_tracking_converges():
    mean_last, max_all = run_tracking("pure_pursuit")
    assert mean_last < 0.45
    assert max_all < 3.5


def test_stanley_tracking_converges():
    mean_last, max_all = run_tracking("stanley")
    assert mean_last < 0.55
    assert max_all < 3.8
