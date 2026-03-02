import math

from docking.config import load_config
from docking.sensors import GlobalPoseSensor, VisionSensor
from docking.types import Obstacle, VehicleState


def test_global_sensor_rate_and_noise_output():
    cfg = load_config()
    sensor = GlobalPoseSensor(cfg.sensors.global_sensor, seed=123)
    state = VehicleState(vehicle_id=1, x=1.0, y=2.0, yaw=0.2, v=0.5, delta=0.1)

    meas = []
    t = 0.0
    for _ in range(30):
        m = sensor.observe(state, t)
        if m is not None:
            meas.append(m)
        t += 0.02

    # About 10Hz in 0.6s => ~6 updates.
    assert 4 <= len(meas) <= 7
    assert all(abs(m.yaw) <= math.pi for m in meas)


def test_vision_sensor_fov_range_and_occlusion():
    cfg = load_config()
    vis = VisionSensor(cfg.sensors.vision, cfg.vehicle, seed=7)

    observer = VehicleState(vehicle_id=1, x=0.0, y=0.0, yaw=0.0, v=0.0, delta=0.0)
    target_visible = VehicleState(vehicle_id=2, x=2.2, y=0.0, yaw=0.0, v=0.0, delta=0.0)

    m = vis.observe_rear_hitch(observer, target_visible, [], timestamp=0.1)
    assert m is not None and m.valid

    # Out of FOV.
    target_side = VehicleState(vehicle_id=2, x=0.0, y=2.0, yaw=0.0, v=0.0, delta=0.0)
    m2 = vis.observe_rear_hitch(observer, target_side, [], timestamp=0.2)
    assert m2 is not None and not m2.valid

    # Occluded.
    obs = [Obstacle(x=1.45, y=0.0, width=0.2, height=1.0)]
    m3 = vis.observe_rear_hitch(observer, target_visible, obs, timestamp=0.3)
    assert m3 is not None and not m3.valid
