from docking.collision import CollisionEngine
from docking.config import load_config
from docking.types import Obstacle, VehicleState


def test_obstacle_collision_detection():
    cfg = load_config()
    ce = CollisionEngine(cfg.vehicle, cfg.safety)

    car = VehicleState(vehicle_id=1, x=0.0, y=0.0, yaw=0.0, v=0.0, delta=0.0)
    obs_hit = Obstacle(x=0.1, y=0.0, width=1.0, height=1.0)
    obs_free = Obstacle(x=6.0, y=0.0, width=1.0, height=1.0)

    assert ce.collide_vehicle_obstacle(car, obs_hit)
    assert not ce.collide_vehicle_obstacle(car, obs_free)


def test_vehicle_vehicle_collision_detection():
    cfg = load_config()
    ce = CollisionEngine(cfg.vehicle, cfg.safety)

    a = VehicleState(vehicle_id=1, x=0.0, y=0.0, yaw=0.0, v=0.0, delta=0.0)
    b = VehicleState(vehicle_id=2, x=0.4, y=0.0, yaw=0.0, v=0.0, delta=0.0)
    c = VehicleState(vehicle_id=3, x=4.0, y=0.0, yaw=0.0, v=0.0, delta=0.0)

    assert ce.collide_vehicle_vehicle(a, b)
    assert not ce.collide_vehicle_vehicle(a, c)


def test_train_collision_apis_cover_whole_train():
    cfg = load_config()
    ce = CollisionEngine(cfg.vehicle, cfg.safety)

    train = [
        VehicleState(vehicle_id=10, x=0.0, y=0.0, yaw=0.0, v=0.0, delta=0.0),
        VehicleState(vehicle_id=11, x=-1.6, y=0.0, yaw=0.0, v=0.0, delta=0.0),
        VehicleState(vehicle_id=12, x=-3.2, y=0.0, yaw=0.0, v=0.0, delta=0.0),
    ]
    obs_hit_tail = [Obstacle(x=-3.2, y=0.0, width=1.0, height=1.0)]
    obs_far = [Obstacle(x=10.0, y=10.0, width=1.0, height=1.0)]

    assert ce.collide_train_obstacles(train, obs_hit_tail, include_clearance=False)
    assert not ce.collide_train_obstacles(train, obs_far, include_clearance=False)

    # Non-adjacent self collision.
    train[-1].x = 0.0
    train[-1].y = 0.0
    assert ce.collide_train_self(train, include_clearance=False, non_adjacent_only=True)
