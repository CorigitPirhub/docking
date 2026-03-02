from docking.config import load_config
from docking.simulation import run_docking_case


def test_two_vehicle_docking_multiple_scenarios():
    cfg = load_config()
    results = []
    for seed in range(20):
        r = run_docking_case(cfg, leader_train_size=1, seed=seed, with_obstacles=(seed % 2 == 0), moving_leader=(seed % 3 != 0))
        results.append(r.success)
        if r.success:
            assert r.final_yaw_error_deg < 10.0

    success_rate = sum(results) / len(results)
    assert success_rate >= 0.95


def test_dock_to_two_vehicle_train():
    cfg = load_config()
    results = []
    for seed in range(24, 44):
        r = run_docking_case(cfg, leader_train_size=2, seed=seed, with_obstacles=(seed % 2 == 1), moving_leader=True)
        results.append(r.success)
        if r.success:
            assert r.final_yaw_error_deg < 10.0

    success_rate = sum(results) / len(results)
    assert success_rate >= 0.95


def test_dock_to_three_vehicle_train():
    cfg = load_config()
    results = []
    for seed in range(60, 80):
        r = run_docking_case(cfg, leader_train_size=3, seed=seed, with_obstacles=(seed % 2 == 0), moving_leader=True)
        results.append(r.success)
        if r.success:
            assert r.final_yaw_error_deg < 10.0

    success_rate = sum(results) / len(results)
    assert success_rate >= 0.95
