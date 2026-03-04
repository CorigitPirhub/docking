from docking.config import load_config
from docking.scenario_support import ScenarioGenerator
from strategy.p2_p4_integrated import IntegratedDemoConfig, IntegratedP2P4Runner
from strategy.p3_reconfig import nmax_at_s


def _leader_train_size(edges: list[list[int]], head: int = 1) -> int:
    child: dict[int, int] = {}
    for a, b in edges:
        child[int(a)] = int(b)
    n = 1
    cur = int(head)
    while cur in child:
        n += 1
        cur = child[cur]
        if n > 1000:
            break
    return int(n)


def _assert_no_nmax_violations(case, out) -> None:
    assert case.labels is not None
    profile = case.labels.n_max_pass_profile
    for row in out.monitor_trace:
        leader_s = float(row["leader_s"])
        nmax = int(nmax_at_s(profile, leader_s))
        size = _leader_train_size(list(row["edges"]), head=1)
        assert size <= nmax


def test_case_a_p2_docking_demo_pass():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    case = gen.generate("A2", n_vehicles=2, seed=51042, initial_dispersion_mode="Clustered_At_A")
    out = IntegratedP2P4Runner(
        cfg,
        case,
        policy="integrated",
        seed=2001,
        demo_cfg=IntegratedDemoConfig(duration_s=60.0, inject_disturbance_for_type_c=False),
    ).run()
    assert out.dock_success_count >= 1
    assert out.collision_count == 0
    assert out.done_time_s < 60.0


def test_case_a_with_sensor_noise_pass():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    case = gen.generate("A2", n_vehicles=2, seed=51042, initial_dispersion_mode="Clustered_At_A")
    out = IntegratedP2P4Runner(
        cfg,
        case,
        policy="integrated",
        seed=2011,
        demo_cfg=IntegratedDemoConfig(duration_s=60.0, inject_disturbance_for_type_c=False, enable_sensor_noise=True),
    ).run()
    assert out.dock_success_count >= 1
    assert out.collision_count == 0
    assert out.done_time_s < 60.0


def test_case_b_p3_split_demo_pass():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    case = gen.generate("B1", n_vehicles=2, seed=62042, initial_dispersion_mode="Uniform_Spread")
    out = IntegratedP2P4Runner(
        cfg,
        case,
        policy="integrated",
        seed=2002,
        demo_cfg=IntegratedDemoConfig(
            duration_s=60.0,
            initial_chain_vehicle_ids=(1, 2),
            pre_split_initial_chain=False,
            free_target_speed=0.85,
            inject_disturbance_for_type_c=False,
        ),
    ).run()
    _assert_no_nmax_violations(case, out)
    assert out.success
    assert out.split_count >= 1
    assert out.collision_count == 0
    assert out.done_time_s < 60.0


def test_case_b_with_sensor_noise_pass():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    case = gen.generate("B1", n_vehicles=2, seed=62042, initial_dispersion_mode="Uniform_Spread")
    out = IntegratedP2P4Runner(
        cfg,
        case,
        policy="integrated",
        seed=2012,
        demo_cfg=IntegratedDemoConfig(
            duration_s=60.0,
            initial_chain_vehicle_ids=(1, 2),
            pre_split_initial_chain=False,
            free_target_speed=0.85,
            inject_disturbance_for_type_c=False,
            enable_sensor_noise=True,
        ),
    ).run()
    _assert_no_nmax_violations(case, out)
    assert out.success
    assert out.split_count >= 1
    assert out.collision_count == 0
    assert out.done_time_s < 60.0


def test_case_c_p4_recovery_demo_pass():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    # Recovery demo runs on the stable A2 geometry with forced disturbance injection.
    case = gen.generate("A2", n_vehicles=2, seed=51042, initial_dispersion_mode="Clustered_At_A")
    out = IntegratedP2P4Runner(
        cfg,
        case,
        policy="integrated",
        seed=2003,
        demo_cfg=IntegratedDemoConfig(
            duration_s=60.0,
            inject_disturbance_for_type_c=True,
            inject_disturbance_all_types=True,
            disturbance_after_docking_s=1.0,
            disturbance_vision_prob=0.9,
            max_retry_per_vehicle=2,
            replan_delay_s=0.5,
        ),
    ).run()
    assert out.p4_interruption_count >= 1
    assert out.p4_replan_success_count >= 1
    assert out.dock_success_count >= 1
    assert out.collision_count == 0
