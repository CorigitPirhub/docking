import numpy as np

from docking.config import load_config
from docking.scenario_support import ScenarioGenerator
from strategy.baseline_scheduler import BaselineScheduler
from docking.costs import SequenceCostWeights, compute_j_seq


def test_jseq_rewards_energy_saving_under_same_time_risk():
    w = SequenceCostWeights(w_catch=1.0, w_dock=1.0, w_energy=3.0, w_risk=1.0, w_switch=0.0)
    c_hi = compute_j_seq(
        t_catch_s=5.0,
        t_dock_s=2.0,
        delta_energy=-0.30,
        risk=0.1,
        n_switch=1.0,
        weights=w,
    )
    c_lo = compute_j_seq(
        t_catch_s=5.0,
        t_dock_s=2.0,
        delta_energy=-0.05,
        risk=0.1,
        n_switch=1.0,
        weights=w,
    )
    assert c_hi.total_cost < c_lo.total_cost


def test_intercept_point_lies_on_leader_path():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    case = gen.generate("A2", seed=123, initial_dispersion_mode="Random_Scattered")
    sch = BaselineScheduler()
    follower = next(v for v in case.vehicles_init if v.vehicle_id != sch.cfg.leader_id)
    c = sch.select_best_intercept(
        follower=follower,
        train_size_now=1,
        tail_ready_time=0.0,
        case=case,
        conservative=False,
        require_energy_gain=False,
    )
    assert c is not None
    p = np.array(c.point_xy, dtype=float)
    d = np.min(np.linalg.norm(case.path_xy - p[None, :], axis=1))
    assert d <= 0.15


def test_adaptive_policy_improves_against_baselines_in_open_case():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    case = gen.generate("A1", seed=77, initial_dispersion_mode="Random_Scattered")
    sch = BaselineScheduler()
    out = sch.rollout_case(case)
    ind = out["independent"]
    fix = out["fixed_sequence"]
    adp = out["adaptive"]

    assert adp.total_energy <= ind.total_energy
    assert adp.total_time_s <= fix.total_time_s
    assert adp.command_exec_rate >= 0.95


def test_build_docking_commands_contains_intercept_path_coordinate():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    case = gen.generate("B2", seed=31, initial_dispersion_mode="Clustered_At_A", overrides={"B2_K": 3})
    sch = BaselineScheduler()
    adp = sch.rollout_case(case)["adaptive"]
    cmds = sch.build_docking_commands(adp, state_seq=5, source="test_p2")
    for c in cmds:
        assert c.header.state_seq == 5
        assert c.header.source == "test_p2"
        assert c.intercept_path_s is not None
