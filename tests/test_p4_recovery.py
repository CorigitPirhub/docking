from docking.config import load_config
from docking.scenario_support import ScenarioGenerator
from strategy.p4_recovery import P4Config, PredictiveRecoveryPlanner


def test_intercept_candidates_are_leader_path_points():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    case = gen.generate("A2", seed=101, initial_dispersion_mode="Random_Scattered")
    planner = PredictiveRecoveryPlanner(P4Config())
    follower = next(v for v in case.vehicles_init if v.vehicle_id != planner.cfg.leader_id)
    ranked = planner.rank_candidates(case=case, follower=follower)
    assert ranked
    p = ranked[0].point_xy
    d = min(((xy[0] - p[0]) ** 2 + (xy[1] - p[1]) ** 2) ** 0.5 for xy in case.path_xy)
    assert d <= 0.15


def test_interruption_recovery_reenters_executable_within_3s():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    case = gen.generate("C3", seed=202, initial_dispersion_mode="Uniform_Spread")
    p4cfg = P4Config(base_interrupt_prob=1.0)
    planner = PredictiveRecoveryPlanner(p4cfg)
    out = planner.plan_case(case, inject_interruptions=True, seed=77)
    assert out.interruption_count >= 1
    assert out.recovery_success_rate >= 0.90
    assert out.reenter_3s_rate >= 0.95


def test_fallback_keeps_no_deadlock_progression():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    case = gen.generate("B1", seed=303, initial_dispersion_mode="Clustered_At_A")
    planner = PredictiveRecoveryPlanner(P4Config(base_interrupt_prob=1.0))
    out = planner.plan_case(case, inject_interruptions=True, seed=99)
    assert out.deadlock_count == 0
    assert out.failure_count == 0


def test_batch_summary_meets_gate4_thresholds():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    planner = PredictiveRecoveryPlanner(P4Config(base_interrupt_prob=0.65))
    cases = [
        gen.generate("A1", seed=401, initial_dispersion_mode="Clustered_At_A"),
        gen.generate("B2", seed=402, initial_dispersion_mode="Random_Scattered", overrides={"B2_K": 2}),
        gen.generate("C1", seed=403, initial_dispersion_mode="Uniform_Spread"),
        gen.generate("C3", seed=404, initial_dispersion_mode="Random_Scattered"),
    ]
    rep = planner.evaluate_batch(cases, seed=505)
    overall = rep["overall"]
    assert overall["recovery_success_rate"] >= 0.90
    assert overall["reenter_3s_rate"] >= 0.95
    assert overall["failure_rate_increase"] <= 0.05
    assert overall["deadlock_total"] == 0
