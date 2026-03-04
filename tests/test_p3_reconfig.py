from docking.config import load_config
from docking.scenario_support import ScenarioGenerator
from strategy.p3_reconfig import AdaptiveSplitDockPlanner, P3Config, nmax_at_s


def test_nmax_query_basic():
    profile = [
        {"s0": 0.0, "s1": 2.0, "n_max": 5},
        {"s0": 2.1, "s1": 3.0, "n_max": 2},
    ]
    assert nmax_at_s(profile, -1.0) == 5
    assert nmax_at_s(profile, 2.5) == 2
    assert nmax_at_s(profile, 5.0) == 2


def test_bottleneck_case_has_zero_overlimit_violation():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    case = gen.generate("B1", seed=123, initial_dispersion_mode="Clustered_At_A")
    planner = AdaptiveSplitDockPlanner(P3Config())
    out = planner.plan_case(case)
    assert out.violation_count == 0
    assert out.collision_count == 0
    assert any(e.event_type == "SPLIT" for e in out.events)


def test_type_c_accuracy_stays_high():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    planner = AdaptiveSplitDockPlanner(P3Config())
    c1 = gen.generate("C1", seed=42, initial_dispersion_mode="Random_Scattered")
    c2 = gen.generate("C3", seed=43, initial_dispersion_mode="Uniform_Spread")
    r1 = planner.plan_case(c1)
    r2 = planner.plan_case(c2)
    assert r1.c_accuracy >= 0.90
    assert r2.c_accuracy >= 0.90


def test_batch_summary_matches_p3_thresholds():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    planner = AdaptiveSplitDockPlanner(P3Config())
    cases = [
        gen.generate("B2", seed=11, initial_dispersion_mode="Clustered_At_A", overrides={"B2_K": 2}),
        gen.generate("B3", seed=12, initial_dispersion_mode="Random_Scattered"),
        gen.generate("C1", seed=13, initial_dispersion_mode="Uniform_Spread"),
        gen.generate("C2", seed=14, initial_dispersion_mode="Clustered_At_A"),
    ]
    rep = planner.evaluate_batch(cases)
    assert rep["type_b"]["violation_total"] == 0
    assert rep["type_c"]["decision_accuracy"] >= 0.90
    assert rep["overall"]["collision_total"] == 0
