import numpy as np

from docking.config import load_config
from docking.scenario_support import ScenarioGenerator, ScenarioValidator


def test_generator_outputs_labels_for_all_subtypes():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    subtypes = ["A1", "A2", "A3", "B1", "B2", "B3", "C1", "C2", "C3"]
    for i, sub in enumerate(subtypes):
        ov = {"B2_K": 2 + (i % 3)} if sub == "B2" else None
        case = gen.generate(sub, seed=cfg.testing.random_seed + i, overrides=ov)
        assert case.labels is not None
        assert case.labels.n_max_pass_global >= 1
        assert len(case.labels.n_max_pass_profile) > 0
        assert case.labels.initial_dispersion_mode in {"Clustered_At_A", "Random_Scattered", "Uniform_Spread"}


def test_bottleneck_types_keep_single_vehicle_feasible():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    for sub in ["B1", "B2", "B3"]:
        ov = {"B2_K": 3} if sub == "B2" else None
        case = gen.generate(sub, seed=77, overrides=ov)
        assert case.labels is not None
        assert case.labels.n_max_pass_global >= 1


def test_initial_dispersion_modes_change_initial_spread():
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    c0 = gen.generate("A1", seed=11, initial_dispersion_mode="Clustered_At_A")
    c1 = gen.generate("A1", seed=11, initial_dispersion_mode="Random_Scattered")
    p0 = np.array([[v.x, v.y] for v in c0.vehicles_init], dtype=float)
    p1 = np.array([[v.x, v.y] for v in c1.vehicles_init], dtype=float)
    d0 = np.linalg.norm(p0 - c0.start_xy[None, :], axis=1).mean()
    d1 = np.linalg.norm(p1 - c1.start_xy[None, :], axis=1).mean()
    assert d1 > d0 + 1.0


def test_batch_validator_reaches_minimum_quality_floor():
    cfg = load_config()
    validator = ScenarioValidator(cfg)
    rep = validator.validate_batch(seeds_per_subtype=4)
    assert rep["overall"]["pass_rate"] >= 0.85
    for sub, m in rep["by_subtype"].items():
        assert m["pass_rate"] >= 0.70, sub


def test_bottleneck_scenarios_are_unavoidable_and_turning():
    cfg = load_config()
    validator = ScenarioValidator(cfg)
    for sub, ov in [("B1", None), ("B2", {"B2_K": 2}), ("B3", None), ("C1", None), ("C2", None), ("C3", None)]:
        case = validator.generator.generate(sub, seed=123, overrides=ov)
        vr = validator.validate_instance(case)
        assert vr.checks.get("baseline_route_exists", False), sub
        assert vr.checks.get("bottleneck_unavoidable", False), sub
        assert vr.checks.get("bottleneck_has_turn", False), sub
