from strategy.p5_multiobjective import (
    default_profile_grid,
    pareto_indices,
    profile_from_tradeoff,
)


def test_profile_factory_outputs_valid_ranges():
    p = profile_from_tradeoff(profile_id="x", w_time=0.8, w_energy=0.6, w_safety=0.4)
    assert p.profile_id == "x"
    assert 0.90 <= p.energy_cfg.eta_nominal <= 0.995
    assert p.scheduler_cfg.chase_speed_mps >= p.scheduler_cfg.leader_speed_mps - 0.3
    assert p.seq_weights.w_energy > 0.0
    assert p.objective.w_time > 0.0


def test_default_profile_grid_has_sufficient_candidates():
    g = default_profile_grid()
    assert len(g) >= 30
    ids = {p.profile_id for p in g}
    assert len(ids) == len(g)


def test_pareto_indices_basic():
    class M:
        def __init__(self, t: float, e: float, s: float):
            self.avg_time_s = t
            self.avg_energy = e
            self.avg_safety = s

    pts = [
        M(10.0, 10.0, 10.0),  # dominated
        M(9.0, 10.0, 10.0),  # frontier
        M(10.0, 9.0, 10.0),  # frontier
        M(10.0, 10.0, 9.0),  # frontier
        M(9.5, 9.5, 9.5),  # frontier
    ]
    idx = set(pareto_indices(pts))
    assert 0 not in idx
    assert {1, 2, 3, 4}.issubset(idx)

