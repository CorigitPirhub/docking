from docking.p_minus1_baselines import ABLATION_METHOD_IDS, CAPABILITY_METHOD_IDS, FULL_METHOD_IDS, METHOD_SPECS, STRONG_METHOD_IDS


def test_stage12_method_pool_satisfies_taskbook_minimums() -> None:
    assert len(STRONG_METHOD_IDS) >= 2
    assert len(CAPABILITY_METHOD_IDS) >= 1
    assert len([m for m in FULL_METHOD_IDS if METHOD_SPECS[m].family == "weak"]) >= 3


def test_stage1_ablation_matrix_is_complete() -> None:
    required = {
        "A_no_stage",
        "A_no_belief_gate",
        "A_no_funnel_gate",
        "A_no_micro_maneuver",
        "A_no_fallback",
        "A_no_safety_projection",
        "A_no_stage_no_belief",
        "A_no_funnel_no_micro",
    }
    assert required.issubset(set(ABLATION_METHOD_IDS))
