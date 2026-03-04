#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.scenario_support import ScenarioGenerator
from strategy.p4_recovery import P4Config, PredictiveRecoveryPlanner


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run P4 predictive-docking recovery experiments.")
    p.add_argument("--seeds-per-subtype", type=int, default=12, help="Number of seeds for each subtype.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    planner = PredictiveRecoveryPlanner(P4Config())

    subtypes = ["A1", "A2", "A3", "B1", "B2", "B3", "C1", "C2", "C3"]
    modes = ["Clustered_At_A", "Random_Scattered", "Uniform_Spread"]
    base_seed = int(cfg.testing.random_seed)

    cases = []
    for si, sub in enumerate(subtypes):
        for k in range(int(args.seeds_per_subtype)):
            seed = base_seed + 30000 * si + k
            mode = modes[k % len(modes)]
            ov = {"B2_K": 2 + (k % 3)} if sub == "B2" else None
            case = gen.generate(
                sub,
                n_vehicles=cfg.scenario.representative_vehicle_count,
                seed=seed,
                initial_dispersion_mode=mode,
                overrides=ov,
            )
            cases.append(case)

    summary = planner.evaluate_batch(cases, seed=base_seed + 991)
    thresholds = {
        "recovery_success_rate_min": 0.90,
        "failure_rate_increase_max": 0.05,
        "reenter_3s_rate_min": 0.95,
        "deadlock_total_max": 0,
    }
    overall = summary["overall"]
    pass_flags = {
        "recovery_success_rate": float(overall["recovery_success_rate"]) >= thresholds["recovery_success_rate_min"],
        "failure_rate_increase": float(overall["failure_rate_increase"]) <= thresholds["failure_rate_increase_max"],
        "reenter_3s_rate": float(overall["reenter_3s_rate"]) >= thresholds["reenter_3s_rate_min"],
        "deadlock_total": int(overall["deadlock_total"]) <= thresholds["deadlock_total_max"],
    }
    gate4_ready = bool(all(pass_flags.values()))

    result = {
        "config": {
            "seeds_per_subtype": int(args.seeds_per_subtype),
            "representative_vehicle_count": int(cfg.scenario.representative_vehicle_count),
        },
        "thresholds": thresholds,
        "summary": summary,
        "pass_flags": pass_flags,
        "gate4_ready": gate4_ready,
    }

    exp_dir = ROOT / "experiments"
    art_dir = ROOT / "artifacts"
    exp_dir.mkdir(exist_ok=True)
    art_dir.mkdir(exist_ok=True)

    json_path = exp_dir / "p4_recovery_results.json"
    md_path = art_dir / "P4_RECOVERY_REPORT.md"
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = [
        "# P4 Recovery Report",
        "",
        f"- num_cases: {overall['num_cases']}",
        f"- interruption_total: {overall['interruption_total']}",
        f"- recovery_success_rate: {overall['recovery_success_rate']:.4f}",
        f"- reenter_3s_rate: {overall['reenter_3s_rate']:.4f}",
        f"- failure_rate_baseline: {overall['failure_rate_baseline']:.4f}",
        f"- failure_rate_injected: {overall['failure_rate_injected']:.4f}",
        f"- failure_rate_increase: {overall['failure_rate_increase']:.4f}",
        f"- deadlock_total: {overall['deadlock_total']}",
        f"- gate4_ready: {gate4_ready}",
        "",
        "Threshold checks:",
        f"- recovery_success_rate >= {thresholds['recovery_success_rate_min']:.2f}: {pass_flags['recovery_success_rate']}",
        f"- failure_rate_increase <= {thresholds['failure_rate_increase_max']:.2f}: {pass_flags['failure_rate_increase']}",
        f"- reenter_3s_rate >= {thresholds['reenter_3s_rate_min']:.2f}: {pass_flags['reenter_3s_rate']}",
        f"- deadlock_total <= {thresholds['deadlock_total_max']}: {pass_flags['deadlock_total']}",
        "",
        f"JSON: {json_path}",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("p4_results_json", json_path)
    print("p4_results_md", md_path)
    print("p4_gate4_ready", gate4_ready)


if __name__ == "__main__":
    main()
