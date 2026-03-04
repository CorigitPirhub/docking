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
from strategy.p3_reconfig import AdaptiveSplitDockPlanner, P3Config


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run P3 reconfiguration experiments.")
    p.add_argument("--seeds-per-subtype", type=int, default=12, help="Number of seeds for each B/C subtype.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    planner = AdaptiveSplitDockPlanner(P3Config())

    subtypes = ["B1", "B2", "B3", "C1", "C2", "C3"]
    modes = ["Clustered_At_A", "Random_Scattered", "Uniform_Spread"]
    base_seed = int(cfg.testing.random_seed)

    cases = []
    for si, sub in enumerate(subtypes):
        for k in range(int(args.seeds_per_subtype)):
            seed = base_seed + 20000 * si + k
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

    report = planner.evaluate_batch(cases)
    thresholds = {
        "type_b_violation_total_max": 0,
        "type_c_decision_accuracy_min": 0.90,
        "collision_total_max": 0,
    }
    pass_flags = {
        "type_b_violation": report["type_b"]["violation_total"] <= thresholds["type_b_violation_total_max"],
        "type_c_accuracy": report["type_c"]["decision_accuracy"] >= thresholds["type_c_decision_accuracy_min"],
        "collision": report["overall"]["collision_total"] <= thresholds["collision_total_max"],
    }
    gate3_ready = bool(all(pass_flags.values()))

    result = {
        "config": {
            "seeds_per_subtype": int(args.seeds_per_subtype),
            "representative_vehicle_count": int(cfg.scenario.representative_vehicle_count),
        },
        "thresholds": thresholds,
        "summary": report,
        "pass_flags": pass_flags,
        "gate3_ready": gate3_ready,
    }

    exp_dir = ROOT / "experiments"
    art_dir = ROOT / "artifacts"
    exp_dir.mkdir(exist_ok=True)
    art_dir.mkdir(exist_ok=True)
    json_path = exp_dir / "p3_reconfig_results.json"
    md_path = art_dir / "P3_RECONFIG_REPORT.md"
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = [
        "# P3 Reconfiguration Report",
        "",
        f"- num_cases: {report['overall']['num_cases']}",
        f"- type_b_violation_total: {report['type_b']['violation_total']}",
        f"- type_c_decision_accuracy: {report['type_c']['decision_accuracy']:.4f}",
        f"- collision_total: {report['overall']['collision_total']}",
        f"- gate3_ready: {gate3_ready}",
        "",
        "Threshold checks:",
        f"- type_b_violation_total <= {thresholds['type_b_violation_total_max']}: {pass_flags['type_b_violation']}",
        f"- type_c_decision_accuracy >= {thresholds['type_c_decision_accuracy_min']:.2f}: {pass_flags['type_c_accuracy']}",
        f"- collision_total <= {thresholds['collision_total_max']}: {pass_flags['collision']}",
        "",
        f"JSON: {json_path}",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("p3_results_json", json_path)
    print("p3_results_md", md_path)
    print("p3_gate3_ready", gate3_ready)


if __name__ == "__main__":
    main()
