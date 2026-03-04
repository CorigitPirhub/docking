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
from strategy.baseline_scheduler import BaselineScheduler, aggregate_rollouts
from docking.costs import EnergyModelConfig


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run P2 sequence baseline experiments.")
    p.add_argument("--seeds-per-subtype", type=int, default=8, help="Number of seeds for each subtype.")
    p.add_argument("--eta-nominal", type=float, default=0.95, help="P2 nominal train energy factor.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    scheduler = BaselineScheduler(energy_cfg=EnergyModelConfig(eta_nominal=float(args.eta_nominal)))

    subtypes = ["A1", "A2", "A3", "B1", "B2", "B3", "C1", "C2", "C3"]
    modes = ["Clustered_At_A", "Random_Scattered", "Uniform_Spread"]
    rows = []
    records = []

    base_seed = int(cfg.testing.random_seed)
    for si, sub in enumerate(subtypes):
        for k in range(int(args.seeds_per_subtype)):
            seed = base_seed + 10000 * si + k
            mode = modes[k % len(modes)]
            override = {"B2_K": 2 + (k % 3)} if sub == "B2" else None
            case = gen.generate(
                sub,
                n_vehicles=cfg.scenario.representative_vehicle_count,
                seed=seed,
                initial_dispersion_mode=mode,
                overrides=override,
            )
            rollout = scheduler.rollout_case(case)
            rows.append(rollout)
            records.append(
                {
                    "scenario_id": case.scenario_id,
                    "subtype": case.subtype,
                    "mode": mode,
                    "labels": case.labels.to_dict() if case.labels is not None else {},
                    "independent": rollout["independent"].to_dict(),
                    "fixed_sequence": rollout["fixed_sequence"].to_dict(),
                    "adaptive": rollout["adaptive"].to_dict(),
                }
            )

    summary = aggregate_rollouts(rows)
    thresholds = {
        "avg_energy_reduction_vs_independent_min": 0.025,
        "avg_time_reduction_vs_fixed_sequence_min": 0.08,
        "adaptive_command_exec_rate_min": 0.95,
    }
    pass_flags = {
        "energy": summary["avg_energy_reduction_vs_independent"] >= thresholds["avg_energy_reduction_vs_independent_min"],
        "time": summary["avg_time_reduction_vs_fixed_sequence"] >= thresholds["avg_time_reduction_vs_fixed_sequence_min"],
        "exec_rate": summary["adaptive_command_exec_rate"] >= thresholds["adaptive_command_exec_rate_min"],
    }
    gate2_ready = bool(all(pass_flags.values()))

    result = {
        "config": {
            "seeds_per_subtype": int(args.seeds_per_subtype),
            "eta_nominal": float(args.eta_nominal),
            "representative_vehicle_count": int(cfg.scenario.representative_vehicle_count),
        },
        "thresholds": thresholds,
        "summary": summary,
        "pass_flags": pass_flags,
        "gate2_ready": gate2_ready,
        "cases": records,
    }

    exp_dir = ROOT / "experiments"
    art_dir = ROOT / "artifacts"
    exp_dir.mkdir(exist_ok=True)
    art_dir.mkdir(exist_ok=True)

    json_path = exp_dir / "p2_sequence_results.json"
    md_path = art_dir / "P2_SEQUENCE_REPORT.md"
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = [
        "# P2 Sequence Baseline Report",
        "",
        f"- num_cases: {summary['num_cases']}",
        f"- avg_energy_reduction_vs_independent: {summary['avg_energy_reduction_vs_independent']:.4f}",
        f"- avg_time_reduction_vs_fixed_sequence: {summary['avg_time_reduction_vs_fixed_sequence']:.4f}",
        f"- adaptive_command_exec_rate: {summary['adaptive_command_exec_rate']:.4f}",
        f"- gate2_ready: {gate2_ready}",
        "",
        "Threshold checks:",
        f"- energy >= {thresholds['avg_energy_reduction_vs_independent_min']:.4f}: {pass_flags['energy']}",
        f"- time >= {thresholds['avg_time_reduction_vs_fixed_sequence_min']:.4f}: {pass_flags['time']}",
        f"- exec_rate >= {thresholds['adaptive_command_exec_rate_min']:.4f}: {pass_flags['exec_rate']}",
        "",
        f"JSON: {json_path}",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("p2_results_json", json_path)
    print("p2_results_md", md_path)
    print("p2_gate2_ready", gate2_ready)


if __name__ == "__main__":
    main()
