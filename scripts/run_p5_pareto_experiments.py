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
from strategy.baseline_scheduler import BaselineScheduler
from strategy.p5_multiobjective import (
    compute_baseline_aggregate,
    default_profile_grid,
    evaluate_profile,
    pareto_indices,
    recommend_profile,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run P5 multi-objective Pareto experiments.")
    p.add_argument("--seeds-per-subtype", type=int, default=8, help="Seed count for each subtype.")
    p.add_argument(
        "--subtypes",
        type=str,
        default="A1,A2,A3,B1,B2,B3,C1,C2,C3",
        help="Comma-separated subtype list.",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    modes = ["Clustered_At_A", "Random_Scattered", "Uniform_Spread"]
    subtypes = [s.strip() for s in str(args.subtypes).split(",") if s.strip()]
    base_seed = int(cfg.testing.random_seed)

    cases = []
    for si, sub in enumerate(subtypes):
        for k in range(int(args.seeds_per_subtype)):
            seed = base_seed + 10000 * si + k
            mode = modes[k % len(modes)]
            override = {"B2_K": 2 + (k % 3)} if sub == "B2" else None
            cases.append(
                gen.generate(
                    sub,
                    n_vehicles=cfg.scenario.representative_vehicle_count,
                    seed=seed,
                    initial_dispersion_mode=mode,
                    overrides=override,
                )
            )

    # P2 adaptive baseline as P5 reference baseline.
    baseline_scheduler = BaselineScheduler()
    baseline_rows = [baseline_scheduler.rollout_case(c) for c in cases]
    baseline = compute_baseline_aggregate(baseline_rows)

    profiles = default_profile_grid()
    metrics = [evaluate_profile(profile=p, cases=cases, baseline=baseline) for p in profiles]

    p_idx = pareto_indices(metrics)
    pass_indices = [i for i, m in enumerate(metrics) if bool(m.profile_pass)]
    if pass_indices:
        rec_idx = min(pass_indices, key=lambda i: float(metrics[i].scalar_score))
    else:
        rec_idx = recommend_profile(metrics, p_idx)
    pareto_points = [metrics[i] for i in p_idx]

    pass_profile_cnt = sum(1 for m in metrics if m.profile_pass)
    gate5_ready = bool(len(pareto_points) >= 10 and pass_profile_cnt >= 1)

    result = {
        "config": {
            "seeds_per_subtype": int(args.seeds_per_subtype),
            "subtypes": subtypes,
            "num_cases": int(len(cases)),
            "representative_vehicle_count": int(cfg.scenario.representative_vehicle_count),
            "num_profiles": int(len(profiles)),
        },
        "baseline": baseline.to_dict(),
        "profiles": [p.to_dict() for p in profiles],
        "metrics": [m.to_dict() for m in metrics],
        "pareto_indices": [int(i) for i in p_idx],
        "pareto_count": int(len(pareto_points)),
        "pass_profile_count": int(pass_profile_cnt),
        "recommended_index": None if rec_idx is None else int(rec_idx),
        "recommended_profile_id": None if rec_idx is None else str(metrics[rec_idx].profile_id),
        "gate5_ready": bool(gate5_ready),
    }

    exp_dir = ROOT / "experiments"
    art_dir = ROOT / "artifacts"
    exp_dir.mkdir(exist_ok=True)
    art_dir.mkdir(exist_ok=True)
    json_path = exp_dir / "p5_pareto_results.json"
    md_path = art_dir / "P5_PARETO_REPORT.md"
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    rec_line = "none"
    if rec_idx is not None:
        rm = metrics[rec_idx]
        rec_line = (
            f"{rm.profile_id} (time={rm.avg_time_s:.3f}, energy={rm.avg_energy:.3f}, "
            f"safety={rm.avg_safety:.3f}, energy_vs_ind={rm.energy_reduction_vs_independent:.4f}, "
            f"time_vs_ind={rm.time_improve_vs_independent:.4f})"
        )

    lines = [
        "# P5 Pareto Report",
        "",
        f"- num_cases: {len(cases)}",
        f"- num_profiles: {len(profiles)}",
        f"- pareto_count: {len(pareto_points)}",
        f"- pass_profile_count: {pass_profile_cnt}",
        f"- gate5_ready: {gate5_ready}",
        "",
        "## Baseline (P2 adaptive reference)",
        f"- avg_time_independent: {baseline.avg_time_independent:.4f}",
        f"- avg_energy_independent: {baseline.avg_energy_independent:.4f}",
        f"- avg_time_p2_adaptive: {baseline.avg_time_p2_adaptive:.4f}",
        f"- avg_energy_p2_adaptive: {baseline.avg_energy_p2_adaptive:.4f}",
        "",
        "## Gate-5 checks",
        f"- pareto_count >= 10: {len(pareto_points) >= 10}",
        f"- exists profile with energy_reduction_vs_independent >= 3% and time not worse: {pass_profile_cnt >= 1}",
        "",
        "## Recommended profile",
        f"- {rec_line}",
        "",
        f"JSON: {json_path}",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("p5_results_json", json_path)
    print("p5_report_md", md_path)
    print("p5_gate5_ready", gate5_ready)


if __name__ == "__main__":
    main()
