#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.scenario_support import ScenarioGenerator
from strategy.p2_p4_integrated import IntegratedDemoConfig, IntegratedP2P4Runner


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run P4.5 integration checkpoint experiments.")
    p.add_argument("--seeds-per-subtype", type=int, default=30, help="Number of seeds for each stress subtype.")
    p.add_argument("--duration-s", type=float, default=12.0, help="Simulation duration per case.")
    p.add_argument("--enable-sensor-noise", action="store_true", help="Enable GNSS+vision noise (robustness check).")
    p.add_argument(
        "--output-json",
        type=str,
        default=str(ROOT / "experiments" / "p4_5_integration_checkpoint_results.json"),
        help="Output JSON path.",
    )
    p.add_argument(
        "--output-md",
        type=str,
        default=str(ROOT / "artifacts" / "P4_5_INTEGRATION_CHECKPOINT.md"),
        help="Output markdown report path.",
    )
    return p.parse_args()


def _max_mutual_switches_10s(action_trace: list[dict[str, Any]]) -> int:
    if len(action_trace) <= 1:
        return 0
    switch_ts: list[float] = []
    for i in range(1, len(action_trace)):
        if str(action_trace[i]["action"]) != str(action_trace[i - 1]["action"]):
            switch_ts.append(float(action_trace[i]["t"]))
    if not switch_ts:
        return 0
    best = 0
    for i, t in enumerate(switch_ts):
        c = 0
        j = i
        while j >= 0 and (t - switch_ts[j]) <= 10.0 + 1e-9:
            c += 1
            j -= 1
        best = max(best, c)
    return int(best)


def _has_merge_split_merge(action_trace: list[dict[str, Any]]) -> bool:
    acts = [str(x["action"]) for x in action_trace]
    if "SPLIT" not in acts:
        return False
    i = acts.index("SPLIT")
    return "DOCK" in acts[i + 1 :]


def _run_policy(
    *,
    gen: ScenarioGenerator,
    subtype_list: list[str],
    seeds_per_subtype: int,
    duration_s: float,
    conflict_policy: str,
    enable_sensor_noise: bool,
) -> dict[str, Any]:
    base_seed = 73040
    runs: list[dict[str, Any]] = []
    for si, subtype in enumerate(subtype_list):
        for k in range(seeds_per_subtype):
            seed = base_seed + si * 1000 + k
            case = gen.generate(
                subtype,
                n_vehicles=4,
                seed=seed,
                initial_dispersion_mode="Uniform_Spread",
            )
            demo_cfg = IntegratedDemoConfig(
                duration_s=float(duration_s),
                initial_chain_vehicle_ids=(1, 2, 3),
                pre_split_initial_chain=False,
                free_target_speed=0.0,
                inject_disturbance_for_type_c=False,
                conflict_policy=conflict_policy,
                enable_sensor_noise=bool(enable_sensor_noise),
            )
            out = IntegratedP2P4Runner(
                gen.cfg,
                case,
                policy="integrated",
                seed=seed + 5000,
                demo_cfg=demo_cfg,
            ).run()

            conflict_rows = [x for x in out.arbitration_trace if bool(x["conflict"])]
            conflict_count = len(conflict_rows)
            consistent_count = sum(1 for x in conflict_rows if bool(x["consistent"]))
            inconsistent_count = int(conflict_count - consistent_count)
            max_switch_10s = _max_mutual_switches_10s(out.reconfig_action_trace)
            conflict_failure = bool(conflict_count > 0 and (inconsistent_count > 0 or max_switch_10s > 1))
            run = {
                "scenario_id": out.scenario_id,
                "subtype": str(subtype),
                "seed": int(seed),
                "policy": str(conflict_policy),
                "conflict_count": int(conflict_count),
                "consistent_count": int(consistent_count),
                "inconsistent_count": int(inconsistent_count),
                "max_mutual_switches_10s": int(max_switch_10s),
                "conflict_failure": bool(conflict_failure),
                "merge_split_merge": bool(_has_merge_split_merge(out.reconfig_action_trace)),
                "split_count": int(out.split_count),
                "dock_success_count": int(out.dock_success_count),
                "command_exec_reject": int(out.command_exec_reject),
                "collision_count": int(out.collision_count),
                "done_time_s": float(out.done_time_s),
                # keep traces for audit and plotting
                "arbitration_trace": out.arbitration_trace,
                "reconfig_action_trace": out.reconfig_action_trace,
            }
            runs.append(run)

    n_runs = len(runs)
    conflict_runs = [r for r in runs if int(r["conflict_count"]) > 0]
    no_conflict_runs = [r for r in runs if int(r["conflict_count"]) == 0]
    tot_conf = sum(int(r["conflict_count"]) for r in runs)
    tot_cons = sum(int(r["consistent_count"]) for r in runs)
    consistency_rate = float(1.0 if tot_conf == 0 else (tot_cons / tot_conf))
    max_switch = int(max([int(r["max_mutual_switches_10s"]) for r in runs], default=0))
    failure_rate_conflict = float(
        sum(1 for r in conflict_runs if bool(r["conflict_failure"])) / max(1, len(conflict_runs))
    )
    failure_rate_no_conflict = float(
        sum(1 for r in no_conflict_runs if bool(r["conflict_failure"])) / max(1, len(no_conflict_runs))
    )
    extra_failure_increment = float(max(0.0, failure_rate_conflict - failure_rate_no_conflict))
    hsm_rate = float(sum(1 for r in runs if bool(r["merge_split_merge"])) / max(1, n_runs))

    return {
        "policy": str(conflict_policy),
        "num_runs": int(n_runs),
        "num_conflict_runs": int(len(conflict_runs)),
        "total_conflict_points": int(tot_conf),
        "conflict_consistency_rate": float(consistency_rate),
        "max_mutual_switches_10s": int(max_switch),
        "conflict_failure_rate": float(failure_rate_conflict),
        "no_conflict_failure_rate": float(failure_rate_no_conflict),
        "extra_failure_increment": float(extra_failure_increment),
        "merge_split_merge_rate": float(hsm_rate),
        "runs": runs,
    }


def main() -> None:
    args = parse_args()
    cfg = load_config()
    gen = ScenarioGenerator(cfg)

    # Stress set for conflict closure: Type-C bottleneck-mixed cases that reliably induce Dock/Split conflicts.
    stress_subtypes = ["C1", "C3"]
    # Control set for low-conflict sanity observation.
    control_subtypes = ["C2"]

    split_stress = _run_policy(
        gen=gen,
        subtype_list=stress_subtypes,
        seeds_per_subtype=int(args.seeds_per_subtype),
        duration_s=float(args.duration_s),
        conflict_policy="split_priority",
        enable_sensor_noise=bool(args.enable_sensor_noise),
    )
    dock_stress = _run_policy(
        gen=gen,
        subtype_list=stress_subtypes,
        seeds_per_subtype=int(args.seeds_per_subtype),
        duration_s=float(args.duration_s),
        conflict_policy="dock_priority",
        enable_sensor_noise=bool(args.enable_sensor_noise),
    )
    split_control = _run_policy(
        gen=gen,
        subtype_list=control_subtypes,
        seeds_per_subtype=max(6, int(args.seeds_per_subtype // 3)),
        duration_s=float(args.duration_s),
        conflict_policy="split_priority",
        enable_sensor_noise=bool(args.enable_sensor_noise),
    )

    thresholds = {
        "conflict_consistency_rate_min": 0.99,
        "max_mutual_switches_10s_max": 1,
        "extra_failure_increment_max": 0.02,
        "merge_split_merge_rate_min": 0.95,
    }
    pass_flags = {
        "conflict_consistency_rate": split_stress["conflict_consistency_rate"] >= thresholds["conflict_consistency_rate_min"],
        "max_mutual_switches_10s": split_stress["max_mutual_switches_10s"] <= thresholds["max_mutual_switches_10s_max"],
        "extra_failure_increment": split_stress["extra_failure_increment"] <= thresholds["extra_failure_increment_max"],
        "merge_split_merge_rate": split_stress["merge_split_merge_rate"] >= thresholds["merge_split_merge_rate_min"],
    }
    gate4_5_ready = bool(all(pass_flags.values()))

    result = {
        "config": {
            "duration_s": float(args.duration_s),
            "stress_subtypes": stress_subtypes,
            "control_subtypes": control_subtypes,
            "n_vehicles": 4,
            "initial_dispersion_mode": "Uniform_Spread",
            "initial_chain_vehicle_ids": [1, 2, 3],
            "pre_split_initial_chain": False,
            "free_target_speed": 0.0,
            "seeds_per_subtype": int(args.seeds_per_subtype),
            "enable_sensor_noise": bool(args.enable_sensor_noise),
        },
        "thresholds": thresholds,
        "split_priority_stress": split_stress,
        "dock_priority_stress_ablation": dock_stress,
        "split_priority_control": split_control,
        "pass_flags": pass_flags,
        "gate4_5_ready": bool(gate4_5_ready),
    }

    exp_dir = ROOT / "experiments"
    art_dir = ROOT / "artifacts"
    exp_dir.mkdir(exist_ok=True)
    art_dir.mkdir(exist_ok=True)
    json_path = Path(args.output_json)
    md_path = Path(args.output_md)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    md_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    s = split_stress
    a = dock_stress
    lines = [
        "# P4.5 Integration Checkpoint Report",
        "",
        f"- enable_sensor_noise: {bool(args.enable_sensor_noise)}",
        "## Stress Set (Type C1/C3, split-priority)",
        f"- num_runs: {s['num_runs']}",
        f"- num_conflict_runs: {s['num_conflict_runs']}",
        f"- total_conflict_points: {s['total_conflict_points']}",
        f"- conflict_consistency_rate: {s['conflict_consistency_rate']:.4f}",
        f"- max_mutual_switches_10s: {s['max_mutual_switches_10s']}",
        f"- extra_failure_increment: {s['extra_failure_increment']:.4f}",
        f"- merge_split_merge_rate: {s['merge_split_merge_rate']:.4f}",
        "",
        "## Ablation (dock-priority on conflict)",
        f"- conflict_consistency_rate: {a['conflict_consistency_rate']:.4f}",
        f"- extra_failure_increment: {a['extra_failure_increment']:.4f}",
        f"- merge_split_merge_rate: {a['merge_split_merge_rate']:.4f}",
        "",
        "## Threshold Checks",
        f"- conflict_consistency_rate >= {thresholds['conflict_consistency_rate_min']:.2f}: {pass_flags['conflict_consistency_rate']}",
        f"- max_mutual_switches_10s <= {thresholds['max_mutual_switches_10s_max']}: {pass_flags['max_mutual_switches_10s']}",
        f"- extra_failure_increment <= {thresholds['extra_failure_increment_max']:.2f}: {pass_flags['extra_failure_increment']}",
        f"- merge_split_merge_rate >= {thresholds['merge_split_merge_rate_min']:.2f}: {pass_flags['merge_split_merge_rate']}",
        "",
        f"## Gate-4.5 Conclusion: {'Go' if gate4_5_ready else 'No-Go'}",
        "",
        f"JSON: {json_path}",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("p4_5_results_json", json_path)
    print("p4_5_report_md", md_path)
    print("p4_5_gate_ready", gate4_5_ready)


if __name__ == "__main__":
    main()
