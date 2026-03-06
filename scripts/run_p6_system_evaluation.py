#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.scenario_support import ScenarioGenerator
from strategy.p2_p4_integrated import IntegratedDemoConfig, IntegratedP2P4Runner
from strategy.p6_system_eval import (
    compare_to_baseline,
    compute_feasibility_proxy_accuracy,
    failure_event_counter,
    per_subtype_aggregate,
    summarize_policy,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run P6 system-level evaluation benchmark.")
    p.add_argument("--seeds-per-subtype", type=int, default=4, help="Seed count for each subtype and dispersion mode.")
    p.add_argument(
        "--subtypes",
        type=str,
        default="A1,A2,A3,B1,B2,B3,C1,C2,C3",
        help="Comma-separated subtype list.",
    )
    p.add_argument(
        "--dispersion-modes",
        type=str,
        default="Clustered_At_A,Random_Scattered,Uniform_Spread",
        help="Comma-separated initial dispersion modes.",
    )
    p.add_argument("--duration-s", type=float, default=60.0, help="Simulation duration upper bound for each run.")
    p.add_argument(
        "--time-budget-s",
        type=float,
        default=60.0,
        help="Time budget threshold (s) for done_within_60 metrics (Gate-6 uses 60s even if simulation runs longer).",
    )
    p.add_argument(
        "--leader-target-speed",
        type=float,
        default=1.25,
        help="Leader cruise speed (m/s) used by the runtime executor (affects completion-time budget).",
    )
    p.add_argument(
        "--free-target-speed",
        type=float,
        default=1.25,
        help="Free vehicle cruise speed (m/s) used by the runtime executor.",
    )
    p.add_argument("--n-vehicles", type=int, default=0, help="Vehicle count, 0 means scenario.representative_vehicle_count.")
    p.add_argument("--base-seed", type=int, default=-1, help="Base random seed, -1 means use config.testing.random_seed.")
    p.add_argument("--inject-disturbance", action="store_true", help="Enable P4 disturbance injection for Type-C docking.")
    p.add_argument(
        "--enable-sensor-noise",
        action="store_true",
        help="Enable GNSS + vision noise (robustness evaluation, slower & harder).",
    )
    p.add_argument(
        "--policy-ids",
        type=str,
        default="",
        help="Comma-separated policy_id filter (e.g. 'integrated_split_priority,independent'). Empty means all.",
    )
    p.add_argument("--max-failure-cases", type=int, default=12, help="How many failure cases to keep in result JSON.")
    p.add_argument(
        "--output-json",
        type=str,
        default=str(ROOT / "experiments" / "p6_system_evaluation_results.json"),
        help="Output JSON path.",
    )
    p.add_argument(
        "--output-md",
        type=str,
        default=str(ROOT / "artifacts" / "P6_SYSTEM_EVALUATION_REPORT.md"),
        help="Output markdown report path.",
    )
    return p.parse_args()


def _policy_specs() -> list[dict[str, str]]:
    return [
        {
            "policy_id": "integrated_split_priority",
            "policy": "integrated",
            "conflict_policy": "split_priority",
        },
        {
            "policy_id": "integrated_dock_priority",
            "policy": "integrated",
            "conflict_policy": "dock_priority",
        },
        {
            "policy_id": "independent",
            "policy": "independent",
            "conflict_policy": "split_priority",
        },
    ]


def _build_case_seed(base_seed: int, subtype_idx: int, mode_idx: int, k: int) -> int:
    # IMPORTANT: scenario seeds must be shared across policies for fair comparison.
    return int(base_seed + 10000 * subtype_idx + 1000 * mode_idx + k)


def _run_one(
    cfg,
    gen: ScenarioGenerator,
    *,
    case,
    policy: str,
    conflict_policy: str,
    seed: int,
    duration_s: float,
    time_budget_s: float,
    leader_target_speed: float,
    free_target_speed: float,
    inject_disturbance: bool,
    enable_sensor_noise: bool,
) -> tuple[dict[str, Any], list[Any], list[Any]]:
    demo_cfg = IntegratedDemoConfig(
        duration_s=float(duration_s),
        leader_target_speed=float(leader_target_speed),
        free_target_speed=float(free_target_speed),
        conflict_policy=str(conflict_policy),
        inject_disturbance_for_type_c=bool(inject_disturbance),
        enable_sensor_noise=bool(enable_sensor_noise),
        stop_on_done=True,
        # P6 benchmark does not need per-tick traces; disabling them makes the
        # evaluation 5-10x faster on CPU-bound geometry checks.
        record_history=False,
        record_monitor_trace=False,
    )
    runner = IntegratedP2P4Runner(cfg, case, policy=policy, seed=int(seed), demo_cfg=demo_cfg)
    out = runner.run()

    feas_correct, feas_total = compute_feasibility_proxy_accuracy(runner.engine.snapshots, out.events)
    done_within_60 = bool(out.success and out.done_time_s <= float(time_budget_s) + 1e-9)
    cmd_total = int(out.command_exec_accept + out.command_exec_reject)
    cmd_exec_rate = 1.0 if cmd_total <= 0 else float(out.command_exec_accept / cmd_total)

    row = {
        "scenario_id": str(out.scenario_id),
        "subtype": str(out.subtype),
        "policy": str(policy),
        "conflict_policy": str(conflict_policy),
        "enable_sensor_noise": bool(enable_sensor_noise),
        "success": bool(out.success),
        "done_within_60": bool(done_within_60),
        "done_time_s": float(out.done_time_s),
        "time_budget_s": float(time_budget_s),
        "total_energy": float(out.total_energy),
        "collision_count": int(out.collision_count),
        "dock_success_count": int(out.dock_success_count),
        "split_count": int(out.split_count),
        "p4_interruption_count": int(out.p4_interruption_count),
        "p4_replan_success_count": int(out.p4_replan_success_count),
        "p4_abort_to_independent_count": int(out.p4_abort_to_independent_count),
        "feasibility_abort_count": int(out.feasibility_abort_count),
        "leader_final_train_size": int(out.leader_final_train_size),
        "commands_submitted": int(out.commands_submitted),
        "commands_accepted": int(out.commands_accepted),
        "command_exec_accept": int(out.command_exec_accept),
        "command_exec_reject": int(out.command_exec_reject),
        "command_exec_rate": float(cmd_exec_rate),
        "feasibility_correct": int(feas_correct),
        "feasibility_total": int(feas_total),
    }
    return row, list(out.events), list(runner.engine.snapshots)


def main() -> None:
    args = parse_args()
    cfg = load_config()
    gen = ScenarioGenerator(cfg)

    subtypes = [s.strip() for s in str(args.subtypes).split(",") if s.strip()]
    modes = [m.strip() for m in str(args.dispersion_modes).split(",") if m.strip()]
    policy_specs = _policy_specs()
    policy_filter = [s.strip() for s in str(args.policy_ids).split(",") if s.strip()]
    if policy_filter:
        policy_specs = [p for p in policy_specs if str(p.get("policy_id", "")) in set(policy_filter)]
        if not policy_specs:
            raise SystemExit(f"--policy-ids filter matched no policies: {policy_filter}")

    n_vehicles = int(args.n_vehicles) if int(args.n_vehicles) > 0 else int(cfg.scenario.representative_vehicle_count)
    base_seed = int(args.base_seed) if int(args.base_seed) >= 0 else int(cfg.testing.random_seed)

    runs: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []

    total_runs = len(policy_specs) * len(subtypes) * len(modes) * int(args.seeds_per_subtype)
    run_idx = 0

    for pi, spec in enumerate(policy_specs):
        for si, subtype in enumerate(subtypes):
            for mi, mode in enumerate(modes):
                for k in range(int(args.seeds_per_subtype)):
                    seed = _build_case_seed(base_seed, si, mi, k)
                    override = None
                    if subtype == "B2":
                        override = {"B2_K": 2 + (k % 3)}
                    case = gen.generate(
                        subtype,
                        n_vehicles=n_vehicles,
                        seed=seed,
                        initial_dispersion_mode=mode,
                        overrides=override,
                    )
                    row, events, snapshots = _run_one(
                        cfg,
                        gen,
                        case=case,
                        policy=spec["policy"],
                        conflict_policy=spec["conflict_policy"],
                        seed=seed + 7000,
                        duration_s=float(args.duration_s),
                        time_budget_s=float(args.time_budget_s),
                        leader_target_speed=float(args.leader_target_speed),
                        free_target_speed=float(args.free_target_speed),
                        inject_disturbance=bool(args.inject_disturbance),
                        enable_sensor_noise=bool(args.enable_sensor_noise),
                    )
                    row.update(
                        {
                            "policy_id": str(spec["policy_id"]),
                            "seed": int(seed),
                            "dispersion_mode": str(mode),
                            "n_vehicles": int(n_vehicles),
                        }
                    )
                    runs.append(row)

                    is_fail = (not bool(row["success"])) or (int(row["collision_count"]) > 0)
                    if is_fail and len(failures) < int(args.max_failure_cases):
                        fail_reason = "timeout_or_unfinished"
                        if int(row["collision_count"]) > 0:
                            fail_reason = "collision"
                        elif int(row["feasibility_abort_count"]) > 0 and int(row["dock_success_count"]) == 0:
                            fail_reason = "feasibility_abort_no_dock"
                        failures.append(
                            {
                                "policy_id": str(spec["policy_id"]),
                                "scenario_id": str(row["scenario_id"]),
                                "subtype": str(row["subtype"]),
                                "seed": int(seed),
                                "dispersion_mode": str(mode),
                                "reason": str(fail_reason),
                                "done_time_s": float(row["done_time_s"]),
                                "collision_count": int(row["collision_count"]),
                                "dock_success_count": int(row["dock_success_count"]),
                                "split_count": int(row["split_count"]),
                                "feasibility_abort_count": int(row["feasibility_abort_count"]),
                                "events_count": failure_event_counter(events),
                                "pending_snapshots": int(sum(1 for s in snapshots if bool(getattr(s, "pending_docks", {})))),
                            }
                        )

                    run_idx += 1
                    print(
                        f"[{run_idx}/{total_runs}] {spec['policy_id']} {subtype} {mode} seed={seed} "
                        f"succ={row['success']} t={row['done_time_s']:.2f} col={row['collision_count']}",
                        flush=True,
                    )

    by_policy: dict[str, list[dict[str, Any]]] = {}
    for spec in policy_specs:
        pid = str(spec["policy_id"])
        by_policy[pid] = [r for r in runs if str(r["policy_id"]) == pid]

    aggregates = {pid: summarize_policy(pid, rows).to_dict() for pid, rows in by_policy.items()}
    subtype_breakdown = {pid: per_subtype_aggregate(rows) for pid, rows in by_policy.items()}

    main_pid = "integrated_split_priority"
    if main_pid not in by_policy:
        main_pid = sorted(by_policy.keys())[0]
    main_agg = summarize_policy(main_pid, by_policy[main_pid])

    cmp_ind = None
    if "independent" in by_policy:
        cmp_ind = compare_to_baseline(main_agg, summarize_policy("independent", by_policy["independent"]))
    cmp_dock = None
    if "integrated_dock_priority" in by_policy:
        cmp_dock = compare_to_baseline(
            main_agg, summarize_policy("integrated_dock_priority", by_policy["integrated_dock_priority"])
        )

    thresholds = {
        "success_rate_min": 0.95,
        "done_within_60_rate_min": 0.90,
        "feasibility_accuracy_min": 0.90,
        "collision_total_max": 0,
    }
    pass_flags = {
        "success_rate": bool(main_agg.success_rate >= thresholds["success_rate_min"]),
        "done_within_60_rate": bool(main_agg.done_within_60_rate >= thresholds["done_within_60_rate_min"]),
        "feasibility_accuracy": bool(main_agg.feasibility_accuracy >= thresholds["feasibility_accuracy_min"]),
        "collision_total": bool(main_agg.collision_total <= thresholds["collision_total_max"]),
    }
    gate6_ready = bool(all(pass_flags.values()))

    result = {
        "config": {
            "duration_s": float(args.duration_s),
            "time_budget_s": float(args.time_budget_s),
            "leader_target_speed": float(args.leader_target_speed),
            "free_target_speed": float(args.free_target_speed),
            "n_vehicles": int(n_vehicles),
            "base_seed": int(base_seed),
            "seeds_per_subtype": int(args.seeds_per_subtype),
            "subtypes": subtypes,
            "dispersion_modes": modes,
            "inject_disturbance": bool(args.inject_disturbance),
            "enable_sensor_noise": bool(args.enable_sensor_noise),
            "total_runs": int(total_runs),
            "policy_specs": policy_specs,
        },
        "thresholds": thresholds,
        "aggregates": aggregates,
        "subtype_breakdown": subtype_breakdown,
        "comparisons": {k: v for k, v in {
            "main_vs_independent": cmp_ind,
            "main_vs_integrated_dock_priority": cmp_dock,
        }.items() if v is not None},
        "pass_flags": pass_flags,
        "gate6_ready": bool(gate6_ready),
        "failure_cases": failures,
        "runs": runs,
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

    lines = [
        "# P6 System Evaluation Report",
        "",
        "## Config",
        f"- duration_s: {float(args.duration_s):.2f}",
        f"- time_budget_s: {float(args.time_budget_s):.2f}",
        f"- leader_target_speed: {float(args.leader_target_speed):.2f}",
        f"- free_target_speed: {float(args.free_target_speed):.2f}",
        f"- enable_sensor_noise: {bool(args.enable_sensor_noise)}",
        f"- inject_disturbance: {bool(args.inject_disturbance)}",
        "",
        f"## Main Policy ({main_pid})",
        f"- num_runs: {main_agg.num_runs}",
        f"- success_rate: {main_agg.success_rate:.4f}",
        f"- done_within_60_rate: {main_agg.done_within_60_rate:.4f}",
        f"- feasibility_accuracy(proxy): {main_agg.feasibility_accuracy:.4f} (samples={main_agg.feasibility_samples})",
        f"- collision_total: {main_agg.collision_total}",
        f"- avg_done_time_s: {main_agg.avg_done_time_s:.4f}",
        f"- avg_energy: {main_agg.avg_energy:.4f}",
        "",
        "## Baseline Comparisons",
        f"- vs independent: {json.dumps(cmp_ind, ensure_ascii=False)}",
        f"- vs integrated_dock_priority: {json.dumps(cmp_dock, ensure_ascii=False)}",
        "",
        "## Gate-6 Checks",
        f"- success_rate >= {thresholds['success_rate_min']:.2f}: {pass_flags['success_rate']}",
        f"- done_within_60_rate >= {thresholds['done_within_60_rate_min']:.2f}: {pass_flags['done_within_60_rate']}",
        f"- feasibility_accuracy >= {thresholds['feasibility_accuracy_min']:.2f}: {pass_flags['feasibility_accuracy']}",
        f"- collision_total <= {thresholds['collision_total_max']}: {pass_flags['collision_total']}",
        "",
        f"## Gate-6 Conclusion: {'Go' if gate6_ready else 'No-Go'}",
        "",
        f"JSON: {json_path}",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("p6_results_json", json_path)
    print("p6_report_md", md_path)
    print("p6_gate6_ready", gate6_ready)


if __name__ == "__main__":
    main()
