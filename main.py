from __future__ import annotations

import argparse
import json
import os
from collections import Counter
from dataclasses import asdict, replace
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

from sim.simulator import DockingSimulation, SimulationConfig
from viz.plotter import SimulationVisualizer


COMPLEX_MODE_ORDER = ["mode_a", "mode_b", "mode_c"]
DIFFICULTY_ORDER = [1, 2, 3]


def _decision_to_dict(result) -> Optional[dict]:
    if result.decision is None:
        return None
    return {
        "hub": {
            "x": float(result.decision.hub.x),
            "y": float(result.decision.hub.y),
            "heading": float(result.decision.hub.heading),
            "openness": float(result.decision.hub.openness),
            "corridor_clearance": float(result.decision.hub.corridor_clearance),
        },
        "sequence": list(result.decision.sequence),
        "predicted_cost": float(result.decision.predicted_cost),
    }


def _run_simulation(
    seed: int,
    config: SimulationConfig,
    render_artifacts: bool,
    artifact_tag: Optional[str] = None,
) -> dict:
    sim = DockingSimulation(seed=seed, config=config)
    result = sim.run()

    os.makedirs("results", exist_ok=True)

    tag = artifact_tag if artifact_tag is not None else f"seed{seed}"
    timeline_path = os.path.join("results", f"timeline_{tag}.png")
    gif_path = os.path.join("results", f"docking_{tag}.gif")

    gif_saved = False
    if render_artifacts:
        specs = {vid: v.spec for vid, v in result.vehicles.items()}
        viz = SimulationVisualizer(world=result.world, vehicle_specs=specs)
        viz.render_timeline(result.frames, timeline_path)
        try:
            viz.render_animation(result.frames, gif_path, stride=5)
            gif_saved = True
        except Exception:
            gif_saved = False

    report = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "seed": seed,
        "success": result.success,
        "message": result.message,
        "failure_code": result.failure_code,
        "decision": _decision_to_dict(result),
        "metrics": result.metrics,
        "config": asdict(config),
        "artifacts": {
            "timeline_png": timeline_path if render_artifacts else None,
            "animation_gif": gif_path if gif_saved else None,
        },
    }
    return report


def run_once(seed: int, config: Optional[SimulationConfig] = None) -> dict:
    cfg = config if config is not None else SimulationConfig()
    report = _run_simulation(seed=seed, config=cfg, render_artifacts=True)

    report_path = os.path.join("results", f"run_report_seed{seed}.json")
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)

    print(json.dumps(report, indent=2))
    print(f"Report saved to: {report_path}")
    return report


def _auto_tune_config(base: SimulationConfig) -> SimulationConfig:
    tuned = replace(
        base,
        hub_samples=base.hub_samples + 50,
        hub_top_k=min(12, base.hub_top_k + 2),
        min_hub_openness=max(0.22, base.min_hub_openness - 0.07),
    )
    if base.top_planner_type.strip().lower() == "game":
        tuned = replace(
            tuned,
            game_max_rounds=base.game_max_rounds + 6,
            game_lambda_cross=max(3.8, base.game_lambda_cross - 0.4),
            game_lambda_conflict=max(1.8, base.game_lambda_conflict - 0.2),
        )
    else:
        tuned = replace(tuned, mcts_iterations_per_hub=base.mcts_iterations_per_hub + 130)
    return tuned


def _run_with_auto_tune(seed: int, base_config: SimulationConfig) -> Tuple[dict, List[dict], bool]:
    primary = _run_simulation(seed=seed, config=base_config, render_artifacts=False)
    attempts = [
        {
            "attempt": 1,
            "seed": seed,
            "config": asdict(base_config),
            "success": primary["success"],
            "failure_code": primary["failure_code"],
            "message": primary["message"],
        }
    ]
    if primary["success"]:
        return primary, attempts, False

    tuned_config = _auto_tune_config(base_config)
    retry = _run_simulation(seed=seed, config=tuned_config, render_artifacts=False)
    attempts.append(
        {
            "attempt": 2,
            "seed": seed,
            "config": asdict(tuned_config),
            "success": retry["success"],
            "failure_code": retry["failure_code"],
            "message": retry["message"],
        }
    )

    retry["auto_tune_retry"] = True
    return retry, attempts, retry["success"]


def _print_ascii_table(title: str, rows: Sequence[Tuple[str, str]]) -> None:
    col1 = max(len("Metric"), max((len(k) for k, _ in rows), default=0))
    col2 = max(len("Value"), max((len(v) for _, v in rows), default=0))
    sep = f"+{'-' * (col1 + 2)}+{'-' * (col2 + 2)}+"

    print(f"\n{title}")
    print(sep)
    print(f"| {'Metric'.ljust(col1)} | {'Value'.ljust(col2)} |")
    print(sep)
    for k, v in rows:
        print(f"| {k.ljust(col1)} | {v.ljust(col2)} |")
    print(sep)


def _scenario_for_index(index: int, campaign: str) -> Tuple[str, int]:
    campaign = campaign.lower()
    if campaign == "simple":
        return "simple", 1
    if campaign in {"mode_a", "mode_b", "mode_c"}:
        d = DIFFICULTY_ORDER[index % len(DIFFICULTY_ORDER)]
        return campaign, d

    combos = [(m, d) for m in COMPLEX_MODE_ORDER for d in DIFFICULTY_ORDER]
    return combos[index % len(combos)]


def _init_mode_diff_stats() -> Dict[str, Dict]:
    mode_stats = {m: {"runs": 0, "success": 0, "failures": Counter()} for m in ["simple", *COMPLEX_MODE_ORDER]}
    diff_stats = {str(d): {"runs": 0, "success": 0, "failures": Counter()} for d in DIFFICULTY_ORDER}
    return {"mode": mode_stats, "difficulty": diff_stats}


def run_batch(
    seed_start: int,
    batch_size: int,
    base_config: Optional[SimulationConfig] = None,
    campaign: str = "complex_mix",
    tag: str = "batch",
    failed_file: str = "results/failed_seeds.json",
) -> dict:
    cfg = base_config if base_config is not None else SimulationConfig()

    total_runs = max(1, batch_size)
    success_count = 0
    rescued_count = 0
    success_sim_times: List[float] = []
    success_top_times: List[float] = []
    success_opt_gaps: List[float] = []

    final_failure_counter = Counter({"PathPlanningFail": 0, "Collision": 0, "Timeout": 0, "Unknown": 0})
    failed_cases = []

    stat = _init_mode_diff_stats()

    consecutive_failures = 0
    aborted_early = False
    max_consecutive_failures = 3

    print(f"Batch validation started: tag={tag}, campaign={campaign}, N={total_runs}, seed_start={seed_start}")

    for idx in range(total_runs):
        seed = seed_start + idx
        scene_mode, diff = _scenario_for_index(idx, campaign)

        run_cfg = replace(cfg, scene_mode=scene_mode, difficulty_level=diff)
        print(f"[{tag} {idx + 1:03d}/{total_runs:03d}] seed={seed}, mode={scene_mode}, diff={diff}")

        final_report, attempts, rescued = _run_with_auto_tune(seed=seed, base_config=run_cfg)

        stat["mode"][scene_mode]["runs"] += 1
        stat["difficulty"][str(diff)]["runs"] += 1

        if final_report["success"]:
            success_count += 1
            consecutive_failures = 0
            sim_time = float(final_report.get("metrics", {}).get("sim_time", 0.0))
            success_sim_times.append(sim_time)
            top_t = final_report.get("metrics", {}).get("top_decision_time_s")
            opt_gap = final_report.get("metrics", {}).get("top_decision_opt_gap_ratio")
            if isinstance(top_t, (int, float)):
                success_top_times.append(float(top_t))
            if isinstance(opt_gap, (int, float)):
                success_opt_gaps.append(float(opt_gap))
            stat["mode"][scene_mode]["success"] += 1
            stat["difficulty"][str(diff)]["success"] += 1
            if rescued:
                rescued_count += 1
        else:
            consecutive_failures += 1
            code = final_report.get("failure_code") or "Unknown"
            if code not in final_failure_counter:
                code = "Unknown"

            final_failure_counter[code] += 1
            stat["mode"][scene_mode]["failures"][code] += 1
            stat["difficulty"][str(diff)]["failures"][code] += 1
            failed_cases.append(
                {
                    "seed": seed,
                    "mode": scene_mode,
                    "difficulty_level": diff,
                    "final_failure_code": code,
                    "final_message": final_report.get("message"),
                    "attempts": attempts,
                }
            )

            if consecutive_failures > max_consecutive_failures:
                aborted_early = True
                print(
                    f"Early stop triggered: consecutive failures={consecutive_failures} (> {max_consecutive_failures})"
                )
                break

    executed = success_count + len(failed_cases)
    success_rate = (success_count / executed) if executed > 0 else 0.0
    avg_sim_time = (sum(success_sim_times) / len(success_sim_times)) if success_sim_times else 0.0
    avg_top_time = (sum(success_top_times) / len(success_top_times)) if success_top_times else 0.0
    avg_opt_gap = (sum(success_opt_gaps) / len(success_opt_gaps)) if success_opt_gaps else 0.0

    os.makedirs("results", exist_ok=True)
    failed_payload = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "tag": tag,
        "campaign": campaign,
        "requested_batch_size": total_runs,
        "executed_runs": executed,
        "failed_seeds": [c["seed"] for c in failed_cases],
        "cases": failed_cases,
    }
    with open(failed_file, "w", encoding="utf-8") as f:
        json.dump(failed_payload, f, indent=2)

    def _clean_stat(raw: Dict) -> Dict:
        out = {}
        for k, v in raw.items():
            out[k] = {
                "runs": int(v["runs"]),
                "success": int(v["success"]),
                "success_rate": (float(v["success"]) / v["runs"]) if v["runs"] > 0 else 0.0,
                "failures": dict(v["failures"]),
            }
        return out

    summary = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "tag": tag,
        "campaign": campaign,
        "requested_batch_size": total_runs,
        "executed_runs": executed,
        "success_count": success_count,
        "success_rate": success_rate,
        "avg_sim_time_success": avg_sim_time,
        "avg_top_decision_time_success": avg_top_time,
        "avg_top_opt_gap_success": avg_opt_gap,
        "rescued_by_auto_tune": rescued_count,
        "final_failures": dict(final_failure_counter),
        "aborted_early": aborted_early,
        "failed_seeds_path": failed_file,
        "base_config": asdict(cfg),
        "tuned_config": asdict(_auto_tune_config(cfg)),
        "mode_stats": _clean_stat(stat["mode"]),
        "difficulty_stats": _clean_stat(stat["difficulty"]),
    }

    batch_report_path = os.path.join("results", f"{tag}_report.json")
    with open(batch_report_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    rows = [
        ("Tag", tag),
        ("Campaign", campaign),
        ("Requested Runs", str(total_runs)),
        ("Executed Runs", str(executed)),
        ("Success Count", str(success_count)),
        ("Success Rate", f"{100.0 * success_rate:.2f}%"),
        ("Avg sim_time (success)", f"{avg_sim_time:.2f}s"),
        ("Avg top decision time (success)", f"{avg_top_time * 1000.0:.2f}ms"),
        ("Avg top opt gap (success)", f"{100.0 * avg_opt_gap:.2f}%"),
        ("Top Planner", cfg.top_planner_type),
        ("Rescued by Auto-tune", str(rescued_count)),
        ("Fail: PathPlanningFail", str(final_failure_counter["PathPlanningFail"])),
        ("Fail: Collision", str(final_failure_counter["Collision"])),
        ("Fail: Timeout", str(final_failure_counter["Timeout"])),
        ("Fail: Unknown", str(final_failure_counter["Unknown"])),
        ("Aborted Early", str(aborted_early)),
        ("Failed Seeds File", failed_file),
    ]
    _print_ascii_table(f"Batch Statistics ({tag})", rows)
    print(f"Batch summary saved to: {batch_report_path}")

    if aborted_early:
        error_report = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "error": "Consecutive failures exceeded threshold",
            "threshold": max_consecutive_failures,
            "summary": summary,
        }
        error_path = os.path.join("results", f"{tag}_error_report.json")
        with open(error_path, "w", encoding="utf-8") as f:
            json.dump(error_report, f, indent=2)
        print(f"Error report saved to: {error_path}")

    summary["failed_cases"] = failed_cases
    return summary


def _select_representative_failure(failed_cases: Sequence[dict]) -> Optional[dict]:
    if not failed_cases:
        return None
    ordered = sorted(
        failed_cases,
        key=lambda c: (int(c.get("difficulty_level", 0)), int(c.get("seed", 0))),
        reverse=True,
    )
    return ordered[0]


def _failure_root_cause_text(case: Optional[dict]) -> str:
    if case is None:
        return "No failure case available."
    mode = case.get("mode")
    code = case.get("final_failure_code")

    if code == "PathPlanningFail":
        return (
            f"{mode}场景下全局可行走廊过于非凸，若仅依赖欧式启发会盲目扩展。"
            "本次已引入holonomic启发以降低该盲区。"
        )
    if code == "Collision":
        return (
            f"{mode}场景出现碰撞，根因通常是窄通道中的局部几何裕量不足，"
            "以及铰接队列姿态传播后对障碍边界的放大效应。"
        )
    if code == "Timeout":
        return (
            f"{mode}场景搜索空间过大导致超时，说明对接锚点与轨迹搜索耦合仍存在效率瓶颈。"
        )
    return "失败原因未归类，需进一步复现并调试。"


def _render_representative_failure(case: Optional[dict]) -> Optional[dict]:
    if case is None:
        return None

    cfg = SimulationConfig(**case["attempts"][0]["config"])
    seed = int(case["seed"])
    tag = f"failure_seed{seed}_{cfg.scene_mode}_d{cfg.difficulty_level}"
    report = _run_simulation(seed=seed, config=cfg, render_artifacts=True, artifact_tag=tag)

    out_path = os.path.join("results", "representative_failure.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    return report


def _print_mode_difficulty_tables(summary: dict, title_prefix: str) -> None:
    mode_rows = []
    for mode in ["mode_a", "mode_b", "mode_c"]:
        s = summary["mode_stats"].get(mode)
        if s is None or s["runs"] == 0:
            continue
        mode_rows.append((f"{mode} success", f"{100.0 * s['success_rate']:.2f}% ({s['success']}/{s['runs']})"))
    if mode_rows:
        _print_ascii_table(f"{title_prefix} - Mode Breakdown", mode_rows)

    diff_rows = []
    for d in ["1", "2", "3"]:
        s = summary["difficulty_stats"].get(d)
        if s is None or s["runs"] == 0:
            continue
        diff_rows.append((f"difficulty {d}", f"{100.0 * s['success_rate']:.2f}% ({s['success']}/{s['runs']})"))
    if diff_rows:
        _print_ascii_table(f"{title_prefix} - Difficulty Breakdown", diff_rows)


def run_generalization_benchmark(seed_start: int, batch_size: int, base_cfg: SimulationConfig) -> dict:
    # Baseline on simple random scenes.
    simple_n = min(30, max(20, batch_size // 4))
    simple_summary = run_batch(
        seed_start=seed_start + 10000,
        batch_size=simple_n,
        base_config=replace(base_cfg, scene_mode="simple", difficulty_level=1),
        campaign="simple",
        tag="simple_baseline",
        failed_file="results/failed_seeds_simple.json",
    )

    # Full pressure test on structured complex scenes (A/B/C x difficulties).
    complex_summary = run_batch(
        seed_start=seed_start,
        batch_size=batch_size,
        base_config=replace(base_cfg, scene_mode="mode_a", difficulty_level=1),
        campaign="complex_mix",
        tag="complex_stress",
        failed_file="results/failed_seeds.json",
    )

    _print_mode_difficulty_tables(complex_summary, "Complex Stress")

    rep_case = _select_representative_failure(complex_summary.get("failed_cases", []))
    rep_report = _render_representative_failure(rep_case)

    generalization = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "simple_baseline": {
            "runs": simple_summary["executed_runs"],
            "success_rate": simple_summary["success_rate"],
            "avg_sim_time_success": simple_summary["avg_sim_time_success"],
            "avg_top_decision_time_success": simple_summary["avg_top_decision_time_success"],
            "avg_top_opt_gap_success": simple_summary["avg_top_opt_gap_success"],
            "failures": simple_summary["final_failures"],
        },
        "complex_stress": {
            "runs": complex_summary["executed_runs"],
            "success_rate": complex_summary["success_rate"],
            "avg_sim_time_success": complex_summary["avg_sim_time_success"],
            "avg_top_decision_time_success": complex_summary["avg_top_decision_time_success"],
            "avg_top_opt_gap_success": complex_summary["avg_top_opt_gap_success"],
            "failures": complex_summary["final_failures"],
            "mode_stats": complex_summary["mode_stats"],
            "difficulty_stats": complex_summary["difficulty_stats"],
        },
        "delta": {
            "success_rate_drop": simple_summary["success_rate"] - complex_summary["success_rate"],
            "avg_sim_time_change": complex_summary["avg_sim_time_success"] - simple_summary["avg_sim_time_success"],
            "avg_top_decision_time_change": (
                complex_summary["avg_top_decision_time_success"] - simple_summary["avg_top_decision_time_success"]
            ),
            "avg_top_opt_gap_change": (
                complex_summary["avg_top_opt_gap_success"] - simple_summary["avg_top_opt_gap_success"]
            ),
        },
        "representative_failure": {
            "case": rep_case,
            "analysis": _failure_root_cause_text(rep_case),
            "artifacts": rep_report["artifacts"] if rep_report is not None else None,
        },
        "notes": [
            "Complex benchmark covers Mode A/B/C with difficulty levels 1/2/3 by round-robin scheduling.",
            "Auto-tune retry adjusts top-layer decision budget but keeps scene identity fixed.",
            "Hybrid A* now uses obstacle-aware holonomic heuristic to improve maze-like global search robustness.",
        ],
    }

    rows = [
        ("Simple success", f"{100.0 * simple_summary['success_rate']:.2f}%"),
        ("Complex success", f"{100.0 * complex_summary['success_rate']:.2f}%"),
        ("Success drop", f"{100.0 * generalization['delta']['success_rate_drop']:.2f}%"),
        ("Simple avg sim_time", f"{simple_summary['avg_sim_time_success']:.2f}s"),
        ("Complex avg sim_time", f"{complex_summary['avg_sim_time_success']:.2f}s"),
        ("Simple top decision", f"{1000.0 * simple_summary['avg_top_decision_time_success']:.2f}ms"),
        ("Complex top decision", f"{1000.0 * complex_summary['avg_top_decision_time_success']:.2f}ms"),
        ("Simple opt gap", f"{100.0 * simple_summary['avg_top_opt_gap_success']:.2f}%"),
        ("Complex opt gap", f"{100.0 * complex_summary['avg_top_opt_gap_success']:.2f}%"),
        ("Complex failed seeds", ",".join(str(x) for x in [c['seed'] for c in complex_summary['failed_cases']]) or "None"),
    ]
    _print_ascii_table("Generalization Comparison", rows)

    out_path = os.path.join("results", "generalization_report.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(generalization, f, indent=2)
    print(f"Generalization report saved to: {out_path}")

    return generalization


def run_planner_comparison(seed_start: int, batch_size: int, base_cfg: SimulationConfig) -> dict:
    seen_cfg = replace(base_cfg, scene_mode="mode_a", difficulty_level=1)
    mcts_cfg = replace(seen_cfg, top_planner_type="mcts")
    game_cfg = replace(seen_cfg, top_planner_type="game")
    seen_tag_mcts = f"mcts_complex_seen_n{batch_size}"
    seen_tag_game = f"game_complex_seen_n{batch_size}"
    unseen_tag_mcts = f"mcts_complex_unseen_n{batch_size}"
    unseen_tag_game = f"game_complex_unseen_n{batch_size}"

    seen_mcts = run_batch(
        seed_start=seed_start,
        batch_size=batch_size,
        base_config=mcts_cfg,
        campaign="complex_mix",
        tag=seen_tag_mcts,
        failed_file=f"results/failed_seeds_{seen_tag_mcts}.json",
    )
    seen_game = run_batch(
        seed_start=seed_start,
        batch_size=batch_size,
        base_config=game_cfg,
        campaign="complex_mix",
        tag=seen_tag_game,
        failed_file=f"results/failed_seeds_{seen_tag_game}.json",
    )

    unseen_seed_start = seed_start + 50000
    unseen_mcts = run_batch(
        seed_start=unseen_seed_start,
        batch_size=batch_size,
        base_config=mcts_cfg,
        campaign="complex_mix",
        tag=unseen_tag_mcts,
        failed_file=f"results/failed_seeds_{unseen_tag_mcts}.json",
    )
    unseen_game = run_batch(
        seed_start=unseen_seed_start,
        batch_size=batch_size,
        base_config=game_cfg,
        campaign="complex_mix",
        tag=unseen_tag_game,
        failed_file=f"results/failed_seeds_{unseen_tag_game}.json",
    )

    def _metrics_block(summary: dict) -> dict:
        return {
            "success_rate": summary["success_rate"],
            "avg_sim_time_success": summary["avg_sim_time_success"],
            "avg_top_decision_time_success": summary["avg_top_decision_time_success"],
            "avg_top_opt_gap_success": summary["avg_top_opt_gap_success"],
            "final_failures": summary["final_failures"],
            "failed_seeds": [c["seed"] for c in summary.get("failed_cases", [])],
        }

    def _delta(game_block: dict, mcts_block: dict) -> dict:
        return {
            "success_rate_gain": game_block["success_rate"] - mcts_block["success_rate"],
            "avg_sim_time_change": game_block["avg_sim_time_success"] - mcts_block["avg_sim_time_success"],
            "avg_top_decision_time_change": (
                game_block["avg_top_decision_time_success"] - mcts_block["avg_top_decision_time_success"]
            ),
            "avg_top_opt_gap_change": (
                game_block["avg_top_opt_gap_success"] - mcts_block["avg_top_opt_gap_success"]
            ),
        }

    seen_block = {"mcts": _metrics_block(seen_mcts), "game": _metrics_block(seen_game)}
    unseen_block = {"mcts": _metrics_block(unseen_mcts), "game": _metrics_block(unseen_game)}
    seen_block["delta_game_minus_mcts"] = _delta(seen_block["game"], seen_block["mcts"])
    unseen_block["delta_game_minus_mcts"] = _delta(unseen_block["game"], unseen_block["mcts"])

    rows_seen = [
        (
            "Success Rate",
            "MCTS {:.2f}% | GAME {:.2f}% | Δ {:+.2f}%".format(
                100.0 * seen_block["mcts"]["success_rate"],
                100.0 * seen_block["game"]["success_rate"],
                100.0 * seen_block["delta_game_minus_mcts"]["success_rate_gain"],
            ),
        ),
        (
            "Avg sim_time",
            "MCTS {:.2f}s | GAME {:.2f}s | Δ {:+.2f}s".format(
                seen_block["mcts"]["avg_sim_time_success"],
                seen_block["game"]["avg_sim_time_success"],
                seen_block["delta_game_minus_mcts"]["avg_sim_time_change"],
            ),
        ),
        (
            "Top decision time",
            "MCTS {:.2f}ms | GAME {:.2f}ms | Δ {:+.2f}ms".format(
                1000.0 * seen_block["mcts"]["avg_top_decision_time_success"],
                1000.0 * seen_block["game"]["avg_top_decision_time_success"],
                1000.0 * seen_block["delta_game_minus_mcts"]["avg_top_decision_time_change"],
            ),
        ),
        (
            "Top opt gap",
            "MCTS {:.2f}% | GAME {:.2f}% | Δ {:+.2f}%".format(
                100.0 * seen_block["mcts"]["avg_top_opt_gap_success"],
                100.0 * seen_block["game"]["avg_top_opt_gap_success"],
                100.0 * seen_block["delta_game_minus_mcts"]["avg_top_opt_gap_change"],
            ),
        ),
    ]
    _print_ascii_table("Innovation Comparison (Seen Complex Scenes)", rows_seen)

    rows_unseen = [
        (
            "Success Rate",
            "MCTS {:.2f}% | GAME {:.2f}% | Δ {:+.2f}%".format(
                100.0 * unseen_block["mcts"]["success_rate"],
                100.0 * unseen_block["game"]["success_rate"],
                100.0 * unseen_block["delta_game_minus_mcts"]["success_rate_gain"],
            ),
        ),
        (
            "Avg sim_time",
            "MCTS {:.2f}s | GAME {:.2f}s | Δ {:+.2f}s".format(
                unseen_block["mcts"]["avg_sim_time_success"],
                unseen_block["game"]["avg_sim_time_success"],
                unseen_block["delta_game_minus_mcts"]["avg_sim_time_change"],
            ),
        ),
        (
            "Top decision time",
            "MCTS {:.2f}ms | GAME {:.2f}ms | Δ {:+.2f}ms".format(
                1000.0 * unseen_block["mcts"]["avg_top_decision_time_success"],
                1000.0 * unseen_block["game"]["avg_top_decision_time_success"],
                1000.0 * unseen_block["delta_game_minus_mcts"]["avg_top_decision_time_change"],
            ),
        ),
        (
            "Top opt gap",
            "MCTS {:.2f}% | GAME {:.2f}% | Δ {:+.2f}%".format(
                100.0 * unseen_block["mcts"]["avg_top_opt_gap_success"],
                100.0 * unseen_block["game"]["avg_top_opt_gap_success"],
                100.0 * unseen_block["delta_game_minus_mcts"]["avg_top_opt_gap_change"],
            ),
        ),
    ]
    _print_ascii_table("Innovation Comparison (Unseen Complex Scenes)", rows_unseen)

    report = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "batch_size": batch_size,
        "seed_start_seen": seed_start,
        "seed_start_unseen": unseen_seed_start,
        "seen": seen_block,
        "unseen": unseen_block,
    }
    out_path = os.path.join("results", f"innovation_compare_n{batch_size}.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    print(f"Innovation comparison saved to: {out_path}")
    if batch_size >= 100:
        canonical_path = os.path.join("results", "innovation_compare.json")
        with open(canonical_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)
        print(f"Canonical innovation report updated: {canonical_path}")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="Multi-Ackermann dynamic docking simulator")
    parser.add_argument("--seed", type=int, default=5, help="random seed")
    parser.add_argument(
        "--retry",
        type=int,
        default=3,
        help="number of seeds to try until a successful run is found",
    )
    parser.add_argument(
        "--batch",
        type=int,
        nargs="?",
        const=20,
        default=0,
        help="batch validation mode; pass N (or use flag alone for N=20)",
    )
    parser.add_argument(
        "--planner",
        type=str,
        default="mcts",
        choices=["mcts", "game"],
        help="top-level docking planner type",
    )
    parser.add_argument(
        "--compare-planners",
        action="store_true",
        help="run MCTS baseline vs game-theory planner comparison on identical seeds",
    )
    args = parser.parse_args()

    base_cfg = SimulationConfig(
        mcts_iterations_per_hub=300,
        hub_samples=140,
        hub_top_k=7,
        min_hub_openness=0.35,
        obstacle_count=9,
        scene_mode="simple",
        difficulty_level=1,
        top_planner_type=args.planner,
        game_max_rounds=18,
        game_lambda_cross=5.0,
        game_lambda_conflict=2.5,
        game_lambda_order_reg=0.55,
    )

    if args.batch > 0:
        if args.compare_planners:
            run_planner_comparison(seed_start=args.seed, batch_size=args.batch, base_cfg=base_cfg)
        else:
            run_generalization_benchmark(seed_start=args.seed, batch_size=args.batch, base_cfg=base_cfg)
        return

    last = None
    for k in range(max(args.retry, 1)):
        seed = args.seed + k
        print(f"\n=== Simulation attempt {k + 1}, seed={seed} ===")
        rep = run_once(seed=seed, config=base_cfg)
        last = rep
        if rep["success"]:
            break

    if last is not None and not last["success"]:
        raise SystemExit("Simulation did not succeed in all retry attempts.")


if __name__ == "__main__":
    main()
