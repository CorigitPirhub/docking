#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.dockbench import family_label, load_representatives, dataset_root
from docking.p_minus1_baselines import FULL_METHOD_IDS, METHOD_SPECS, STRONG_METHOD_IDS

STAGE1_SCRIPT = ROOT / "scripts" / "run_p_minus1_stage1_docking.py"


@dataclass(frozen=True)
class SceneCase:
    case_id: str
    scene_id: str
    family: str
    difficulty: str
    subset_tag: str
    scenario_json: str
    max_time_s: float
    methods: list[str]


@dataclass(frozen=True)
class Stage1Row:
    case_id: str
    scene_id: str
    family_code: str
    difficulty: str
    subset_tag: str
    method_id: str
    family: str
    success: bool
    reason: str
    failure_category: str
    done_time_s: float
    collision: bool
    trajectory_cost: float
    min_clearance_m: float
    fallback_count: int
    visual_loss_count: int
    replan_count: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="P-1 Stage-1 representative DockBench suite.")
    p.add_argument("--dataset-root", type=str, default=str(dataset_root()), help="DockBench dataset root.")
    p.add_argument("--out-dir", type=str, default=str(ROOT / "artifacts"), help="Output directory.")
    p.add_argument("--max-time-s", type=float, default=40.0, help="Global cap for scene horizon.")
    p.add_argument("--skip-gifs", action="store_true", help="Skip GIF generation.")
    return p.parse_args()


def _case_methods(case_id: str) -> list[str]:
    if case_id == "switching":
        return [*FULL_METHOD_IDS, "A_no_belief_gate", "A_no_fallback"]
    if case_id == "funnel":
        return [*FULL_METHOD_IDS, "A_no_funnel_gate", "A_no_micro_maneuver"]
    if case_id == "extension":
        return [*FULL_METHOD_IDS, "A_no_stage", "A_no_stage_no_belief"]
    if case_id == "lane":
        return [*FULL_METHOD_IDS, "A_no_stage", "A_no_corridor_reciprocity", "A_no_lc_hybrid_search"]
    return list(FULL_METHOD_IDS)


def _build_cases(dataset_root_path: Path) -> list[SceneCase]:
    reps = load_representatives(dataset_root_path)
    mapping = [
        ("common", reps["CF_L2"]),
        ("switching", reps["SC_L2"]),
        ("funnel", reps["FC_L2"]),
        ("extension", reps["EC_L2"]),
        ("lane", reps["LC_L2"]),
    ]
    return [
        SceneCase(
            case_id=case_id,
            scene_id=spec.scene_id,
            family=spec.family,
            difficulty=spec.difficulty,
            subset_tag=spec.subset_tag,
            scenario_json=spec.scenario_json,
            max_time_s=float(spec.max_time_s),
            methods=_case_methods(case_id),
        )
        for case_id, spec in mapping
    ]


def _run_case(case: SceneCase, *, out_dir: Path, max_time_s: float, skip_gifs: bool) -> dict[str, Any]:
    cmd = [
        sys.executable,
        str(STAGE1_SCRIPT),
        "--scenario-json",
        str(case.scenario_json),
        "--methods",
        ",".join(case.methods),
        "--max-time-s",
        str(min(float(max_time_s), float(case.max_time_s))),
        "--out-dir",
        str(out_dir),
    ]
    if skip_gifs:
        cmd.append("--skip-gifs")
    subprocess.run(cmd, check=True, cwd=str(ROOT), capture_output=True, text=True)
    payload = json.loads(Path(case.scenario_json).read_text(encoding="utf-8"))
    seed = int(payload["scenario"]["seed"])
    return json.loads((out_dir / f"p_minus1_stage1_results_seed{seed}.json").read_text(encoding="utf-8"))


def _extract_rows(case: SceneCase, result: dict[str, Any]) -> list[Stage1Row]:
    rows: list[Stage1Row] = []
    for run in result["runs"]:
        metrics = run["metrics"]
        method_id = str(metrics["method_id"])
        rows.append(
            Stage1Row(
                case_id=case.case_id,
                scene_id=case.scene_id,
                family_code=case.family,
                difficulty=case.difficulty,
                subset_tag=case.subset_tag,
                method_id=method_id,
                family=METHOD_SPECS.get(method_id, METHOD_SPECS["co_bcfd"]).family,
                success=bool(metrics["success"]),
                reason=str(metrics["reason"]),
                failure_category=str(metrics.get("failure_category", metrics["reason"])),
                done_time_s=float(metrics["done_time_s"]),
                collision=bool(metrics["collision"]),
                trajectory_cost=float(metrics["trajectory_cost"]),
                min_clearance_m=float(metrics["min_clearance_m"]),
                fallback_count=int(metrics.get("fallback_count", 0)),
                visual_loss_count=int(metrics.get("visual_loss_count", 0)),
                replan_count=int(metrics.get("replan_count", 0)),
            )
        )
    return rows


def _metric(rows: list[Stage1Row], case_id: str, method_id: str, attr: str) -> float | None:
    rel = [getattr(row, attr) for row in rows if row.case_id == case_id and row.method_id == method_id]
    if not rel:
        return None
    arr = np.asarray(rel, dtype=float)
    return float(np.mean(arr))


def _success(rows: list[Stage1Row], case_id: str, method_id: str) -> float:
    value = _metric(rows, case_id, method_id, "success")
    return float(value or 0.0)


def _best_strong(rows: list[Stage1Row], case_id: str) -> str:
    best_mid = STRONG_METHOD_IDS[0]
    best_key = (1.0, math.inf)
    for mid in STRONG_METHOD_IDS:
        succ = _success(rows, case_id, mid)
        done = _metric(rows, case_id, mid, "done_time_s")
        key = (-succ, math.inf if done is None else done)
        if key < best_key:
            best_mid = mid
            best_key = key
    return best_mid


def _save_method_compare(rows: list[Stage1Row], out_path: Path, cases: list[SceneCase]) -> None:
    methods = ["co_bcfd", "T_hard_switch", "T_lattice_pbvs", "T_parking_hierarchical", "T_coop_dist_blend"]
    fig, axes = plt.subplots(2, 2, figsize=(14.8, 8.6), dpi=170, sharey=True)
    for ax, case in zip(axes.flat, cases):
        vals = [_success(rows, case.case_id, mid) for mid in methods]
        ax.bar(np.arange(len(methods)), vals, color=["tab:blue" if mid == "co_bcfd" else "tab:gray" for mid in methods])
        ax.set_title(f"{case.case_id}: {case.family}-{case.difficulty}")
        ax.set_ylim(0.0, 1.05)
        ax.set_xticks(np.arange(len(methods)))
        ax.set_xticklabels(methods, rotation=35, ha="right", fontsize=8)
        ax.grid(alpha=0.25, axis="y")
    axes[0, 0].set_ylabel("success")
    axes[1, 0].set_ylabel("success")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _save_mechanism_compare(rows: list[Stage1Row], out_path: Path) -> None:
    cases = [
        ("switching", ["co_bcfd", "A_no_belief_gate", "A_no_fallback"]),
        ("funnel", ["co_bcfd", "A_no_funnel_gate", "A_no_micro_maneuver"]),
        ("extension", ["co_bcfd", "A_no_stage", "A_no_stage_no_belief"]),
    ]
    fig, axes = plt.subplots(1, len(cases), figsize=(15.6, 4.8), dpi=170, sharey=True)
    for ax, (case_id, mids) in zip(axes, cases):
        succ = [_success(rows, case_id, mid) for mid in mids]
        ax.bar(np.arange(len(mids)), succ, color=["tab:blue", "tab:orange", "tab:red"][: len(mids)])
        ax.set_title(case_id)
        ax.set_ylim(0.0, 1.05)
        ax.set_xticks(np.arange(len(mids)))
        ax.set_xticklabels(mids, rotation=30, ha="right", fontsize=8)
        ax.grid(alpha=0.25, axis="y")
    axes[0].set_ylabel("success")
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _write_report(out_path: Path, *, cases: list[SceneCase], rows: list[Stage1Row], plots: dict[str, str], gif_paths: list[str]) -> None:
    checks = {
        "common_best_strong": _best_strong(rows, "common"),
        "common_strong_success": any(_success(rows, "common", mid) > 0.0 for mid in STRONG_METHOD_IDS),
        "switching_signal": bool((_metric(rows, "switching", "co_bcfd", "visual_loss_count") or 0.0) > 0.0 or (_metric(rows, "switching", "co_bcfd", "fallback_count") or 0.0) > 0.0),
        "funnel_ablation_gap": bool(_success(rows, "funnel", "co_bcfd") > _success(rows, "funnel", "A_no_funnel_gate")),
        "extension_stage_needed": bool(_success(rows, "extension", "co_bcfd") > _success(rows, "extension", "A_no_stage")),
    }
    lines = [
        "# P-1 Stage-1 DockBench Representative Suite\n\n",
        "## Representative Scenes\n",
    ]
    for case in cases:
        lines.append(f"- `{case.case_id}`: `{case.scene_id}` / `{case.family}` / `{case.difficulty}` / `{case.subset_tag}`\n")
    lines.append("\n## Validation Checks\n")
    for key, value in checks.items():
        lines.append(f"- `{key}`: `{value}`\n")
    for case in cases:
        lines.append(f"\n## {case.case_id} ({case.family}-{case.difficulty})\n\n")
        lines.append("| method | family | success | T_done[s] | collision | min_clear[m] | traj_cost | fallback | visual_loss | replans |\n")
        lines.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for row in [row for row in rows if row.case_id == case.case_id]:
            lines.append(
                f"| `{row.method_id}` | `{row.family}` | `{row.success}` | {row.done_time_s:.2f} | `{row.collision}` | {row.min_clearance_m:.3f} | {row.trajectory_cost:.3f} | {row.fallback_count} | {row.visual_loss_count} | {row.replan_count} |\n"
            )
    lines.append("\n## Plots\n")
    for key, value in plots.items():
        lines.append(f"- `{key}`: `{value}`\n")
    lines.append("\n## GIFs\n")
    for path in gif_paths:
        lines.append(f"- `{path}`\n")
    out_path.write_text("".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    ds_root = dataset_root(args.dataset_root)
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    cases = _build_cases(ds_root)
    rows: list[Stage1Row] = []
    gif_paths: list[str] = []
    for case in cases:
        result = _run_case(case, out_dir=out_dir, max_time_s=float(args.max_time_s), skip_gifs=bool(args.skip_gifs))
        rows.extend(_extract_rows(case, result))
        if not bool(args.skip_gifs):
            for mid in ["co_bcfd", _best_strong(rows, case.case_id)] + [m for m in case.methods if m.startswith("A_")][:1]:
                payload = json.loads(Path(case.scenario_json).read_text(encoding="utf-8"))
                seed = int(payload["scenario"]["seed"])
                gif_paths.append(str((out_dir / f"p_minus1_stage1_{mid}_seed{seed}.gif").resolve()))
    method_plot = out_dir / "p_minus1_stage1_suite_method_compare.png"
    mech_plot = out_dir / "p_minus1_stage1_suite_mechanism_compare.png"
    _save_method_compare(rows, method_plot, cases)
    _save_mechanism_compare(rows, mech_plot)
    summary = {
        "dataset_root": str(ds_root),
        "cases": [asdict(case) for case in cases],
        "rows": [row.to_dict() for row in rows],
        "plots": {
            "method_compare": str(method_plot.resolve()),
            "mechanism_compare": str(mech_plot.resolve()),
        },
        "representative_gifs": gif_paths,
    }
    json_path = out_dir / "p_minus1_stage1_suite_results.json"
    md_path = out_dir / "P_MINUS1_STAGE1_SUITE_REPORT.md"
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    _write_report(md_path, cases=cases, rows=rows, plots=summary["plots"], gif_paths=gif_paths)
    print("stage1_suite_json", json_path)
    print("stage1_suite_md", md_path)
    print("stage1_suite_plot", method_plot)
    print("stage1_suite_plot", mech_plot)
    for path in gif_paths:
        print("stage1_suite_gif", path)


if __name__ == "__main__":
    main()
