#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.dockbench import FAMILIES, dataset_root, family_label, label_fidelity, load_manifest, load_representatives
from docking.p_minus1_baselines import STRONG_METHOD_IDS

STAGE1_SCRIPT = ROOT / "scripts" / "run_p_minus1_stage1_docking.py"
_METHODS_BY_FAMILY: dict[str, list[str]] = {
    "CF": ["co_bcfd", "T_lattice_pbvs", "T_parking_hierarchical"],
    "SC": ["co_bcfd", "T_lattice_pbvs", "T_parking_hierarchical", "T_hard_switch", "A_no_belief_gate", "A_no_fallback"],
    "FC": ["co_bcfd", "T_lattice_pbvs", "T_parking_hierarchical", "A_no_funnel_gate", "A_no_micro_maneuver"],
    "EC": ["co_bcfd", "T_lattice_pbvs", "T_parking_hierarchical", "A_no_stage"],
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Validate DockBench-v1 Stage-0 admission rules.")
    p.add_argument("--dataset-root", type=str, default=str(dataset_root()), help="DockBench dataset root.")
    p.add_argument("--out-dir", type=str, default="", help="Audit output directory. Defaults to dataset root.")
    return p.parse_args()


def _run_scene(*, scene_json: str, methods: list[str], out_dir: Path, max_time_s: float, cache_dirs: list[Path] | None = None) -> dict[str, Any]:
    payload = json.loads(Path(scene_json).read_text(encoding="utf-8"))
    seed = int(payload["scenario"]["seed"])
    scenario_signature = payload.get("scenario", {})
    for cache_dir in cache_dirs or []:
        result_path = cache_dir / f"p_minus1_stage1_results_seed{seed}.json"
        if result_path.exists():
            cached = json.loads(result_path.read_text(encoding="utf-8"))
            got = {str(run["metrics"]["method_id"]) for run in cached.get("runs", [])}
            cached_signature = cached.get("scenario", {})
            if scenario_signature == cached_signature and set(methods).issubset(got):
                return cached
    cmd = [
        sys.executable,
        str(STAGE1_SCRIPT),
        "--scenario-json",
        str(scene_json),
        "--methods",
        ",".join(methods),
        "--out-dir",
        str(out_dir),
        "--max-time-s",
        str(float(max_time_s)),
        "--skip-gifs",
    ]
    subprocess.run(cmd, check=True, cwd=str(ROOT), capture_output=True, text=True)
    return json.loads((out_dir / f"p_minus1_stage1_results_seed{seed}.json").read_text(encoding="utf-8"))


def _lookup(result: dict[str, Any], method_id: str) -> dict[str, Any]:
    for run in result["runs"]:
        metrics = run["metrics"]
        if str(metrics["method_id"]) == method_id:
            return metrics
    raise KeyError(method_id)


def _gap_signal(full: dict[str, Any], other: dict[str, Any], *, time_margin: float = 1.0, cost_margin: float = 0.8) -> bool:
    if not other:
        return False
    if bool(full.get("success", False)) and not bool(other.get("success", True)):
        return True
    if bool(other.get("collision", False)):
        return True
    if bool(full.get("success", False)) and bool(other.get("success", False)):
        if float(other.get("done_time_s", 0.0)) >= float(full.get("done_time_s", 0.0)) + float(time_margin):
            return True
        if float(other.get("trajectory_cost", 0.0)) >= float(full.get("trajectory_cost", 0.0)) + float(cost_margin):
            return True
    if int(other.get("fallback_count", 0)) >= int(full.get("fallback_count", 0)) + 1:
        return True
    if int(other.get("visual_loss_count", 0)) >= int(full.get("visual_loss_count", 0)) + 1:
        return True
    return False


def _evaluate_scene(scene: dict[str, Any], result: dict[str, Any]) -> dict[str, Any]:
    family = str(scene["family"]).upper()
    methods = {mid: _lookup(result, mid) for mid in _METHODS_BY_FAMILY[family]}
    descriptors = scene["descriptors"]
    co = methods["co_bcfd"]
    strong_success = any(bool(methods[mid]["success"]) for mid in methods if mid in STRONG_METHOD_IDS)
    checks: dict[str, bool] = {
        "label_match": bool(
            label_fidelity(
                family=family,
                difficulty=str(scene["difficulty"]),
                descriptors=descriptors,
                direct_los_blocked=bool(scene["scenario"]["direct_los_blocked"]),
            )
        ),
        "co_success": bool(co["success"]),
    }
    extras: dict[str, Any] = {
        "strong_baseline_success": bool(strong_success),
    }
    if family == "CF":
        checks["strong_baseline_success"] = bool(strong_success)
        checks["limited_relocation"] = float(scene["scenario"]["leader_relocation_m"]) <= 0.40 + 1e-9
        checks["not_blocked"] = not bool(scene["scenario"]["direct_los_blocked"])
    elif family == "SC":
        checks["blocked"] = bool(scene["scenario"]["direct_los_blocked"])
        checks["switching_signal"] = bool(
            int(co.get("visual_loss_count", 0)) > 0
            or int(co.get("fallback_count", 0)) > 0
            or _gap_signal(co, methods.get("A_no_belief_gate", {}), time_margin=0.6, cost_margin=0.4)
            or _gap_signal(co, methods.get("A_no_fallback", {}), time_margin=0.6, cost_margin=0.4)
        )
    elif family == "FC":
        checks["tight_dock_zone"] = float(descriptors["dock_zone_clearance_m"]) <= 0.95 + 1e-9
        checks["funnel_signal"] = bool(
            _gap_signal(co, methods.get("A_no_funnel_gate", {}), time_margin=0.8, cost_margin=0.5)
            or _gap_signal(co, methods.get("A_no_micro_maneuver", {}), time_margin=0.8, cost_margin=0.5)
            or int(co.get("fallback_count", 0)) >= 1
            or int(co.get("visual_loss_count", 0)) >= 2
            or int(co.get("replan_count", 0)) > 0
        )
    elif family == "EC":
        checks["requires_staging"] = float(scene["scenario"]["leader_relocation_m"]) >= 0.72 - 1e-9
        checks["no_stage_fails"] = not bool(methods["A_no_stage"]["success"])
        checks["blocked"] = bool(scene["scenario"]["direct_los_blocked"])
    checks["cell_pass"] = bool(all(checks.values()))
    return {
        "scene_id": str(scene["scene_id"]),
        "family": family,
        "difficulty": str(scene["difficulty"]),
        "split": str(scene["split"]),
        "checks": checks,
        "extras": extras,
        "metrics": methods,
    }


def _write_report(out_path: Path, *, tuning_rows: list[dict[str, Any]], rep_rows: list[dict[str, Any]], family_admission: dict[str, Any], summary: dict[str, Any]) -> None:
    lines = [
        "# DockBench-v1 Stage-0 Audit Report\n\n",
        "## Summary\n",
        f"- dataset_root: `{summary['dataset_root']}`\n",
        f"- tuning_scene_count: `{summary['tuning_scene_count']}`\n",
        f"- stage0_pass: `{summary['pass']}`\n",
        "\n## Family Admission\n",
    ]
    for family in FAMILIES:
        lines.append(f"- `{family}` / `{family_label(family)}`: `{json.dumps(family_admission[family], ensure_ascii=False)}`\n")
    lines.extend([
        "\n## Tuning Cell Checks\n\n",
        "| scene_id | family | difficulty | pass | highlights |\n",
        "|---|---|---|---:|---|\n",
    ])
    for row in tuning_rows:
        highlights = ", ".join(f"{k}={v}" for k, v in row["checks"].items() if k != "cell_pass")
        lines.append(f"| `{row['scene_id']}` | `{row['family']}` | `{row['difficulty']}` | `{row['checks']['cell_pass']}` | {highlights} |\n")
    lines.extend([
        "\n## Representative Readiness\n\n",
        "| scene_id | family | difficulty | split | pass | highlights |\n",
        "|---|---|---|---|---:|---|\n",
    ])
    for row in rep_rows:
        highlights = ", ".join(f"{k}={v}" for k, v in row["checks"].items() if k != "cell_pass")
        lines.append(f"| `{row['scene_id']}` | `{row['family']}` | `{row['difficulty']}` | `{row['split']}` | `{row['checks']['cell_pass']}` | {highlights} |\n")
    out_path.write_text("".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    ds_root = dataset_root(args.dataset_root)
    out_dir = Path(args.out_dir).expanduser().resolve() if str(args.out_dir).strip() else ds_root
    out_dir.mkdir(parents=True, exist_ok=True)
    stage1_tmp = out_dir / "stage0_stage1_runs"
    stage1_tmp.mkdir(parents=True, exist_ok=True)

    tuning_specs = load_manifest(ds_root, split="tuning")
    tuning_rows: list[dict[str, Any]] = []
    for spec in tuning_specs:
        result = _run_scene(scene_json=spec.scenario_json, methods=_METHODS_BY_FAMILY[spec.family], out_dir=stage1_tmp, max_time_s=spec.max_time_s, cache_dirs=[ds_root / "stage0_full_audit_runs", stage1_tmp])
        scene_payload = json.loads(Path(spec.scenario_json).read_text(encoding="utf-8"))
        tuning_rows.append(_evaluate_scene(scene_payload, result))

    reps = load_representatives(ds_root)
    rep_rows: list[dict[str, Any]] = []
    for key, spec in reps.items():
        result = _run_scene(scene_json=spec.scenario_json, methods=_METHODS_BY_FAMILY[spec.family], out_dir=stage1_tmp, max_time_s=spec.max_time_s, cache_dirs=[ds_root / "stage0_full_audit_runs", stage1_tmp])
        scene_payload = json.loads(Path(spec.scenario_json).read_text(encoding="utf-8"))
        row = _evaluate_scene(scene_payload, result)
        row["representative_key"] = key
        rep_rows.append(row)

    family_admission: dict[str, Any] = {}
    for family in FAMILIES:
        rows = [row for row in tuning_rows if row["family"] == family]
        rep_row = next((row for row in rep_rows if row["family"] == family and row["difficulty"] == "L2"), None)
        strong_available = True if family == "EC" else any(bool(row["extras"]["strong_baseline_success"]) for row in rows)
        family_pass = bool(rows) and all(row["checks"]["cell_pass"] for row in rows) and strong_available and bool(rep_row and rep_row["split"] == "test" and rep_row["checks"]["cell_pass"])
        family_admission[family] = {
            "num_cells": len(rows),
            "num_pass": sum(1 for row in rows if row["checks"]["cell_pass"]),
            "all_tuning_cells_pass": bool(rows) and all(row["checks"]["cell_pass"] for row in rows),
            "strong_baseline_available": bool(strong_available),
            "representative_ready": bool(rep_row and rep_row["split"] == "test" and rep_row["checks"]["cell_pass"]),
            "pass": bool(family_pass),
        }

    summary = {
        "dataset_root": str(ds_root),
        "tuning_scene_count": len(tuning_specs),
        "family_admission": family_admission,
        "tuning_rows": tuning_rows,
        "representatives": rep_rows,
        "pass": bool(all(item["pass"] for item in family_admission.values())),
    }
    json_path = out_dir / "dockbench_v1_stage0_audit.json"
    md_path = out_dir / "DOCKBENCH_V1_STAGE0_AUDIT.md"
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    _write_report(md_path, tuning_rows=tuning_rows, rep_rows=rep_rows, family_admission=family_admission, summary=summary)
    print("stage0_json", json_path)
    print("stage0_md", md_path)
    if not summary["pass"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
