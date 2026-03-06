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

from docking.dockbench import FAMILIES, load_manifest, dataset_root, family_label
from docking.p_minus1_baselines import STRONG_METHOD_IDS

STAGE1_SCRIPT = ROOT / "scripts" / "run_p_minus1_stage1_docking.py"

_METHODS_BY_FAMILY: dict[str, list[str]] = {
    "CF": ["co_bcfd", "T_lattice_pbvs", "T_parking_hierarchical"],
    "SC": ["co_bcfd", "T_lattice_pbvs", "T_hard_switch", "A_no_belief_gate", "A_no_fallback"],
    "FC": ["co_bcfd", "T_lattice_pbvs", "A_no_funnel_gate", "A_no_micro_maneuver"],
    "EC": ["co_bcfd", "T_lattice_pbvs", "A_no_stage"],
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Validate DockBench-v1 Stage-0 admission rules on tuning split.")
    p.add_argument("--dataset-root", type=str, default=str(dataset_root()), help="DockBench dataset root.")
    p.add_argument("--out-dir", type=str, default="", help="Audit output directory. Defaults to dataset root.")
    p.add_argument("--skip-gifs", action="store_true", help="Skip GIF generation.")
    return p.parse_args()


def _run_scene(*, scene_json: str, methods: list[str], out_dir: Path, max_time_s: float) -> dict[str, Any]:
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
    ]
    cmd.append("--skip-gifs")
    subprocess.run(cmd, check=True, cwd=str(ROOT), capture_output=True, text=True)
    payload = json.loads(Path(scene_json).read_text(encoding="utf-8"))
    seed = int(payload["scenario"]["seed"])
    return json.loads((out_dir / f"p_minus1_stage1_results_seed{seed}.json").read_text(encoding="utf-8"))


def _lookup(result: dict[str, Any], method_id: str) -> dict[str, Any]:
    for run in result["runs"]:
        if str(run["metrics"]["method_id"]) == method_id:
            return run["metrics"]
    raise KeyError(method_id)


def _evaluate_cell(scene: dict[str, Any], result: dict[str, Any]) -> dict[str, Any]:
    family = str(scene["family"]).upper()
    scene_id = str(scene["scene_id"])
    methods = {mid: _lookup(result, mid) for mid in _METHODS_BY_FAMILY[family]}
    strong_success = any(bool(methods[mid]["success"]) for mid in methods if mid in STRONG_METHOD_IDS)
    descriptors = scene["descriptors"]
    co = methods["co_bcfd"]
    checks: dict[str, bool] = {
        "co_success": bool(co["success"]),
        "strong_baseline_success": bool(strong_success),
    }
    if family == "CF":
        checks["limited_relocation"] = float(scene["scenario"]["leader_relocation_m"]) <= 0.40 + 1e-9
        checks["not_blocked"] = not bool(scene["scenario"]["direct_los_blocked"])
    elif family == "SC":
        checks["blocked"] = bool(scene["scenario"]["direct_los_blocked"])
        checks["switching_signal"] = bool(int(co.get("visual_loss_count", 0)) > 0 or int(co.get("fallback_count", 0)) > 0)
    elif family == "FC":
        no_funnel = methods.get("A_no_funnel_gate", {})
        no_micro = methods.get("A_no_micro_maneuver", {})
        checks["tight_dock_zone"] = float(descriptors["dock_zone_clearance_m"]) <= 0.85 + 1e-9
        checks["funnel_signal"] = bool((not bool(no_funnel.get("success", True))) or (not bool(no_micro.get("success", True))) or int(co.get("replan_count", 0)) > 0)
    elif family == "EC":
        no_stage = methods["A_no_stage"]
        checks["requires_staging"] = float(scene["scenario"]["leader_relocation_m"]) >= 0.72 - 1e-9
        checks["no_stage_fails"] = not bool(no_stage["success"])
        checks["blocked"] = bool(scene["scenario"]["direct_los_blocked"])
    checks["cell_pass"] = bool(all(checks.values()))
    return {
        "scene_id": scene_id,
        "family": family,
        "difficulty": str(scene["difficulty"]),
        "checks": checks,
        "metrics": methods,
    }


def _write_report(out_path: Path, *, audit_rows: list[dict[str, Any]], family_admission: dict[str, Any], summary: dict[str, Any]) -> None:
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
    lines.append("\n## Cell Checks\n\n")
    lines.append("| scene_id | family | difficulty | pass | highlights |\n")
    lines.append("|---|---|---|---:|---|\n")
    for row in audit_rows:
        checks = row["checks"]
        highlights = ", ".join(f"{k}={v}" for k, v in checks.items() if k != "cell_pass")
        lines.append(f"| `{row['scene_id']}` | `{row['family']}` | `{row['difficulty']}` | `{checks['cell_pass']}` | {highlights} |\n")
    out_path.write_text("".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    ds_root = dataset_root(args.dataset_root)
    out_dir = Path(args.out_dir).expanduser().resolve() if str(args.out_dir).strip() else ds_root
    out_dir.mkdir(parents=True, exist_ok=True)
    stage1_tmp = out_dir / "stage0_stage1_runs"
    stage1_tmp.mkdir(parents=True, exist_ok=True)

    tuning_specs = load_manifest(ds_root, split="tuning")
    audit_rows: list[dict[str, Any]] = []
    for spec in tuning_specs:
        result = _run_scene(scene_json=spec.scenario_json, methods=_METHODS_BY_FAMILY[spec.family], out_dir=stage1_tmp, max_time_s=spec.max_time_s)
        scene_payload = json.loads(Path(spec.scenario_json).read_text(encoding="utf-8"))
        audit_rows.append(_evaluate_cell(scene_payload, result))

    family_admission: dict[str, Any] = {}
    for family in FAMILIES:
        rows = [row for row in audit_rows if row["family"] == family]
        family_admission[family] = {
            "num_cells": len(rows),
            "num_pass": sum(1 for row in rows if row["checks"]["cell_pass"]),
            "pass": bool(rows) and all(row["checks"]["cell_pass"] for row in rows),
        }
    summary = {
        "dataset_root": str(ds_root),
        "tuning_scene_count": len(tuning_specs),
        "family_admission": family_admission,
        "rows": audit_rows,
        "pass": bool(all(item["pass"] for item in family_admission.values())),
    }
    json_path = out_dir / "dockbench_v1_stage0_audit.json"
    md_path = out_dir / "DOCKBENCH_V1_STAGE0_AUDIT.md"
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    _write_report(md_path, audit_rows=audit_rows, family_admission=family_admission, summary=summary)
    print("stage0_json", json_path)
    print("stage0_md", md_path)
    if not summary["pass"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
