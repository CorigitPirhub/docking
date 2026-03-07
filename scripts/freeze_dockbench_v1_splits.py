#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.dockbench import DIFFICULTIES, FAMILIES, DockBenchSceneSpec, dataset_root, family_label, label_fidelity, save_manifest_files
from docking.p_minus1_baselines import STRONG_METHOD_IDS

STAGE1_SCRIPT = ROOT / "scripts" / "run_p_minus1_stage1_docking.py"
METHODS_BY_FAMILY: dict[str, list[str]] = {
    "CF": ["co_bcfd", "T_lattice_pbvs", "T_parking_hierarchical"],
    "SC": ["co_bcfd", "T_lattice_pbvs", "T_parking_hierarchical", "T_hard_switch", "A_no_belief_gate", "A_no_fallback"],
    "FC": ["co_bcfd", "T_lattice_pbvs", "T_parking_hierarchical", "A_no_funnel_gate", "A_no_micro_maneuver"],
    "EC": ["co_bcfd", "T_lattice_pbvs", "T_parking_hierarchical", "A_no_stage"],
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Freeze DockBench-v1 tuning/test/challenge splits via support-aware audit.")
    p.add_argument("--dataset-root", type=str, default=str(dataset_root()), help="DockBench dataset root.")
    p.add_argument("--workers", type=int, default=4, help="Parallel scene workers.")
    return p.parse_args()


def _run_scene(spec: DockBenchSceneSpec, out_dir: Path) -> dict[str, Any]:
    payload = json.loads(Path(spec.scenario_json).read_text(encoding="utf-8"))
    seed = int(payload["scenario"]["seed"])
    result_path = out_dir / f"p_minus1_stage1_results_seed{seed}.json"
    if result_path.exists():
        cached = json.loads(result_path.read_text(encoding="utf-8"))
        got = {str(run["metrics"]["method_id"]) for run in cached.get("runs", [])}
        if cached.get("scenario", {}) == payload.get("scenario", {}) and set(METHODS_BY_FAMILY[spec.family]).issubset(got):
            return cached
    cmd = [
        sys.executable,
        str(STAGE1_SCRIPT),
        "--scenario-json",
        str(spec.scenario_json),
        "--methods",
        ",".join(METHODS_BY_FAMILY[spec.family]),
        "--out-dir",
        str(out_dir),
        "--max-time-s",
        str(float(spec.max_time_s)),
        "--skip-gifs",
    ]
    subprocess.run(cmd, check=True, cwd=str(ROOT), capture_output=True, text=True)
    return json.loads(result_path.read_text(encoding="utf-8"))


def _metric(result: dict[str, Any], method_id: str) -> dict[str, Any]:
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


def _scene_score(scene: dict[str, Any], result: dict[str, Any]) -> dict[str, Any]:
    family = str(scene["family"])
    metrics = {mid: _metric(result, mid) for mid in METHODS_BY_FAMILY[family]}
    co = metrics["co_bcfd"]
    strong_success = any(bool(metrics[mid]["success"]) for mid in metrics if mid in STRONG_METHOD_IDS)
    descriptors = scene["descriptors"]
    label_match = bool(label_fidelity(family=family, difficulty=str(scene["difficulty"]), descriptors=descriptors, direct_los_blocked=bool(scene["scenario"]["direct_los_blocked"])))
    if family == "CF":
        mechanism = int(not scene["scenario"]["direct_los_blocked"])
        score = 10 * int(co["success"]) + 5 * int(strong_success) + 2 * mechanism
    elif family == "SC":
        mechanism = int(
            int(co.get("visual_loss_count", 0)) > 0
            or int(co.get("fallback_count", 0)) > 0
            or _gap_signal(co, metrics.get("A_no_belief_gate", {}), time_margin=0.6, cost_margin=0.4)
            or _gap_signal(co, metrics.get("A_no_fallback", {}), time_margin=0.6, cost_margin=0.4)
        )
        score = 10 * int(co["success"]) + 4 * int(strong_success) + 3 * mechanism + 2 * int(scene["scenario"]["direct_los_blocked"])
    elif family == "FC":
        mechanism = int(
            _gap_signal(co, metrics.get("A_no_funnel_gate", {}), time_margin=0.8, cost_margin=0.5)
            or _gap_signal(co, metrics.get("A_no_micro_maneuver", {}), time_margin=0.8, cost_margin=0.5)
            or int(co.get("fallback_count", 0)) >= 1
            or int(co.get("visual_loss_count", 0)) >= 2
            or int(co.get("replan_count", 0)) > 0
        )
        score = 10 * int(co["success"]) + 4 * int(strong_success) + 4 * mechanism + 2 * int(float(descriptors["dock_zone_clearance_m"]) <= 0.95 + 1e-9)
    else:
        mechanism = int(not metrics["A_no_stage"]["success"])
        score = 12 * int(co["success"]) + 4 * mechanism + 2 * int(scene["scenario"]["leader_relocation_m"] >= 0.72 - 1e-9)
    time_penalty = float(co["done_time_s"]) if bool(co["success"]) else 999.0
    hardness = float(descriptors["occlusion_index"] + descriptors["staging_shift_required_m"] + descriptors["detour_factor"] + max(0.0, 1.0 - descriptors["dock_zone_clearance_m"]))
    return {
        "scene_id": scene["scene_id"],
        "family": family,
        "difficulty": scene["difficulty"],
        "score": float(score),
        "label_match": bool(label_match),
        "mechanism_signal": bool(mechanism),
        "co_success": bool(co["success"]),
        "strong_success": bool(strong_success),
        "co_time": time_penalty,
        "hardness": hardness,
        "metrics": metrics,
    }


def _rewrite_scene_split(scene_path: Path, split: str) -> None:
    payload = json.loads(scene_path.read_text(encoding="utf-8"))
    payload["split"] = str(split)
    scene_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> None:
    args = parse_args()
    ds_root = dataset_root(args.dataset_root)
    manifest_path = ds_root / "dockbench_v1_manifest.json"
    manifest = [DockBenchSceneSpec(**item) for item in json.loads(manifest_path.read_text(encoding="utf-8"))]
    stage1_tmp = ds_root / "stage0_full_audit_runs"
    stage1_tmp.mkdir(parents=True, exist_ok=True)

    grouped: dict[tuple[str, str], list[DockBenchSceneSpec]] = defaultdict(list)
    for spec in manifest:
        grouped[(spec.family, spec.difficulty)].append(spec)

    result_map: dict[str, dict[str, Any]] = {}
    with ThreadPoolExecutor(max_workers=max(1, int(args.workers))) as executor:
        futures = {executor.submit(_run_scene, spec, stage1_tmp): spec.scene_id for spec in manifest}
        for fut in as_completed(futures):
            result_map[futures[fut]] = fut.result()

    scored_by_cell: dict[str, list[dict[str, Any]]] = {}
    new_records: list[DockBenchSceneSpec] = []
    for family in FAMILIES:
        for difficulty in DIFFICULTIES:
            cell_specs = sorted(grouped[(family, difficulty)], key=lambda spec: spec.scene_id)
            scored: list[dict[str, Any]] = []
            for spec in cell_specs:
                scene = json.loads(Path(spec.scenario_json).read_text(encoding="utf-8"))
                result = result_map[spec.scene_id]
                scored.append(_scene_score(scene, result))
            scored.sort(key=lambda item: (-int(item["label_match"]), -int(item["co_success"]), -item["score"], item["co_time"], -item["hardness"], item["scene_id"]))
            tuning_id = scored[0]["scene_id"]
            remaining = [item for item in scored if item["scene_id"] != tuning_id]
            remaining.sort(key=lambda item: (item["co_success"], item["score"], -item["hardness"], item["scene_id"]))
            challenge_id = remaining[0]["scene_id"] if remaining else tuning_id
            for spec in cell_specs:
                if spec.scene_id == tuning_id:
                    split = "tuning"
                elif spec.scene_id == challenge_id:
                    split = "challenge"
                else:
                    split = "test"
                _rewrite_scene_split(Path(spec.scenario_json), split)
                new_records.append(
                    DockBenchSceneSpec(
                        scene_id=spec.scene_id,
                        split=split,
                        family=spec.family,
                        difficulty=spec.difficulty,
                        seed=spec.seed,
                        style=spec.style,
                        subset_tag=spec.subset_tag,
                        max_time_s=spec.max_time_s,
                        scenario_json=spec.scenario_json,
                    )
                )
            scored_by_cell[f"{family}-{difficulty}"] = scored

    representatives: dict[str, DockBenchSceneSpec] = {}
    for family in FAMILIES:
        key = f"{family}_L2"
        candidates = [spec for spec in new_records if spec.family == family and spec.difficulty == "L2" and spec.split == "test"]
        candidate_scores = {item["scene_id"]: item for item in scored_by_cell[f"{family}-L2"]}
        candidates.sort(
            key=lambda spec: (
                -int(candidate_scores[spec.scene_id]["co_success"]),
                -candidate_scores[spec.scene_id]["score"],
                candidate_scores[spec.scene_id]["co_time"],
                spec.scene_id,
            )
        )
        if not candidates:
            raise RuntimeError(f"missing test representative for {family}-L2")
        representatives[key] = candidates[0]

    quality_report_path = ds_root / "dockbench_v1_quality_report.json"
    quality_report = json.loads(quality_report_path.read_text(encoding="utf-8")) if quality_report_path.exists() else {}
    quality_report["split_freeze"] = {
        "method": "support-aware audit selection",
        "family_labels": {family: family_label(family) for family in FAMILIES},
        "cells": scored_by_cell,
    }
    save_manifest_files(ds_root, records=new_records, representatives=representatives, quality_report=quality_report)
    summary = {
        "dataset_root": str(ds_root),
        "cell_scores": scored_by_cell,
        "representatives": {key: value.to_dict() for key, value in representatives.items()},
    }
    (ds_root / "dockbench_v1_split_freeze_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"dataset_root": str(ds_root), "summary": str((ds_root / 'dockbench_v1_split_freeze_summary.json').resolve())}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
