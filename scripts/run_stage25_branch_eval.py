#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.p_minus1_baselines import METHOD_SPECS

STAGE1_SCRIPT = ROOT / "scripts" / "run_p_minus1_stage1_docking.py"


def _run_scene(scene: dict[str, Any], *, methods: list[str], out_dir: Path, max_time_s: float, skip_gifs: bool) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable,
        str(STAGE1_SCRIPT),
        "--scenario-json",
        str(scene["scenario_json"]),
        "--methods",
        ",".join(methods),
        "--out-dir",
        str(out_dir),
        "--max-time-s",
        str(min(float(max_time_s), float(scene["max_time_s"]))),
    ]
    if skip_gifs:
        cmd.append("--skip-gifs")
    subprocess.run(cmd, check=True, cwd=str(ROOT), capture_output=True, text=True)
    result_files = sorted(out_dir.glob("p_minus1_stage1_results_seed*.json"))
    if not result_files:
        raise FileNotFoundError(f"No stage1 result json found in {out_dir}")
    return json.loads(result_files[-1].read_text(encoding="utf-8"))


def _summarize(method_id: str, rows: list[dict[str, Any]]) -> dict[str, Any]:
    total = len(rows)
    succ_rows = [row for row in rows if bool(row["success"])]
    families: dict[str, dict[str, float]] = {}
    for family in sorted({str(row["family"]) for row in rows}):
        fam_rows = [row for row in rows if str(row["family"]) == family]
        fam_succ = [row for row in fam_rows if bool(row["success"])]
        families[family] = {
            "success": float(len(fam_succ) / max(1, len(fam_rows))),
            "collision": float(sum(1 for row in fam_rows if bool(row["collision"])) / max(1, len(fam_rows))),
            "avg_time": float(sum(float(row["done_time_s"]) for row in fam_succ) / max(1, len(fam_succ))) if fam_succ else float("nan"),
        }
    fails = [row for row in rows if not bool(row["success"])]
    return {
        "method_id": method_id,
        "overall_success": float(len(succ_rows) / max(1, total)),
        "overall_collision": float(sum(1 for row in rows if bool(row["collision"])) / max(1, total)),
        "overall_time": float(sum(float(row["done_time_s"]) for row in succ_rows) / max(1, len(succ_rows))) if succ_rows else float("nan"),
        "fails": fails,
        "families": families,
        "rows": rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Stage-2.5 branch evaluation on frozen DockBench split.")
    parser.add_argument("--dataset-root", type=str, default=str(ROOT / "data" / "dockbench_v1"))
    parser.add_argument("--split", type=str, default="test", choices=["tuning", "test", "challenge"])
    parser.add_argument("--methods", type=str, default="S25_cgfl_only,S25_tvt_only,S25_vpcr_only,co_bcfd")
    parser.add_argument("--out-dir", type=str, default=str(ROOT / "artifacts" / "stage25_branch_eval"))
    parser.add_argument("--max-time-s", type=float, default=45.0)
    parser.add_argument("--skip-gifs", action="store_true")
    args = parser.parse_args()

    dataset_root = Path(args.dataset_root).expanduser().resolve()
    split_path = dataset_root / f"dockbench_v1_split_{args.split}.json"
    scenes = json.loads(split_path.read_text(encoding="utf-8"))
    methods = [mid.strip() for mid in str(args.methods).split(",") if mid.strip()]
    for mid in methods:
        if mid not in METHOD_SPECS:
            raise KeyError(f"Unknown method id: {mid}")

    out_dir = Path(args.out_dir).expanduser().resolve()
    cache_dir = out_dir / "scene_runs"
    cache_dir.mkdir(parents=True, exist_ok=True)

    rows_by_method: dict[str, list[dict[str, Any]]] = defaultdict(list)
    scene_index: list[dict[str, Any]] = []
    for scene in scenes:
        scene_id = str(scene["scene_id"])
        scene_out = cache_dir / scene_id
        result = _run_scene(scene, methods=methods, out_dir=scene_out, max_time_s=float(args.max_time_s), skip_gifs=bool(args.skip_gifs))
        scene_index.append({
            "scene_id": scene_id,
            "family": str(scene["family"]),
            "difficulty": str(scene["difficulty"]),
            "scenario_json": str(scene["scenario_json"]),
            "result_json": str(sorted(scene_out.glob("p_minus1_stage1_results_seed*.json"))[-1]),
        })
        run_map = {str(run["metrics"]["method_id"]): run for run in result.get("runs", [])}
        for mid in methods:
            metrics = dict(run_map[mid]["metrics"])
            rows_by_method[mid].append(
                {
                    "scene_id": scene_id,
                    "family": str(scene["family"]),
                    "difficulty": str(scene["difficulty"]),
                    "success": bool(metrics["success"]),
                    "done_time_s": float(metrics["done_time_s"]),
                    "collision": bool(metrics["collision"]),
                    "failure_category": str(metrics.get("failure_category", metrics.get("reason", "unknown"))),
                    "trajectory_cost": float(metrics.get("trajectory_cost", 0.0)),
                    "fallback_count": int(metrics.get("fallback_count", 0)),
                    "visual_loss_count": int(metrics.get("visual_loss_count", 0)),
                    "replan_count": int(metrics.get("replan_count", 0)),
                }
            )

    summaries = {mid: _summarize(mid, rows_by_method[mid]) for mid in methods}
    (out_dir / "branch_eval_index.json").write_text(
        json.dumps({"split": args.split, "methods": methods, "scenes": scene_index}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    for mid, summary in summaries.items():
        (out_dir / f"{mid}_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    (out_dir / "branch_eval_summary.json").write_text(json.dumps(summaries, ensure_ascii=False, indent=2), encoding="utf-8")
    for mid in methods:
        spec = METHOD_SPECS[mid]
        summary = summaries[mid]
        print(
            f"{mid}\t{spec.summary}\tsuccess={summary['overall_success']:.4f}\tcollision={summary['overall_collision']:.4f}\ttime={summary['overall_time']:.2f}"
        )


if __name__ == "__main__":
    main()
