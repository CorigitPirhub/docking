#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from statistics import mean
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.scenario_support import ScenarioGenerator
from strategy.p2_p4_integrated import IntegratedDemoConfig, IntegratedP2P4Runner


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Multi-seed B1/60s deadlock-plateau checker")
    p.add_argument("--num-seeds", type=int, default=12)
    p.add_argument("--base-seed", type=int, default=62042)
    p.add_argument("--runner-seed-base", type=int, default=2002)
    p.add_argument("--freeze-threshold-s", type=float, default=2.0)
    return p.parse_args()


def _effective_freeze_duration(run: dict[str, Any], tol_leader: float, tol_free: float) -> float:
    mon = run.get("monitor_trace", [])
    if len(mon) <= 1:
        return 0.0
    max_freeze = 0.0
    cur = 0.0
    for k in range(1, len(mon)):
        p = mon[k - 1]
        c = mon[k]
        dt = max(0.0, float(c["t"]) - float(p["t"]))
        if dt <= 0.0:
            continue
        freeze_now = True
        for vid_str, st in c.get("states", {}).items():
            mode = str(st.get("mode", "FREE"))
            if mode != "FREE":
                continue
            vid = int(vid_str)
            goal_tol = tol_leader if vid == 1 else tol_free
            goal_dist = float(st.get("goal_dist", 0.0))
            v = abs(float(st.get("v", 0.0)))
            if goal_dist > goal_tol + 0.05 and v > 0.02:
                freeze_now = False
                break
        if freeze_now:
            cur += dt
            if cur > max_freeze:
                max_freeze = cur
        else:
            cur = 0.0
    return float(max_freeze)


def main() -> None:
    args = parse_args()
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    demo_cfg = IntegratedDemoConfig(
        duration_s=60.0,
        initial_chain_vehicle_ids=(1, 2),
        pre_split_initial_chain=False,
        free_target_speed=0.85,
        inject_disturbance_for_type_c=False,
    )

    runs: list[dict[str, Any]] = []
    for i in range(int(args.num_seeds)):
        seed = int(args.base_seed + i)
        case = gen.generate("B1", n_vehicles=2, seed=seed, initial_dispersion_mode="Uniform_Spread")
        out = IntegratedP2P4Runner(
            cfg,
            case,
            policy="integrated",
            seed=int(args.runner_seed_base + i),
            demo_cfg=demo_cfg,
        ).run()
        out_d = out.to_dict()
        max_freeze = _effective_freeze_duration(
            out_d,
            tol_leader=float(demo_cfg.leader_goal_tol_m),
            tol_free=float(demo_cfg.vehicle_goal_tol_m),
        )
        stall = bool(max_freeze >= float(args.freeze_threshold_s))
        runs.append(
            {
                "seed": int(seed),
                "scenario_id": str(out_d["scenario_id"]),
                "success": bool(out_d["success"]),
                "done_time_s": float(out_d["done_time_s"]),
                "collision_count": int(out_d["collision_count"]),
                "split_count": int(out_d["split_count"]),
                "dock_success_count": int(out_d["dock_success_count"]),
                "max_effective_freeze_s": float(max_freeze),
                "stall_like_plateau": bool(stall),
            }
        )

    success_rate = float(sum(1 for r in runs if r["success"]) / max(1, len(runs)))
    no_collision_rate = float(sum(1 for r in runs if int(r["collision_count"]) == 0) / max(1, len(runs)))
    no_plateau_rate = float(sum(1 for r in runs if not bool(r["stall_like_plateau"])) / max(1, len(runs)))
    avg_done = float(mean(r["done_time_s"] for r in runs))

    summary = {
        "num_runs": int(len(runs)),
        "success_rate": float(success_rate),
        "no_collision_rate": float(no_collision_rate),
        "no_plateau_rate": float(no_plateau_rate),
        "avg_done_time_s": float(avg_done),
        "freeze_threshold_s": float(args.freeze_threshold_s),
        "pass": bool(success_rate >= 0.95 and no_collision_rate == 1.0 and no_plateau_rate >= 0.95),
    }
    result = {
        "config": {
            "num_seeds": int(args.num_seeds),
            "base_seed": int(args.base_seed),
            "runner_seed_base": int(args.runner_seed_base),
            "duration_s": 60.0,
            "scenario_subtype": "B1",
        },
        "summary": summary,
        "runs": runs,
    }

    exp_dir = ROOT / "experiments"
    art_dir = ROOT / "artifacts"
    exp_dir.mkdir(exist_ok=True)
    art_dir.mkdir(exist_ok=True)
    json_path = exp_dir / "p2_p4_B60_multiseed_check.json"
    md_path = art_dir / "P2_P4_B60_MULTI_SEED_CHECK.md"
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = [
        "# B1 60s Multi-seed Plateau Check",
        "",
        f"- num_runs: {summary['num_runs']}",
        f"- success_rate: {summary['success_rate']:.4f}",
        f"- no_collision_rate: {summary['no_collision_rate']:.4f}",
        f"- no_plateau_rate: {summary['no_plateau_rate']:.4f}",
        f"- avg_done_time_s: {summary['avg_done_time_s']:.2f}",
        f"- freeze_threshold_s: {summary['freeze_threshold_s']:.2f}",
        f"- pass: {summary['pass']}",
        "",
        f"JSON: {json_path}",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("b60_multiseed_json", json_path)
    print("b60_multiseed_md", md_path)
    print("b60_multiseed_pass", summary["pass"])


if __name__ == "__main__":
    main()

