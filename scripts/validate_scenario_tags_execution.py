#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.scenario_support import ScenarioGenerator
from strategy.p2_p4_integrated import IntegratedDemoConfig, IntegratedP2P4Runner
from strategy.p3_reconfig import in_zone, nmax_at_s


@dataclass(frozen=True)
class CaseExecutionTagAudit:
    scenario_id: str
    subtype: str
    seed: int
    dispersion_mode: str
    n_vehicles: int
    enable_sensor_noise: bool
    inject_disturbance: bool
    success: bool
    collision_count: int
    done_time_s: float
    dock_success_count: int
    split_count: int
    nmax_violation_count: int
    nmax_max_over: int
    dock_locked_outside_zone: int
    docking_visual_valid_ratio: float
    docking_fallback_ratio: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _leader_train_size(edges: list[list[int]], head: int) -> int:
    child: dict[int, int] = {}
    for a, b in edges:
        child[int(a)] = int(b)
    n = 1
    cur = int(head)
    while cur in child:
        n += 1
        cur = child[cur]
        if n > 1000:
            break
    return int(n)


def _leader_s_at(monitor_trace: list[dict[str, Any]], t: float) -> float:
    if not monitor_trace:
        return 0.0
    ts = np.array([float(r["t"]) for r in monitor_trace], dtype=float)
    idx = int(np.searchsorted(ts, float(t), side="right") - 1)
    idx = max(0, min(idx, len(monitor_trace) - 1))
    return float(monitor_trace[idx]["leader_s"])


def _audit_one(
    cfg,
    *,
    subtype: str,
    seed: int,
    mode: str,
    n_vehicles: int,
    duration_s: float,
    enable_sensor_noise: bool,
    inject_disturbance: bool,
) -> CaseExecutionTagAudit:
    gen = ScenarioGenerator(cfg)
    case = gen.generate(subtype, n_vehicles=n_vehicles, seed=seed, initial_dispersion_mode=mode)
    assert case.labels is not None
    labels = case.labels

    demo_cfg = IntegratedDemoConfig(
        duration_s=float(duration_s),
        enable_sensor_noise=bool(enable_sensor_noise),
        inject_disturbance_for_type_c=bool(inject_disturbance),
        stop_on_done=True,
    )
    out = IntegratedP2P4Runner(cfg, case, policy="integrated", seed=int(seed + 7000), demo_cfg=demo_cfg).run()

    violation_count = 0
    max_over = 0
    for row in out.monitor_trace:
        leader_s = float(row["leader_s"])
        nmax = int(nmax_at_s(labels.n_max_pass_profile, leader_s))
        size = _leader_train_size(list(row["edges"]), head=1)
        if size > nmax:
            violation_count += 1
            max_over = max(max_over, int(size - nmax))

    dock_outside = 0
    for e in out.events:
        if str(e.event) != "DOCK_LOCKED":
            continue
        s_evt = _leader_s_at(out.monitor_trace, float(e.t))
        if labels.dock_friendly_zones and (not in_zone(labels.dock_friendly_zones, s_evt)):
            dock_outside += 1

    dock_ticks = 0
    vis_ok = 0
    fallback = 0
    for row in out.monitor_trace:
        for st in row.get("states", {}).values():
            if str(st.get("mode", "")) != "DOCKING":
                continue
            dbg = st.get("dock_debug", None)
            if not isinstance(dbg, dict):
                continue
            dock_ticks += 1
            if float(dbg.get("visual_valid", 0.0)) >= 0.5:
                vis_ok += 1
            if float(dbg.get("fallback_global", 0.0)) >= 0.5:
                fallback += 1
    vis_ratio = 1.0 if dock_ticks <= 0 else float(vis_ok / dock_ticks)
    fallback_ratio = 0.0 if dock_ticks <= 0 else float(fallback / dock_ticks)

    return CaseExecutionTagAudit(
        scenario_id=str(out.scenario_id),
        subtype=str(out.subtype),
        seed=int(seed),
        dispersion_mode=str(mode),
        n_vehicles=int(n_vehicles),
        enable_sensor_noise=bool(enable_sensor_noise),
        inject_disturbance=bool(inject_disturbance),
        success=bool(out.success),
        collision_count=int(out.collision_count),
        done_time_s=float(out.done_time_s),
        dock_success_count=int(out.dock_success_count),
        split_count=int(out.split_count),
        nmax_violation_count=int(violation_count),
        nmax_max_over=int(max_over),
        dock_locked_outside_zone=int(dock_outside),
        docking_visual_valid_ratio=float(vis_ratio),
        docking_fallback_ratio=float(fallback_ratio),
    )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Validate scenario tags vs real execution (with optional sensor noise).")
    p.add_argument("--seeds-per-subtype", type=int, default=2, help="Seed count for each subtype and dispersion mode.")
    p.add_argument("--duration-s", type=float, default=60.0, help="Simulation duration for each run.")
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
    p.add_argument("--n-vehicles", type=int, default=0, help="Vehicle count, 0 means scenario.representative_vehicle_count.")
    p.add_argument("--base-seed", type=int, default=-1, help="Base random seed, -1 means use config.testing.random_seed.")
    p.add_argument("--enable-sensor-noise", action="store_true", help="Enable GNSS+vision noise (recommended).")
    p.add_argument("--inject-disturbance", action="store_true", help="Enable disturbance injection (P4-style).")
    p.add_argument(
        "--output-json",
        type=str,
        default=str(ROOT / "artifacts" / "scenario_tag_execution_report.json"),
        help="Output JSON path.",
    )
    p.add_argument(
        "--output-md",
        type=str,
        default=str(ROOT / "artifacts" / "SCENARIO_TAG_EXECUTION_REPORT.md"),
        help="Output markdown report path.",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config()

    subtypes = [s.strip() for s in str(args.subtypes).split(",") if s.strip()]
    modes = [m.strip() for m in str(args.dispersion_modes).split(",") if m.strip()]
    n_vehicles = int(args.n_vehicles) if int(args.n_vehicles) > 0 else int(cfg.scenario.representative_vehicle_count)
    base_seed = int(args.base_seed) if int(args.base_seed) >= 0 else int(cfg.testing.random_seed)

    audits: list[CaseExecutionTagAudit] = []
    for si, subtype in enumerate(subtypes):
        for mi, mode in enumerate(modes):
            for k in range(int(args.seeds_per_subtype)):
                seed = int(base_seed + 10000 * si + 1000 * mi + k)
                audits.append(
                    _audit_one(
                        cfg,
                        subtype=subtype,
                        seed=seed,
                        mode=mode,
                        n_vehicles=n_vehicles,
                        duration_s=float(args.duration_s),
                        enable_sensor_noise=bool(args.enable_sensor_noise),
                        inject_disturbance=bool(args.inject_disturbance),
                    )
                )
                a = audits[-1]
                print(
                    f"{a.subtype} {a.dispersion_mode} seed={a.seed} "
                    f"succ={a.success} col={a.collision_count} nmax_vio={a.nmax_violation_count} dock_out={a.dock_locked_outside_zone}",
                    flush=True,
                )

    by_subtype: dict[str, dict[str, Any]] = {}
    for sub in subtypes:
        rs = [a for a in audits if a.subtype == sub]
        ok = [a for a in rs if (a.collision_count == 0 and a.nmax_violation_count == 0)]
        by_subtype[sub] = {
            "total": int(len(rs)),
            "ok_rate_hard": float(len(ok) / max(1, len(rs))),
            "collision_free_rate": float(sum(1 for a in rs if a.collision_count == 0) / max(1, len(rs))),
            "nmax_no_violation_rate": float(sum(1 for a in rs if a.nmax_violation_count == 0) / max(1, len(rs))),
            "avg_docking_visual_valid_ratio": float(
                sum(float(a.docking_visual_valid_ratio) for a in rs) / max(1, len(rs))
            ),
            "avg_docking_fallback_ratio": float(sum(float(a.docking_fallback_ratio) for a in rs) / max(1, len(rs))),
        }

    overall_ok = [a for a in audits if (a.collision_count == 0 and a.nmax_violation_count == 0)]
    report = {
        "config": {
            "seeds_per_subtype": int(args.seeds_per_subtype),
            "duration_s": float(args.duration_s),
            "subtypes": subtypes,
            "dispersion_modes": modes,
            "n_vehicles": int(n_vehicles),
            "base_seed": int(base_seed),
            "enable_sensor_noise": bool(args.enable_sensor_noise),
            "inject_disturbance": bool(args.inject_disturbance),
        },
        "overall": {
            "total": int(len(audits)),
            "hard_ok": int(len(overall_ok)),
            "hard_ok_rate": float(len(overall_ok) / max(1, len(audits))),
        },
        "by_subtype": by_subtype,
        "cases": [a.to_dict() for a in audits],
    }

    out_json = Path(args.output_json)
    out_md = Path(args.output_md)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = [
        "# Scenario Tag vs Execution Report",
        "",
        "Hard OK means: collision_count==0 and nmax_violation_count==0.",
        "",
        "## Overall",
        f"- total: {report['overall']['total']}",
        f"- hard_ok: {report['overall']['hard_ok']}",
        f"- hard_ok_rate: {report['overall']['hard_ok_rate']:.4f}",
        "",
        "## By Subtype",
    ]
    for sub in subtypes:
        s = report["by_subtype"][sub]
        lines.append(
            f"- {sub}: ok_rate_hard={s['ok_rate_hard']:.4f}, collision_free_rate={s['collision_free_rate']:.4f}, "
            f"nmax_no_violation_rate={s['nmax_no_violation_rate']:.4f}, vis_valid_avg={s['avg_docking_visual_valid_ratio']:.4f}, "
            f"fallback_avg={s['avg_docking_fallback_ratio']:.4f}"
        )
    lines += ["", f"JSON: {out_json}"]
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("scenario_tag_exec_json", out_json)
    print("scenario_tag_exec_md", out_md)


if __name__ == "__main__":
    main()
