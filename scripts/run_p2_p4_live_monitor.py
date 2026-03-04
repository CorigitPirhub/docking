#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.scenario_support import ScenarioGenerator
from strategy.p2_p4_integrated import IntegratedDemoConfig, IntegratedP2P4Runner


def _demo_preset(name: str) -> dict[str, Any]:
    if name == "A":
        return {
            "subtype": "A2",
            "seed": 51042,
            "mode": "Clustered_At_A",
            "n_vehicles": 2,
            "demo_cfg": {
                "duration_s": 60.0,
                "inject_disturbance_for_type_c": False,
            },
        }
    if name == "B":
        return {
            "subtype": "B1",
            "seed": 62042,
            "mode": "Uniform_Spread",
            "n_vehicles": 2,
            "demo_cfg": {
                "duration_s": 60.0,
                "initial_chain_vehicle_ids": (1, 2),
                "pre_split_initial_chain": False,
                "free_target_speed": 0.85,
                "inject_disturbance_for_type_c": False,
            },
        }
    if name == "C":
        return {
            "subtype": "A2",
            "seed": 51042,
            "mode": "Clustered_At_A",
            "n_vehicles": 2,
            "demo_cfg": {
                "duration_s": 60.0,
                "inject_disturbance_for_type_c": True,
                "inject_disturbance_all_types": True,
                "disturbance_after_docking_s": 1.0,
                "disturbance_vision_prob": 0.9,
                "max_retry_per_vehicle": 2,
                "replan_delay_s": 0.5,
            },
        }
    raise ValueError(f"Unsupported preset: {name}")


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Run integrated P2-P4 with live monitor/step mode")
    p.add_argument("--preset", choices=["A", "B", "C"], default="A", help="demo preset")
    p.add_argument("--policy", choices=["integrated", "independent"], default="integrated")
    p.add_argument("--print-every", type=int, default=2, help="print every N low ticks")
    p.add_argument("--step", action="store_true", help="step-by-step execution (Enter next / c continue / q quit)")
    p.add_argument("--pause-on-alert", action="store_true", help="pause when alerts are emitted")
    p.add_argument("--seed-offset", type=int, default=0, help="runner random seed offset")
    p.add_argument("--jsonl-out", type=str, default="", help="per-tick monitor jsonl output path")
    p.add_argument("--result-out", type=str, default="", help="result json output path")
    p.add_argument("--summary-out", type=str, default="", help="summary markdown output path")
    p.add_argument("--timeline-out", type=str, default="", help="timeline png output path")
    return p.parse_args()


def _fmt_tick(row: dict[str, Any], vehicle_ids: list[int]) -> str:
    t = float(row["t"])
    leader_s = float(row["leader_s"])
    pending = len(row.get("pending_docks", {}))
    edges = row.get("edges", [])
    alerts = row.get("alerts", [])
    energy = float(row.get("total_energy", 0.0))
    collision_count = int(row.get("collision_count", 0))
    parts = [
        f"t={t:6.2f}",
        f"leader_s={leader_s:6.2f}",
        f"E={energy:8.3f}",
        f"pending={pending}",
        f"edges={len(edges)}",
        f"coll={collision_count}",
    ]
    for vid in vehicle_ids:
        st = row["states"].get(str(vid), {})
        if not st:
            continue
        mode = str(st.get("mode", "UNK"))
        v = float(st.get("v", 0.0))
        yawr = float(st.get("yaw_rate_est", 0.0))
        gdist = float(st.get("goal_dist", 0.0))
        dock = st.get("dock_debug", None)
        stage = dock.get("stage", "-") if isinstance(dock, dict) else "-"
        parts.append(f"v{vid}:{mode[:4]} v={v:4.2f} yr={yawr:5.2f} gd={gdist:5.2f} st={stage[:7]}")
    if alerts:
        parts.append("ALERT=" + ",".join(str(a.get("type", "?")) for a in alerts))
    return " | ".join(parts)


def _render_timeline(result: dict[str, Any], out_png: Path) -> None:
    mon = result.get("monitor_trace", [])
    if not mon:
        return
    t = [float(x["t"]) for x in mon]
    e = [float(x.get("total_energy", 0.0)) for x in mon]
    c = [int(x.get("collision_count", 0)) for x in mon]
    leader_s = [float(x.get("leader_s", 0.0)) for x in mon]
    dock_d = []
    dock_yaw = []
    for x in mon:
        pairs = x.get("dock_pairs", [])
        if pairs:
            p = pairs[0]
            dock_d.append(float(p.get("hitch_distance", 0.0)))
            dock_yaw.append(float(p.get("yaw_error_deg", 0.0)))
        else:
            dock_d.append(float("nan"))
            dock_yaw.append(float("nan"))

    fig, axes = plt.subplots(4, 1, figsize=(11.0, 9.0), dpi=150, sharex=True)
    axes[0].plot(t, leader_s, color="#1d3557", linewidth=1.6)
    axes[0].set_ylabel("leader_s [m]")
    axes[0].grid(alpha=0.25)

    axes[1].plot(t, e, color="#2a9d8f", linewidth=1.6)
    axes[1].set_ylabel("total_energy")
    axes[1].grid(alpha=0.25)

    axes[2].plot(t, dock_d, color="#e76f51", linewidth=1.4, label="hitch distance")
    axes[2].plot(t, dock_yaw, color="#6d597a", linewidth=1.1, label="yaw err [deg]")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].set_ylabel("dock metrics")
    axes[2].grid(alpha=0.25)

    axes[3].plot(t, c, color="#d62828", linewidth=1.5)
    axes[3].set_ylabel("collision_count")
    axes[3].set_xlabel("time [s]")
    axes[3].grid(alpha=0.25)

    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)


def main() -> None:
    args = _parse_args()
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    preset = _demo_preset(args.preset)
    case = gen.generate(
        preset["subtype"],
        n_vehicles=int(preset["n_vehicles"]),
        seed=int(preset["seed"]),
        initial_dispersion_mode=str(preset["mode"]),
        overrides=None,
    )
    dcfg = IntegratedDemoConfig(**preset["demo_cfg"])

    exp_dir = ROOT / "experiments"
    art_dir = ROOT / "artifacts"
    exp_dir.mkdir(exist_ok=True)
    art_dir.mkdir(exist_ok=True)
    tag = f"p2_p4_monitor_{args.preset}_{args.policy.lower()}"
    jsonl_path = Path(args.jsonl_out) if args.jsonl_out else (exp_dir / f"{tag}_ticks.jsonl")
    result_path = Path(args.result_out) if args.result_out else (exp_dir / f"{tag}_result.json")
    summary_path = Path(args.summary_out) if args.summary_out else (art_dir / f"{tag}_summary.md")
    timeline_path = Path(args.timeline_out) if args.timeline_out else (art_dir / f"{tag}_timeline.png")

    vehicle_ids = sorted(int(v.vehicle_id) for v in case.vehicles_init)
    jsonl_path.parent.mkdir(parents=True, exist_ok=True)
    fout = jsonl_path.open("w", encoding="utf-8")
    step_state = {"continuous": not args.step}

    def tick_observer(row: dict[str, Any]) -> None:
        fout.write(json.dumps(row, ensure_ascii=False) + "\n")
        t_idx = int(round(float(row["t"]) / max(cfg.control.dt, 1e-6)))
        do_print = (t_idx % max(1, int(args.print_every))) == 0 or bool(row.get("alerts"))
        if do_print:
            print(_fmt_tick(row, vehicle_ids))

    def step_decider(row: dict[str, Any]) -> bool:
        if step_state["continuous"]:
            if args.pause_on_alert and row.get("alerts"):
                step_state["continuous"] = False
            else:
                return True
        while True:
            cmd = input("[step] Enter=next, c=continue, q=quit > ").strip().lower()
            if cmd == "q":
                return False
            if cmd == "c":
                step_state["continuous"] = True
                return True
            if cmd == "":
                return True
            print("invalid input")

    runner = IntegratedP2P4Runner(
        cfg,
        case,
        policy=args.policy,
        seed=2000 + int(args.seed_offset),
        demo_cfg=dcfg,
        tick_observer=tick_observer,
        step_decider=step_decider if (args.step or args.pause_on_alert) else None,
    )
    out = runner.run()
    fout.close()

    res = out.to_dict()
    result_path.write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding="utf-8")
    _render_timeline(res, timeline_path)

    alert_counter = Counter()
    alert_vehicle = defaultdict(Counter)
    for row in res.get("monitor_trace", []):
        for a in row.get("alerts", []):
            typ = str(a.get("type", "unknown"))
            vid = int(a.get("vehicle_id", -1))
            alert_counter[typ] += 1
            alert_vehicle[vid][typ] += 1

    lines = [
        f"# P2-P4 Live Monitor Summary ({args.preset}/{args.policy})",
        "",
        f"- scenario_id: {res['scenario_id']}",
        f"- subtype: {res['subtype']}",
        f"- success: {res['success']}",
        f"- done_time_s: {res['done_time_s']:.2f}",
        f"- total_energy: {res['total_energy']:.4f}",
        f"- dock_success_count: {res['dock_success_count']}",
        f"- split_count: {res['split_count']}",
        f"- p4_interruption_count: {res['p4_interruption_count']}",
        f"- p4_replan_success_count: {res['p4_replan_success_count']}",
        f"- collision_count: {res['collision_count']}",
        "",
        "## Alerts",
    ]
    if alert_counter:
        for k, v in sorted(alert_counter.items()):
            lines.append(f"- {k}: {int(v)}")
    else:
        lines.append("- none")
    lines += [
        "",
        "## Alert By Vehicle",
    ]
    if alert_vehicle:
        for vid in sorted(alert_vehicle.keys()):
            pairs = ", ".join(f"{k}:{int(v)}" for k, v in sorted(alert_vehicle[vid].items()))
            lines.append(f"- vehicle {vid}: {pairs}")
    else:
        lines.append("- none")
    lines += [
        "",
        f"- tick_jsonl: {jsonl_path}",
        f"- result_json: {result_path}",
        f"- timeline_png: {timeline_path}",
    ]
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("monitor_jsonl", jsonl_path)
    print("monitor_result", result_path)
    print("monitor_summary", summary_path)
    print("monitor_timeline", timeline_path)


if __name__ == "__main__":
    main()
