#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from standalone_framework_demo.run_demo import FrameworkProfile, StandaloneEngine, save_plots, write_report


def profiles() -> list[FrameworkProfile]:
    return [
        FrameworkProfile(
            name="ground_transport_convoy",
            goal_x=36.0,
            spacing=1.6,
            leader_cruise_speed=1.2,
            free_cruise_speed=1.0,
            phase_a_end_x=12.0,
            phase_b_end_x=23.0,
            rebuild_target_size=3,
            wait_trigger_t=9.0,
            wait_vehicle_id=5,
            wait_duration_s=1.5,
            hard_bottleneck_x_min=16.0,
            hard_bottleneck_x_max=22.0,
            feasibility_dock_time_coeff=1.6,
            feasibility_margin=0.8,
        ),
        FrameworkProfile(
            name="warehouse_tugger_convoy",
            goal_x=28.0,
            spacing=1.4,
            leader_cruise_speed=1.0,
            free_cruise_speed=0.85,
            docking_lock_pos_err=0.14,
            docking_lock_vel_err=0.40,
            phase_a_end_x=8.5,
            phase_b_end_x=18.0,
            rebuild_target_size=2,
            wait_trigger_t=6.0,
            wait_vehicle_id=4,
            wait_duration_s=1.2,
            hard_bottleneck_x_min=11.0,
            hard_bottleneck_x_max=16.0,
            feasibility_dock_time_coeff=1.3,
            feasibility_margin=0.6,
        ),
    ]


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    out_root = root / "artifacts" / "standalone_framework_transferability"
    out_root.mkdir(parents=True, exist_ok=True)

    summary: dict[str, object] = {"cases": []}

    for p in profiles():
        engine = StandaloneEngine(profile=p)
        metrics = engine.run(duration_s=34.0)
        case_dir = out_root / p.name
        case_dir.mkdir(parents=True, exist_ok=True)
        (case_dir / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
        save_plots(engine, case_dir)
        write_report(case_dir, metrics)
        summary["cases"].append(
            {
                "profile": p.name,
                "ok": bool(metrics["ok"]),
                "invariant_ok": bool(metrics["invariant_ok"]),
                "execute_accept": int(metrics["execute_accept"]),
                "execute_reject": int(metrics["execute_reject"]),
                "dock_locked": int(metrics["event_count"].get("DOCK_LOCKED", 0)),
                "split_done": int(metrics["event_count"].get("SPLIT_DONE", 0)),
                "hard_bottleneck_samples": int(metrics["hard_bottleneck_samples"]),
                "max_leader_train_size_hard_bottleneck": float(metrics["max_leader_train_size_hard_bottleneck"]),
            }
        )

    cases = summary["cases"]
    summary["all_ok"] = bool(all(c["ok"] for c in cases))
    summary["case_count"] = len(cases)

    (out_root / "transferability_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = [
        "# Standalone Transferability Suite",
        "",
        "- Goal: verify the same decoupled framework can migrate across similar problem settings by profile adaptation only.",
        f"- case_count: {summary['case_count']}",
        f"- all_ok: {summary['all_ok']}",
        "",
        "## Case Results",
    ]
    for c in cases:
        lines.append(
            f"- {c['profile']}: ok={c['ok']}, invariant_ok={c['invariant_ok']}, "
            f"accept={c['execute_accept']}, reject={c['execute_reject']}, "
            f"DOCK_LOCKED={c['dock_locked']}, SPLIT_DONE={c['split_done']}, "
            f"hard_samples={c['hard_bottleneck_samples']}, "
            f"max_leader_train_size_hard={c['max_leader_train_size_hard_bottleneck']}"
        )
    (out_root / "TRANSFERABILITY_SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("transferability_summary", out_root / "transferability_summary.json")
    print("transferability_report", out_root / "TRANSFERABILITY_SUMMARY.md")
    print("transferability_ok", summary["all_ok"])


if __name__ == "__main__":
    main()
