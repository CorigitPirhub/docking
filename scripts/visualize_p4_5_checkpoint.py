#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _plot_summary(data: dict, out_path: Path, *, title_suffix: str = "") -> None:
    s = data["split_priority_stress"]
    a = data["dock_priority_stress_ablation"]

    names = ["consistency", "max_switch_10s", "extra_fail_inc", "merge_split_merge"]
    split_vals = [
        float(s["conflict_consistency_rate"]),
        float(s["max_mutual_switches_10s"]),
        float(s["extra_failure_increment"]),
        float(s["merge_split_merge_rate"]),
    ]
    dock_vals = [
        float(a["conflict_consistency_rate"]),
        float(a["max_mutual_switches_10s"]),
        float(a["extra_failure_increment"]),
        float(a["merge_split_merge_rate"]),
    ]
    thresholds = data["thresholds"]
    th_vals = [
        float(thresholds["conflict_consistency_rate_min"]),
        float(thresholds["max_mutual_switches_10s_max"]),
        float(thresholds["extra_failure_increment_max"]),
        float(thresholds["merge_split_merge_rate_min"]),
    ]

    x = np.arange(len(names), dtype=float)
    w = 0.33
    fig, ax = plt.subplots(figsize=(9.6, 4.6), dpi=180)
    ax.bar(x - 0.5 * w, split_vals, width=w, label="split-priority", color="#2a9d8f")
    ax.bar(x + 0.5 * w, dock_vals, width=w, label="dock-priority (ablation)", color="#e76f51")
    ax.plot(x, th_vals, "k--", linewidth=1.2, label="threshold")
    ax.set_xticks(x, names)
    ax.set_title(f"P4.5 Conflict-Closure Metrics {title_suffix}".strip())
    ax.grid(alpha=0.24, axis="y")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _plot_timeline(data: dict, out_path: Path, *, title_suffix: str = "") -> None:
    runs = data["split_priority_stress"]["runs"]
    rep = None
    for r in runs:
        if int(r["conflict_count"]) > 0 and bool(r["merge_split_merge"]):
            rep = r
            break
    if rep is None and runs:
        rep = runs[0]
    if rep is None:
        return

    arb = rep["arbitration_trace"]
    acts = rep["reconfig_action_trace"]
    t = np.array([float(x["t"]) for x in arb], dtype=float)
    conflict = np.array([1.0 if bool(x["conflict"]) else 0.0 for x in arb], dtype=float)
    chosen = [str(x["chosen_action"]) for x in arb]
    code = {"NONE": 0.0, "HOLD": 0.5, "DOCK": 1.0, "SPLIT": 2.0}
    y = np.array([code.get(c, -0.2) for c in chosen], dtype=float)

    fig, axes = plt.subplots(2, 1, figsize=(10.0, 5.8), dpi=180, sharex=True)
    axes[0].step(t, y, where="post", color="#1d3557", linewidth=1.7)
    axes[0].set_yticks([0.0, 0.5, 1.0, 2.0], ["NONE", "HOLD", "DOCK", "SPLIT"])
    axes[0].set_ylabel("Chosen Action")
    axes[0].grid(alpha=0.24)

    axes[1].plot(t, conflict, color="#e63946", linewidth=1.4, label="conflict flag")
    for a in acts:
        color = "#2a9d8f" if str(a["action"]) == "SPLIT" else "#f4a261"
        axes[1].axvline(float(a["t"]), color=color, linewidth=1.0, alpha=0.75)
    axes[1].set_ylim(-0.1, 1.1)
    axes[1].set_ylabel("Conflict")
    axes[1].set_xlabel("t [s]")
    axes[1].grid(alpha=0.24)
    axes[1].legend(loc="upper right", fontsize=8)

    fig.suptitle(f"P4.5 Representative Timeline {title_suffix} | {rep['scenario_id']} | {rep['subtype']}".strip())
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main() -> None:
    import argparse

    p = argparse.ArgumentParser(description="Visualize P4.5 checkpoint results.")
    p.add_argument(
        "--input",
        type=str,
        default=str(ROOT / "experiments" / "p4_5_integration_checkpoint_results.json"),
        help="Input JSON path.",
    )
    p.add_argument("--tag", type=str, default="", help="Optional tag appended to output filenames.")
    args = p.parse_args()

    in_path = Path(args.input)
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)
    data = _load(in_path)

    cfg = dict(data.get("config", {}))
    noise = bool(cfg.get("enable_sensor_noise", False))
    suffix = f"(noise={'on' if noise else 'off'})"
    tag = str(args.tag).strip()
    summary_png = out_dir / (f"p4_5_metrics_summary_{tag}.png" if tag else "p4_5_metrics_summary.png")
    timeline_png = out_dir / (f"p4_5_representative_timeline_{tag}.png" if tag else "p4_5_representative_timeline.png")
    _plot_summary(data, summary_png, title_suffix=suffix)
    _plot_timeline(data, timeline_png, title_suffix=suffix)
    print("p4_5_summary_png", summary_png)
    print("p4_5_timeline_png", timeline_png)


if __name__ == "__main__":
    main()
