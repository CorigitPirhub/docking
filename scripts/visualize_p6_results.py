#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Visualize P6 system-level results.")
    p.add_argument(
        "--input",
        type=str,
        default=str(ROOT / "experiments" / "p6_system_evaluation_results.json"),
        help="Input P6 JSON path.",
    )
    return p.parse_args()


def _save_policy_metrics(data: dict, out_dir: Path, *, title_suffix: str = "") -> Path:
    aggs = data["aggregates"]
    policy_ids = list(aggs.keys())

    success = [float(aggs[p]["success_rate"]) for p in policy_ids]
    done60 = [float(aggs[p]["done_within_60_rate"]) for p in policy_ids]
    feas = [float(aggs[p]["feasibility_accuracy"]) for p in policy_ids]

    x = np.arange(len(policy_ids), dtype=float)
    w = 0.24

    fig, ax = plt.subplots(figsize=(9, 4.8), dpi=160)
    ax.bar(x - w, success, width=w, label="success_rate")
    ax.bar(x, done60, width=w, label="done_within_60_rate")
    ax.bar(x + w, feas, width=w, label="feasibility_accuracy")
    ax.set_ylim(0.0, 1.05)
    ax.set_xticks(x)
    ax.set_xticklabels(policy_ids, rotation=15)
    ax.set_ylabel("Rate")
    ax.set_title(f"P6 Policy-Level Metrics {title_suffix}".strip())
    ax.grid(alpha=0.25, linestyle="--", axis="y")
    ax.legend(loc="lower right")
    fig.tight_layout()

    out = out_dir / "p6_policy_metrics.png"
    fig.savefig(out)
    plt.close(fig)
    return out


def _save_subtype_heatmap(data: dict, out_dir: Path, *, title_suffix: str = "") -> Path:
    breakdown = data["subtype_breakdown"]
    policy_ids = list(breakdown.keys())
    subtype_ids = sorted({s for pid in policy_ids for s in breakdown[pid].keys()})

    mat = np.zeros((len(policy_ids), len(subtype_ids)), dtype=float)
    for i, pid in enumerate(policy_ids):
        for j, sid in enumerate(subtype_ids):
            mat[i, j] = float(breakdown[pid][sid]["success_rate"])

    fig, ax = plt.subplots(figsize=(11, 4.6), dpi=160)
    im = ax.imshow(mat, aspect="auto", cmap="viridis", vmin=0.0, vmax=1.0)
    ax.set_xticks(np.arange(len(subtype_ids)))
    ax.set_xticklabels(subtype_ids)
    ax.set_yticks(np.arange(len(policy_ids)))
    ax.set_yticklabels(policy_ids)
    ax.set_title(f"P6 Success Rate by Subtype {title_suffix}".strip())
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            ax.text(j, i, f"{mat[i, j]:.2f}", ha="center", va="center", color="white", fontsize=7)
    fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02, label="success_rate")
    fig.tight_layout()

    out = out_dir / "p6_subtype_success_heatmap.png"
    fig.savefig(out)
    plt.close(fig)
    return out


def _save_time_energy_scatter(data: dict, out_dir: Path, *, title_suffix: str = "") -> Path:
    runs = data["runs"]
    colors = {
        "integrated_split_priority": "tab:blue",
        "integrated_dock_priority": "tab:orange",
        "independent": "tab:green",
    }

    fig, ax = plt.subplots(figsize=(8.8, 5.2), dpi=160)
    for pid in colors:
        rr = [r for r in runs if str(r["policy_id"]) == pid]
        if not rr:
            continue
        x = np.array([float(r["done_time_s"]) for r in rr], dtype=float)
        y = np.array([float(r["total_energy"]) for r in rr], dtype=float)
        ax.scatter(x, y, s=16, alpha=0.45, label=pid, c=colors[pid])

    ax.set_xlabel("done_time_s")
    ax.set_ylabel("total_energy")
    ax.set_title(f"P6 Time-Energy Scatter {title_suffix}".strip())
    ax.grid(alpha=0.25, linestyle="--")
    ax.legend(loc="best")
    fig.tight_layout()

    out = out_dir / "p6_time_energy_scatter.png"
    fig.savefig(out)
    plt.close(fig)
    return out


def main() -> None:
    args = parse_args()
    data = json.loads(Path(args.input).read_text(encoding="utf-8"))
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)

    cfg = dict(data.get("config", {}))
    noise = bool(cfg.get("enable_sensor_noise", False))
    disturbance = bool(cfg.get("inject_disturbance", False))
    suffix = f"(noise={'on' if noise else 'off'}, disturbance={'on' if disturbance else 'off'})"

    p1 = _save_policy_metrics(data, out_dir, title_suffix=suffix)
    p2 = _save_subtype_heatmap(data, out_dir, title_suffix=suffix)
    p3 = _save_time_energy_scatter(data, out_dir, title_suffix=suffix)

    print("p6_fig_policy_metrics", p1)
    print("p6_fig_subtype_heatmap", p2)
    print("p6_fig_time_energy", p3)


if __name__ == "__main__":
    main()
