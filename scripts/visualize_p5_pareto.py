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


def main() -> None:
    exp_path = ROOT / "experiments" / "p5_pareto_results.json"
    if not exp_path.exists():
        raise FileNotFoundError(f"Missing results JSON: {exp_path}")
    data = json.loads(exp_path.read_text(encoding="utf-8"))
    metrics = data.get("metrics", [])
    pidx = set(int(i) for i in data.get("pareto_indices", []))
    ridx = data.get("recommended_index", None)

    t = np.array([float(m["avg_time_s"]) for m in metrics], dtype=float)
    e = np.array([float(m["avg_energy"]) for m in metrics], dtype=float)
    s = np.array([float(m["avg_safety"]) for m in metrics], dtype=float)
    pass_mask = np.array([bool(m["profile_pass"]) for m in metrics], dtype=bool)

    art_dir = ROOT / "artifacts"
    art_dir.mkdir(exist_ok=True)
    p1 = art_dir / "p5_pareto_frontier.png"
    p2 = art_dir / "p5_profile_tradeoff.png"

    fig, ax = plt.subplots(figsize=(8.8, 6.6), dpi=160)
    sc = ax.scatter(t, e, c=s, cmap="viridis", s=34, alpha=0.78, label="profiles")
    if pass_mask.any():
        ax.scatter(t[pass_mask], e[pass_mask], facecolors="none", edgecolors="#e76f51", s=72, linewidths=1.2, label="gate-pass")
    if pidx:
        px = np.array([t[i] for i in sorted(pidx)], dtype=float)
        py = np.array([e[i] for i in sorted(pidx)], dtype=float)
        order = np.argsort(px)
        ax.plot(px[order], py[order], color="#1d3557", linewidth=1.8, label="pareto-front")
        ax.scatter(px, py, color="#1d3557", s=42)
    if ridx is not None:
        i = int(ridx)
        ax.scatter([t[i]], [e[i]], marker="*", s=220, color="#d62828", edgecolors="black", linewidths=0.8, label="recommended")
    cb = fig.colorbar(sc, ax=ax)
    cb.set_label("avg_safety_proxy")
    ax.set_xlabel("avg_time_s (lower is better)")
    ax.set_ylabel("avg_energy (lower is better)")
    ax.set_title("P5 Pareto Frontier (Time-Energy-Safety)")
    ax.grid(alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(p1)
    plt.close(fig)

    ids = [str(m["profile_id"]) for m in metrics]
    scores = np.array([float(m["scalar_score"]) for m in metrics], dtype=float)
    ord_idx = np.argsort(scores)[:15]
    fig2, ax2 = plt.subplots(figsize=(10.0, 6.0), dpi=160)
    bars = ax2.bar(np.arange(len(ord_idx)), scores[ord_idx], color="#457b9d")
    for k, i in enumerate(ord_idx):
        if i in pidx:
            bars[k].set_color("#1d3557")
        if ridx is not None and int(i) == int(ridx):
            bars[k].set_color("#d62828")
    ax2.set_xticks(np.arange(len(ord_idx)))
    ax2.set_xticklabels([ids[i] for i in ord_idx], rotation=45, ha="right")
    ax2.set_ylabel("scalar_score (lower is better)")
    ax2.set_title("Top-15 Profiles by Scalar Score")
    ax2.grid(axis="y", alpha=0.25)
    fig2.tight_layout()
    fig2.savefig(p2)
    plt.close(fig2)

    print("p5_viz_file", p1)
    print("p5_viz_file", p2)


if __name__ == "__main__":
    main()

