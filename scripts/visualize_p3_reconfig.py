#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _score_result(r: dict[str, Any]) -> float:
    n_events = len(r["events"])
    acc = float(r.get("c_accuracy", 1.0))
    return float(0.6 * n_events + 2.0 * acc)


def _pick_rep(results: list[dict[str, Any]], prefix: str) -> dict[str, Any]:
    rows = [r for r in results if str(r["subtype"]).startswith(prefix)]
    if not rows:
        raise ValueError(f"no result with prefix {prefix}")
    rows.sort(key=_score_result, reverse=True)
    return rows[0]


def _plot_timeline(r: dict[str, Any], out_path: Path, title: str) -> None:
    trace = r["size_profile"]
    s = np.array([float(x["s"]) for x in trace], dtype=float)
    n = np.array([float(x["train_size"]) for x in trace], dtype=float)
    nmax = np.array([float(x["nmax"]) for x in trace], dtype=float)

    fig, ax = plt.subplots(figsize=(10.5, 4.2), dpi=170)
    ax.plot(s, nmax, color="#6c757d", linewidth=2.0, label="n_max_pass(s)")
    ax.plot(s, n, color="#2a9d8f", linewidth=2.2, label="planned_train_size(s)")
    ax.fill_between(s, n, nmax, where=(n > nmax), color="#ef233c", alpha=0.20, label="over-limit")

    for ev in r["events"]:
        se = float(ev["s"])
        if ev["event_type"] == "SPLIT":
            ax.axvline(se, color="#e63946", alpha=0.35, linewidth=1.1)
        else:
            ax.axvline(se, color="#3a86ff", alpha=0.35, linewidth=1.1)

    txt = (
        f"scenario={r['scenario_id']}  subtype={r['subtype']}\n"
        f"events={len(r['events'])}  violations={r['violation_count']}  "
        f"collisions={r['collision_count']}  c_acc={float(r.get('c_accuracy', 1.0)):.3f}"
    )
    ax.text(0.01, 0.98, txt, transform=ax.transAxes, va="top", ha="left", fontsize=8.2)
    ax.set_xlabel("Path Coordinate s [m]")
    ax.set_ylabel("Train Size")
    ax.set_title(title)
    ax.grid(alpha=0.25)
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _plot_accuracy_hist(results: list[dict[str, Any]], out_path: Path) -> None:
    c = [float(r.get("c_accuracy", 1.0)) for r in results if str(r["subtype"]).startswith("C")]
    fig, ax = plt.subplots(figsize=(7.4, 4.4), dpi=170)
    ax.hist(c, bins=12, color="#2a9d8f", edgecolor="black", alpha=0.75)
    ax.axvline(0.90, color="#e63946", linestyle="--", linewidth=1.3, label="threshold 0.90")
    ax.set_xlabel("Type-C Decision Accuracy")
    ax.set_ylabel("Count")
    ax.set_title("P3 Type-C Decision Accuracy Distribution")
    ax.grid(alpha=0.22, axis="y")
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main() -> None:
    in_path = ROOT / "experiments" / "p3_reconfig_results.json"
    data = _load(in_path)
    results = data["summary"]["results"]
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)

    rep_b = _pick_rep(results, "B")
    rep_c = _pick_rep(results, "C")

    p_b = out_dir / f"p3_timeline_B_{rep_b['scenario_id']}.png"
    p_c = out_dir / f"p3_timeline_C_{rep_c['scenario_id']}.png"
    p_h = out_dir / "p3_typec_accuracy_hist.png"
    _plot_timeline(rep_b, p_b, title="P3 Reconfig Timeline (Type B Representative)")
    _plot_timeline(rep_c, p_c, title="P3 Reconfig Timeline (Type C Representative)")
    _plot_accuracy_hist(results, p_h)

    alias_b = out_dir / "p3_timeline_B_representative.png"
    alias_c = out_dir / "p3_timeline_C_representative.png"
    alias_b.write_bytes(p_b.read_bytes())
    alias_c.write_bytes(p_c.read_bytes())

    print("p3_viz_file", p_b)
    print("p3_viz_file", p_c)
    print("p3_viz_file", p_h)
    print("p3_viz_file", alias_b)
    print("p3_viz_file", alias_c)


if __name__ == "__main__":
    main()
