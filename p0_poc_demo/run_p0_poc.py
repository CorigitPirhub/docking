#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from collections import Counter
from dataclasses import asdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.runtime_support import ReconfigRuntimeEngine
from p0_poc_demo.simple_strategy import P0RuleStrategy


def largest_train_size(edges: tuple[tuple[int, int], ...], vehicle_ids: tuple[int, ...]) -> int:
    parent = {v: None for v in vehicle_ids}
    child = {v: None for v in vehicle_ids}
    for p, c in edges:
        parent[c] = p
        child[p] = c
    heads = [v for v in vehicle_ids if parent[v] is None]
    best = 0
    for h in heads:
        n = 1
        cur = child[h]
        while cur is not None:
            n += 1
            cur = child[cur]
        best = max(best, n)
    return best


def run_demo(duration_s: float = 24.0) -> tuple[ReconfigRuntimeEngine, dict]:
    cfg = load_config()
    vehicle_ids = [1, 2, 3, 4, 5]
    strategy = P0RuleStrategy(vehicle_ids)
    engine = ReconfigRuntimeEngine(cfg, vehicle_ids)
    pending_seen: dict[int, float] = {}

    def leader_remaining_fn(t: float) -> float:
        # Keep docking feasibility mostly stable in PoC to focus on command-closed-loop behavior.
        return max(0.0, 120.0 - 0.5 * t)

    def low_hook(rt: ReconfigRuntimeEngine, t: float) -> None:
        strategy.on_tick(rt, t)

        # Emulate successful lock after short approach delay.
        for f, l in list(rt.topology.docking_target.items()):
            pending_seen.setdefault(f, t)
            if t - pending_seen[f] >= 0.8:
                locked = rt.mark_docking_locked(f, l, now=t)
                if (not locked) and (t - pending_seen[f] >= 2.0):
                    rt.topology.abort_docking(f, now=t)
        for f in list(pending_seen.keys()):
            if f not in rt.topology.docking_target:
                pending_seen.pop(f, None)

    engine.run(duration_s=duration_s, leader_remaining_fn=leader_remaining_fn, low_tick_hook=low_hook)

    inv_ok, inv_reason = engine.check_invariants()
    execute_acks = [a for a in engine.acks if a.stage == "execute"]
    accept_n = sum(1 for a in execute_acks if a.accepted)
    reject_n = sum(1 for a in execute_acks if not a.accepted)

    ev_counter = Counter(e.event_type.value for e in engine.events)
    snapshots = engine.snapshots
    ts = np.array([s.t for s in snapshots], dtype=float)
    size = np.array([largest_train_size(s.edges, engine.topology.vehicle_ids) for s in snapshots], dtype=float)
    feasible = np.array([1.0 if s.feasible else 0.0 for s in snapshots], dtype=float)

    def max_size(mask: np.ndarray) -> float:
        if not np.any(mask):
            return 0.0
        return float(np.max(size[mask]))

    m_build = max_size(ts < 8.0)
    m_bottle_prep = max_size((ts >= 8.0) & (ts < 12.0))
    m_bottle_hard = max_size((ts >= 12.0) & (ts < 14.0))
    m_after = max_size(ts >= 14.0)

    ok = bool(
        inv_ok
        and ev_counter["DOCK_LOCKED"] >= 2
        and ev_counter["SPLIT_DONE"] >= 1
        and m_bottle_hard <= 1.0 + 1e-9
    )
    metrics = {
        "duration_s": duration_s,
        "invariant_ok": inv_ok,
        "invariant_reason": inv_reason,
        "execute_accept": int(accept_n),
        "execute_reject": int(reject_n),
        "events": dict(ev_counter),
        "max_train_size_build_phase": m_build,
        "max_train_size_bottleneck_prep_phase": m_bottle_prep,
        "max_train_size_bottleneck_hard_phase": m_bottle_hard,
        "max_train_size_after_phase": m_after,
        "final_edges": [list(e) for e in engine.topology.edges()],
        "ok": ok,
    }
    return engine, metrics


def save_plot(engine: ReconfigRuntimeEngine, out_png: Path) -> None:
    snapshots = engine.snapshots
    ts = np.array([s.t for s in snapshots], dtype=float)
    size = np.array([largest_train_size(s.edges, engine.topology.vehicle_ids) for s in snapshots], dtype=float)
    feasible = np.array([1.0 if s.feasible else 0.0 for s in snapshots], dtype=float)

    fig, ax = plt.subplots(figsize=(10, 4), dpi=160)
    ax.plot(ts, size, label="largest_train_size", linewidth=2.0)
    ax.plot(ts, feasible, label="feasible_flag", linewidth=1.6, linestyle="--")
    ax.axvspan(8.0, 14.0, color="tab:red", alpha=0.08, label="bottleneck_window")
    for e in engine.events:
        if e.event_type.value in {"DOCK_LOCKED", "SPLIT_DONE", "DOCKING_ABORTED"}:
            ax.axvline(e.t, color="gray", alpha=0.2, linewidth=0.8)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("size / flag")
    ax.set_title("P0 PoC Timeline: Dock -> Split -> Re-Dock")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)


def main() -> None:
    engine, metrics = run_demo()
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)
    json_path = out_dir / "p0_poc_metrics.json"
    md_path = out_dir / "P0_POC_REPORT.md"
    png_path = out_dir / "p0_poc_timeline.png"

    json_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    save_plot(engine, png_path)

    lines = [
        "# P0 Minimal PoC Report",
        "",
        "- Goal: validate `strategy command -> runtime execution -> feedback` closed loop",
        f"- invariant_ok: {metrics['invariant_ok']} ({metrics['invariant_reason']})",
        f"- execute_accept: {metrics['execute_accept']}",
        f"- execute_reject: {metrics['execute_reject']}",
        f"- DOCK_LOCKED: {metrics['events'].get('DOCK_LOCKED', 0)}",
        f"- SPLIT_DONE: {metrics['events'].get('SPLIT_DONE', 0)}",
        f"- max_train_size_build_phase: {metrics['max_train_size_build_phase']}",
        f"- max_train_size_bottleneck_prep_phase: {metrics['max_train_size_bottleneck_prep_phase']}",
        f"- max_train_size_bottleneck_hard_phase: {metrics['max_train_size_bottleneck_hard_phase']}",
        f"- max_train_size_after_phase: {metrics['max_train_size_after_phase']}",
        f"- final_edges: {metrics['final_edges']}",
        f"- ok: {metrics['ok']}",
        "",
        f"JSON: {json_path}",
        f"Figure: {png_path}",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("p0_poc_json", json_path)
    print("p0_poc_md", md_path)
    print("p0_poc_png", png_path)
    print("p0_poc_ok", metrics["ok"])


if __name__ == "__main__":
    main()
