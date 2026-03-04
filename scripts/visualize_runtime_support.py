#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.runtime_support import LayerName, ReconfigRuntimeEngine
from runtime.command_bus import CommandHeader, DockingCommand, SplitCommand, WaitCommand


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


def run_demo():
    cfg = load_config()
    engine = ReconfigRuntimeEngine(cfg, [1, 2, 3, 4, 5])

    schedule = [
        (0.0, "dock", (2, 1)),
        (2.0, "dock", (3, 2)),
        (4.8, "wait", (5, 3.0)),
        (6.2, "split", (2, 3)),
        (7.0, "dock", (4, 2)),
        (11.8, "dock", (5, 4)),
    ]
    submit_idx = {"i": 0}
    pending_seen: dict[int, float] = {}

    def leader_remaining_fn(t: float) -> float:
        if t < 12.0:
            return max(0.0, 45.0 - 2.2 * t)
        return max(0.0, 8.0 - 1.5 * (t - 12.0))

    def low_hook(rt: ReconfigRuntimeEngine, t: float) -> None:
        while submit_idx["i"] < len(schedule) and t >= schedule[submit_idx["i"]][0] - 1e-9:
            ts, typ, payload = schedule[submit_idx["i"]]
            cid = f"demo_{submit_idx['i']}"
            hdr = CommandHeader(
                command_id=cid,
                state_seq=int(rt.state_seq),
                issued_at=float(ts),
                deadline_at=float(ts + cfg.coordinator.command_ttl_s),
                priority=5,
                source="visualize_runtime_support",
            )
            if typ == "dock":
                f, l = payload
                rt.submit_command(DockingCommand(header=hdr, follower_id=f, leader_id=l), now=t)
            elif typ == "split":
                p, c = payload
                rt.submit_command(SplitCommand(header=hdr, parent_id=p, child_id=c, reason="demo_split"), now=t)
            else:
                v, dur = payload
                rt.submit_command(WaitCommand(header=hdr, vehicle_id=v, duration_s=dur, reason="demo_wait"), now=t)
            submit_idx["i"] += 1

        # Emulate successful docking after stable approach delay.
        for f, l in list(rt.topology.docking_target.items()):
            pending_seen.setdefault(f, t)
            if t - pending_seen[f] >= 0.9:
                rt.mark_docking_locked(f, l, now=t)
        for f in list(pending_seen.keys()):
            if f not in rt.topology.docking_target:
                pending_seen.pop(f, None)

    stats = engine.run(duration_s=16.0, leader_remaining_fn=leader_remaining_fn, low_tick_hook=low_hook)
    return cfg, engine, stats


def plot_ticks(stats, out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 3), dpi=160)
    ymap = {LayerName.HIGH.value: 2, LayerName.MID.value: 1, LayerName.LOW.value: 0}
    colors = {LayerName.HIGH.value: "tab:red", LayerName.MID.value: "tab:orange", LayerName.LOW.value: "tab:blue"}

    for t, layer in stats.tick_trace:
        ax.scatter([t], [ymap[layer]], s=6, c=colors[layer])

    ax.set_yticks([0, 1, 2], ["LOW", "MID", "HIGH"])
    ax.set_xlabel("time [s]")
    ax.set_title("Multi-Rate Tick Schedule")
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def plot_topology_timeline(engine: ReconfigRuntimeEngine, out_path: Path) -> None:
    ts = np.array([s.t for s in engine.snapshots], dtype=float)
    edge_n = np.array([len(s.edges) for s in engine.snapshots], dtype=float)
    pending_n = np.array([len(s.pending_docks) for s in engine.snapshots], dtype=float)
    largest_n = np.array([largest_train_size(s.edges, engine.topology.vehicle_ids) for s in engine.snapshots], dtype=float)
    feasible = np.array([1.0 if s.feasible else 0.0 for s in engine.snapshots], dtype=float)

    fig, ax = plt.subplots(figsize=(10, 4), dpi=160)
    ax.plot(ts, edge_n, label="edge_count", linewidth=1.8)
    ax.plot(ts, largest_n, label="largest_train_size", linewidth=1.8)
    ax.plot(ts, pending_n, label="pending_docks", linewidth=1.6)
    ax.plot(ts, feasible, label="feasible_flag", linewidth=1.4, linestyle="--")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("count / flag")
    ax.set_title("Topology Evolution Timeline")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def plot_feasibility(engine: ReconfigRuntimeEngine, out_path: Path) -> None:
    if not engine.decisions:
        return
    t = np.array([d.t for d in engine.decisions], dtype=float)
    leader_r = np.array([d.leader_remaining_s for d in engine.decisions], dtype=float)
    t_est = np.array([d.t_est for d in engine.decisions], dtype=float)
    feasible = np.array([1.0 if d.feasible else 0.0 for d in engine.decisions], dtype=float)

    fig, ax = plt.subplots(figsize=(10, 4), dpi=160)
    ax.plot(t, leader_r, label="leader_remaining_s", linewidth=1.9)
    ax.plot(t, t_est, label="T_est", linewidth=1.9)
    ax.fill_between(t, 0.0, np.max(np.maximum(leader_r, t_est)) * feasible, color="tab:green", alpha=0.08, label="feasible")

    for e in engine.events:
        if e.event_type.value in {"DOCKING_ABORTED", "DOCK_LOCKED", "SPLIT_DONE"}:
            ax.axvline(e.t, color="gray", alpha=0.15, linewidth=0.8)

    ax.set_xlabel("time [s]")
    ax.set_ylabel("time budget [s]")
    ax.set_title("Task Feasibility Monitor (T_est vs Leader Remaining)")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main() -> None:
    _, engine, stats = run_demo()
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)

    p_ticks = out_dir / "runtime_multirate_ticks.png"
    p_topo = out_dir / "runtime_topology_timeline.png"
    p_feas = out_dir / "runtime_feasibility_timeline.png"

    plot_ticks(stats, p_ticks)
    plot_topology_timeline(engine, p_topo)
    plot_feasibility(engine, p_feas)

    print("runtime_support_png", p_ticks)
    print("runtime_support_png", p_topo)
    print("runtime_support_png", p_feas)


if __name__ == "__main__":
    main()
