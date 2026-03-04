#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.runtime_support import ReconfigRuntimeEngine
from runtime.command_bus import (
    CommandHeader,
    DockingCommand,
    FeedbackStage,
    FeedbackStatus,
    SplitCommand,
    WaitCommand,
)


@dataclass
class EpisodeMetrics:
    seed: int
    invariant_ok: bool
    command_submit_total: int
    command_execute_accept: int
    command_execute_reject: int
    dock_locked: int
    split_done: int
    docking_aborted: int
    feasibility_flips: int
    ticks_high: int
    ticks_mid: int
    ticks_low: int


def run_episode(seed: int, duration_s: float = 30.0) -> EpisodeMetrics:
    cfg = load_config()
    rng = np.random.default_rng(seed)
    engine = ReconfigRuntimeEngine(cfg, [1, 2, 3, 4, 5, 6])
    submit_count = 0
    next_submit = 0.0
    issued_ids = 0
    first_seen_pending: dict[int, float] = {}

    def leader_remaining_fn(t: float) -> float:
        decay = 1.3 + 0.6 * (seed % 5) / 4.0
        if t < 18.0:
            return max(0.0, 60.0 - decay * t)
        return max(0.0, 10.0 - 2.0 * (t - 18.0))

    def issue_random_cmd(t: float) -> None:
        nonlocal issued_ids, submit_count
        choice = float(rng.random())
        cmd_id = f"ep{seed}_cmd{issued_ids}"
        issued_ids += 1
        ttl = float(cfg.coordinator.command_ttl_s)
        hdr = CommandHeader(
            command_id=cmd_id,
            state_seq=int(engine.state_seq),
            issued_at=float(t),
            deadline_at=float(t + ttl),
            priority=0,
            source="validate_runtime_support",
        )
        if choice < 0.6:
            follower = int(rng.integers(2, 7))
            leader = int(rng.integers(1, 7))
            cmd = DockingCommand(
                header=hdr,
                follower_id=follower,
                leader_id=leader,
            )
        elif choice < 0.8:
            # Random split; may be rejected, this is expected in stress tests.
            parent = int(rng.integers(1, 7))
            child = int(rng.integers(1, 7))
            cmd = SplitCommand(
                header=hdr,
                parent_id=parent,
                child_id=child,
                reason="random_split",
            )
        else:
            v = int(rng.integers(1, 7))
            cmd = WaitCommand(
                header=hdr,
                vehicle_id=v,
                duration_s=float(rng.uniform(0.2, 2.5)),
                reason="random_wait",
            )
        engine.submit_command(cmd, now=t)
        submit_count += 1

    def low_hook(rt: ReconfigRuntimeEngine, t: float) -> None:
        nonlocal next_submit
        if t >= next_submit - 1e-9:
            n_cmd = int(rng.integers(1, 4))
            for _ in range(n_cmd):
                issue_random_cmd(t)
            next_submit += float(rng.uniform(0.15, 0.75))

        # Emulate docking completion with a random lag after docking starts.
        for f, l in list(rt.topology.docking_target.items()):
            first_seen_pending.setdefault(f, t)
            lag = t - first_seen_pending[f]
            if lag >= float(rng.uniform(0.4, 1.8)):
                if rt.mark_docking_locked(f, l, now=t):
                    first_seen_pending.pop(f, None)
        for f in list(first_seen_pending.keys()):
            if f not in rt.topology.docking_target:
                first_seen_pending.pop(f, None)

    stats = engine.run(duration_s=duration_s, leader_remaining_fn=leader_remaining_fn, low_tick_hook=low_hook)
    inv_ok, _ = engine.check_invariants()

    exec_fbs = [f for f in engine.feedbacks if f.stage == FeedbackStage.EXEC]
    ev = engine.events
    return EpisodeMetrics(
        seed=seed,
        invariant_ok=inv_ok,
        command_submit_total=submit_count,
        command_execute_accept=sum(1 for f in exec_fbs if f.status == FeedbackStatus.RUNNING),
        command_execute_reject=sum(1 for f in exec_fbs if f.status == FeedbackStatus.REJECTED),
        dock_locked=sum(1 for e in ev if e.event_type.value == "DOCK_LOCKED"),
        split_done=sum(1 for e in ev if e.event_type.value == "SPLIT_DONE"),
        docking_aborted=sum(1 for e in ev if e.event_type.value == "DOCKING_ABORTED"),
        feasibility_flips=sum(1 for e in ev if e.event_type.value == "FEASIBILITY_FLIP"),
        ticks_high=stats.ticks_high,
        ticks_mid=stats.ticks_mid,
        ticks_low=stats.ticks_low,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate command/topology/feasibility/multi-rate support")
    parser.add_argument("--episodes", type=int, default=180)
    args = parser.parse_args()

    cfg = load_config()
    seeds = [cfg.testing.random_seed + i for i in range(args.episodes)]
    metrics = [run_episode(s) for s in seeds]

    invariant_ok_count = sum(1 for m in metrics if m.invariant_ok)
    avg_accept = float(np.mean([m.command_execute_accept for m in metrics])) if metrics else 0.0
    avg_reject = float(np.mean([m.command_execute_reject for m in metrics])) if metrics else 0.0
    avg_locked = float(np.mean([m.dock_locked for m in metrics])) if metrics else 0.0
    avg_abort = float(np.mean([m.docking_aborted for m in metrics])) if metrics else 0.0
    ticks_ok = all((m.ticks_low >= m.ticks_mid >= m.ticks_high > 0) for m in metrics)

    report = {
        "episodes": args.episodes,
        "invariant_ok_rate": invariant_ok_count / max(1, args.episodes),
        "ticks_order_ok": ticks_ok,
        "avg_command_execute_accept": avg_accept,
        "avg_command_execute_reject": avg_reject,
        "avg_dock_locked": avg_locked,
        "avg_docking_aborted": avg_abort,
        "ok": bool(invariant_ok_count == args.episodes and ticks_ok),
        "samples": [asdict(m) for m in metrics[:20]],
    }

    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)
    json_path = out_dir / "runtime_support_validation.json"
    md_path = out_dir / "RUNTIME_SUPPORT_VALIDATION_REPORT.md"

    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = [
        "# Runtime Support Validation Report",
        "",
        f"- episodes: {report['episodes']}",
        f"- invariant_ok_rate: {report['invariant_ok_rate']:.4f}",
        f"- ticks_order_ok: {report['ticks_order_ok']}",
        f"- avg_command_execute_accept: {report['avg_command_execute_accept']:.2f}",
        f"- avg_command_execute_reject: {report['avg_command_execute_reject']:.2f}",
        f"- avg_dock_locked: {report['avg_dock_locked']:.2f}",
        f"- avg_docking_aborted: {report['avg_docking_aborted']:.2f}",
        f"- ok: {report['ok']}",
        "",
        f"JSON: {json_path}",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("runtime_validation_json", json_path)
    print("runtime_validation_md", md_path)
    print("runtime_validation_ok", report["ok"])


if __name__ == "__main__":
    main()
