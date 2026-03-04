from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class PolicyAggregate:
    policy_id: str
    num_runs: int
    success_rate: float
    done_within_60_rate: float
    collision_total: int
    avg_done_time_s: float
    avg_energy: float
    feasibility_accuracy: float
    feasibility_samples: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "policy_id": str(self.policy_id),
            "num_runs": int(self.num_runs),
            "success_rate": float(self.success_rate),
            "done_within_60_rate": float(self.done_within_60_rate),
            "collision_total": int(self.collision_total),
            "avg_done_time_s": float(self.avg_done_time_s),
            "avg_energy": float(self.avg_energy),
            "feasibility_accuracy": float(self.feasibility_accuracy),
            "feasibility_samples": int(self.feasibility_samples),
        }


def _ev_value(ev: Any, key: str, default: Any = None) -> Any:
    if hasattr(ev, key):
        return getattr(ev, key)
    if isinstance(ev, dict):
        return ev.get(key, default)
    return default


def _snap_value(snap: Any, key: str, default: Any = None) -> Any:
    if hasattr(snap, key):
        return getattr(snap, key)
    if isinstance(snap, dict):
        return snap.get(key, default)
    return default


def compute_feasibility_proxy_accuracy(snapshots: list[Any], events: list[Any]) -> tuple[int, int]:
    """
    Proxy definition:
    - Prediction: runtime snapshot `feasible` when there are pending dock followers.
    - Oracle: all currently pending followers eventually receive DOCK_LOCKED (at t >= snapshot.t).

    Returns (correct_count, sample_count).
    """
    lock_time: dict[int, float] = {}
    for ev in events:
        if str(_ev_value(ev, "event", "")).upper() != "DOCK_LOCKED":
            continue
        follower = _ev_value(ev, "vehicle_id", None)
        t = float(_ev_value(ev, "t", 0.0))
        if follower is None:
            continue
        fid = int(follower)
        if fid not in lock_time:
            lock_time[fid] = t
        else:
            lock_time[fid] = min(lock_time[fid], t)

    correct = 0
    total = 0
    for snap in snapshots:
        pending = _snap_value(snap, "pending_docks", {})
        if not pending:
            continue
        followers = [int(fid) for fid in pending.keys()]
        t = float(_snap_value(snap, "t", 0.0))
        pred = bool(_snap_value(snap, "feasible", True))
        oracle = all((fid in lock_time) and (lock_time[fid] >= t - 1e-9) for fid in followers)
        total += 1
        correct += int(pred == oracle)
    return int(correct), int(total)


def summarize_policy(policy_id: str, rows: list[dict[str, Any]]) -> PolicyAggregate:
    n = len(rows)
    if n == 0:
        return PolicyAggregate(
            policy_id=policy_id,
            num_runs=0,
            success_rate=0.0,
            done_within_60_rate=0.0,
            collision_total=0,
            avg_done_time_s=0.0,
            avg_energy=0.0,
            feasibility_accuracy=1.0,
            feasibility_samples=0,
        )

    succ = sum(1 for r in rows if bool(r["success"]))
    done_60 = sum(1 for r in rows if bool(r["done_within_60"]))
    coll = sum(int(r["collision_count"]) for r in rows)
    avg_t = sum(float(r["done_time_s"]) for r in rows) / n
    avg_e = sum(float(r["total_energy"]) for r in rows) / n

    feas_correct = sum(int(r["feasibility_correct"]) for r in rows)
    feas_total = sum(int(r["feasibility_total"]) for r in rows)
    feas_acc = 1.0 if feas_total <= 0 else (feas_correct / feas_total)

    return PolicyAggregate(
        policy_id=policy_id,
        num_runs=n,
        success_rate=float(succ / n),
        done_within_60_rate=float(done_60 / n),
        collision_total=int(coll),
        avg_done_time_s=float(avg_t),
        avg_energy=float(avg_e),
        feasibility_accuracy=float(feas_acc),
        feasibility_samples=int(feas_total),
    )


def per_subtype_aggregate(rows: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for r in rows:
        groups[str(r["subtype"])].append(r)
    out: dict[str, dict[str, Any]] = {}
    for subtype, g in sorted(groups.items()):
        out[subtype] = summarize_policy(policy_id=subtype, rows=g).to_dict()
    return out


def compare_to_baseline(main: PolicyAggregate, baseline: PolicyAggregate) -> dict[str, float]:
    eps = 1e-9
    return {
        "delta_success_rate": float(main.success_rate - baseline.success_rate),
        "delta_done_within_60_rate": float(main.done_within_60_rate - baseline.done_within_60_rate),
        "delta_avg_done_time_s": float(main.avg_done_time_s - baseline.avg_done_time_s),
        "delta_avg_energy_pct": float((main.avg_energy - baseline.avg_energy) / max(baseline.avg_energy, eps)),
        "delta_collision_total": float(main.collision_total - baseline.collision_total),
    }


def failure_event_counter(events: list[Any]) -> dict[str, int]:
    c = Counter(str(_ev_value(ev, "event", "")) for ev in events)
    return {k: int(v) for k, v in sorted(c.items()) if k}
