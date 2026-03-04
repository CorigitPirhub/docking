from __future__ import annotations

from dataclasses import dataclass

from strategy.p6_system_eval import compute_feasibility_proxy_accuracy, summarize_policy


@dataclass(frozen=True)
class Snap:
    t: float
    pending_docks: dict[int, int]
    feasible: bool


@dataclass(frozen=True)
class Ev:
    t: float
    event: str
    vehicle_id: int | None = None


def test_feasibility_proxy_accuracy_basic() -> None:
    snapshots = [
        Snap(t=1.0, pending_docks={2: 1}, feasible=True),
        Snap(t=2.0, pending_docks={2: 1}, feasible=False),
        Snap(t=3.0, pending_docks={}, feasible=True),
    ]
    events = [
        Ev(t=1.8, event="DOCK_LOCKED", vehicle_id=2),
    ]
    c, n = compute_feasibility_proxy_accuracy(snapshots, events)
    assert n == 2
    assert c == 2


def test_summarize_policy() -> None:
    rows = [
        {
            "success": True,
            "done_within_60": True,
            "collision_count": 0,
            "done_time_s": 32.0,
            "total_energy": 12.0,
            "feasibility_correct": 8,
            "feasibility_total": 10,
        },
        {
            "success": False,
            "done_within_60": False,
            "collision_count": 1,
            "done_time_s": 60.0,
            "total_energy": 20.0,
            "feasibility_correct": 1,
            "feasibility_total": 2,
        },
    ]
    agg = summarize_policy("p", rows)
    assert agg.num_runs == 2
    assert abs(agg.success_rate - 0.5) < 1e-9
    assert agg.collision_total == 1
    assert abs(agg.avg_done_time_s - 46.0) < 1e-9
    assert abs(agg.feasibility_accuracy - 0.75) < 1e-9
