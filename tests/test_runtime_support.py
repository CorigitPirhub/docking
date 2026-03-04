import numpy as np

from docking.config import load_config
from docking.runtime_support import (
    MultiRateExecutor,
    ReconfigRuntimeEngine,
    TaskFeasibilityMonitor,
    TopologyStateMachine,
    estimate_leader_remaining_time,
)
from docking.types import VehicleMode
from runtime.command_bus import CommandHeader, DockingCommand


def test_command_interface_and_docking_execution():
    cfg = load_config()
    engine = ReconfigRuntimeEngine(cfg, [1, 2, 3])

    ack0 = engine.submit_command(
        DockingCommand(
            header=CommandHeader(command_id="dock_1", state_seq=0, issued_at=0.0, deadline_at=5.0, priority=5),
            follower_id=2,
            leader_id=1,
        ),
        now=0.0,
    )
    assert ack0.accepted

    # duplicate command id should be rejected
    ack_dup = engine.submit_command(
        DockingCommand(
            header=CommandHeader(command_id="dock_1", state_seq=0, issued_at=0.0, deadline_at=5.0, priority=5),
            follower_id=3,
            leader_id=1,
        ),
        now=0.0,
    )
    assert not ack_dup.accepted

    locked = {"done": False}

    def low_hook(rt: ReconfigRuntimeEngine, t: float) -> None:
        if (not locked["done"]) and t >= 0.2:
            locked["done"] = rt.mark_docking_locked(follower_id=2, leader_id=1, now=t)

    stats = engine.run(duration_s=0.6, leader_remaining_fn=lambda _t: 60.0, low_tick_hook=low_hook)
    assert stats.ticks_low > 0
    assert locked["done"]
    assert (1, 2) in set(engine.topology.edges())
    assert engine.topology.mode[2] == VehicleMode.TRAIN_FOLLOW


def test_topology_state_machine_split_and_cycle_guard():
    topo = TopologyStateMachine([1, 2, 3])
    ok, _ = topo.start_docking(2, 1, now=0.0)
    assert ok
    ok, _ = topo.finalize_docking(2, 1, now=0.1)
    assert ok

    ok, _ = topo.start_docking(3, 2, now=0.2)
    assert ok
    ok, _ = topo.finalize_docking(3, 2, now=0.3)
    assert ok

    can, reason = topo.can_start_docking(1, 3)
    assert not can
    assert reason == "cycle_risk"

    ok, _ = topo.apply_split(2, 3, now=0.4)
    assert ok
    assert topo.parent[3] is None
    assert topo.child[2] is None
    inv_ok, _ = topo.check_invariants()
    assert inv_ok


def test_feasibility_abort_switch_logic():
    cfg = load_config()
    engine = ReconfigRuntimeEngine(cfg, [1, 2, 3])
    engine.submit_command(
        DockingCommand(
            header=CommandHeader(command_id="dock_abort", state_seq=0, issued_at=0.0, deadline_at=5.0, priority=5),
            follower_id=2,
            leader_id=1,
        ),
        now=0.0,
    )

    def leader_remaining_fn(t: float) -> float:
        # At t=0 feasible for one pending dock: 10 + 5 <= 20
        # At t>=1 infeasible: 10 + 5 > 4
        return 20.0 if t < 1.0 else 4.0

    engine.run(duration_s=2.1, leader_remaining_fn=leader_remaining_fn)

    abort_events = [e for e in engine.events if e.event_type.value == "DOCKING_ABORTED"]
    assert len(abort_events) >= 1
    assert engine.topology.mode[2] == VehicleMode.FREE
    assert any(d.flipped and (not d.feasible) for d in engine.decisions)


def test_feasibility_monitor_no_pending_is_always_feasible():
    cfg = load_config()
    mon = TaskFeasibilityMonitor(cfg.coordinator)
    d0 = mon.evaluate(pending_docks=0, leader_remaining_s=0.0, now=0.0)
    assert d0.feasible
    assert d0.reason == "ok_no_pending_docks"

    # Even if previously infeasible, no pending docks should recover immediately.
    _ = mon.evaluate(pending_docks=2, leader_remaining_s=0.5, now=1.0)
    d2 = mon.evaluate(pending_docks=0, leader_remaining_s=0.1, now=2.0)
    assert d2.feasible
    assert d2.reason == "ok_no_pending_docks"


def test_multirate_executor_counts_and_order():
    cfg = load_config()
    ex = MultiRateExecutor(cfg)
    calls: list[tuple[float, str]] = []

    stats = ex.run(
        duration_s=2.0,
        on_high=lambda t, _k: calls.append((t, "HIGH")),
        on_mid=lambda t, _k: calls.append((t, "MID")),
        on_low=lambda t, _k: calls.append((t, "LOW")),
    )

    # With inclusive t=0 and t=2.0
    assert stats.ticks_high == 3
    assert stats.ticks_mid == 21
    assert stats.ticks_low == 41

    # For timestamps where all layers tick together, order should be HIGH->MID->LOW.
    by_t: dict[float, list[str]] = {}
    for t, layer in calls:
        by_t.setdefault(round(t, 6), []).append(layer)
    for k in (0.0, 1.0, 2.0):
        layers = by_t[round(k, 6)]
        assert layers[0] == "HIGH"
        assert ("MID" in layers) and ("LOW" in layers)
        assert layers.index("MID") < layers.index("LOW")


def test_estimate_leader_remaining_time_basic_properties():
    t1 = estimate_leader_remaining_time(distance_remaining=10.0, speed_now=0.2, v_max=2.0, a_max=1.0)
    t2 = estimate_leader_remaining_time(distance_remaining=20.0, speed_now=0.2, v_max=2.0, a_max=1.0)
    assert t2 > t1 > 0.0

    t_fast = estimate_leader_remaining_time(distance_remaining=10.0, speed_now=1.5, v_max=2.0, a_max=1.0)
    assert t_fast < t1


def test_topology_randomized_invariants_stress():
    rng = np.random.default_rng(7)
    topo = TopologyStateMachine([1, 2, 3, 4, 5, 6])

    for k in range(1200):
        now = 0.05 * k
        op = int(rng.integers(0, 6))
        ids = [1, 2, 3, 4, 5, 6]

        if op == 0:
            f = int(rng.choice(ids))
            l = int(rng.choice(ids))
            topo.start_docking(f, l, now)
        elif op == 1:
            if topo.docking_target:
                f = int(rng.choice(list(topo.docking_target.keys())))
                topo.finalize_docking(f, topo.docking_target[f], now)
        elif op == 2:
            if topo.docking_target:
                f = int(rng.choice(list(topo.docking_target.keys())))
                topo.abort_docking(f, now)
        elif op == 3:
            f = int(rng.choice(ids))
            topo.apply_wait(f, duration_s=float(rng.uniform(0.0, 0.5)), now=now)
        elif op == 4:
            edges = topo.edges()
            if edges:
                p, c = edges[int(rng.integers(0, len(edges)))]
                topo.apply_split(p, c, now)
        else:
            topo.tick(now)

        ok, reason = topo.check_invariants()
        assert ok, reason
