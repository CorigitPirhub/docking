from __future__ import annotations

import random

from runtime.command_bus import (
    CommandBus,
    CommandFeedback,
    CommandHeader,
    CommandKind,
    DockingCommand,
    ErrorCode,
    FeedbackStage,
    FeedbackStatus,
    SplitCommand,
    WaitCommand,
    command_resources,
)


def _mk_dock(cid: str, prio: int, issued: float, deadline: float, seq: int, follower: int, leader: int) -> DockingCommand:
    return DockingCommand(
        header=CommandHeader(
            command_id=cid,
            state_seq=seq,
            issued_at=issued,
            deadline_at=deadline,
            priority=prio,
        ),
        follower_id=follower,
        leader_id=leader,
    )


def _mk_split(cid: str, prio: int, issued: float, deadline: float, seq: int, parent: int, child: int) -> SplitCommand:
    return SplitCommand(
        header=CommandHeader(
            command_id=cid,
            state_seq=seq,
            issued_at=issued,
            deadline_at=deadline,
            priority=prio,
        ),
        parent_id=parent,
        child_id=child,
    )


def _mk_wait(cid: str, prio: int, issued: float, deadline: float, seq: int, vid: int, dur: float = 1.0) -> WaitCommand:
    return WaitCommand(
        header=CommandHeader(
            command_id=cid,
            state_seq=seq,
            issued_at=issued,
            deadline_at=deadline,
            priority=prio,
        ),
        vehicle_id=vid,
        duration_s=dur,
    )


def test_interface_consistency_fields_present() -> None:
    cmd = _mk_dock("c1", 3, 0.0, 1.0, 0, 2, 1)
    fb = CommandFeedback(
        command_id=cmd.header.command_id,
        kind=cmd.kind,
        stage=FeedbackStage.SUBMIT,
        status=FeedbackStatus.QUEUED,
        accepted=True,
        error_code=ErrorCode.OK,
        t=0.0,
    )
    row = fb.to_dict()
    assert set(row.keys()) == {
        "command_id",
        "kind",
        "stage",
        "status",
        "accepted",
        "error_code",
        "t",
        "detail",
    }


def test_dedup_reject_100_percent() -> None:
    # Keep queue capacity above trial count so this test isolates dedup behavior.
    bus = CommandBus(max_queue=256)
    now = 0.0
    current_seq = 0
    dup_reject = 0
    trials = 100
    for i in range(trials):
        cid = f"dup_{i}"
        c1 = _mk_wait(cid, 1, now, now + 10.0, current_seq, 1)
        c2 = _mk_wait(cid, 1, now, now + 10.0, current_seq, 1)
        fb1 = bus.submit(c1, now, current_seq)
        fb2 = bus.submit(c2, now, current_seq)
        assert fb1.accepted
        if (not fb2.accepted) and fb2.error_code == ErrorCode.DUPLICATE_COMMAND_ID:
            dup_reject += 1
    assert dup_reject == trials


def test_expiry_reject_100_percent() -> None:
    bus = CommandBus(max_queue=32)
    now = 10.0
    current_seq = 0
    reject = 0
    trials = 100
    for i in range(trials):
        cmd = _mk_wait(f"exp_{i}", 0, issued=0.0, deadline=5.0, seq=current_seq, vid=1)
        fb = bus.submit(cmd, now, current_seq)
        if (not fb.accepted) and fb.error_code == ErrorCode.EXPIRED:
            reject += 1
    assert reject == trials


def test_arbitration_priority_and_tie() -> None:
    bus = CommandBus(max_queue=16)
    now = 0.0
    seq = 0

    # conflict on same leader tail
    c_hi = _mk_dock("c_hi", 10, 0.0, 10.0, seq, follower=2, leader=1)
    c_lo = _mk_dock("c_lo", 5, 0.0, 10.0, seq, follower=3, leader=1)
    bus.submit(c_lo, now, seq)
    bus.submit(c_hi, now, seq)
    out = bus.dispatch(now=0.1, current_state_seq=seq, max_dispatch=10)
    assert [c.header.command_id for c in out] == ["c_hi"]

    # tie by priority, earlier issued wins
    bus = CommandBus(max_queue=16)
    a = _mk_wait("a", 1, issued=1.0, deadline=10.0, seq=0, vid=4)
    b = _mk_wait("b", 1, issued=2.0, deadline=10.0, seq=0, vid=4)
    bus.submit(b, 2.0, 0)
    bus.submit(a, 2.0, 0)
    out = bus.dispatch(now=2.1, current_state_seq=0, max_dispatch=10)
    assert [c.header.command_id for c in out] == ["a"]


def test_arbitration_accuracy_random_stress() -> None:
    rng = random.Random(7)
    trials = 1200
    exact_match = 0

    for t in range(trials):
        bus = CommandBus(max_queue=512)
        now = 10.0 + 0.01 * t
        seq = 5
        cmds = []
        n = 18
        for i in range(n):
            kind = rng.choice([CommandKind.DOCK, CommandKind.SPLIT, CommandKind.WAIT])
            cid = f"t{t}_c{i}"
            prio = rng.randint(0, 12)
            issued = now - rng.random()
            deadline = now + 5.0
            if kind == CommandKind.DOCK:
                leader = rng.randint(1, 6)
                follower = rng.randint(1, 6)
                if follower == leader:
                    follower = 6 if leader != 6 else 5
                cmd = _mk_dock(cid, prio, issued, deadline, seq, follower, leader)
            elif kind == CommandKind.SPLIT:
                p = rng.randint(1, 6)
                c = rng.randint(1, 6)
                if c == p:
                    c = 6 if p != 6 else 5
                cmd = _mk_split(cid, prio, issued, deadline, seq, p, c)
            else:
                v = rng.randint(1, 6)
                cmd = _mk_wait(cid, prio, issued, deadline, seq, v, dur=1.0)
            fb = bus.submit(cmd, now, seq)
            assert fb.accepted
            cmds.append(cmd)

        # independent expected arbitration (same frozen rule).
        ordered = sorted(cmds, key=lambda c: (-c.header.priority, c.header.issued_at, c.header.command_id))
        expected: list[str] = []
        occupied: set[tuple[str, int]] = set()
        for c in ordered:
            res = command_resources(c)
            if res & occupied:
                continue
            expected.append(c.header.command_id)
            occupied |= res

        out = bus.dispatch(now=now + 0.001, current_state_seq=seq, max_dispatch=999)
        got = [c.header.command_id for c in out]
        if got == expected:
            exact_match += 1

    accuracy = exact_match / float(trials)
    assert accuracy >= 0.995
