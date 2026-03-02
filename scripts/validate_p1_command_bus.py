#!/usr/bin/env python3
from __future__ import annotations

import json
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from runtime.command_bus import (  # noqa: E402
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


def mk_dock(cid: str, prio: int, issued: float, deadline: float, seq: int, follower: int, leader: int) -> DockingCommand:
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


def mk_split(cid: str, prio: int, issued: float, deadline: float, seq: int, parent: int, child: int) -> SplitCommand:
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


def mk_wait(cid: str, prio: int, issued: float, deadline: float, seq: int, vid: int, dur: float = 1.0) -> WaitCommand:
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


def check_interface_consistency() -> tuple[float, dict]:
    cmd = mk_dock("ifc_1", 1, 0.0, 1.0, 0, 2, 1)
    fb = CommandFeedback(
        command_id=cmd.header.command_id,
        kind=cmd.kind,
        stage=FeedbackStage.SUBMIT,
        status=FeedbackStatus.QUEUED,
        accepted=True,
        error_code=ErrorCode.OK,
        t=0.0,
    )
    checks = {
        "header_has_state_seq": hasattr(cmd.header, "state_seq"),
        "header_has_priority": hasattr(cmd.header, "priority"),
        "header_has_deadline_at": hasattr(cmd.header, "deadline_at"),
        "header_has_revoke": hasattr(cmd.header, "revoke"),
        "feedback_has_stage": hasattr(fb, "stage"),
        "feedback_has_status": hasattr(fb, "status"),
        "feedback_has_error_code": hasattr(fb, "error_code"),
        "feedback_has_accepted": hasattr(fb, "accepted"),
    }
    passed = sum(1 for v in checks.values() if v)
    return passed / max(1, len(checks)), checks


def check_dedup(trials: int = 200) -> float:
    bus = CommandBus(max_queue=512)
    now = 0.0
    seq = 0
    ok = 0
    for i in range(trials):
        cid = f"dup_{i}"
        c1 = mk_wait(cid, 0, now, now + 10.0, seq, 1)
        c2 = mk_wait(cid, 0, now, now + 10.0, seq, 1)
        f1 = bus.submit(c1, now, seq)
        f2 = bus.submit(c2, now, seq)
        if f1.accepted and (not f2.accepted) and f2.error_code == ErrorCode.DUPLICATE_COMMAND_ID:
            ok += 1
    return ok / float(trials)


def check_expiry(trials: int = 200) -> float:
    bus = CommandBus(max_queue=512)
    seq = 0
    ok = 0
    for i in range(trials):
        now = 10.0 + i * 0.01
        cmd = mk_wait(f"exp_{i}", 0, issued=0.0, deadline=5.0, seq=seq, vid=1)
        fb = bus.submit(cmd, now, seq)
        if (not fb.accepted) and fb.error_code == ErrorCode.EXPIRED:
            ok += 1
    return ok / float(trials)


def check_arbitration_accuracy(trials: int = 1500) -> float:
    rng = random.Random(123)
    exact = 0
    for t in range(trials):
        bus = CommandBus(max_queue=512)
        now = 10.0 + 0.02 * t
        seq = 4
        cmds = []
        for i in range(20):
            kind = rng.choice([CommandKind.DOCK, CommandKind.SPLIT, CommandKind.WAIT])
            cid = f"t{t}_c{i}"
            prio = rng.randint(0, 15)
            issued = now - rng.random()
            deadline = now + 3.0
            if kind == CommandKind.DOCK:
                l = rng.randint(1, 7)
                f = rng.randint(1, 7)
                if f == l:
                    f = 7 if l != 7 else 6
                c = mk_dock(cid, prio, issued, deadline, seq, f, l)
            elif kind == CommandKind.SPLIT:
                p = rng.randint(1, 7)
                ch = rng.randint(1, 7)
                if ch == p:
                    ch = 7 if p != 7 else 6
                c = mk_split(cid, prio, issued, deadline, seq, p, ch)
            else:
                c = mk_wait(cid, prio, issued, deadline, seq, rng.randint(1, 7), 1.0)
            fb = bus.submit(c, now, seq)
            if fb.accepted:
                cmds.append(c)

        ordered = sorted(cmds, key=lambda c: (-c.header.priority, c.header.issued_at, c.header.command_id))
        expected: list[str] = []
        used: set[tuple[str, int]] = set()
        for c in ordered:
            res = command_resources(c)
            if res & used:
                continue
            expected.append(c.header.command_id)
            used |= res

        out = bus.dispatch(now=now + 0.001, current_state_seq=seq, max_dispatch=1000)
        got = [c.header.command_id for c in out]
        if got == expected:
            exact += 1
    return exact / float(trials)


def main() -> None:
    interface_rate, interface_checks = check_interface_consistency()
    dedup_rate = check_dedup()
    expiry_rate = check_expiry()
    arbitration_rate = check_arbitration_accuracy()

    report = {
        "interface_consistency_rate": interface_rate,
        "dedup_accuracy": dedup_rate,
        "expiry_reject_accuracy": expiry_rate,
        "arbitration_accuracy": arbitration_rate,
        "thresholds": {
            "interface_consistency_rate": 1.0,
            "dedup_accuracy": 1.0,
            "expiry_reject_accuracy": 1.0,
            "arbitration_accuracy": 0.995,
        },
        "interface_checks": interface_checks,
    }
    report["ok"] = bool(
        report["interface_consistency_rate"] >= report["thresholds"]["interface_consistency_rate"]
        and report["dedup_accuracy"] >= report["thresholds"]["dedup_accuracy"]
        and report["expiry_reject_accuracy"] >= report["thresholds"]["expiry_reject_accuracy"]
        and report["arbitration_accuracy"] >= report["thresholds"]["arbitration_accuracy"]
    )

    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)
    json_path = out_dir / "p1_command_bus_validation.json"
    md_path = out_dir / "P1_COMMAND_BUS_REPORT.md"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = [
        "# P1 Command Bus Validation",
        "",
        f"- interface_consistency_rate: {report['interface_consistency_rate']:.4f}",
        f"- dedup_accuracy: {report['dedup_accuracy']:.4f}",
        f"- expiry_reject_accuracy: {report['expiry_reject_accuracy']:.4f}",
        f"- arbitration_accuracy: {report['arbitration_accuracy']:.4f}",
        f"- ok: {report['ok']}",
        "",
        f"JSON: {json_path}",
    ]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("p1_validation_json", json_path)
    print("p1_validation_md", md_path)
    print("p1_validation_ok", report["ok"])


if __name__ == "__main__":
    main()

