from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Iterable


class CommandKind(str, Enum):
    DOCK = "DOCK"
    SPLIT = "SPLIT"
    WAIT = "WAIT"


class FeedbackStage(str, Enum):
    SUBMIT = "SUBMIT"
    EXEC = "EXEC"
    DONE = "DONE"


class FeedbackStatus(str, Enum):
    QUEUED = "QUEUED"
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    REJECTED = "REJECTED"


class ErrorCode(str, Enum):
    OK = "OK"
    DUPLICATE_COMMAND_ID = "DUPLICATE_COMMAND_ID"
    INVALID_HEADER = "INVALID_HEADER"
    INVALID_PAYLOAD = "INVALID_PAYLOAD"
    FUTURE_STATE_SEQ = "FUTURE_STATE_SEQ"
    EXPIRED = "EXPIRED"
    QUEUE_FULL = "QUEUE_FULL"
    ARBITRATION_LOST = "ARBITRATION_LOST"
    UNKNOWN_COMMAND = "UNKNOWN_COMMAND"
    EXECUTION_FAILED = "EXECUTION_FAILED"
    REVOKE_APPLIED = "REVOKE_APPLIED"


@dataclass(frozen=True)
class CommandHeader:
    command_id: str
    state_seq: int
    issued_at: float
    deadline_at: float
    priority: int = 0
    cancel_tag: str | None = None
    revoke: bool = False
    can_preempt: bool = False
    rollback_on_reject: bool = False
    source: str = "strategy"

    def is_expired(self, now: float) -> bool:
        return now > self.deadline_at + 1e-9

    def is_valid(self) -> bool:
        if not self.command_id:
            return False
        if self.state_seq < 0:
            return False
        if self.deadline_at < self.issued_at:
            return False
        return True


@dataclass(frozen=True)
class DockingCommand:
    header: CommandHeader
    follower_id: int
    leader_id: int
    intercept_path_s: float | None = None
    kind: CommandKind = CommandKind.DOCK


@dataclass(frozen=True)
class SplitCommand:
    header: CommandHeader
    parent_id: int
    child_id: int
    reason: str = "split"
    kind: CommandKind = CommandKind.SPLIT


@dataclass(frozen=True)
class WaitCommand:
    header: CommandHeader
    vehicle_id: int
    duration_s: float
    reason: str = "wait"
    kind: CommandKind = CommandKind.WAIT


UnifiedCommand = DockingCommand | SplitCommand | WaitCommand


@dataclass(frozen=True)
class CommandFeedback:
    command_id: str
    kind: CommandKind
    stage: FeedbackStage
    status: FeedbackStatus
    accepted: bool
    error_code: ErrorCode
    t: float
    detail: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


def _compare_key(cmd: UnifiedCommand) -> tuple[int, float, str]:
    # Higher priority first, then earlier issue time, then command id.
    return (-int(cmd.header.priority), float(cmd.header.issued_at), cmd.header.command_id)


def command_resources(cmd: UnifiedCommand) -> set[tuple[str, int]]:
    if isinstance(cmd, DockingCommand):
        return {
            ("veh", int(cmd.follower_id)),
            ("tail", int(cmd.leader_id)),
        }
    if isinstance(cmd, SplitCommand):
        return {
            ("veh", int(cmd.parent_id)),
            ("veh", int(cmd.child_id)),
            ("edge_parent", int(cmd.parent_id)),
            ("edge_child", int(cmd.child_id)),
        }
    return {("veh", int(cmd.vehicle_id))}


class CommandBus:
    """A deterministic command bus for strategy-control decoupling."""

    def __init__(self, max_queue: int = 256):
        self.max_queue = max(1, int(max_queue))
        self.queue: list[UnifiedCommand] = []
        self.seen_command_ids: set[str] = set()
        self.inflight: dict[str, UnifiedCommand] = {}
        self.feedbacks: list[CommandFeedback] = []

    def _append_feedback(
        self,
        cmd: UnifiedCommand,
        stage: FeedbackStage,
        status: FeedbackStatus,
        accepted: bool,
        error: ErrorCode,
        now: float,
        detail: str = "",
    ) -> CommandFeedback:
        fb = CommandFeedback(
            command_id=cmd.header.command_id,
            kind=cmd.kind,
            stage=stage,
            status=status,
            accepted=accepted,
            error_code=error,
            t=float(now),
            detail=detail,
        )
        self.feedbacks.append(fb)
        return fb

    @staticmethod
    def _validate_payload(cmd: UnifiedCommand) -> tuple[bool, str]:
        if isinstance(cmd, DockingCommand):
            if cmd.follower_id <= 0 or cmd.leader_id <= 0:
                return False, "vehicle_id_must_be_positive"
            if cmd.follower_id == cmd.leader_id:
                return False, "self_docking_forbidden"
            return True, "ok"
        if isinstance(cmd, SplitCommand):
            if cmd.parent_id <= 0 or cmd.child_id <= 0:
                return False, "vehicle_id_must_be_positive"
            if cmd.parent_id == cmd.child_id:
                return False, "self_split_forbidden"
            return True, "ok"
        if cmd.vehicle_id <= 0:
            return False, "vehicle_id_must_be_positive"
        if cmd.duration_s < 0.0:
            return False, "wait_duration_negative"
        return True, "ok"

    def submit(self, cmd: UnifiedCommand, now: float, current_state_seq: int) -> CommandFeedback:
        if not cmd.header.is_valid():
            return self._append_feedback(
                cmd, FeedbackStage.SUBMIT, FeedbackStatus.REJECTED, False, ErrorCode.INVALID_HEADER, now
            )
        if cmd.header.command_id in self.seen_command_ids:
            return self._append_feedback(
                cmd,
                FeedbackStage.SUBMIT,
                FeedbackStatus.REJECTED,
                False,
                ErrorCode.DUPLICATE_COMMAND_ID,
                now,
            )

        self.seen_command_ids.add(cmd.header.command_id)
        if cmd.header.is_expired(now):
            return self._append_feedback(
                cmd, FeedbackStage.SUBMIT, FeedbackStatus.REJECTED, False, ErrorCode.EXPIRED, now
            )
        if int(cmd.header.state_seq) > int(current_state_seq):
            return self._append_feedback(
                cmd, FeedbackStage.SUBMIT, FeedbackStatus.REJECTED, False, ErrorCode.FUTURE_STATE_SEQ, now
            )
        ok, reason = self._validate_payload(cmd)
        if not ok:
            return self._append_feedback(
                cmd,
                FeedbackStage.SUBMIT,
                FeedbackStatus.REJECTED,
                False,
                ErrorCode.INVALID_PAYLOAD,
                now,
                detail=reason,
            )
        if len(self.queue) >= self.max_queue:
            return self._append_feedback(
                cmd, FeedbackStage.SUBMIT, FeedbackStatus.REJECTED, False, ErrorCode.QUEUE_FULL, now
            )

        if cmd.header.revoke:
            removed = self.revoke(cancel_tag=cmd.header.cancel_tag, command_id=cmd.header.command_id)
            return self._append_feedback(
                cmd,
                FeedbackStage.SUBMIT,
                FeedbackStatus.QUEUED,
                True,
                ErrorCode.REVOKE_APPLIED,
                now,
                detail=f"revoked={removed}",
            )

        self.queue.append(cmd)
        return self._append_feedback(cmd, FeedbackStage.SUBMIT, FeedbackStatus.QUEUED, True, ErrorCode.OK, now)

    def revoke(self, cancel_tag: str | None = None, command_id: str | None = None) -> int:
        before = len(self.queue)
        kept: list[UnifiedCommand] = []
        for cmd in self.queue:
            if cancel_tag is not None and cmd.header.cancel_tag == cancel_tag:
                continue
            if command_id is not None and cmd.header.command_id == command_id:
                continue
            kept.append(cmd)
        self.queue = kept
        return before - len(self.queue)

    def _drop_expired_in_queue(self, now: float) -> None:
        kept: list[UnifiedCommand] = []
        for cmd in self.queue:
            if cmd.header.is_expired(now):
                self._append_feedback(cmd, FeedbackStage.EXEC, FeedbackStatus.REJECTED, False, ErrorCode.EXPIRED, now)
            else:
                kept.append(cmd)
        self.queue = kept

    def dispatch(
        self,
        now: float,
        current_state_seq: int,
        active_commands: Iterable[UnifiedCommand] | None = None,
        max_dispatch: int | None = None,
    ) -> list[UnifiedCommand]:
        self._drop_expired_in_queue(now)
        selected: list[UnifiedCommand] = []
        losers: list[UnifiedCommand] = []
        deferred: list[UnifiedCommand] = []

        occupied: set[tuple[str, int]] = set()
        if active_commands is not None:
            for cmd in active_commands:
                occupied |= command_resources(cmd)

        eligible = [cmd for cmd in self.queue if int(cmd.header.state_seq) <= int(current_state_seq)]
        future = [cmd for cmd in self.queue if int(cmd.header.state_seq) > int(current_state_seq)]
        eligible.sort(key=_compare_key)

        cap = max_dispatch if max_dispatch is not None else 1_000_000
        for cmd in eligible:
            res = command_resources(cmd)
            if len(selected) >= cap:
                deferred.append(cmd)
                continue
            if res & occupied:
                losers.append(cmd)
                continue
            selected.append(cmd)
            occupied |= res

        self.queue = future + deferred

        for cmd in losers:
            self._append_feedback(
                cmd,
                FeedbackStage.EXEC,
                FeedbackStatus.REJECTED,
                False,
                ErrorCode.ARBITRATION_LOST,
                now,
            )

        for cmd in selected:
            self.inflight[cmd.header.command_id] = cmd
            self._append_feedback(
                cmd,
                FeedbackStage.EXEC,
                FeedbackStatus.RUNNING,
                True,
                ErrorCode.OK,
                now,
            )

        return selected

    def complete(self, command_id: str, success: bool, now: float, detail: str = "") -> CommandFeedback:
        cmd = self.inflight.pop(command_id, None)
        if cmd is None:
            fb = CommandFeedback(
                command_id=command_id,
                kind=CommandKind.WAIT,
                stage=FeedbackStage.DONE,
                status=FeedbackStatus.REJECTED,
                accepted=False,
                error_code=ErrorCode.UNKNOWN_COMMAND,
                t=float(now),
                detail="command_id_not_inflight",
            )
            self.feedbacks.append(fb)
            return fb

        if success:
            return self._append_feedback(
                cmd,
                FeedbackStage.DONE,
                FeedbackStatus.SUCCESS,
                True,
                ErrorCode.OK,
                now,
                detail=detail,
            )
        return self._append_feedback(
            cmd,
            FeedbackStage.DONE,
            FeedbackStatus.FAILED,
            False,
            ErrorCode.EXECUTION_FAILED,
            now,
            detail=detail,
        )

