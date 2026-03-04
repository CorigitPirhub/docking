from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Callable

from .config import Config, CoordinatorConfig
from .types import VehicleMode
from runtime.command_bus import (
    CommandBus,
    CommandFeedback,
    CommandKind,
    DockingCommand,
    FeedbackStage,
    FeedbackStatus,
    SplitCommand,
    UnifiedCommand,
    WaitCommand,
)


class LayerName(str, Enum):
    HIGH = "HIGH"
    MID = "MID"
    LOW = "LOW"


class EventType(str, Enum):
    COMMAND_QUEUED = "COMMAND_QUEUED"
    COMMAND_REJECTED = "COMMAND_REJECTED"
    DOCKING_STARTED = "DOCKING_STARTED"
    DOCK_LOCKED = "DOCK_LOCKED"
    DOCKING_ABORTED = "DOCKING_ABORTED"
    SPLIT_DONE = "SPLIT_DONE"
    WAIT_STARTED = "WAIT_STARTED"
    WAIT_ENDED = "WAIT_ENDED"
    FEASIBILITY_FLIP = "FEASIBILITY_FLIP"


@dataclass(frozen=True)
class RuntimeEvent:
    t: float
    event_type: EventType
    vehicle_id: int | None = None
    peer_id: int | None = None
    detail: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class FeasibilityDecision:
    t: float
    feasible: bool
    flipped: bool
    pending_docks: int
    t_est: float
    leader_remaining_s: float
    reason: str

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class RuntimeSnapshot:
    t: float
    edges: tuple[tuple[int, int], ...]
    modes: dict[int, str]
    pending_docks: dict[int, int]
    feasible: bool

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class MultiRateStats:
    ticks_high: int
    ticks_mid: int
    ticks_low: int
    tick_trace: tuple[tuple[float, str], ...]

    def to_dict(self) -> dict:
        return asdict(self)


class TopologyStateMachine:
    def __init__(self, vehicle_ids: list[int]):
        ids = sorted(set(int(v) for v in vehicle_ids))
        if not ids:
            raise ValueError("TopologyStateMachine requires at least one vehicle")
        self.vehicle_ids: tuple[int, ...] = tuple(ids)
        self.parent: dict[int, int | None] = {v: None for v in self.vehicle_ids}
        self.child: dict[int, int | None] = {v: None for v in self.vehicle_ids}
        self.mode: dict[int, VehicleMode] = {v: VehicleMode.FREE for v in self.vehicle_ids}
        self.docking_target: dict[int, int] = {}
        self.wait_until: dict[int, float] = {}
        self.time_now: float = 0.0

    def _contains(self, v: int) -> bool:
        return v in self.parent

    def _is_ancestor(self, ancestor: int, node: int) -> bool:
        cur = self.parent.get(node, None)
        while cur is not None:
            if cur == ancestor:
                return True
            cur = self.parent.get(cur, None)
        return False

    def descendants(self, root: int) -> list[int]:
        out: list[int] = []
        cur = self.child.get(root, None)
        while cur is not None:
            out.append(cur)
            cur = self.child.get(cur, None)
        return out

    def edges(self) -> tuple[tuple[int, int], ...]:
        out = []
        for child, parent in self.parent.items():
            if parent is not None:
                out.append((parent, child))
        out.sort()
        return tuple(out)

    def heads(self) -> tuple[int, ...]:
        out = [v for v in self.vehicle_ids if self.parent[v] is None]
        out.sort()
        return tuple(out)

    def train_size(self, head: int) -> int:
        if not self._contains(head):
            return 0
        if self.parent[head] is not None:
            return 0
        return 1 + len(self.descendants(head))

    def can_start_docking(self, follower: int, leader: int) -> tuple[bool, str]:
        if not self._contains(follower) or not self._contains(leader):
            return False, "unknown_vehicle"
        if follower == leader:
            return False, "self_docking"
        if follower in self.wait_until:
            return False, "follower_waiting"
        if self.parent[follower] is not None:
            return False, "follower_not_head"
        if self.child[leader] is not None:
            return False, "leader_tail_occupied"
        if self._is_ancestor(follower, leader):
            return False, "cycle_risk"
        return True, "ok"

    def start_docking(self, follower: int, leader: int, now: float) -> tuple[bool, str]:
        ok, reason = self.can_start_docking(follower, leader)
        if not ok:
            return False, reason
        self.time_now = now
        self.docking_target[follower] = leader
        self._refresh_modes(now)
        return True, "ok"

    def finalize_docking(self, follower: int, leader: int, now: float) -> tuple[bool, str]:
        if self.docking_target.get(follower, None) != leader:
            return False, "docking_target_mismatch"
        ok, reason = self.can_start_docking(follower, leader)
        if not ok:
            return False, reason
        self.time_now = now
        self.parent[follower] = leader
        self.child[leader] = follower
        self.docking_target.pop(follower, None)
        self._refresh_modes(now)
        return True, "ok"

    def abort_docking(self, follower: int, now: float) -> bool:
        if follower not in self.docking_target:
            return False
        self.time_now = now
        self.docking_target.pop(follower, None)
        self._refresh_modes(now)
        return True

    def can_split(self, parent: int, child: int) -> tuple[bool, str]:
        if not self._contains(parent) or not self._contains(child):
            return False, "unknown_vehicle"
        if self.parent[child] != parent:
            return False, "edge_not_found"
        if self.child[parent] != child:
            return False, "edge_inconsistent"
        return True, "ok"

    def apply_split(self, parent: int, child: int, now: float) -> tuple[bool, str]:
        ok, reason = self.can_split(parent, child)
        if not ok:
            return False, reason
        self.time_now = now
        self.parent[child] = None
        self.child[parent] = None
        self._refresh_modes(now)
        return True, "ok"

    def apply_wait(self, vehicle_id: int, duration_s: float, now: float) -> tuple[bool, str]:
        if not self._contains(vehicle_id):
            return False, "unknown_vehicle"
        if duration_s < 0.0:
            return False, "negative_wait_duration"
        self.time_now = now
        self.wait_until[vehicle_id] = now + duration_s
        self._refresh_modes(now)
        return True, "ok"

    def tick(self, now: float) -> list[int]:
        self.time_now = now
        finished = [v for v, until in self.wait_until.items() if now >= until - 1e-9]
        for v in finished:
            self.wait_until.pop(v, None)
        self._refresh_modes(now)
        return finished

    def _refresh_modes(self, now: float) -> None:
        for v in self.vehicle_ids:
            if now < self.wait_until.get(v, -1e9):
                self.mode[v] = VehicleMode.WAIT
            elif v in self.docking_target:
                self.mode[v] = VehicleMode.DOCKING
            elif self.parent[v] is None:
                self.mode[v] = VehicleMode.FREE
            else:
                self.mode[v] = VehicleMode.TRAIN_FOLLOW

    def check_invariants(self) -> tuple[bool, str]:
        # Per-node in/out degree <= 1 is guaranteed by dict structure; verify consistency.
        for v in self.vehicle_ids:
            p = self.parent[v]
            c = self.child[v]
            if p is not None and self.child.get(p, None) != v:
                return False, f"parent_child_mismatch_{v}"
            if c is not None and self.parent.get(c, None) != v:
                return False, f"child_parent_mismatch_{v}"
        # Cycle check.
        for v in self.vehicle_ids:
            seen = set()
            cur = v
            while cur is not None:
                if cur in seen:
                    return False, f"cycle_detected_{v}"
                seen.add(cur)
                cur = self.parent[cur]
        return True, "ok"


class TaskFeasibilityMonitor:
    def __init__(self, cfg: CoordinatorConfig):
        self.cfg = cfg
        self._last_feasible = True

    def evaluate(self, pending_docks: int, leader_remaining_s: float, now: float) -> FeasibilityDecision:
        pending = max(0, int(pending_docks))
        leader_remaining = max(0.0, float(leader_remaining_s))
        if pending <= 0:
            t_est = 0.0
            feasible = True
            flipped = feasible != self._last_feasible
            self._last_feasible = feasible
            return FeasibilityDecision(
                t=now,
                feasible=feasible,
                flipped=flipped,
                pending_docks=pending,
                t_est=float(t_est),
                leader_remaining_s=float(leader_remaining),
                reason="ok_no_pending_docks",
            )

        t_est = pending * self.cfg.feasibility_dock_time_est_s + self.cfg.feasibility_margin_s

        if self._last_feasible:
            feasible = t_est <= leader_remaining + 1e-9
        else:
            feasible = t_est <= (leader_remaining - self.cfg.feasibility_recover_hysteresis_s)

        flipped = feasible != self._last_feasible
        self._last_feasible = feasible
        reason = "ok" if feasible else "insufficient_leader_remaining_time"
        return FeasibilityDecision(
            t=now,
            feasible=feasible,
            flipped=flipped,
            pending_docks=pending,
            t_est=float(t_est),
            leader_remaining_s=float(leader_remaining),
            reason=reason,
        )


class MultiRateExecutor:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.periods = {
            LayerName.HIGH: 1.0 / max(cfg.control.high_rate_hz, 1e-6),
            LayerName.MID: 1.0 / max(cfg.control.mid_rate_hz, 1e-6),
            LayerName.LOW: 1.0 / max(cfg.control.main_rate_hz, 1e-6),
        }
        self.base_dt = min(cfg.control.dt, self.periods[LayerName.LOW])

    def run(
        self,
        duration_s: float,
        on_high: Callable[[float, int], None],
        on_mid: Callable[[float, int], None],
        on_low: Callable[[float, int], None],
    ) -> MultiRateStats:
        eps = 1e-9
        t = 0.0
        next_due = {k: 0.0 for k in self.periods}
        ticks = {k: 0 for k in self.periods}
        trace: list[tuple[float, str]] = []

        callbacks = {
            LayerName.HIGH: on_high,
            LayerName.MID: on_mid,
            LayerName.LOW: on_low,
        }
        order = (LayerName.HIGH, LayerName.MID, LayerName.LOW)
        while t <= duration_s + eps:
            for layer in order:
                while t + eps >= next_due[layer]:
                    callbacks[layer](float(next_due[layer]), ticks[layer])
                    trace.append((float(next_due[layer]), layer.value))
                    ticks[layer] += 1
                    next_due[layer] += self.periods[layer]
            t += self.base_dt

        return MultiRateStats(
            ticks_high=ticks[LayerName.HIGH],
            ticks_mid=ticks[LayerName.MID],
            ticks_low=ticks[LayerName.LOW],
            tick_trace=tuple(trace),
        )


def estimate_leader_remaining_time(distance_remaining: float, speed_now: float, v_max: float, a_max: float) -> float:
    d = max(0.0, float(distance_remaining))
    if d <= 1e-9:
        return 0.0
    v0 = max(0.0, float(speed_now))
    vmax = max(1e-6, float(v_max))
    a = max(1e-6, float(a_max))

    if v0 >= vmax:
        return d / vmax

    t_acc = (vmax - v0) / a
    d_acc = (v0 + vmax) * 0.5 * t_acc
    if d_acc >= d:
        # Solve d = v0*t + 0.5*a*t^2
        return (-v0 + math.sqrt(max(v0 * v0 + 2.0 * a * d, 0.0))) / a
    return t_acc + (d - d_acc) / vmax


class ReconfigRuntimeEngine:
    def __init__(self, cfg: Config, vehicle_ids: list[int]):
        self.cfg = cfg
        self.topology = TopologyStateMachine(vehicle_ids)
        self.feasibility = TaskFeasibilityMonitor(cfg.coordinator)
        self.bus = CommandBus(max_queue=cfg.coordinator.max_command_queue)
        self.events: list[RuntimeEvent] = []
        self.decisions: list[FeasibilityDecision] = []
        self.snapshots: list[RuntimeSnapshot] = []
        self.multi_rate_stats: MultiRateStats | None = None
        self.state_seq: int = 0
        self._dock_cmd_id_by_follower: dict[int, str] = {}
        self._wait_cmd_id_by_vehicle: dict[int, str] = {}

    def _emit(self, t: float, event_type: EventType, vehicle_id: int | None = None, peer_id: int | None = None, detail: str = "") -> None:
        self.events.append(RuntimeEvent(t=float(t), event_type=event_type, vehicle_id=vehicle_id, peer_id=peer_id, detail=detail))

    @property
    def feedbacks(self) -> list[CommandFeedback]:
        return self.bus.feedbacks

    def submit_command(self, cmd: UnifiedCommand, now: float) -> CommandFeedback:
        fb = self.bus.submit(cmd, now=now, current_state_seq=self.state_seq)
        if fb.accepted:
            self._emit(now, EventType.COMMAND_QUEUED, detail=f"{cmd.kind}")
        else:
            self._emit(now, EventType.COMMAND_REJECTED, detail=f"{cmd.kind}:{fb.error_code.value}:{fb.detail}")
        return fb

    def mark_docking_locked(self, follower_id: int, leader_id: int, now: float) -> bool:
        ok, reason = self.topology.finalize_docking(follower_id, leader_id, now)
        cmd_id = self._dock_cmd_id_by_follower.pop(int(follower_id), None)
        if ok:
            self._emit(now, EventType.DOCK_LOCKED, vehicle_id=follower_id, peer_id=leader_id, detail="locked")
        else:
            self._emit(now, EventType.COMMAND_REJECTED, vehicle_id=follower_id, peer_id=leader_id, detail=f"lock_fail:{reason}")
        if cmd_id is not None:
            self.bus.complete(command_id=cmd_id, success=ok, now=now, detail=f"dock_lock:{reason}")
        return ok

    def abort_docking(self, follower_id: int, now: float, detail: str = "abort") -> bool:
        follower_id = int(follower_id)
        leader = self.topology.docking_target.get(follower_id, None)
        ok = self.topology.abort_docking(follower_id, now)
        cmd_id = self._dock_cmd_id_by_follower.pop(follower_id, None)
        if ok:
            self._emit(now, EventType.DOCKING_ABORTED, vehicle_id=follower_id, peer_id=leader, detail=detail)
        if cmd_id is not None:
            self.bus.complete(command_id=cmd_id, success=False, now=now, detail=f"dock_abort:{detail}")
        return ok

    def _process_command_mid(self, now: float) -> None:
        selected = self.bus.dispatch(
            now=now,
            current_state_seq=self.state_seq,
            active_commands=list(self.bus.inflight.values()),
            max_dispatch=1,
        )
        if not selected:
            return
        for cmd in selected:
            if isinstance(cmd, DockingCommand):
                ok, reason = self.topology.start_docking(cmd.follower_id, cmd.leader_id, now)
                if ok:
                    self._dock_cmd_id_by_follower[int(cmd.follower_id)] = cmd.header.command_id
                    self._emit(now, EventType.DOCKING_STARTED, vehicle_id=cmd.follower_id, peer_id=cmd.leader_id, detail="started")
                else:
                    self._emit(now, EventType.COMMAND_REJECTED, vehicle_id=cmd.follower_id, peer_id=cmd.leader_id, detail=f"dock:{reason}")
                    self.bus.complete(command_id=cmd.header.command_id, success=False, now=now, detail=f"dock_start:{reason}")
                continue

            if isinstance(cmd, SplitCommand):
                ok, reason = self.topology.apply_split(cmd.parent_id, cmd.child_id, now)
                if ok:
                    self._emit(now, EventType.SPLIT_DONE, vehicle_id=cmd.child_id, peer_id=cmd.parent_id, detail=cmd.reason)
                else:
                    self._emit(now, EventType.COMMAND_REJECTED, vehicle_id=cmd.child_id, peer_id=cmd.parent_id, detail=f"split:{reason}")
                self.bus.complete(command_id=cmd.header.command_id, success=ok, now=now, detail=f"split:{reason}")
                continue

            assert isinstance(cmd, WaitCommand)
            ok, reason = self.topology.apply_wait(cmd.vehicle_id, cmd.duration_s, now)
            if ok:
                self._wait_cmd_id_by_vehicle[int(cmd.vehicle_id)] = cmd.header.command_id
                self._emit(now, EventType.WAIT_STARTED, vehicle_id=cmd.vehicle_id, detail=cmd.reason)
            else:
                self._emit(now, EventType.COMMAND_REJECTED, vehicle_id=cmd.vehicle_id, detail=f"wait:{reason}")
                self.bus.complete(command_id=cmd.header.command_id, success=False, now=now, detail=f"wait:{reason}")

    def _handle_high_tick(self, now: float, leader_remaining_s: float) -> None:
        pending = len(self.topology.docking_target)
        decision = self.feasibility.evaluate(pending_docks=pending, leader_remaining_s=leader_remaining_s, now=now)
        self.decisions.append(decision)
        if decision.flipped:
            self._emit(now, EventType.FEASIBILITY_FLIP, detail=f"feasible={decision.feasible}")

        if not decision.feasible and self.topology.docking_target:
            followers = sorted(list(self.topology.docking_target.keys()))
            for f in followers:
                self.abort_docking(f, now=now, detail="feasibility_infeasible")

    def _handle_low_tick(self, now: float) -> None:
        ended = self.topology.tick(now)
        for vid in ended:
            self._emit(now, EventType.WAIT_ENDED, vehicle_id=vid, detail="timeout")
            cmd_id = self._wait_cmd_id_by_vehicle.pop(int(vid), None)
            if cmd_id is not None:
                self.bus.complete(command_id=cmd_id, success=True, now=now, detail="wait_done")

        feasible = self.decisions[-1].feasible if self.decisions else True
        self.snapshots.append(
            RuntimeSnapshot(
                t=now,
                edges=self.topology.edges(),
                modes={k: self.topology.mode[k].value for k in self.topology.vehicle_ids},
                pending_docks=dict(self.topology.docking_target),
                feasible=feasible,
            )
        )
        self.state_seq += 1

    def run(
        self,
        duration_s: float,
        leader_remaining_fn: Callable[[float], float],
        low_tick_hook: Callable[["ReconfigRuntimeEngine", float], None] | None = None,
    ) -> MultiRateStats:
        executor = MultiRateExecutor(self.cfg)

        def _high_cb(t: float, _idx: int) -> None:
            self._handle_high_tick(t, float(leader_remaining_fn(t)))

        def _mid_cb(t: float, _idx: int) -> None:
            self._process_command_mid(t)

        def _low_cb(t: float, _idx: int) -> None:
            if low_tick_hook is not None:
                low_tick_hook(self, t)
            self._handle_low_tick(t)

        self.multi_rate_stats = executor.run(duration_s=duration_s, on_high=_high_cb, on_mid=_mid_cb, on_low=_low_cb)
        return self.multi_rate_stats

    def check_invariants(self) -> tuple[bool, str]:
        return self.topology.check_invariants()
