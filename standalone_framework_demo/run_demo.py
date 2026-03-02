#!/usr/bin/env python3
from __future__ import annotations

import json
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


class Mode(str, Enum):
    FREE = "FREE"
    DOCKING = "DOCKING"
    TRAIN_FOLLOW = "TRAIN_FOLLOW"
    WAIT = "WAIT"


class CmdKind(str, Enum):
    DOCK = "DOCK"
    SPLIT = "SPLIT"
    WAIT = "WAIT"


class EventType(str, Enum):
    COMMAND_QUEUED = "COMMAND_QUEUED"
    COMMAND_REJECTED = "COMMAND_REJECTED"
    DOCKING_STARTED = "DOCKING_STARTED"
    DOCK_LOCKED = "DOCK_LOCKED"
    SPLIT_DONE = "SPLIT_DONE"
    WAIT_STARTED = "WAIT_STARTED"
    WAIT_ENDED = "WAIT_ENDED"
    FEASIBILITY_FLIP = "FEASIBILITY_FLIP"
    DOCKING_ABORTED = "DOCKING_ABORTED"


@dataclass(frozen=True)
class CommandHeader:
    command_id: str
    parent_state_seq: int
    issued_at: float
    valid_for_s: float
    priority: int
    source: str = "standalone_strategy"
    can_preempt: bool = True
    rollback_on_reject: bool = False

    @property
    def expires_at(self) -> float:
        return self.issued_at + max(0.0, self.valid_for_s)


@dataclass(frozen=True)
class DockingCommand:
    header: CommandHeader
    follower_id: int
    leader_id: int
    intercept_path_s: float | None = None
    kind: CmdKind = CmdKind.DOCK


@dataclass(frozen=True)
class SplitCommand:
    header: CommandHeader
    parent_id: int
    child_id: int
    reason: str = "split"
    kind: CmdKind = CmdKind.SPLIT


@dataclass(frozen=True)
class WaitCommand:
    header: CommandHeader
    vehicle_id: int
    duration_s: float
    reason: str = "wait"
    kind: CmdKind = CmdKind.WAIT


UnifiedCommand = DockingCommand | SplitCommand | WaitCommand


@dataclass(frozen=True)
class Ack:
    command_id: str
    kind: CmdKind
    stage: str
    accepted: bool
    reason: str
    t: float


@dataclass(frozen=True)
class Event:
    t: float
    event_type: EventType
    vehicle_id: int | None = None
    peer_id: int | None = None
    detail: str = ""


@dataclass
class Vehicle:
    vehicle_id: int
    x: float
    v: float = 0.0


@dataclass(frozen=True)
class Snapshot:
    t: float
    state_seq: int
    x: dict[int, float]
    v: dict[int, float]
    edges: tuple[tuple[int, int], ...]
    pending_docks: dict[int, int]
    feasible: bool


@dataclass(frozen=True)
class FrameworkProfile:
    name: str = "default_convoy"
    goal_x: float = 36.0
    spacing: float = 1.6
    leader_cruise_speed: float = 1.2
    free_cruise_speed: float = 1.0
    docking_lock_pos_err: float = 0.12
    docking_lock_vel_err: float = 0.35
    phase_a_end_x: float = 12.0
    phase_b_end_x: float = 23.0
    rebuild_target_size: int = 3
    wait_trigger_t: float = 9.0
    wait_vehicle_id: int = 5
    wait_duration_s: float = 1.5
    hard_bottleneck_x_min: float = 16.0
    hard_bottleneck_x_max: float = 22.0
    feasibility_dock_time_coeff: float = 1.6
    feasibility_margin: float = 0.8


class Topology:
    def __init__(self, vehicle_ids: list[int]):
        ids = sorted(set(vehicle_ids))
        self.vehicle_ids = tuple(ids)
        self.parent = {v: None for v in ids}
        self.child = {v: None for v in ids}
        self.mode = {v: Mode.FREE for v in ids}
        self.wait_until: dict[int, float] = {}
        self.docking_target: dict[int, int] = {}

    def edges(self) -> tuple[tuple[int, int], ...]:
        out = []
        for c, p in self.parent.items():
            if p is not None:
                out.append((p, c))
        out.sort()
        return tuple(out)

    def _is_ancestor(self, anc: int, node: int) -> bool:
        cur = self.parent.get(node, None)
        while cur is not None:
            if cur == anc:
                return True
            cur = self.parent.get(cur, None)
        return False

    def descendants(self, root: int) -> list[int]:
        out = []
        cur = self.child[root]
        while cur is not None:
            out.append(cur)
            cur = self.child[cur]
        return out

    def chain_from_head(self, head: int) -> list[int]:
        if self.parent.get(head, None) is not None:
            return []
        out = [head]
        cur = self.child.get(head, None)
        while cur is not None:
            out.append(cur)
            cur = self.child.get(cur, None)
        return out

    def heads(self) -> list[int]:
        return sorted(v for v in self.vehicle_ids if self.parent[v] is None)

    def largest_train_size(self) -> int:
        best = 1
        for h in self.heads():
            best = max(best, len(self.chain_from_head(h)))
        return best

    def can_start_dock(self, follower: int, leader: int) -> tuple[bool, str]:
        if follower == leader:
            return False, "self_dock"
        if follower not in self.parent or leader not in self.parent:
            return False, "unknown_vehicle"
        if self.parent[follower] is not None:
            return False, "follower_not_head"
        if self.child[leader] is not None:
            return False, "leader_tail_occupied"
        if leader in self.docking_target.values():
            return False, "leader_tail_reserved"
        if follower in self.docking_target:
            return False, "follower_already_docking"
        if self._is_ancestor(follower, leader):
            return False, "cycle_risk"
        if follower in self.wait_until:
            return False, "follower_waiting"
        return True, "ok"

    def start_dock(self, follower: int, leader: int, now: float) -> tuple[bool, str]:
        ok, reason = self.can_start_dock(follower, leader)
        if not ok:
            return False, reason
        self.docking_target[follower] = leader
        self.refresh_modes(now)
        return True, "ok"

    def finalize_dock(self, follower: int, leader: int, now: float) -> tuple[bool, str]:
        if self.docking_target.get(follower, None) != leader:
            return False, "target_mismatch"
        if self.parent[follower] is not None:
            return False, "follower_not_head"
        if self.child[leader] is not None:
            return False, "leader_tail_occupied"
        for ff, ll in self.docking_target.items():
            if ff != follower and ll == leader:
                return False, "leader_tail_reserved"
        if self._is_ancestor(follower, leader):
            return False, "cycle_risk"
        self.parent[follower] = leader
        self.child[leader] = follower
        self.docking_target.pop(follower, None)
        self.refresh_modes(now)
        return True, "ok"

    def abort_dock(self, follower: int, now: float) -> bool:
        if follower not in self.docking_target:
            return False
        self.docking_target.pop(follower, None)
        self.refresh_modes(now)
        return True

    def apply_split(self, parent: int, child: int, now: float) -> tuple[bool, str]:
        if self.parent.get(child, None) != parent:
            return False, "edge_not_found"
        if self.child.get(parent, None) != child:
            return False, "edge_inconsistent"
        self.parent[child] = None
        self.child[parent] = None
        self.refresh_modes(now)
        return True, "ok"

    def apply_wait(self, vehicle_id: int, duration_s: float, now: float) -> tuple[bool, str]:
        if vehicle_id not in self.parent:
            return False, "unknown_vehicle"
        if duration_s < 0.0:
            return False, "negative_duration"
        self.wait_until[vehicle_id] = now + duration_s
        self.refresh_modes(now)
        return True, "ok"

    def tick(self, now: float) -> list[int]:
        ended = [v for v, until in self.wait_until.items() if now >= until - 1e-9]
        for v in ended:
            self.wait_until.pop(v, None)
        self.refresh_modes(now)
        return ended

    def refresh_modes(self, now: float) -> None:
        for v in self.vehicle_ids:
            if now < self.wait_until.get(v, -1.0):
                self.mode[v] = Mode.WAIT
            elif v in self.docking_target:
                self.mode[v] = Mode.DOCKING
            elif self.parent[v] is None:
                self.mode[v] = Mode.FREE
            else:
                self.mode[v] = Mode.TRAIN_FOLLOW

    def check_invariants(self) -> tuple[bool, str]:
        for v in self.vehicle_ids:
            p = self.parent[v]
            c = self.child[v]
            if p is not None and self.child.get(p, None) != v:
                return False, f"parent_child_mismatch_{v}"
            if c is not None and self.parent.get(c, None) != v:
                return False, f"child_parent_mismatch_{v}"
        for v in self.vehicle_ids:
            seen = set()
            cur = v
            while cur is not None:
                if cur in seen:
                    return False, f"cycle_{v}"
                seen.add(cur)
                cur = self.parent[cur]
        return True, "ok"


@dataclass(frozen=True)
class StrategySnapshot:
    t: float
    state_seq: int
    leader_x: float
    parent: dict[int, int | None]
    child: dict[int, int | None]
    pending_docks: dict[int, int]
    modes: dict[int, Mode]


class RuleStrategy:
    def __init__(self, vehicle_ids: list[int], profile: FrameworkProfile, leader_id: int = 1):
        self.vehicle_ids = sorted(set(vehicle_ids))
        self.leader_id = leader_id
        self.profile = profile
        self.cmd_seq = 0
        self.wait_issued = False

    def _new_id(self) -> str:
        self.cmd_seq += 1
        return f"standalone_cmd_{self.cmd_seq:04d}"

    @staticmethod
    def _chain(parent: dict[int, int | None], child: dict[int, int | None], head: int) -> list[int]:
        if parent.get(head, None) is not None:
            return []
        out = [head]
        cur = child.get(head, None)
        while cur is not None:
            out.append(cur)
            cur = child.get(cur, None)
        return out

    def _mk_dock(self, t: float, state_seq: int, follower: int, leader: int, priority: int) -> DockingCommand:
        return DockingCommand(
            header=CommandHeader(
                command_id=self._new_id(),
                parent_state_seq=state_seq,
                issued_at=t,
                valid_for_s=3.0,
                priority=priority,
            ),
            follower_id=follower,
            leader_id=leader,
        )

    def _mk_split(self, t: float, state_seq: int, parent_id: int, child_id: int, priority: int) -> SplitCommand:
        return SplitCommand(
            header=CommandHeader(
                command_id=self._new_id(),
                parent_state_seq=state_seq,
                issued_at=t,
                valid_for_s=2.0,
                priority=priority,
            ),
            parent_id=parent_id,
            child_id=child_id,
            reason="bottleneck_constraint",
        )

    def _mk_wait(self, t: float, state_seq: int, vehicle_id: int) -> WaitCommand:
        return WaitCommand(
            header=CommandHeader(
                command_id=self._new_id(),
                parent_state_seq=state_seq,
                issued_at=t,
                valid_for_s=2.0,
                priority=10,
            ),
            vehicle_id=vehicle_id,
            duration_s=self.profile.wait_duration_s,
            reason="merge_gap_management",
        )

    def decide(self, s: StrategySnapshot) -> list[UnifiedCommand]:
        cmds: list[UnifiedCommand] = []
        chain = self._chain(s.parent, s.child, self.leader_id)

        # once-only wait command to demonstrate protocol completeness
        wait_vid = self.profile.wait_vehicle_id
        if (not self.wait_issued) and s.t >= self.profile.wait_trigger_t:
            if s.modes.get(wait_vid, Mode.FREE) == Mode.FREE:
                cmds.append(self._mk_wait(s.t, s.state_seq, wait_vid))
                self.wait_issued = True
                return cmds

        # Phase A: open area, encourage docking to full train
        if s.leader_x < self.profile.phase_a_end_x:
            if len(chain) < len(self.vehicle_ids):
                tail = chain[-1]
                if tail in s.pending_docks.values():
                    return cmds
                free_heads = [
                    v
                    for v in self.vehicle_ids
                    if s.parent[v] is None and v not in s.pending_docks and v not in chain and s.modes[v] == Mode.FREE
                ]
                if free_heads:
                    cmds.append(self._mk_dock(s.t, s.state_seq, min(free_heads), tail, priority=4))
            return cmds

        # Phase B: bottleneck approach and crossing, enforce split to single vehicle
        if s.leader_x < self.profile.phase_b_end_x:
            if len(chain) > 1:
                cmds.append(self._mk_split(s.t, s.state_seq, chain[-2], chain[-1], priority=9))
            return cmds

        # Phase C: open after bottleneck, rebuild a smaller train (size=3)
        if len(chain) < self.profile.rebuild_target_size:
            tail = chain[-1]
            if tail in s.pending_docks.values():
                return cmds
            free_heads = [
                v
                for v in self.vehicle_ids
                if s.parent[v] is None and v not in s.pending_docks and v not in chain and s.modes[v] == Mode.FREE
            ]
            if free_heads:
                cmds.append(self._mk_dock(s.t, s.state_seq, min(free_heads), tail, priority=5))
        return cmds


class StandaloneEngine:
    def __init__(self, profile: FrameworkProfile | None = None):
        self.profile = profile or FrameworkProfile()
        self.vehicle_ids = [1, 2, 3, 4, 5]
        self.vehicles = {
            1: Vehicle(1, 0.0, 0.0),
            2: Vehicle(2, -2.8, 0.0),
            3: Vehicle(3, -5.4, 0.0),
            4: Vehicle(4, -8.0, 0.0),
            5: Vehicle(5, -10.5, 0.0),
        }
        self.topology = Topology(self.vehicle_ids)
        self.strategy = RuleStrategy(self.vehicle_ids, profile=self.profile, leader_id=1)
        self.queue: list[UnifiedCommand] = []
        self.command_ids: set[str] = set()
        self.acks: list[Ack] = []
        self.events: list[Event] = []
        self.snapshots: list[Snapshot] = []
        self.state_seq = 0

        self.dt_low = 0.05   # 20 Hz
        self.dt_mid = 0.2    # 5 Hz
        self.dt_high = 1.0   # 1 Hz

        self.goal_x = self.profile.goal_x
        self.spacing = self.profile.spacing
        self.infeasible_last = False

    def emit(self, t: float, et: EventType, vehicle: int | None = None, peer: int | None = None, detail: str = "") -> None:
        self.events.append(Event(t=t, event_type=et, vehicle_id=vehicle, peer_id=peer, detail=detail))

    def submit(self, cmd: UnifiedCommand, now: float) -> None:
        if cmd.header.command_id in self.command_ids:
            self.acks.append(Ack(cmd.header.command_id, cmd.kind, "submit", False, "duplicate_command_id", now))
            self.emit(now, EventType.COMMAND_REJECTED, detail=f"{cmd.kind}:duplicate_command_id")
            return
        if cmd.header.valid_for_s <= 0:
            self.acks.append(Ack(cmd.header.command_id, cmd.kind, "submit", False, "non_positive_ttl", now))
            self.emit(now, EventType.COMMAND_REJECTED, detail=f"{cmd.kind}:non_positive_ttl")
            return
        self.command_ids.add(cmd.header.command_id)
        self.queue.append(cmd)
        self.acks.append(Ack(cmd.header.command_id, cmd.kind, "submit", True, "queued", now))
        self.emit(now, EventType.COMMAND_QUEUED, detail=cmd.kind.value)

    def _leader_remaining_time(self) -> float:
        lead = self.vehicles[1]
        remain_d = max(0.0, self.goal_x - lead.x)
        # use nominal cruise speed for robust feasibility estimation in this minimal demo
        return remain_d / max(1e-6, self.profile.leader_cruise_speed)

    def _feasibility_high_tick(self, now: float) -> None:
        pending = len(self.topology.docking_target)
        t_est = pending * self.profile.feasibility_dock_time_coeff + self.profile.feasibility_margin
        feasible = t_est <= self._leader_remaining_time()
        if feasible != (not self.infeasible_last):
            self.emit(now, EventType.FEASIBILITY_FLIP, detail=f"feasible={feasible}")
        if not feasible and self.topology.docking_target:
            for f in sorted(list(self.topology.docking_target.keys())):
                l = self.topology.docking_target.get(f, None)
                self.topology.abort_dock(f, now)
                self.emit(now, EventType.DOCKING_ABORTED, vehicle=f, peer=l, detail="infeasible_abort")
        self.infeasible_last = not feasible

    def _make_strategy_snapshot(self, now: float) -> StrategySnapshot:
        return StrategySnapshot(
            t=now,
            state_seq=self.state_seq,
            leader_x=self.vehicles[1].x,
            parent=dict(self.topology.parent),
            child=dict(self.topology.child),
            pending_docks=dict(self.topology.docking_target),
            modes=dict(self.topology.mode),
        )

    def _high_tick(self, now: float) -> None:
        self._feasibility_high_tick(now)
        snapshot = self._make_strategy_snapshot(now)
        for cmd in self.strategy.decide(snapshot):
            self.submit(cmd, now)

    def _mid_tick(self, now: float) -> None:
        if not self.queue:
            return
        self.queue.sort(key=lambda c: c.header.priority, reverse=True)
        cmd = self.queue.pop(0)

        if now > cmd.header.expires_at + 1e-9:
            self.acks.append(Ack(cmd.header.command_id, cmd.kind, "execute", False, "expired", now))
            self.emit(now, EventType.COMMAND_REJECTED, detail=f"{cmd.kind}:expired")
            return

        if cmd.header.parent_state_seq > self.state_seq:
            self.acks.append(Ack(cmd.header.command_id, cmd.kind, "execute", False, "future_state_seq", now))
            self.emit(now, EventType.COMMAND_REJECTED, detail=f"{cmd.kind}:future_state_seq")
            return

        if isinstance(cmd, DockingCommand):
            ok, reason = self.topology.start_dock(cmd.follower_id, cmd.leader_id, now)
            self.acks.append(Ack(cmd.header.command_id, cmd.kind, "execute", ok, reason, now))
            if ok:
                self.emit(now, EventType.DOCKING_STARTED, cmd.follower_id, cmd.leader_id, "started")
            else:
                self.emit(now, EventType.COMMAND_REJECTED, cmd.follower_id, cmd.leader_id, f"dock:{reason}")
            return

        if isinstance(cmd, SplitCommand):
            ok, reason = self.topology.apply_split(cmd.parent_id, cmd.child_id, now)
            self.acks.append(Ack(cmd.header.command_id, cmd.kind, "execute", ok, reason, now))
            if ok:
                self.emit(now, EventType.SPLIT_DONE, cmd.child_id, cmd.parent_id, cmd.reason)
            else:
                self.emit(now, EventType.COMMAND_REJECTED, cmd.child_id, cmd.parent_id, f"split:{reason}")
            return

        assert isinstance(cmd, WaitCommand)
        ok, reason = self.topology.apply_wait(cmd.vehicle_id, cmd.duration_s, now)
        self.acks.append(Ack(cmd.header.command_id, cmd.kind, "execute", ok, reason, now))
        if ok:
            self.emit(now, EventType.WAIT_STARTED, cmd.vehicle_id, detail=cmd.reason)
        else:
            self.emit(now, EventType.COMMAND_REJECTED, cmd.vehicle_id, detail=f"wait:{reason}")

    def _low_tick(self, now: float) -> None:
        ended = self.topology.tick(now)
        for v in ended:
            self.emit(now, EventType.WAIT_ENDED, v, detail="timeout")

        # leader dynamics
        leader = self.vehicles[1]
        if self.topology.mode[1] == Mode.WAIT:
            leader.v = 0.0
        else:
            leader.v = self.profile.leader_cruise_speed if leader.x < self.goal_x else 0.0
        leader.x += leader.v * self.dt_low

        # chained followers inherit parent position/speed
        for head in self.topology.heads():
            chain = self.topology.chain_from_head(head)
            for idx in range(1, len(chain)):
                p = self.vehicles[chain[idx - 1]]
                c = self.vehicles[chain[idx]]
                c.x = p.x - self.spacing
                c.v = p.v

        # free / docking vehicles integrate independently
        for v_id in self.vehicle_ids:
            if v_id == 1:
                continue
            if self.topology.parent[v_id] is not None:
                continue
            mode = self.topology.mode[v_id]
            veh = self.vehicles[v_id]
            if mode == Mode.WAIT:
                veh.v = 0.0
            elif mode == Mode.DOCKING:
                l_id = self.topology.docking_target.get(v_id, None)
                if l_id is None:
                    veh.v = 0.0
                else:
                    leader_v = self.vehicles[l_id].v
                    target = self.vehicles[l_id].x - self.spacing
                    err = target - veh.x
                    veh.v = float(np.clip(leader_v + 1.0 * err, -1.5, 1.8))
            else:
                if veh.x < self.goal_x:
                    veh.v = self.profile.free_cruise_speed
                else:
                    veh.v = 0.0
            veh.x += veh.v * self.dt_low

        # docking lock check
        for f, l in list(self.topology.docking_target.items()):
            target = self.vehicles[l].x - self.spacing
            pos_err = abs(self.vehicles[f].x - target)
            vel_err = abs(self.vehicles[f].v - self.vehicles[l].v)
            if pos_err < self.profile.docking_lock_pos_err and vel_err < self.profile.docking_lock_vel_err:
                ok, reason = self.topology.finalize_dock(f, l, now)
                if ok:
                    self.emit(now, EventType.DOCK_LOCKED, f, l, "locked")
                else:
                    self.emit(now, EventType.COMMAND_REJECTED, f, l, f"lock:{reason}")

        # Hard safety fallback in bottleneck: enforce single-vehicle chain.
        leader_x = self.vehicles[1].x
        if self.profile.hard_bottleneck_x_min <= leader_x < self.profile.hard_bottleneck_x_max:
            chain = self.topology.chain_from_head(1)
            while len(chain) > 1:
                p, c = chain[-2], chain[-1]
                ok, _ = self.topology.apply_split(p, c, now)
                if ok:
                    self.emit(now, EventType.SPLIT_DONE, c, p, "safety_auto_split")
                chain = self.topology.chain_from_head(1)

        self.state_seq += 1
        self.snapshots.append(
            Snapshot(
                t=now,
                state_seq=self.state_seq,
                x={i: self.vehicles[i].x for i in self.vehicle_ids},
                v={i: self.vehicles[i].v for i in self.vehicle_ids},
                edges=self.topology.edges(),
                pending_docks=dict(self.topology.docking_target),
                feasible=(not self.infeasible_last),
            )
        )

    def run(self, duration_s: float = 32.0) -> dict:
        t = 0.0
        next_high = 0.0
        next_mid = 0.0
        next_low = 0.0
        eps = 1e-9

        invariant_ok = True
        invariant_reason = "ok"

        while t <= duration_s + eps:
            while t + eps >= next_high:
                self._high_tick(next_high)
                next_high += self.dt_high
            while t + eps >= next_mid:
                self._mid_tick(next_mid)
                next_mid += self.dt_mid
            while t + eps >= next_low:
                self._low_tick(next_low)
                ok, reason = self.topology.check_invariants()
                if not ok:
                    invariant_ok = False
                    invariant_reason = reason
                next_low += self.dt_low
            t += self.dt_low

        execute_acks = [a for a in self.acks if a.stage == "execute"]
        accept_n = sum(1 for a in execute_acks if a.accepted)
        reject_n = sum(1 for a in execute_acks if not a.accepted)
        ev_count = {}
        for e in self.events:
            ev_count[e.event_type.value] = ev_count.get(e.event_type.value, 0) + 1

        ts = np.array([s.t for s in self.snapshots], dtype=float)
        leader_x = np.array([s.x[1] for s in self.snapshots], dtype=float)
        largest_sizes = np.array(
            [self._largest_size_from_edges(s.edges, vehicle_ids=self.vehicle_ids) for s in self.snapshots],
            dtype=float,
        )
        leader_sizes = np.array(
            [self._leader_size_from_edges(s.edges, vehicle_ids=self.vehicle_ids, head=1) for s in self.snapshots],
            dtype=float,
        )
        hard_mask = (leader_x >= self.profile.hard_bottleneck_x_min) & (leader_x < self.profile.hard_bottleneck_x_max)
        hard_max_leader_size = float(np.max(leader_sizes[hard_mask])) if np.any(hard_mask) else 0.0
        hard_max_largest_size = float(np.max(largest_sizes[hard_mask])) if np.any(hard_mask) else 0.0

        ok = bool(
            invariant_ok
            and ev_count.get("DOCK_LOCKED", 0) >= 3
            and ev_count.get("SPLIT_DONE", 0) >= 2
            and hard_max_leader_size <= 1.0 + 1e-9
        )

        return {
            "duration_s": duration_s,
            "invariant_ok": invariant_ok,
            "invariant_reason": invariant_reason,
            "execute_accept": accept_n,
            "execute_reject": reject_n,
            "event_count": ev_count,
            "hard_bottleneck_samples": int(np.sum(hard_mask)),
            "max_leader_train_size_hard_bottleneck": hard_max_leader_size,
            "max_largest_train_size_hard_bottleneck": hard_max_largest_size,
            "final_edges": [list(e) for e in self.topology.edges()],
            "ok": ok,
        }

    @staticmethod
    def _largest_size_from_edges(edges: tuple[tuple[int, int], ...], vehicle_ids: list[int] | tuple[int, ...]) -> int:
        parent = {i: None for i in vehicle_ids}
        child = {i: None for i in vehicle_ids}
        for p, c in edges:
            parent[c] = p
            child[p] = c
        heads = [v for v in vehicle_ids if parent[v] is None]
        best = 1
        for h in heads:
            n = 1
            cur = child[h]
            while cur is not None:
                n += 1
                cur = child[cur]
            best = max(best, n)
        return best

    @staticmethod
    def _leader_size_from_edges(
        edges: tuple[tuple[int, int], ...], vehicle_ids: list[int] | tuple[int, ...], head: int = 1
    ) -> int:
        child = {i: None for i in vehicle_ids}
        for p, c in edges:
            child[p] = c
        n = 1
        cur = child.get(head, None)
        while cur is not None:
            n += 1
            cur = child.get(cur, None)
        return n


def save_plots(engine: StandaloneEngine, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = np.array([s.t for s in engine.snapshots], dtype=float)
    leader_x = np.array([s.x[1] for s in engine.snapshots], dtype=float)

    # Plot 1: x(t) trajectories
    fig1, ax1 = plt.subplots(figsize=(10, 4), dpi=160)
    for vid in engine.vehicle_ids:
        xs = np.array([s.x[vid] for s in engine.snapshots], dtype=float)
        ax1.plot(ts, xs, label=f"veh{vid}", linewidth=1.6)
    _shade_bottleneck_time_windows(
        ax1,
        ts=ts,
        leader_x=leader_x,
        x_min=engine.profile.hard_bottleneck_x_min,
        x_max=engine.profile.hard_bottleneck_x_max,
        label="hard_bottleneck_window",
    )
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("x [m]")
    ax1.set_title("Standalone Demo: Vehicle Longitudinal Trajectories")
    ax1.grid(alpha=0.25)
    ax1.legend(ncol=3, fontsize=8)
    fig1.tight_layout()
    fig1.savefig(out_dir / "standalone_trajectories.png")
    plt.close(fig1)

    # Plot 2: topology size and feasibility
    largest_sizes = np.array(
        [engine._largest_size_from_edges(s.edges, vehicle_ids=engine.vehicle_ids) for s in engine.snapshots], dtype=float
    )
    leader_sizes = np.array(
        [engine._leader_size_from_edges(s.edges, vehicle_ids=engine.vehicle_ids, head=1) for s in engine.snapshots],
        dtype=float,
    )
    feasible = np.array([1.0 if s.feasible else 0.0 for s in engine.snapshots], dtype=float)
    fig2, ax2 = plt.subplots(figsize=(10, 4), dpi=160)
    ax2.plot(ts, largest_sizes, linewidth=1.8, label="largest_train_size")
    ax2.plot(ts, leader_sizes, linewidth=2.0, label="leader_train_size")
    ax2.plot(ts, feasible, linewidth=1.4, linestyle="--", label="feasible_flag")
    _shade_bottleneck_time_windows(
        ax2,
        ts=ts,
        leader_x=leader_x,
        x_min=engine.profile.hard_bottleneck_x_min,
        x_max=engine.profile.hard_bottleneck_x_max,
        label="hard_bottleneck_window",
    )
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("size / flag")
    ax2.set_title("Standalone Demo: Topology Evolution")
    ax2.grid(alpha=0.25)
    ax2.legend(fontsize=8)
    fig2.tight_layout()
    fig2.savefig(out_dir / "standalone_topology_timeline.png")
    plt.close(fig2)

    # Plot 3: command/event timeline
    event_types = [
        EventType.COMMAND_QUEUED.value,
        EventType.DOCKING_STARTED.value,
        EventType.DOCK_LOCKED.value,
        EventType.SPLIT_DONE.value,
        EventType.WAIT_STARTED.value,
        EventType.WAIT_ENDED.value,
        EventType.DOCKING_ABORTED.value,
        EventType.FEASIBILITY_FLIP.value,
        EventType.COMMAND_REJECTED.value,
    ]
    y_map = {name: idx for idx, name in enumerate(event_types)}
    fig3, ax3 = plt.subplots(figsize=(10, 4), dpi=160)
    for e in engine.events:
        ax3.scatter([e.t], [y_map[e.event_type.value]], s=14)
    ax3.set_yticks(list(y_map.values()), list(y_map.keys()))
    ax3.set_xlabel("time [s]")
    ax3.set_title("Standalone Demo: Event Timeline")
    ax3.grid(alpha=0.25)
    fig3.tight_layout()
    fig3.savefig(out_dir / "standalone_events_timeline.png")
    plt.close(fig3)


def _shade_bottleneck_time_windows(
    ax: plt.Axes, ts: np.ndarray, leader_x: np.ndarray, x_min: float, x_max: float, label: str
) -> None:
    mask = (leader_x >= x_min) & (leader_x < x_max)
    if not np.any(mask):
        return
    idx = np.where(mask)[0]
    groups: list[tuple[int, int]] = []
    start = idx[0]
    prev = idx[0]
    for i in idx[1:]:
        if i == prev + 1:
            prev = i
            continue
        groups.append((start, prev))
        start = i
        prev = i
    groups.append((start, prev))
    first = True
    for s, e in groups:
        ax.axvspan(ts[s], ts[e], color="tab:red", alpha=0.08, label=label if first else None)
        first = False


def write_report(out_dir: Path, metrics: dict) -> None:
    report_md = out_dir / "STANDALONE_FRAMEWORK_DEMO_REPORT.md"
    lines = [
        "# Standalone Framework Demo Report",
        "",
        "- This demo is fully standalone and does not import `docking/` platform modules.",
        "- Purpose: validate the feasibility of the decoupled strategy-control framework.",
        "",
        "## Metrics",
        f"- duration_s: {metrics['duration_s']}",
        f"- invariant_ok: {metrics['invariant_ok']} ({metrics['invariant_reason']})",
        f"- execute_accept: {metrics['execute_accept']}",
        f"- execute_reject: {metrics['execute_reject']}",
        f"- DOCK_LOCKED: {metrics['event_count'].get('DOCK_LOCKED', 0)}",
        f"- SPLIT_DONE: {metrics['event_count'].get('SPLIT_DONE', 0)}",
        f"- max_leader_train_size_hard_bottleneck: {metrics['max_leader_train_size_hard_bottleneck']}",
        f"- max_largest_train_size_hard_bottleneck: {metrics['max_largest_train_size_hard_bottleneck']}",
        f"- final_edges: {metrics['final_edges']}",
        f"- ok: {metrics['ok']}",
        "",
        "## Generated Visualizations",
        "- standalone_trajectories.png",
        "- standalone_topology_timeline.png",
        "- standalone_events_timeline.png",
    ]
    report_md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    out_dir = root / "artifacts" / "standalone_framework_demo"
    engine = StandaloneEngine()
    metrics = engine.run(duration_s=32.0)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "standalone_metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    save_plots(engine, out_dir)
    write_report(out_dir, metrics)

    print("standalone_metrics", out_dir / "standalone_metrics.json")
    print("standalone_report", out_dir / "STANDALONE_FRAMEWORK_DEMO_REPORT.md")
    print("standalone_plot", out_dir / "standalone_trajectories.png")
    print("standalone_plot", out_dir / "standalone_topology_timeline.png")
    print("standalone_plot", out_dir / "standalone_events_timeline.png")
    print("standalone_ok", metrics["ok"])


if __name__ == "__main__":
    main()
