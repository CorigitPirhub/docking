from __future__ import annotations

from dataclasses import dataclass

from docking.runtime_support import CommandHeader, DockingCommand, ReconfigRuntimeEngine, SplitCommand, WaitCommand


@dataclass(frozen=True)
class P0StrategyConfig:
    leader_id: int = 1
    decision_period_s: float = 0.25
    build_phase_end_s: float = 8.0
    bottleneck_end_s: float = 14.0
    rebuild_target_size: int = 3
    bottleneck_max_size: int = 1
    wait_vehicle_id: int = 5
    wait_start_s: float = 10.5
    wait_duration_s: float = 2.0


class P0RuleStrategy:
    """Minimal strategy PoC: build train -> split for bottleneck -> rebuild."""

    def __init__(self, vehicle_ids: list[int], cfg: P0StrategyConfig | None = None):
        if not vehicle_ids:
            raise ValueError("vehicle_ids must not be empty")
        self.vehicle_ids = tuple(sorted(set(int(v) for v in vehicle_ids)))
        self.cfg = cfg or P0StrategyConfig()
        self._last_decision_t = -1e9
        self._cmd_seq = 0
        self._wait_issued = False

    def _next_id(self) -> str:
        self._cmd_seq += 1
        return f"p0_cmd_{self._cmd_seq:04d}"

    @staticmethod
    def _chain_from_head(engine: ReconfigRuntimeEngine, head: int) -> list[int]:
        chain = [head]
        cur = head
        while True:
            nxt = engine.topology.child.get(cur, None)
            if nxt is None:
                return chain
            chain.append(nxt)
            cur = nxt

    def _submit_dock(self, engine: ReconfigRuntimeEngine, now: float, follower: int, leader: int) -> None:
        hdr = CommandHeader(
            command_id=self._next_id(),
            issued_at=now,
            valid_for_s=engine.cfg.coordinator.command_ttl_s,
            source="p0_rule_strategy",
        )
        engine.submit_command(DockingCommand(header=hdr, follower_id=follower, leader_id=leader), now=now)

    def _submit_split(self, engine: ReconfigRuntimeEngine, now: float, parent: int, child: int, reason: str) -> None:
        hdr = CommandHeader(
            command_id=self._next_id(),
            issued_at=now,
            valid_for_s=engine.cfg.coordinator.command_ttl_s,
            source="p0_rule_strategy",
        )
        engine.submit_command(SplitCommand(header=hdr, parent_id=parent, child_id=child, reason=reason), now=now)

    def _submit_wait(self, engine: ReconfigRuntimeEngine, now: float, vehicle_id: int, duration_s: float) -> None:
        hdr = CommandHeader(
            command_id=self._next_id(),
            issued_at=now,
            valid_for_s=engine.cfg.coordinator.command_ttl_s,
            source="p0_rule_strategy",
        )
        engine.submit_command(
            WaitCommand(
                header=hdr,
                vehicle_id=vehicle_id,
                duration_s=duration_s,
                reason="p0_bottleneck_wait",
            ),
            now=now,
        )

    def _free_heads(self, engine: ReconfigRuntimeEngine) -> list[int]:
        out = []
        for v in self.vehicle_ids:
            if engine.topology.parent[v] is None and v not in engine.topology.docking_target:
                out.append(v)
        return out

    def _build_chain_step(self, engine: ReconfigRuntimeEngine, now: float, target_size: int) -> None:
        chain = self._chain_from_head(engine, self.cfg.leader_id)
        if len(chain) >= target_size:
            return
        tail = chain[-1]
        if engine.topology.child[tail] is not None:
            return
        # Avoid issuing multiple followers to the same leader before previous lock completes.
        if tail in engine.topology.docking_target.values():
            return
        candidates = [v for v in self._free_heads(engine) if v not in chain and v != self.cfg.leader_id]
        if not candidates:
            return
        follower = min(candidates)
        self._submit_dock(engine, now, follower=follower, leader=tail)

    def _enforce_split_step(self, engine: ReconfigRuntimeEngine, now: float, max_size: int) -> None:
        chain = self._chain_from_head(engine, self.cfg.leader_id)
        if len(chain) <= max_size:
            return
        parent, child = chain[-2], chain[-1]
        self._submit_split(engine, now, parent=parent, child=child, reason="bottleneck_split")

    def on_tick(self, engine: ReconfigRuntimeEngine, now: float) -> None:
        if now - self._last_decision_t < self.cfg.decision_period_s - 1e-9:
            return
        self._last_decision_t = now

        # Avoid queueing too aggressively in PoC.
        if engine.queue:
            return

        if (not self._wait_issued) and now >= self.cfg.wait_start_s:
            self._submit_wait(engine, now, vehicle_id=self.cfg.wait_vehicle_id, duration_s=self.cfg.wait_duration_s)
            self._wait_issued = True
            return

        if now < self.cfg.build_phase_end_s:
            self._build_chain_step(engine, now, target_size=len(self.vehicle_ids))
            return

        if now < self.cfg.bottleneck_end_s:
            self._enforce_split_step(engine, now, max_size=self.cfg.bottleneck_max_size)
            return

        self._build_chain_step(engine, now, target_size=self.cfg.rebuild_target_size)
