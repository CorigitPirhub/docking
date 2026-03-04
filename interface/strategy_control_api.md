# Strategy-Control API v1.0

## 1. Scope

This document freezes the strategy-control interface for P1:

1. Strategy layer emits discrete reconfiguration commands only.
2. Control/runtime layer executes accepted commands and emits lifecycle feedback.
3. Both sides communicate through `runtime/command_bus.py`.

Acceptance & regression entrypoints: `interface/acceptance_runbook.md`.

---

## 2. Strategy Input (Platform-Agnostic Snapshot)

```text
StrategyStateSnapshot
- timestamp: float
- state_seq: int                 # monotonic state version
- vehicles: VehicleSummary[]     # pose/velocity/mode summary
- topology: TopologySummary      # G^k head-chain structure
- scenario_tags: ScenarioSummary # N_max_pass_profile, split zones, dock zones
- feasibility: FeasibilitySummary# T_est vs T_leader_remain
- risk: RiskSummary              # clearance/collision/fov related risk
```

`state_seq` is mandatory for command consistency.

---

## 3. Command Header (Frozen)

```text
CommandHeader
- command_id: str                # globally unique in runtime horizon
- state_seq: int                 # parent version used by strategy
- issued_at: float               # issue time
- deadline_at: float             # hard deadline
- priority: int                  # larger = higher priority
- cancel_tag: str | None         # optional revoke group
- revoke: bool                   # if true, apply revoke action
- can_preempt: bool              # reserved preemption semantic
- rollback_on_reject: bool       # reserved rollback semantic
- source: str                    # strategy identifier
```

Execution guard:

1. `deadline_at >= issued_at`
2. `command_id` non-empty
3. `state_seq <= current_state_seq` at submit/dispatch time

---

## 4. Command Types

```text
DockingCommand
- header: CommandHeader
- follower_id: int
- leader_id: int
- intercept_path_s: float | None
```

```text
SplitCommand
- header: CommandHeader
- parent_id: int
- child_id: int
- reason: str
```

```text
WaitCommand
- header: CommandHeader
- vehicle_id: int
- duration_s: float
- reason: str
```

---

## 5. Feedback Protocol

Feedback stages:

1. `SUBMIT`
2. `EXEC`
3. `DONE`

Feedback schema:

```text
CommandFeedback
- command_id: str
- kind: DOCK | SPLIT | WAIT
- stage: SUBMIT | EXEC | DONE
- status: QUEUED | RUNNING | SUCCESS | FAILED | REJECTED
- accepted: bool
- error_code: ErrorCode
- t: float
- detail: str
```

Lifecycle:

1. Strategy submits command -> `SUBMIT` feedback.
2. Runtime arbitration/dispatch -> `EXEC` feedback.
3. Runtime completion -> `DONE` feedback.

---

## 6. Conflict Arbitration (Frozen Rule)

At same dispatch cycle:

1. Sort by `priority DESC`, `issued_at ASC`, `command_id ASC`.
2. Resolve resource conflicts greedily.
3. Losers receive `EXEC + REJECTED + ARBITRATION_LOST`.

Resource conflicts:

1. `DockingCommand`: follower vehicle + leader tail.
2. `SplitCommand`: parent/child vehicles + edge ownership.
3. `WaitCommand`: vehicle ownership.

---

## 7. Consistency Guarantees

1. Duplicate `command_id` is always rejected.
2. Expired commands are always rejected before execution.
3. Future-version commands (`state_seq > current_state_seq`) are rejected.
4. Arbitration outcome is deterministic under same input set.

Version: `v1.0`  
Status: `Frozen at P1`

Related:
- Foundation cost-query API: `interface/foundation_cost_api.md`
