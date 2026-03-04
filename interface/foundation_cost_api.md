# Foundation Cost API v1.0

This document freezes the **cost-query interface** exposed by the foundation layer for the strategy layer.

Source of truth: `docking/costs.py`.

Acceptance & regression entrypoints: `interface/acceptance_runbook.md`.

## 1. Scope

Strategy code should **not** implement its own time/energy/risk surrogates. Instead, it should reuse the APIs in
`docking/costs.py` for candidate evaluation and objective aggregation.

Covered action types:

- Single-vehicle local planning (one-step/horizon proxy)
- Docking (catch + dock, energy delta, risk)
- Train-follow (time + energy + risk proxy)
- Split / Wait (time + energy + risk proxy)

All costs expose:

- `feasible: bool`
- `time_s: float`
- `energy_delta: float` (can be absolute energy for some actions; see breakdown)
- `risk: float`
- `total_cost: float`

## 2. Core Types

### 2.1 Sequence cost (P2-style)

- `EnergyModelConfig`: energy proxy model (single vs train)
- `SequenceCostWeights`: weights for `T_catch/T_dock/ΔE/R/N_switch`
- `SequenceCostBreakdown`: structured output of `compute_j_seq(...)`

APIs:

- `estimate_travel_time(distance_m, speed_now, v_max, a_max) -> float`
- `compute_j_seq(...) -> SequenceCostBreakdown`
- `risk_score(...) -> float` (P2 sequence risk proxy)
- `recovery_risk_score(...) -> float` (P4 recovery risk proxy)

### 2.2 Action cost (command-level)

- `ActionCostWeights`: generic weights for time/energy/risk
- `ActionCost`: normalized action-level output (`time_s/energy_delta/risk/feasible/total_cost`)

APIs:

- `local_planning_action_cost(...) -> ActionCost`
- `docking_action_cost(...) -> ActionCost`
- `train_follow_action_cost(...) -> ActionCost`
- `split_action_cost(...) -> ActionCost`
- `wait_action_cost(...) -> ActionCost`

## 3. Regression / Acceptance Cases

The following tests act as the minimum regression suite for “strategy → foundation cost + runtime” integration:

- `tests/test_runtime_support.py` (CommandBus-backed runtime + topology + feasibility)
- `tests/test_p2_scheduler.py` (sequence cost + intercept selection)
- `tests/test_p2_p4_integrated.py` (closed-loop integrated execution; includes sensor-noise enabled cases)
