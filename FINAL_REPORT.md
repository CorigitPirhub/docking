# FINAL_REPORT

## 1. System Architecture Summary
This project delivers a complete multi-vehicle docking prototype with a layered architecture:

- **Environment Layer**: `WorldMap` + `SceneFactory`
  - Supports `simple`, `mode_a` (dense clutter), `mode_b` (narrow corridor), `mode_c` (maze/dead-corner)
  - Difficulty scaling via `difficulty_level=1/2/3`
- **Vehicle & Dynamics Layer**: Ackermann single-vehicle model and articulated-train model
  - Dynamic switch from decoupled vehicles to coupled chain after each successful dock
- **Top Decision Layer**: Topology MCTS (`TopologyMCTSPlanner`)
  - Jointly decides docking hub and docking order
  - Uses cost-map risk/open-space terms and teacher-prior guidance
- **Bottom Planning/Control Layer**:
  - Hybrid A* (Ackermann-constrained)
  - Docking-stage fallback planning (higher search budget only when needed)
  - Docking closed-loop geometric alignment for final insertion
- **Evaluation Layer**:
  - Batch benchmark, failure taxonomy, auto-tune retry, early-stop guard
  - Visualization and structured reports in `results/`

## 2. Strategies for Three Core Problems
### 2.1 Docking Sequence (Who docks next)
- Top-level MCTS outputs an initial sequence.
- During execution, a feasibility-aware selector reorders remaining trailers by lightweight reachability score (distance + corridor risk + heading mismatch), reducing dead-end sequence risk.

### 2.2 Docking Position (Where to dock)
- Candidate hubs are sampled from a cost map using:
  - obstacle clearance
  - local openness
  - rearward corridor clearance for chain growth
- Top objective combines travel cost + risk + maneuverability terms.

### 2.3 Model Evolution (Single -> Articulated)
- Before dock: each vehicle follows Ackermann nonholonomic kinematics.
- After dock lock: follower front port is constrained to predecessor rear port; joint propagation updates follower pose from hitch motion.
- Whole chain motion uses articulated propagation with collision checks for each unit.

## 3. Seed 66 Deep Diagnosis
### 3.1 Reproduction
- Scenario: `seed=66`, `mode_c`, `difficulty=2`
- Historical failure: `Docking failed for R4 (PathPlanningFail)`
- Failure snapshot saved in `results/seed66_geometry_diag.json`.

### 3.2 Geometric Feasibility Check at Failure Stage
Failure instant (R4 stage):
- Train already attached: `['R1', 'R5', 'R2', 'R3']`
- Active trailer: `R4`
- Start pose: `(16.80, 86.43, 2.78 rad)`
- Goal pose: `(74.49, 49.01, -pi)`

Tests on the **same fixed geometry**:
- Current planner budget: **no path**
- Increased Hybrid A* budgets:
  - `T1`: path found (length 74)
  - `T2`: path found (length 86)
  - `T3`: path found (length 103)
- Holonomic heuristic at start is finite (`72.83`), proving topological reachability in occupancy graph.

### 3.3 Judgment
**Conclusion: Seed 66 is a planning-budget insufficiency case, not a geometric no-solution trap.**

Reason:
- Under same world and same vehicle states, higher-budget planners find feasible paths.
- Therefore failure is not caused by unavoidable physical impossibility.

### 3.4 Targeted Fix Applied
- Added docking-stage adaptive fallback planning (higher Hybrid A* budget tiers) when default planner fails.
- No collision-threshold relaxation was introduced.

### 3.5 Post-fix Verification
- `seed=66` rerun now succeeds: `results/seed66_after_fix_report.json`.
- Additional complex regression batch (`N=30`, includes the same difficulty region) reached 100% success:
  - `results/complex_regression_30_report.json`

## 4. Performance Evaluation Summary
| Benchmark | Runs | Success Rate | Avg sim_time (success) | Failure Breakdown |
|---|---:|---:|---:|---|
| Simple Baseline | 25 | 100.00% | 163.74s | none |
| Complex Stress (pre-fix full test) | 100 | 99.00% | 171.84s | 1 x PathPlanningFail (seed 66) |
| Complex Regression (post-fix) | 30 | 100.00% | 170.52s | none |

Reference reports:
- `results/simple_baseline_report.json`
- `results/complex_stress_report.json`
- `results/complex_regression_30_report.json`
- `results/generalization_report.json`

## 5. Known Limitations
1. Runtime in dense/maze scenes is still high due nonholonomic search cost.
2. Planner currently uses forward primitives only (no explicit reverse maneuver primitive), which may hurt efficiency in very tight maneuvers.
3. Full 100-run post-fix re-benchmark was not rerun in this final pass; post-fix evidence is provided via targeted seed66 replay + 30-run complex regression.

## 6. Deliverables Checklist
- [x] Multi-mode structured scene factory with difficulty control
- [x] Batch benchmark and failure taxonomy
- [x] Seed66 deep diagnosis with professional conclusion
- [x] Targeted algorithmic fix (budget-aware fallback, no collision-threshold cheating)
- [x] `README.md` with architecture and run guide
- [x] `FINAL_REPORT.md` complete
