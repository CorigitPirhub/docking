# DockBench-v1 Stage-0 Audit Report

## Summary
- dataset_root: `/home/zzy/TrajectoryPlanning/docking/data/dockbench_v1`
- tuning_scene_count: `12`
- stage0_pass: `False`

## Family Admission
- `CF` / `Common-Feasible`: `{"num_cells": 3, "num_pass": 3, "all_tuning_cells_pass": true, "strong_baseline_available": true, "representative_ready": true, "pass": true}`
- `SC` / `Switching-Critical`: `{"num_cells": 3, "num_pass": 2, "all_tuning_cells_pass": false, "strong_baseline_available": false, "representative_ready": true, "pass": false}`
- `FC` / `Funnel-Critical`: `{"num_cells": 3, "num_pass": 2, "all_tuning_cells_pass": false, "strong_baseline_available": true, "representative_ready": true, "pass": false}`
- `EC` / `Extension-Critical`: `{"num_cells": 3, "num_pass": 0, "all_tuning_cells_pass": false, "strong_baseline_available": true, "representative_ready": false, "pass": false}`

## Tuning Cell Checks

| scene_id | family | difficulty | pass | highlights |
|---|---|---|---:|---|
| `DBv1-CF-L1-003` | `CF` | `L1` | `True` | label_match=True, co_success=True, strong_baseline_success=True, limited_relocation=True, not_blocked=True |
| `DBv1-CF-L2-008` | `CF` | `L2` | `True` | label_match=True, co_success=True, strong_baseline_success=True, limited_relocation=True, not_blocked=True |
| `DBv1-CF-L3-015` | `CF` | `L3` | `True` | label_match=True, co_success=True, strong_baseline_success=True, limited_relocation=True, not_blocked=True |
| `DBv1-SC-L1-022` | `SC` | `L1` | `True` | label_match=True, co_success=True, blocked=True, switching_signal=True |
| `DBv1-SC-L2-026` | `SC` | `L2` | `False` | label_match=True, co_success=False, blocked=True, switching_signal=True |
| `DBv1-SC-L3-032` | `SC` | `L3` | `True` | label_match=True, co_success=True, blocked=True, switching_signal=True |
| `DBv1-FC-L1-041` | `FC` | `L1` | `False` | label_match=True, co_success=True, tight_dock_zone=True, funnel_signal=False |
| `DBv1-FC-L2-046` | `FC` | `L2` | `True` | label_match=True, co_success=True, tight_dock_zone=True, funnel_signal=True |
| `DBv1-FC-L3-050` | `FC` | `L3` | `True` | label_match=True, co_success=True, tight_dock_zone=True, funnel_signal=True |
| `DBv1-EC-L1-058` | `EC` | `L1` | `False` | label_match=True, co_success=False, requires_staging=True, no_stage_fails=True, blocked=True |
| `DBv1-EC-L2-064` | `EC` | `L2` | `False` | label_match=True, co_success=False, requires_staging=True, no_stage_fails=True, blocked=True |
| `DBv1-EC-L3-070` | `EC` | `L3` | `False` | label_match=True, co_success=False, requires_staging=True, no_stage_fails=True, blocked=True |

## Representative Readiness

| scene_id | family | difficulty | split | pass | highlights |
|---|---|---|---|---:|---|
| `DBv1-CF-L2-010` | `CF` | `L2` | `test` | `True` | label_match=True, co_success=True, strong_baseline_success=True, limited_relocation=True, not_blocked=True |
| `DBv1-SC-L2-028` | `SC` | `L2` | `test` | `True` | label_match=True, co_success=True, blocked=True, switching_signal=True |
| `DBv1-FC-L2-043` | `FC` | `L2` | `test` | `True` | label_match=True, co_success=True, tight_dock_zone=True, funnel_signal=True |
| `DBv1-EC-L2-065` | `EC` | `L2` | `test` | `False` | label_match=True, co_success=False, requires_staging=True, no_stage_fails=True, blocked=True |
