# P-1 Stage-1 Docking Report
## Scenario
- seed: `20470716`
- style: `funnel_critical`
- obstacles: `3`
- leader: `(8.23, -0.27, yaw=16.3deg)`
- follower: `(0.70, -1.34, yaw=3.2deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `False`
- subset_tag: `Funnel-Critical`
- leader_relocation_m: `0.26`
- follower_detour_ratio: `1.15`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 11.50 | `success` | `False` | 0.644 | 0.439 | 0 |
| `T_lattice_pbvs` | `strong` | `True` | 8.60 | `success` | `False` | 0.617 | 0.164 | 0 |
| `T_parking_hierarchical` | `strong` | `True` | 8.70 | `success` | `False` | 0.661 | 0.222 | 0 |
| `A_no_funnel_gate` | `ablation` | `True` | 11.45 | `success` | `False` | 0.624 | 0.518 | 0 |
| `A_no_micro_maneuver` | `ablation` | `True` | 11.50 | `success` | `False` | 0.644 | 0.439 | 0 |
