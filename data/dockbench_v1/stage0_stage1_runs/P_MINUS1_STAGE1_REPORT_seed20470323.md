# P-1 Stage-1 Docking Report
## Scenario
- seed: `20470323`
- style: `funnel_critical`
- obstacles: `4`
- leader: `(8.40, 0.02, yaw=19.4deg)`
- follower: `(1.17, -2.90, yaw=5.7deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `False`
- subset_tag: `Funnel-Critical`
- leader_relocation_m: `0.31`
- follower_detour_ratio: `1.15`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 10.95 | `success` | `False` | 0.741 | 0.442 | 0 |
| `T_lattice_pbvs` | `strong` | `True` | 8.40 | `success` | `False` | 0.738 | 0.166 | 0 |
| `T_parking_hierarchical` | `strong` | `True` | 8.45 | `success` | `False` | 0.689 | 0.221 | 0 |
| `A_no_funnel_gate` | `ablation` | `True` | 8.65 | `success` | `False` | 0.731 | 0.345 | 0 |
| `A_no_micro_maneuver` | `ablation` | `True` | 10.95 | `success` | `False` | 0.741 | 0.442 | 0 |
