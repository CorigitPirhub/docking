# P-1 Stage-1 Docking Report
## Scenario
- seed: `20370454`
- style: `switching_critical`
- obstacles: `3`
- leader: `(5.24, 0.13, yaw=-14.1deg)`
- follower: `(-3.00, 2.19, yaw=-7.0deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `True`
- subset_tag: `Switching-Critical`
- leader_relocation_m: `0.26`
- follower_detour_ratio: `1.11`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 28.05 | `geometric_deadlock` | `False` | 0.101 | 0.776 | 3 |
| `T_hard_switch` | `weak` | `False` | 8.05 | `collision` | `True` | 0.118 | 0.121 | 0 |
| `T_lattice_pbvs` | `strong` | `False` | 28.05 | `geometric_deadlock` | `False` | 0.655 | 0.168 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 28.05 | `geometric_deadlock` | `False` | 0.655 | 0.168 | 0 |
| `A_no_belief_gate` | `ablation` | `False` | 28.05 | `geometric_deadlock` | `False` | 0.101 | 0.776 | 3 |
| `A_no_fallback` | `ablation` | `False` | 28.05 | `geometric_deadlock` | `False` | 0.101 | 0.776 | 3 |
