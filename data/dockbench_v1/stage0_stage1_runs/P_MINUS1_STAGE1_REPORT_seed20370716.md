# P-1 Stage-1 Docking Report
## Scenario
- seed: `20370716`
- style: `switching_critical`
- obstacles: `3`
- leader: `(8.42, 1.38, yaw=20.1deg)`
- follower: `(0.53, -1.80, yaw=28.1deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `True`
- subset_tag: `Switching-Critical`
- leader_relocation_m: `0.26`
- follower_detour_ratio: `1.11`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 27.25 | `success` | `False` | 1.162 | 0.712 | 0 |
| `T_hard_switch` | `weak` | `False` | 3.00 | `collision` | `True` | 0.049 | 0.075 | 0 |
| `T_lattice_pbvs` | `strong` | `False` | 28.05 | `geometric_deadlock` | `False` | 0.850 | 0.135 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 28.05 | `geometric_deadlock` | `False` | 0.850 | 0.135 | 0 |
| `A_no_belief_gate` | `ablation` | `True` | 27.00 | `success` | `False` | 1.162 | 0.706 | 0 |
| `A_no_fallback` | `ablation` | `True` | 27.25 | `success` | `False` | 1.162 | 0.712 | 0 |
