# P-1 Stage-1 Docking Report
## Scenario
- seed: `20360716`
- style: `switching_critical`
- obstacles: `2`
- leader: `(9.50, -1.38, yaw=1.1deg)`
- follower: `(1.86, -0.92, yaw=7.9deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `True`
- subset_tag: `Switching-Critical`
- leader_relocation_m: `0.23`
- follower_detour_ratio: `1.09`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 29.30 | `success` | `False` | 0.954 | 2.050 | 0 |
| `T_hard_switch` | `weak` | `False` | 3.95 | `collision` | `True` | 0.096 | 0.083 | 0 |
| `T_lattice_pbvs` | `strong` | `False` | 30.05 | `geometric_deadlock` | `False` | 0.100 | 1.652 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 30.05 | `geometric_deadlock` | `False` | 0.100 | 1.652 | 0 |
| `A_no_belief_gate` | `ablation` | `True` | 29.20 | `success` | `False` | 0.954 | 2.028 | 0 |
| `A_no_fallback` | `ablation` | `True` | 29.30 | `success` | `False` | 0.954 | 2.050 | 0 |
