# P-1 Stage-1 Docking Report
## Scenario
- seed: `20380454`
- style: `switching_critical`
- obstacles: `3`
- leader: `(6.10, -2.91, yaw=-2.8deg)`
- follower: `(-3.72, -2.21, yaw=-13.0deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `True`
- subset_tag: `Switching-Critical`
- leader_relocation_m: `0.27`
- follower_detour_ratio: `1.12`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 28.15 | `success` | `False` | 1.044 | 0.962 | 0 |
| `T_hard_switch` | `weak` | `False` | 4.05 | `collision` | `True` | 0.076 | 0.086 | 0 |
| `T_lattice_pbvs` | `strong` | `False` | 32.05 | `geometric_deadlock` | `False` | 1.125 | 0.116 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 32.05 | `geometric_deadlock` | `False` | 1.125 | 0.116 | 0 |
| `A_no_belief_gate` | `ablation` | `True` | 27.85 | `success` | `False` | 1.044 | 0.902 | 0 |
| `A_no_fallback` | `ablation` | `True` | 28.15 | `success` | `False` | 1.044 | 0.962 | 0 |
