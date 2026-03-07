# P-1 Stage-1 Docking Report
## Scenario
- seed: `20570716`
- style: `extension_critical`
- obstacles: `4`
- leader: `(-2.58, 0.19, yaw=-0.7deg)`
- follower: `(-12.84, 0.49, yaw=0.4deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `True`
- subset_tag: `Extension-Critical`
- leader_relocation_m: `1.44`
- follower_detour_ratio: `1.27`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 38.05 | `geometric_deadlock` | `False` | 0.505 | 1.375 | 1 |
| `T_lattice_pbvs` | `strong` | `False` | 38.05 | `geometric_deadlock` | `False` | 1.302 | 0.094 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 38.05 | `geometric_deadlock` | `False` | 1.302 | 0.094 | 0 |
| `A_no_stage` | `ablation` | `False` | 14.00 | `collision` | `True` | 0.097 | 0.151 | 1 |
