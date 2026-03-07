# P-1 Stage-1 Docking Report
## Scenario
- seed: `20580716`
- style: `extension_critical`
- obstacles: `5`
- leader: `(-6.45, -0.56, yaw=17.6deg)`
- follower: `(-16.98, -2.34, yaw=8.9deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `True`
- subset_tag: `Extension-Critical`
- leader_relocation_m: `1.45`
- follower_detour_ratio: `1.30`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 44.05 | `geometric_deadlock` | `False` | 1.102 | 0.071 | 3 |
| `T_lattice_pbvs` | `strong` | `False` | 44.05 | `geometric_deadlock` | `False` | 1.102 | 0.086 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 44.05 | `geometric_deadlock` | `False` | 1.102 | 0.086 | 0 |
| `A_no_stage` | `ablation` | `False` | 44.05 | `geometric_deadlock` | `False` | 1.102 | 0.071 | 3 |
