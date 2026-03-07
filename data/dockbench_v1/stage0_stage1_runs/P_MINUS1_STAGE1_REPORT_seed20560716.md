# P-1 Stage-1 Docking Report
## Scenario
- seed: `20560716`
- style: `extension_critical`
- obstacles: `2`
- leader: `(-3.11, -1.73, yaw=-3.5deg)`
- follower: `(-11.55, -0.01, yaw=-1.8deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `True`
- subset_tag: `Extension-Critical`
- leader_relocation_m: `1.27`
- follower_detour_ratio: `1.20`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 32.05 | `geometric_deadlock` | `False` | 0.436 | 2.593 | 0 |
| `T_lattice_pbvs` | `strong` | `False` | 32.05 | `geometric_deadlock` | `False` | 0.205 | 0.173 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 32.05 | `geometric_deadlock` | `False` | 0.205 | 0.173 | 0 |
| `A_no_stage` | `ablation` | `False` | 4.05 | `collision` | `True` | 0.099 | 0.118 | 0 |
