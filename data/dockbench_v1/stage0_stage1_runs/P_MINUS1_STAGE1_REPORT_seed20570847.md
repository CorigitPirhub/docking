# P-1 Stage-1 Docking Report
## Scenario
- seed: `20570847`
- style: `extension_critical`
- obstacles: `4`
- leader: `(-7.09, -1.94, yaw=10.0deg)`
- follower: `(-17.30, -2.85, yaw=4.2deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `True`
- subset_tag: `Extension-Critical`
- leader_relocation_m: `1.44`
- follower_detour_ratio: `1.27`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 38.05 | `geometric_deadlock` | `False` | 0.475 | 4.512 | 3 |
| `T_lattice_pbvs` | `strong` | `False` | 38.05 | `geometric_deadlock` | `False` | 1.302 | 0.163 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 38.05 | `geometric_deadlock` | `False` | 1.302 | 0.163 | 0 |
| `A_no_stage` | `ablation` | `False` | 13.65 | `collision` | `True` | 0.094 | 0.147 | 1 |
