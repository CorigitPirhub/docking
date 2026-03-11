# P-1 Stage-1 Docking Report
## Scenario
- seed: `20660585`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(4.43, 1.41, yaw=6.9deg)`
- follower: `(-2.13, -0.05, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.34`
- follower_detour_ratio: `1.16`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 16.00 | `lc_geometric_deadlock` | `False` | 0.239 | 0.290 | 0 |
