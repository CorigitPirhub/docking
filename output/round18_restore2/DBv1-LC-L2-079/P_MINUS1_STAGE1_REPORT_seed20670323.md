# P-1 Stage-1 Docking Report
## Scenario
- seed: `20670323`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(3.91, 2.91, yaw=27.4deg)`
- follower: `(-2.78, 1.56, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.24`
- follower_detour_ratio: `1.17`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 17.00 | `lc_geometric_deadlock` | `False` | 0.197 | 0.090 | 0 |
