# P-1 Stage-1 Docking Report
## Scenario
- seed: `20660978`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(3.94, -0.27, yaw=6.1deg)`
- follower: `(-3.10, -1.75, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.25`
- follower_detour_ratio: `1.16`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 16.00 | `lc_geometric_deadlock` | `False` | 0.286 | 0.269 | 0 |
