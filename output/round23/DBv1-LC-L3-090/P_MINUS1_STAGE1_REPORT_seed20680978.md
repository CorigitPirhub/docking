# P-1 Stage-1 Docking Report
## Scenario
- seed: `20680978`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(3.24, 0.37, yaw=-47.0deg)`
- follower: `(-3.66, 1.63, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.64`
- follower_detour_ratio: `1.19`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 25.15 | `lc_geometric_deadlock` | `False` | 0.176 | 0.414 | 0 |
