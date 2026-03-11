# P-1 Stage-1 Docking Report
## Scenario
- seed: `20680454`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(3.71, 0.05, yaw=-42.2deg)`
- follower: `(-2.93, 1.26, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.36`
- follower_detour_ratio: `1.18`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 30.55 | `lc_geometric_deadlock` | `False` | 0.218 | 1.133 | 0 |
