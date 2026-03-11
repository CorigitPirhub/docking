# P-1 Stage-1 Docking Report
## Scenario
- seed: `20680585`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(5.25, -1.37, yaw=-51.7deg)`
- follower: `(-3.02, -0.24, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `False`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `0.98`
- follower_detour_ratio: `1.18`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 17.00 | `lc_geometric_deadlock` | `False` | 0.246 | 0.196 | 0 |
