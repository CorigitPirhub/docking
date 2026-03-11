# P-1 Stage-1 Docking Report
## Scenario
- seed: `20680716`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(3.67, 0.07, yaw=-44.9deg)`
- follower: `(-4.45, 1.32, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.30`
- follower_detour_ratio: `1.18`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 32.05 | `timeout` | `False` | 0.200 | 1.030 | 0 |
