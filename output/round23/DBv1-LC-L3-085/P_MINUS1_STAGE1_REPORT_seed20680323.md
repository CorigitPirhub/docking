# P-1 Stage-1 Docking Report
## Scenario
- seed: `20680323`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(3.25, -2.87, yaw=-43.2deg)`
- follower: `(-3.76, -1.53, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.27`
- follower_detour_ratio: `1.18`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 32.05 | `timeout` | `False` | 0.204 | 0.830 | 0 |
