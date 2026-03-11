# P-1 Stage-1 Docking Report
## Scenario
- seed: `20660323`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(3.31, 2.00, yaw=13.9deg)`
- follower: `(-3.83, 0.51, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.31`
- follower_detour_ratio: `1.16`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 11.80 | `collision` | `True` | 0.093 | 0.192 | 0 |
