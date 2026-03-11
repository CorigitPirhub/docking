# P-1 Stage-1 Docking Report
## Scenario
- seed: `20660716`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(5.20, 0.15, yaw=12.3deg)`
- follower: `(-1.38, -1.21, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.12`
- follower_detour_ratio: `1.16`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 15.25 | `success` | `False` | 0.153 | 3.196 | 0 |
