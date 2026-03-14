# P-1 Stage-1 Docking Report
## Scenario
- seed: `20670454`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(5.18, -0.12, yaw=22.6deg)`
- follower: `(-1.75, -1.41, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.13`
- follower_detour_ratio: `1.17`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 14.35 | `success` | `False` | 0.100 | 1.932 | 0 |
