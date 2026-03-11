# P-1 Stage-1 Docking Report
## Scenario
- seed: `20670716`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(3.55, -0.77, yaw=-24.7deg)`
- follower: `(-3.73, 0.50, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `False`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `0.93`
- follower_detour_ratio: `1.17`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 27.35 | `success` | `False` | 0.174 | 2.814 | 0 |
