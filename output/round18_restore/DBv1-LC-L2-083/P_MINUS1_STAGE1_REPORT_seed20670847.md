# P-1 Stage-1 Docking Report
## Scenario
- seed: `20670847`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(4.32, -1.05, yaw=-27.1deg)`
- follower: `(-2.67, 0.19, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `False`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `0.94`
- follower_detour_ratio: `1.17`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 15.45 | `success` | `False` | 0.179 | 2.633 | 0 |
