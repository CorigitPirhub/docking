# P-1 Stage-1 Docking Report
## Scenario
- seed: `20670585`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(4.82, -0.42, yaw=22.9deg)`
- follower: `(-2.49, -1.71, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.03`
- follower_detour_ratio: `1.17`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 16.40 | `success` | `False` | 0.152 | 3.257 | 0 |
