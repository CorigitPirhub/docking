# P-1 Stage-1 Docking Report
## Scenario
- seed: `20670978`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(4.11, -0.25, yaw=-27.4deg)`
- follower: `(-3.89, 1.03, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.04`
- follower_detour_ratio: `1.17`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 21.65 | `lc_geometric_deadlock` | `False` | 0.164 | 0.264 | 0 |
