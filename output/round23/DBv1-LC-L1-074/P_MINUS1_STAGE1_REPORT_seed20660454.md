# P-1 Stage-1 Docking Report
## Scenario
- seed: `20660454`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(5.53, -0.81, yaw=-10.5deg)`
- follower: `(-1.89, 0.61, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.18`
- follower_detour_ratio: `1.16`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 14.00 | `success` | `False` | 0.103 | 3.614 | 0 |
