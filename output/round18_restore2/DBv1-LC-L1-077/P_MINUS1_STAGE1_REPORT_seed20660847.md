# P-1 Stage-1 Docking Report
## Scenario
- seed: `20660847`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(4.42, -1.89, yaw=-9.5deg)`
- follower: `(-2.02, -0.42, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `True`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.30`
- follower_detour_ratio: `1.16`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 17.00 | `lc_geometric_deadlock` | `False` | 0.232 | 0.108 | 0 |
