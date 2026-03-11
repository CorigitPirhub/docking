# P-1 Stage-1 Docking Report
## Scenario
- seed: `20680847`
- style: `lane_constrained`
- obstacles: `4`
- leader: `(3.30, 0.00, yaw=-54.8deg)`
- follower: `(-5.03, 1.05, yaw=0.0deg)`
- large_obstacle_present: `True`
- direct_los_blocked: `False`
- subset_tag: `Lane-Constrained`
- leader_relocation_m: `1.20`
- follower_detour_ratio: `1.18`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `False` | 17.00 | `lc_geometric_deadlock` | `False` | 0.242 | 0.166 | 0 |
