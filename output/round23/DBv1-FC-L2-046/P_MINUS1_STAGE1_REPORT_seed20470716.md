# P-1 Stage-1 Docking Report
## Scenario
- seed: `20470716`
- style: `funnel_critical`
- obstacles: `3`
- leader: `(8.81, -2.49, yaw=-8.8deg)`
- follower: `(1.35, -1.23, yaw=-23.7deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `False`
- subset_tag: `Funnel-Critical`
- leader_relocation_m: `0.26`
- follower_detour_ratio: `1.15`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 9.15 | `success` | `False` | 0.696 | 0.428 | 0 |
