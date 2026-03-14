# P-1 Stage-1 Docking Report
## Scenario
- seed: `20370454`
- style: `switching_critical`
- obstacles: `3`
- leader: `(10.28, -1.95, yaw=-16.9deg)`
- follower: `(1.94, 0.22, yaw=-10.2deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `True`
- subset_tag: `Switching-Critical`
- leader_relocation_m: `0.26`
- follower_detour_ratio: `1.11`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 15.05 | `success` | `False` | 0.619 | 0.578 | 0 |
