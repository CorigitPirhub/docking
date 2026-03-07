# P-1 Stage-1 Docking Report
## Scenario
- seed: `20270454`
- style: `common_feasible`
- obstacles: `4`
- leader: `(10.56, -2.05, yaw=-1.7deg)`
- follower: `(2.24, -1.32, yaw=-7.2deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `False`
- subset_tag: `Common-Feasible`
- leader_relocation_m: `0.03`
- follower_detour_ratio: `1.04`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 7.45 | `success` | `False` | 1.255 | 0.313 | 0 |
| `T_lattice_pbvs` | `strong` | `True` | 8.45 | `success` | `False` | 1.223 | 0.120 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 7.40 | `collision` | `True` | 1.223 | 0.142 | 0 |
