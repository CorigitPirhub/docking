# P-1 Stage-1 Docking Report
## Scenario
- seed: `20260585`
- style: `common_feasible`
- obstacles: `3`
- leader: `(6.26, 1.10, yaw=-0.4deg)`
- follower: `(-1.15, 1.17, yaw=-3.2deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `False`
- subset_tag: `Common-Feasible`
- leader_relocation_m: `0.03`
- follower_detour_ratio: `1.03`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 6.80 | `success` | `False` | 2.292 | 0.366 | 0 |
| `T_lattice_pbvs` | `strong` | `True` | 7.30 | `success` | `False` | 2.307 | 0.117 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 6.30 | `collision` | `True` | 2.296 | 0.138 | 0 |
