# P-1 Stage-1 Docking Report
## Scenario
- seed: `20270716`
- style: `common_feasible`
- obstacles: `4`
- leader: `(6.73, -0.67, yaw=-13.1deg)`
- follower: `(-1.41, 1.23, yaw=-7.6deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `False`
- subset_tag: `Common-Feasible`
- leader_relocation_m: `0.03`
- follower_detour_ratio: `1.04`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 7.40 | `success` | `False` | 3.566 | 0.325 | 0 |
| `T_lattice_pbvs` | `strong` | `True` | 8.35 | `success` | `False` | 3.566 | 0.119 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 7.35 | `collision` | `True` | 3.566 | 0.140 | 0 |
