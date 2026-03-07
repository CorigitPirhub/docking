# P-1 Stage-1 Docking Report
## Scenario
- seed: `20280585`
- style: `common_feasible`
- obstacles: `5`
- leader: `(7.51, 0.87, yaw=-1.3deg)`
- follower: `(-1.92, 0.75, yaw=6.0deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `False`
- subset_tag: `Common-Feasible`
- leader_relocation_m: `0.04`
- follower_detour_ratio: `1.05`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 8.15 | `success` | `False` | 1.153 | 0.340 | 0 |
| `T_lattice_pbvs` | `strong` | `True` | 9.60 | `success` | `False` | 1.153 | 0.124 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 8.60 | `collision` | `True` | 1.153 | 0.144 | 0 |
