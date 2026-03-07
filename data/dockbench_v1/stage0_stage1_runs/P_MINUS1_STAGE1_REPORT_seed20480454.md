# P-1 Stage-1 Docking Report
## Scenario
- seed: `20480454`
- style: `funnel_critical`
- obstacles: `4`
- leader: `(6.46, -1.04, yaw=16.5deg)`
- follower: `(-1.42, -3.47, yaw=-3.5deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `False`
- subset_tag: `Funnel-Critical`
- leader_relocation_m: `0.27`
- follower_detour_ratio: `1.18`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 28.55 | `success` | `False` | 0.575 | 0.842 | 0 |
| `T_lattice_pbvs` | `strong` | `False` | 36.05 | `geometric_deadlock` | `False` | 1.129 | 0.111 | 0 |
| `T_parking_hierarchical` | `strong` | `False` | 36.05 | `geometric_deadlock` | `False` | 1.129 | 0.111 | 0 |
| `A_no_funnel_gate` | `ablation` | `True` | 27.30 | `success` | `False` | 0.575 | 0.823 | 0 |
| `A_no_micro_maneuver` | `ablation` | `True` | 28.55 | `success` | `False` | 0.575 | 0.842 | 0 |
