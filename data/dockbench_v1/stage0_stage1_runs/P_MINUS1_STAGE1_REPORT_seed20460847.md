# P-1 Stage-1 Docking Report
## Scenario
- seed: `20460847`
- style: `funnel_critical`
- obstacles: `2`
- leader: `(7.41, 0.17, yaw=-12.9deg)`
- follower: `(0.97, 2.41, yaw=-5.5deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `False`
- subset_tag: `Funnel-Critical`
- leader_relocation_m: `0.26`
- follower_detour_ratio: `1.13`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 10.00 | `success` | `False` | 0.774 | 0.479 | 0 |
| `T_lattice_pbvs` | `strong` | `True` | 7.25 | `success` | `False` | 0.688 | 0.176 | 0 |
| `T_parking_hierarchical` | `strong` | `True` | 7.25 | `success` | `False` | 0.791 | 0.230 | 0 |
| `A_no_funnel_gate` | `ablation` | `True` | 10.55 | `success` | `False` | 0.759 | 0.676 | 0 |
| `A_no_micro_maneuver` | `ablation` | `True` | 10.00 | `success` | `False` | 0.774 | 0.479 | 0 |
