# P-1 Stage-1 Docking Report
## Scenario
- seed: `20570847`
- style: `extension_critical`
- obstacles: `4`
- leader: `(-7.09, -1.94, yaw=10.0deg)`
- follower: `(-17.30, -2.85, yaw=4.2deg)`
- large_obstacle_present: `False`
- direct_los_blocked: `True`
- subset_tag: `Extension-Critical`
- leader_relocation_m: `1.44`
- follower_detour_ratio: `1.27`

## Results (single-scene)

| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |
|---|---|---:|---:|---|---:|---:|---:|---:|
| `co_bcfd` | `full` | `True` | 25.45 | `success` | `False` | 0.372 | 3.737 | 0 |
