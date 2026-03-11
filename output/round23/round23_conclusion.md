# Round 23 Conclusion

- Local LC family size: 18 scenes.
- LC success rate: 5/18 = 27.8%
- LC collision rate: 1/18 = 5.6%
- Successful LC mean |leader yaw error|: 38.01 deg
- LC failure modes: {'collision_obstacle': 1, 'locked': 5, 'lc_geometric_deadlock': 10, 'timeout': 2}

## L2-081 Comparison

- Round 21 baseline: goal yaw `15.00°`, final yaw `40.64°`, success `True`.
- Round 23: goal yaw `15.00°`, final yaw `40.64°`, success `True`.
- Round 23 front clearance for selected L2-081 anchor: `2.75` m.
- Round 23 longitudinal offset for selected L2-081 anchor: `0.20000000000000018` m.

## Interpretation

- The planner now uses an explicit soft cost with path length, clearance, yaw error, front mobility, and longitudinal slack terms.
- This improves candidate introspection and preserves the Round 21 timeout-to-lock safety net, but it does not yet achieve the required L2-081 yaw improvement while keeping success.
- In the stable configuration, L2-081 remains successful but keeps the same 15° anchor family and large final yaw error.
- A stronger yaw bias can reduce the chosen goal yaw, but in prior probes it regressed to geometric deadlock. The added front-clearance and longitudinal terms were not sufficient to change that trade-off in the stable setting.

## Guard Check

- DBv1-CF-L2-010: success=`True`, collision=`False`, done_time_s=`7.40`.
- DBv1-SC-L2-026: success=`True`, collision=`False`, done_time_s=`15.05`.
- DBv1-FC-L2-046: success=`True`, collision=`False`, done_time_s=`9.15`.
- DBv1-EC-L2-065: success=`True`, collision=`False`, done_time_s=`25.45`.
