# Round 24 Conclusion

- Local LC family size: 18 scenes.
- LC success rate: 4/18 = 22.2%
- LC collision rate: 0/18 = 0.0%
- Successful LC mean |leader yaw error|: 29.13 deg
- LC failure modes: {'lc_geometric_deadlock': 12, 'locked': 4, 'timeout': 2}

## L2-081 Comparison

- Round 23/early baseline: goal yaw `15.00°`, final yaw `40.64°`, success `True`.
- Round 24: goal yaw `15.00°`, final yaw `28.99°`, success `True`.
- Round 24 selected anchor front clearance: `2.75` m.

## Interpretation

- The hard clearance floor removed the Round 23 collision on `DBv1-LC-L1-073` by aborting low-clearance certificate-ascent motion before contact.
- The post-align divergence guard now cuts off harmful yaw growth; `L2-081` remains successful and its final leader yaw drops back toward the pre-divergence level.
- This is a stop-loss round: safety is restored and the yaw blow-up is suppressed, but the planner still does not discover a substantially better terminal orientation for the successful LC cases.

## Guard Check

- DBv1-CF-L2-010: success=`True`, collision=`False`, done_time_s=`7.40`.
- DBv1-SC-L2-026: success=`True`, collision=`False`, done_time_s=`15.05`.
- DBv1-FC-L2-046: success=`True`, collision=`False`, done_time_s=`9.15`.
- DBv1-EC-L2-065: success=`True`, collision=`False`, done_time_s=`25.45`.
