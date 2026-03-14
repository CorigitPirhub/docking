# Round 25 Research Notes

## Dominant Remaining Failure Mode

- Round 24 shows LC failures are now dominated by `lc_geometric_deadlock` rather than collision.
- The common pattern is: a semantically reasonable terminal anchor exists, but the leader cannot reach an execution-valid release basin without either stalling in `LC_LEADER_SETTLE` or over-rotating during post-align.

## Primary-Source References Used For Method Ideation

- KAT / Hybrid A* with local confined-space trajectory optimization: https://arxiv.org/abs/1709.05443
- Safer Gap / keyhole-style passage selection under narrow-space constraints: https://arxiv.org/abs/2303.08243
- SPARROWS / local collision-set representations for repeated constrained planning: https://arxiv.org/abs/2402.08857
- CIAO* / constrained incremental search in dynamic obstacle fields: https://arxiv.org/pdf/2001.05449
- TAPOM / coupled planning under tight parking constraints: https://arxiv.org/abs/2511.05052

## Rejected Next-Step Prototype

- I sketched a `keyframe reachability` idea inspired by waypoint / keyhole decomposition, but I did **not** keep it in the active LC planning path because it was not validated and would have changed the accepted Round 24 behavior.
- The accepted state remains the Round 24 safety-restored planner with hard clearance filtering and post-align divergence suppression.
