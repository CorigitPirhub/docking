# Round18 Restore3 Compare

| Scene | Archived Result | Restored Result | Archived Plan | Restored Plan | Match |
|---|---|---|---|---|---|
| `DBv1-LC-L3-085` | `lc_geometric_deadlock` | `lc_geometric_deadlock` | `lc_corridor_semantic_reference` | `lc_corridor_semantic_reference` | `True` |
| `DBv1-LC-L3-087` | `locked` | `locked` | `lc_corridor_exec_shape` | `lc_corridor_exec_shape` | `True` |

## Notes
- `DBv1-LC-L3-085` now matches the archived Round18 behavior exactly: `success=False`, `reason=lc_geometric_deadlock`, `goal_yaw=0°`, `final_leader_yaw≈-86.14°`.
- `DBv1-LC-L3-087` remains aligned with the archived successful `lc_corridor_exec_shape` branch.
- The restoration patch is the `combined_robust < 0.18` acceptance floor inside `docking/lc_corridor.py` for `lc_corridor_exec_shape`.
