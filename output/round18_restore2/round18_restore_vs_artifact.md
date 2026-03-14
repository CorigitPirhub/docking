# Round18 Restore vs Archived Artifact

| Scene | Archived Success | Restored Success | Archived Reason | Restored Reason | Match |
|---|---:|---:|---|---|---:|
| DBv1-LC-L1-073 | False | False | collision_obstacle | collision_obstacle | True |
| DBv1-LC-L1-074 | True | True | locked | locked | True |
| DBv1-LC-L1-075 | False | False | lc_geometric_deadlock | lc_geometric_deadlock | True |
| DBv1-LC-L1-076 | True | True | locked | locked | True |
| DBv1-LC-L1-077 | False | False | lc_geometric_deadlock | lc_geometric_deadlock | True |
| DBv1-LC-L1-078 | False | False | lc_geometric_deadlock | lc_geometric_deadlock | True |
| DBv1-LC-L2-079 | False | False | lc_geometric_deadlock | lc_geometric_deadlock | True |
| DBv1-LC-L2-080 | True | True | locked | locked | True |
| DBv1-LC-L2-081 | True | True | locked | locked | True |
| DBv1-LC-L2-082 | False | False | lc_geometric_deadlock | lc_geometric_deadlock | True |
| DBv1-LC-L2-083 | True | True | locked | locked | True |
| DBv1-LC-L2-084 | False | False | lc_geometric_deadlock | lc_geometric_deadlock | True |
| DBv1-LC-L3-085 | False | True | lc_geometric_deadlock | locked | False |
| DBv1-LC-L3-086 | False | False | lc_geometric_deadlock | lc_geometric_deadlock | True |
| DBv1-LC-L3-087 | True | True | locked | locked | True |
| DBv1-LC-L3-088 | False | False | lc_geometric_deadlock | lc_geometric_deadlock | True |
| DBv1-LC-L3-089 | False | False | lc_geometric_deadlock | lc_geometric_deadlock | True |
| DBv1-LC-L3-090 | False | False | lc_geometric_deadlock | lc_geometric_deadlock | True |
| DBv1-CF-L2-010 | True | True | locked | locked | True |
| DBv1-SC-L2-026 | True | True | locked | locked | True |
| DBv1-FC-L2-046 | True | True | locked | locked | True |
| DBv1-EC-L2-065 | True | True | locked | locked | True |

- Exact scene-level success/reason match: 21/22.
- Key gate scenes `L1-073 / L2-081 / L2-083` are restored to the archived Round18 behavior.
- One notable deviation remains: `DBv1-LC-L3-085` succeeds in the restored run while the archived artifact recorded `lc_geometric_deadlock`.
