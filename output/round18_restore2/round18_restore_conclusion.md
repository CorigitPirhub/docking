# Round 18 Restore Validation

- Local LC family size: 18 scenes.
- LC success rate: 7/18 = 38.9%
- LC collision rate: 1/18 = 5.6%
- Successful LC mean |yaw error|: 32.65 deg
- Successful LC median |yaw error|: 32.75 deg
- LC failure-mode counts: {'collision_obstacle': 1, 'locked': 7, 'lc_geometric_deadlock': 10}

## LC Rows

| Scene | Success | Reason | Collision | T_done (s) | Min clear (m) | Goal yaw (deg) | Final yaw (deg) | |yaw diff| (deg) | Plan | Artifact |
|---|---:|---|---:|---:|---:|---:|---:|---:|---|---|
| DBv1-LC-L1-073 | False | collision_obstacle | True | 11.75 | 0.094 | 0.00 | 24.82 | 24.82 | lc_corridor_semantic_cert_anchor | `output/round18_restore2/DBv1-LC-L1-073/p_minus1_stage1_results_seed20660323.json` |
| DBv1-LC-L1-074 | True | locked | False | 13.05 | 0.103 | -25.46 | -31.06 | 31.06 | lc_corridor_smallgap_escape | `output/round18_restore2/DBv1-LC-L1-074/p_minus1_stage1_results_seed20660454.json` |
| DBv1-LC-L1-075 | False | lc_geometric_deadlock | False | 16.00 | 0.239 | 0.00 | -35.62 | 35.62 | lc_corridor_semantic_reference | `output/round18_restore2/DBv1-LC-L1-075/p_minus1_stage1_results_seed20660585.json` |
| DBv1-LC-L1-076 | True | locked | False | 12.70 | 0.153 | 27.26 | 32.87 | 32.87 | lc_corridor_smallgap_escape | `output/round18_restore2/DBv1-LC-L1-076/p_minus1_stage1_results_seed20660716.json` |
| DBv1-LC-L1-077 | False | lc_geometric_deadlock | False | 17.00 | 0.232 | 0.00 | 7.39 | 7.39 | lc_corridor_semantic_cert_anchor | `output/round18_restore2/DBv1-LC-L1-077/p_minus1_stage1_results_seed20660847.json` |
| DBv1-LC-L1-078 | False | lc_geometric_deadlock | False | 16.00 | 0.286 | 0.00 | -36.31 | 36.31 | lc_corridor_semantic_reference | `output/round18_restore2/DBv1-LC-L1-078/p_minus1_stage1_results_seed20660978.json` |
| DBv1-LC-L2-079 | False | lc_geometric_deadlock | False | 17.00 | 0.197 | 0.00 | -19.00 | 19.00 | lc_corridor_semantic_cert_anchor | `output/round18_restore2/DBv1-LC-L2-079/p_minus1_stage1_results_seed20670323.json` |
| DBv1-LC-L2-080 | True | locked | False | 14.35 | 0.100 | 24.54 | 24.20 | 24.20 | lc_corridor_macro_escape | `output/round18_restore2/DBv1-LC-L2-080/p_minus1_stage1_results_seed20670454.json` |
| DBv1-LC-L2-081 | True | locked | False | 15.10 | 0.153 | 15.00 | 30.08 | 30.08 | lc_corridor_semantic_cert_anchor | `output/round18_restore2/DBv1-LC-L2-081/p_minus1_stage1_results_seed20670585.json` |
| DBv1-LC-L2-082 | False | lc_geometric_deadlock | False | 31.10 | 0.222 | -15.00 | -15.04 | 15.04 | lc_corridor_semantic_cert_anchor | `output/round18_restore2/DBv1-LC-L2-082/p_minus1_stage1_results_seed20670716.json` |
| DBv1-LC-L2-083 | True | locked | False | 15.55 | 0.178 | -15.00 | -34.37 | 34.37 | lc_corridor_semantic_cert_anchor | `output/round18_restore2/DBv1-LC-L2-083/p_minus1_stage1_results_seed20670847.json` |
| DBv1-LC-L2-084 | False | lc_geometric_deadlock | False | 21.65 | 0.164 | 0.00 | -46.63 | 46.63 | lc_corridor_semantic_reference | `output/round18_restore2/DBv1-LC-L2-084/p_minus1_stage1_results_seed20670978.json` |
| DBv1-LC-L3-085 | True | locked | False | 19.40 | 0.195 | -35.83 | -43.25 | 43.25 | lc_corridor_exec_shape | `output/round18_restore2/DBv1-LC-L3-085/p_minus1_stage1_results_seed20680323.json` |
| DBv1-LC-L3-086 | False | lc_geometric_deadlock | False | 16.00 | 0.223 | 0.00 | -70.95 | 70.95 | lc_corridor_semantic_reference | `output/round18_restore2/DBv1-LC-L3-086/p_minus1_stage1_results_seed20680454.json` |
| DBv1-LC-L3-087 | True | locked | False | 10.75 | 0.112 | -33.39 | -32.75 | 32.75 | lc_corridor_exec_shape | `output/round18_restore2/DBv1-LC-L3-087/p_minus1_stage1_results_seed20680585.json` |
| DBv1-LC-L3-088 | False | lc_geometric_deadlock | False | 16.50 | 0.201 | 0.00 | -73.83 | 73.83 | lc_corridor_semantic_reference | `output/round18_restore2/DBv1-LC-L3-088/p_minus1_stage1_results_seed20680716.json` |
| DBv1-LC-L3-089 | False | lc_geometric_deadlock | False | 17.00 | 0.240 | 0.00 | -60.24 | 60.24 | lc_corridor_semantic_cert_anchor | `output/round18_restore2/DBv1-LC-L3-089/p_minus1_stage1_results_seed20680847.json` |
| DBv1-LC-L3-090 | False | lc_geometric_deadlock | False | 25.30 | 0.176 | 0.00 | -4.39 | 4.39 | lc_corridor_semantic_cert_anchor | `output/round18_restore2/DBv1-LC-L3-090/p_minus1_stage1_results_seed20680978.json` |

## Guard Rows

- DBv1-CF-L2-010: success=`True`, collision=`False`, done_time_s=`7.40`.
- DBv1-SC-L2-026: success=`True`, collision=`False`, done_time_s=`15.05`.
- DBv1-FC-L2-046: success=`True`, collision=`False`, done_time_s=`9.15`.
- DBv1-EC-L2-065: success=`True`, collision=`False`, done_time_s=`25.45`.
