# P0 跨同类问题迁移证据（补强版）

## 1. 目的

用于补足当前任务目标中的缺口：

> “启发性证据不足：还没展示跨同类问题迁移案例（不仅是当前平台可用）”

---

## 2. 迁移验证方法

采用同一套框架代码（命令协议、总线语义、事件反馈、仲裁顺序不变），
仅通过 `FrameworkProfile` 参数适配不同任务语境：

1. Case-A：`ground_transport_convoy`
- 语义：地面运输编组场景
- 参数：更长路径、更大间距

2. Case-B：`warehouse_tugger_convoy`
- 语义：仓储牵引编组场景
- 参数：更短路径、更紧凑间距、不同瓶颈区间与等待策略

执行脚本：

```bash
python standalone_framework_demo/run_transferability_suite.py
```

---

## 3. 结果摘要

来源：`artifacts/standalone_framework_transferability/transferability_summary.json`

1. case_count: `2`
2. all_ok: `true`

分案例结果：

1. `ground_transport_convoy`
- ok=true
- invariant_ok=true
- execute_accept=9, execute_reject=0
- DOCK_LOCKED=5, SPLIT_DONE=2
- max_leader_train_size_hard_bottleneck=1.0

2. `warehouse_tugger_convoy`
- ok=true
- invariant_ok=true
- execute_accept=7, execute_reject=0
- DOCK_LOCKED=4, SPLIT_DONE=2
- max_leader_train_size_hard_bottleneck=1.0

---

## 4. 证据文件清单

1. 总结报告  
`artifacts/standalone_framework_transferability/TRANSFERABILITY_SUMMARY.md`

2. 总结JSON  
`artifacts/standalone_framework_transferability/transferability_summary.json`

3. Case-A 可视化与指标  
`artifacts/standalone_framework_transferability/ground_transport_convoy/*`

4. Case-B 可视化与指标  
`artifacts/standalone_framework_transferability/warehouse_tugger_convoy/*`

---

## 5. 启发性结论

1. 同一“策略-规控解耦 + 命令总线”框架可在同类重构任务间复用；
2. 迁移仅依赖 profile 参数调整，而非重写命令语义或总线机制；
3. 框架具备论文层面的“可迁移性论据”，可支撑“对同类问题启发性强”的主张。
