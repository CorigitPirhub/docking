# 核心需求任务书（Taskbook）

> **核心问题**：见 `CORE_PROBLEM_SPEC.md`。
> **场景基准**：见 `SCENARIO_CLASSIFICATION.md`。

## 0. 文档目的
- 目标：为“动态可重组多智能体系统的时空资源协同优化与自适应编队规划”提供分阶段、可量化、可验收的执行路线。
- 范围：仅针对“核心需求层”及其与现有基础支撑的集成；不重复基础支撑已完成能力。
- 约束：静态障碍场景、全局信息共享、无动态障碍。

---

## P0（必须先完成）：资料调研 + 创新框架搭建 + 先评估后实现

> **硬约束（Gate-0）**：在 P0 评估通过前，不进入核心框架的正式实现。  
> 允许：为验证可落地性，做极简原型 Demo（PoC），但不得进入工程化开发。

### P0.1 目标
- 广泛搜集相关领域资料，形成可复用的知识基线。
- 提出具备创新性的“策略层-规控层”解耦框架草案（平台无关）。
- 对框架进行“创新度 + 可实现性 + 风险”自评，结论为 `Go` 后才能进入 P1。

### P0.2 关键任务
1. 文献与方案扫描（至少覆盖以下方向）
- 多智能体任务分配：拍卖、CBBA、MCTS、滚动优化、分布式 MPC。
- 混合系统与编队重构：拓扑切换、事件触发控制、混合模式稳定性。
- 对接/解编策略：主动截获、可完成性判定、失败回退机制。
- 多目标优化：时间-能耗-安全 Pareto 与在线权重自适应。
- 执行要求：必须自主检索并筛选文献，完成深入阅读与跨论文联想，不得仅停留在二手综述摘录。

2. 形成框架草案（必须包含）
- 策略层输入/输出定义（离散决策变量、状态摘要、可行性反馈）。
- 规控层能力边界定义（可执行指令集、失败码、时序约束）。
- 两层异步一致性机制（命令有效期、版本号、冲突仲裁、回滚）。

3. 完成“创新度与可实现性评估”
- 创新点论证：与常规工程堆叠方案差异化对比。
- 可实现性论证：映射到当前平台已有能力，列出缺口与复杂度估计。
- 风险清单：算法风险、系统风险、实时性风险、评估风险。

4. 极简 PoC（可选，但推荐）
- 只验证关键链路：`策略命令 -> 规控执行 -> 反馈` 能闭环。
- 不追求性能最优，只验证框架可执行性。

### P0.2.1 执行风险提示（必须显式管理）
- 风险R1：文献覆盖广但理解浅，导致框架“看似新颖、实则拼装”。
- 风险R2：仅参考本领域，忽略相邻领域可迁移机制（如混合系统验证、任务分配博弈论）。
- 风险R3：过早实现导致架构冻结，后续创新空间被工程惯性锁死。
- 应对要求：每个候选创新点都要附“来源证据 + 差异化论证 + 最小可验证实验设想”。

### P0.3 产出物
- `reviews/P0_LITERATURE_REVIEW.md`（调研综述 + 方法族对比）
- `reviews/P0_FRAMEWORK_DESIGN.md`（架构图 + 接口规范 + 时序）
- `reviews/P0_INNOVATION_FEASIBILITY_REVIEW.md`（评分、风险、Go/No-Go）
- 可选：`p0_poc_demo/`（最小原型代码与说明）

### P0.4 量化评估指标（必须全部达标）
- 文献覆盖数量：`>= 30` 篇（其中近5年 `>= 20`，高相关 `>= 15`）。
- 竞品/基线对比：`>= 6` 套方法（含至少 2 套非学习方法 + 2 套学习方法）。
- 创新度评分（满分 10）：  
  - 新颖性 `>= 7.5`  
  - 解耦完整性 `>= 8.0`  
  - 泛化潜力 `>= 7.5`
- 可实现性评分（满分 10）：  
  - 工程可落地性 `>= 8.0`  
  - 与现有基础支撑兼容性 `>= 8.5`
- 风险可控性：高风险项必须有缓释策略，且未缓释高风险数 `= 0`。

### P0.5 阶段通过条件（Gate-0）
- 通过架构评审（Go/No-Go 结论为 `Go`）。
- 明确 P1 的“最小可实现子集（MVP）”与接口冻结版本。

### P0 完成记录（2026-03-02）

状态：`Completed`

交付物：

1. `reviews/P0_LITERATURE_REVIEW.md`
2. `reviews/P0_FRAMEWORK_DESIGN.md`
3. `reviews/P0_INNOVATION_FEASIBILITY_REVIEW.md`
4. `p0_poc_demo/`（最小闭环 PoC）
5. `artifacts/P0_POC_REPORT.md`
6. `artifacts/p0_poc_metrics.json`
7. `artifacts/p0_poc_timeline.png`
8. `artifacts/p0_literature_metrics.json`

量化结果：

1. 文献覆盖数量：`53`（阈值 `>=30`）
2. 近5年文献数量（2021-2026）：`39`（阈值 `>=20`）
3. 高相关文献数量：`44`（阈值 `>=15`）
4. 基线/竞品方法族：`8`（其中非学习 `5`、学习 `3`；阈值 `>=6`）
5. 创新度评分：新颖性 `8.2`、解耦完整性 `8.8`、泛化潜力 `8.1`
6. 可实现性评分：工程可落地性 `8.4`、兼容性 `9.0`
7. 未缓释高风险数量：`0`

PoC 验证结果（闭环链路）：

1. 拓扑不变量：通过
2. `Dock -> Split -> Re-Dock` 事件链路：通过
3. 硬瓶颈窗口（12s-14s）最大列车规模约束：通过（`<=1`）

Gate-0 结论：`Go`

---

## P1：核心接口冻结与策略执行总线落地（解耦工程化）

### P1.1 目标
- 将 P0 框架固化为可执行接口与运行时总线。
- 保障策略层与规控层可独立迭代、异步运行、可回溯。

### P1.2 关键任务
1. 策略层标准输入状态（平台无关摘要）定义
- 车辆摘要、拓扑摘要、场景标签摘要、可行性摘要、风险摘要。

2. 输出指令协议冻结
- `DockingCommand` / `SplitCommand` / `WaitCommand` 扩展字段（优先级、截止时间、版本号、撤销标记）。

3. 执行反馈协议
- `ACK`（accepted/rejected）+ 执行态 `EXEC` + 终态 `DONE/FAIL` + 失败码标准化。

4. 异步一致性机制
- 命令 TTL、重复命令去重、冲突仲裁、命令抢占与回滚。

### P1.3 产出物
- `interface/strategy_control_api.md`
- `interface/error_codes.md`
- `runtime/command_bus.py`（或等价模块）

### P1.4 量化指标
- 接口一致性测试通过率 `= 100%`
- 命令去重正确率 `= 100%`
- 过期命令拒绝正确率 `= 100%`
- 冲突命令仲裁正确率 `>= 99.5%`（随机压测）

### P1.5 Gate-1
- 协议冻结版本 `v1.0` 发布。
- 任意策略器可在不修改规控代码前提下完成接入。

### P1 完成记录（2026-03-02）

状态：`Completed`

交付物：

1. `interface/strategy_control_api.md`
2. `interface/error_codes.md`
3. `runtime/command_bus.py`
4. `tests/test_command_bus.py`
5. `scripts/validate_p1_command_bus.py`
6. `artifacts/p1_command_bus_validation.json`
7. `artifacts/P1_COMMAND_BUS_REPORT.md`

量化结果：

1. 接口一致性通过率：`1.0000`（阈值 `=1.0`）
2. 命令去重正确率：`1.0000`（阈值 `=1.0`）
3. 过期命令拒绝正确率：`1.0000`（阈值 `=1.0`）
4. 冲突仲裁正确率：`1.0000`（阈值 `>=0.995`）
5. 专项测试：`pytest -q tests/test_command_bus.py` 通过
6. 全量回归：`pytest -q` 通过

Gate-1 结论：`Go`

---

## P2：时空协同序列决策基线（时间-能耗混合代价）

### P2.1 目标
- 实现“谁先对接、何时对接、何时不对接”的可运行基线策略。
- 默认头车不停等，后车主动截获；保留“等待权”作为可选动作。

### P2.2 关键任务
1. 构建混合代价函数 `J_seq`
- 组成项：追击时间、对接耗时、能耗收益、风险惩罚、重构惩罚。
- 能耗项在 P2 采用“标称简化模型”：例如列车状态能耗系数固定 `eta_nominal=0.95`（可配区间 `0.95~0.97`），用于先跑通序列决策；P5 再替换为精细化/在线估计模型。

2. 任务分配器实现（建议两条线并行）
- 线A：拍卖/CBBA 风格分布式分配（高实时）
- 线B：滚动时域优化（短视窗 MPC/启发式搜索）

3. 主动截获点选择
- 基于头车共享轨迹，选取可达、低风险、对接友好的截获点。

### P2.3 产出物
- `strategy/baseline_scheduler.py`
- `docking/costs.py`
- `interface/foundation_cost_api.md`
- `experiments/p2_sequence_results.json`

### P2.4 量化指标
- 与“全独立行驶”相比：平均总能耗下降 `>= 2.5%`
- 与“固定序列对接”相比：平均总耗时下降 `>= 8%`
- 指令层面可执行率（被规控层接受并完成） `>= 95%`

### P2.5 Gate-2
- 基线策略在 A/B/C 三类场景均可稳定运行且优于至少一个朴素基线。

### P2 完成记录（2026-03-02）

状态：`Completed`

交付物：

1. `docking/costs.py`（序列代价/能耗/风险模型 + 行为代价查询接口）
2. `interface/foundation_cost_api.md`（冻结 cost-query 协议）
3. `strategy/baseline_scheduler.py`
4. `strategy/__init__.py`
5. `scripts/run_p2_sequence_experiments.py`
6. `tests/test_p2_scheduler.py`
7. `experiments/p2_sequence_results.json`
8. `artifacts/P2_SEQUENCE_REPORT.md`

量化结果（`seeds_per_subtype=8`，共 `72` 案例）：

1. 相对“全独立行驶”平均总能耗下降：`2.5043%`（阈值 `>=2.5%`）
2. 相对“固定序列对接”平均总耗时下降：`16.8403%`（阈值 `>=8%`）
3. 指令层面可执行率（adaptive）：`100.00%`（阈值 `>=95%`）
4. 批量结果判定：`gate2_ready=True`
5. 专项单测：`pytest -q tests/test_p2_scheduler.py` 通过
6. 全量回归：`pytest -q` 通过

Gate-2 结论：`Go`

---

## P3：动态通行能力判定与自适应解编策略

### P3.1 目标
- 将场景标签（`N_max_pass_profile`、`split_mandatory_zones`）转化为在线决策约束。
- 支持“尽量晚解编 + 安全可通过”的平衡控制。

### P3.2 关键任务
1. 在线可通过性判定器
- 输入局部路径段曲率/净空/编组长度，输出可通过编组上限。

2. 解编时机优化
- 在进入瓶颈前选取最晚安全解编点。

3. 过瓶颈后的重构判定
- 在 `dock_friendly_zones` 中触发重构机会评估。

### P3.3 量化指标
- Type B 场景：违规通行（超过 `N_max_pass`）次数 `= 0`
- Type C 场景：重构决策正确率（相对离线标注） `>= 90%`
- 因解编/重构导致的碰撞数 `= 0`

### P3.4 Gate-3
- B/C 场景通行与重构行为满足安全硬约束并达到目标准确率。

### P3 完成记录（2026-03-02）

状态：`Completed`

交付物：

1. `strategy/p3_reconfig.py`
2. `scripts/run_p3_reconfig_experiments.py`
3. `scripts/visualize_p3_reconfig.py`
4. `scripts/visualize_p3_reconfig_gif.py`
5. `tests/test_p3_reconfig.py`
6. `experiments/p3_reconfig_results.json`
7. `artifacts/P3_RECONFIG_REPORT.md`
8. `artifacts/P3_CONVERGENCE_NOTE.md`
9. `artifacts/p3_reconfig_B_representative.gif`
10. `artifacts/p3_reconfig_C_representative.gif`

量化结果（B/C 共 `90` 案例，`seeds_per_subtype=15`）：

1. Type B 违规通行总次数：`0`（阈值 `=0`）
2. Type C 重构决策准确率（相对离线标签）：`1.0000`（阈值 `>=0.90`）
3. 因解编/重构导致碰撞总数（约束违规代理）：`0`（阈值 `=0`）
4. 批量结果判定：`gate3_ready=True`
5. 专项单测：`pytest -q tests/test_p3_reconfig.py` 通过
6. 全量回归：`pytest -q` 通过

收敛性说明：

1. 已输出 P3 收敛与稳定性说明文档：`artifacts/P3_CONVERGENCE_NOTE.md`
2. 结论：有限路径 + 有界冷却 + 安全投影下，事件序列有限且保持通行约束不变式。

Gate-3 结论：`Go`

---

## P4：预测性主动对接与异常回退联动

### P4.1 目标
- 从“被动追尾对接”升级为“预测截获 + 异常联动回退”。

### P4.2 关键任务
1. 截获预测模块
- 使用头车轨迹预测 `T_pred` 区间候选对接点并排序。
- 明确定义：截获点是“头车未来轨迹上的路径点（path point）”，不是任意空间点；后车以该路径点为时空锚点执行主动截获。

2. 异常联动
- FOV丢失、路径阻塞、可完成性翻转触发重规划或转独立行驶。

3. 失败闭环
- 失败后不死锁，保证系统继续向终点推进。

### P4.3 量化指标
- 对接中断后的恢复成功率 `>= 90%`
- 因异常回退导致的任务失败率增量 `<= 5%`
- 中断后 3s 内重新进入可执行状态比例 `>= 95%`

### P4.4 Gate-4
- 异常触发场景下系统保持鲁棒，不出现长时间僵死。

### P4 完成记录（2026-03-02）

状态：`Completed`

交付物：

1. `strategy/p4_recovery.py`
2. `scripts/run_p4_recovery_experiments.py`
3. `scripts/visualize_p4_demo.py`
4. `tests/test_p4_recovery.py`
5. `experiments/p4_recovery_results.json`
6. `artifacts/P4_RECOVERY_REPORT.md`
7. `artifacts/p4_demo_timeline.png`
8. `artifacts/p4_demo_recovery.gif`

量化结果（A/B/C 共 `108` 案例，`seeds_per_subtype=12`）：

1. 对接中断总次数：`150`
2. 对接中断后恢复成功率：`1.0000`（阈值 `>=0.90`）
3. 中断后 3s 内重新进入可执行状态比例：`1.0000`（阈值 `>=0.95`）
4. 异常回退导致的任务失败率增量：`0.0000`（阈值 `<=0.05`）
5. 长时间僵死/死锁总数：`0`（Gate-4 鲁棒性要求）
6. 批量结果判定：`gate4_ready=True`
7. 专项单测：`pytest -q tests/test_p4_recovery.py` 通过
8. 全量回归：`pytest -q` 通过

Gate-4 结论：`Go`

---

## P4.5 Integration Checkpoint（新增，P5 前必过）

### 目标
- 验证 P2（对接序列策略）与 P3（解编约束策略）在混合场景下的协同一致性，避免策略“打架”。

### 关键检查项
1. 冲突一致性
- 当 P2 倾向“立即对接”而 P3 判定“即将进入瓶颈需解编”时，系统应按统一仲裁规则输出唯一动作。

2. 时序一致性
- 命令级联下不出现“刚对接即立刻解编”的抖振（除非安全硬约束触发）。

3. 约束优先级
- 安全与可通行性约束优先于能耗收益；在 Type C 场景中必须稳定生效。

### 量化指标
- Type C 场景冲突决策一致率 `>= 99%`。
- 互斥命令抖振次数（10s 滑窗）`<= 1`。
- 因策略冲突导致的额外失败率增量 `<= 2%`。

### 通过条件（Gate-4.5）
- 通过后才允许进入 P5 的多目标权重优化与 Pareto 收敛阶段。

### P4.5 完成记录（2026-03-02）

状态：`Completed`

交付物：

1. `strategy/p2_p4_integrated.py`（新增冲突观测、优先级仲裁、去抖振记录）
2. `scripts/run_p4_5_integration_checkpoint.py`
3. `scripts/visualize_p4_5_checkpoint.py`
4. `experiments/p4_5_integration_checkpoint_results.json`
5. `artifacts/P4_5_INTEGRATION_CHECKPOINT.md`
6. `artifacts/p4_5_metrics_summary.png`
7. `artifacts/p4_5_representative_timeline.png`

量化结果（Type C 压测集：`C1/C3`，`60` runs）：

1. 冲突一致率：`1.0000`（阈值 `>=0.99`）
2. 10s 窗口互斥抖振最大值：`1`（阈值 `<=1`）
3. 因策略冲突导致额外失败率增量：`0.0000`（阈值 `<=0.02`）
4. 先合后分后合链路稳定率：`1.0000`（内部稳定性门槛 `>=0.95`）

对照消融（`dock_priority`）：

1. 冲突一致率：`0.0000`
2. 额外失败率增量：`1.0000`
3. 先合后分后合稳定率：`0.0000`

Gate-4.5 结论：`Go`

### P4.5 修复后复核（2026-03-03）

针对“B 场景后段卡停（feasibility_block + 终点拥堵）”修复后执行复核：

1. Gate-B 验收口径修正：`integrated_success=True` 为必需条件（不再仅检查 `split_count` 与 `leader_s`）。
2. Type-C 冲突闭环复跑：`scripts/run_p4_5_integration_checkpoint.py --seeds-per-subtype 12`
3. B1 60s 多 seed 平台期检查：`scripts/run_b1_60_multiseed_check.py --num-seeds 12`

复核结果：

1. Gate-4.5（Type C1/C3, 24 runs）  
   冲突一致率 `1.0000`、10s 抖振最大值 `1`、额外失败率增量 `0.0000`、先合后分后合稳定率 `1.0000`，`gate4_5_ready=True`。
2. B1 60s 多 seed（12 runs）  
   成功率 `1.0000`、无碰撞率 `1.0000`、无平台停死率 `1.0000`、平均完成时间 `53.80s`。

复核结论：`Go`（满足进入 P5 的稳定性前置条件，样本规模为快速复核批次）。

---

## P5：多目标协同优化（时间-能耗-安全 Pareto）

### P5.1 目标
- 在策略层实现可调权重的多目标优化并输出 Pareto 前沿。

### P5.2 关键任务
1. 能耗收益模型接入（含列车节能系数）
- 将 P2 的 `eta_nominal` 简化模型升级为精细化模型（可含工况相关项 `eta(n, kappa, v)`），并支持在线校正/自适应估计。

2. 目标聚合与权重自适应
- `J = w_T T_done + w_E E_tot + w_R N_reconfig + w_S R_safety ...`

3. 策略比较与消融
- 去掉某项约束/目标后的性能变化分析。

### P5.3 量化指标
- 在 A/B/C 总体测试集：  
  - 总能耗较单车独立基线下降 `>= 3%`  
  - 总耗时不劣于基线（或下降 `>= 5%`）  
  - 碰撞数 `= 0`
- 输出至少 `>= 10` 个 Pareto 有效点。

### P5.4 Gate-5
- 形成可解释的 Pareto 曲线与推荐权重策略。

### P5 完成记录（2026-03-03）

状态：`Completed`

交付物：

1. `strategy/p5_multiobjective.py`
2. `scripts/run_p5_pareto_experiments.py`
3. `scripts/visualize_p5_pareto.py`
4. `tests/test_p5_multiobjective.py`
5. `experiments/p5_pareto_results.json`
6. `artifacts/P5_PARETO_REPORT.md`
7. `artifacts/p5_pareto_frontier.png`
8. `artifacts/p5_profile_tradeoff.png`

量化结果（`A/B/C` 共 `72` 案例，权重候选 `75` 组）：

1. Pareto 有效点数量：`36`（阈值 `>=10`）
2. 满足 Gate-5 约束的候选权重数：`1`
3. 推荐权重配置：`p061`
   - 相对单车独立基线平均能耗下降：`3.6463%`（阈值 `>=3%`）
   - 相对单车独立基线平均耗时改善：`0.2381%`（满足“不劣于基线”）
   - 碰撞数：`0`
   - 指令执行率：`100%`
4. 批量结果判定：`gate5_ready=True`
5. 专项测试：`pytest -q tests/test_p5_multiobjective.py` 通过
6. 全量回归：`pytest -q` 通过

Gate-5 结论：`Go`

---

## P6：系统级评估与论文级证据整理

### P6.1 目标
- 建立“可复现、可对比、可发布”的评测闭环。

### P6.2 关键任务
1. 大规模基准测试
- 每类场景多种初始分散模式与随机种子重复实验。

2. 对比与消融
- 与至少 2 个基线策略对比。

3. 证据包整理
- 指标表、关键可视化、失败案例分析、配置与代码版本锁定。

### P6.3 量化指标
- 总体任务成功率 `>= 95%`
- `T_done <= 60s` 达成率 `>= 90%`
- 可完成性判定准确率 `>= 90%`
- 全测试集碰撞次数 `= 0`

### P6.4 Gate-6（项目验收）
- 指标达标 + 复现实验脚本完整 + 文档可审计。

### P6 完成记录（2026-03-03）

状态：`Completed (Gate-6: No-Go)`

交付物：

1. `strategy/p6_system_eval.py`
2. `scripts/run_p6_system_evaluation.py`
3. `scripts/visualize_p6_results.py`
4. `tests/test_p6_system_eval.py`
5. `experiments/p6_system_evaluation_results.json`
6. `artifacts/P6_SYSTEM_EVALUATION_REPORT.md`
7. `artifacts/p6_policy_metrics.png`
8. `artifacts/p6_subtype_success_heatmap.png`
9. `artifacts/p6_time_energy_scatter.png`

评测配置（本轮）：

1. 场景覆盖：`A1/A2/A3/B1/B2/B3/C1/C2/C3`
2. 初始分散模式：`Random_Scattered`、`Uniform_Spread`
3. 每子类种子数：`1`
4. 车辆数：`2`
5. 策略对比：`integrated_split_priority`（主策略）、`integrated_dock_priority`（消融）、`independent`（基线）
6. 总运行数：`54`

量化结果（主策略 `integrated_split_priority`）：

1. 总体成功率：`0.4444`（阈值 `>=0.95`，未达标）
2. `T_done <= 60s` 达成率：`0.4444`（阈值 `>=0.90`，未达标）
3. 可完成性判定准确率（proxy）：`0.1013`（阈值 `>=0.90`，未达标）
4. 全测试集碰撞总数：`6`（阈值 `=0`，未达标）

对比结果：

1. 相对 `independent`：成功率 `-11.11%`，平均时间 `-0.097s`，平均能耗 `-12.36%`，碰撞总数持平（`6` vs `6`）。
2. 相对 `integrated_dock_priority`：成功率 `+5.56%`，平均时间 `-3.64s`，平均能耗 `-7.44%`，碰撞总数 `-2`。

失败聚类（摘要）：

1. 复杂瓶颈/混合场景（`B3/C1/C2/C3`）出现集中超时与碰撞。
2. `dock_priority` 在 Type-C 冲突场景失败更重，与 P4.5 冲突仲裁结论一致。

测试与回归：

1. `pytest -q tests/test_p6_system_eval.py tests/test_p5_multiobjective.py tests/test_p2_p4_integrated.py` 通过。
2. `pytest -q` 全量回归通过。

Gate-6 结论：`No-Go`

---

## 阶段依赖关系（必须遵守）
1. `P0 -> P1`：P0 不通过，禁止进入 P1 正式开发。
2. `P1 -> P2/P3/P4`：接口未冻结，禁止并行推进算法实现。
3. `P2/P3/P4 -> P5`：基线策略未稳定，禁止做权重调参与 Pareto 优化。
4. `P5 -> P6`：无稳定 Pareto 结果，不进入系统级结论汇总。

---

## 当前任务目标
保持在 P6 修复闭环：针对 `B3/C1/C2/C3` 的超时与碰撞问题做稳定性修复后重跑系统级评估，目标是把 Gate-6 从 `No-Go` 提升为 `Go`。
下一轮需优先收敛：`success_rate`、`T_done<=60s`、`collision_total` 三项硬门槛。

### P2-P4 演示修复记录（2026-03-02）

已完成“效果优先”的 P2-P4 联调修复与可视化重生成，结果作为 P4.5 前置演示基线：

1. 新增/修复集成执行链路：`strategy/p2_p4_integrated.py`
2. 重新定义 A/B/C 演示用例并固化：`scripts/run_p2_p4_integrated_demo.py`
3. 重做可视化与 GIF：`scripts/visualize_p2_p4_integrated_demo.py`
4. 效果验收测试通过：`tests/test_p2_p4_integrated.py`
5. 结果与报告：
   - `experiments/p2_p4_integrated_demo_results.json`
   - `artifacts/P2_P4_INTEGRATED_DEMO_REPORT.md`
   - `artifacts/p2_p4_demo_A.gif`
   - `artifacts/p2_p4_demo_B.gif`
   - `artifacts/p2_p4_demo_C.gif`
   - `artifacts/p4_demo_recovery.gif`（已覆盖为新版）

当前三类演示结论（case-pass）：

1. A（P2 对接）：`pass`
2. B（P3 解编）：`pass`
3. C（P4 恢复）：`pass`

汇总：`all_case_pass=true`（见 `experiments/p2_p4_integrated_demo_results.json`）。
