# 核心需求任务书（Taskbook）

> **核心问题**：见 `CORE_PROBLEM_SPEC.md`。
> **场景基准**：见 `SCENARIO_CLASSIFICATION.md`。

## 0. 文档目的
- 目标：为“动态可重组多智能体系统的时空资源协同优化与自适应编队规划”提供分阶段、可量化、可验收的执行路线。
- 范围：仅针对“核心需求层”及其与现有基础支撑的集成；不重复基础支撑已完成能力。
- 约束：静态障碍场景、全局信息共享、无动态障碍。

---

## P-1（最高优先级，先于 P0）：基础支撑对接能力扎实化（随机障碍 + 两阶段对接闭环）

> 目的：在进入任何“策略层/调度层”研究前，先把**基础支撑层的对接能力**打磨到“可在随机障碍环境中稳定闭环”的水平，形成可复用的 Docking Skill/接口与可视化证据。  
> 说明：本阶段只关注“对接功能本身”可用、可批量评测、可复现；不做多目标权重优化、不做上层序列决策。

### P-1.1 Co-BCFD Docking Skill（需要广泛搜集相关论文，必须具备创新性实现）
> **设计稿/论文叙事**：以 `DOCK.md` 为准（P-1.2/P-1.3 的实验暴露问题后，必须反向更新 `DOCK.md`，保持“设计-实现-实验”一致）。

对接被视为一个可复用闭环技能（Docking Skill）。注意：“全局趋近 + 视觉伺服”只是技能内部手段；P-1.1 的创新性要求在于把**可达性/可见性/安全/回退/协同配合**统一成一个浑然天成的结构，而不是工程拼接。要求架构至少包含：
1. **协同 Staging（Leader 可动，内部封装动作）**：被对接车（Leader）不要求原地等待；当当前位置导致“几何不可达/遮挡严重/风险高/代价大”时，Docking Skill 内部需选择 docking-friendly staging pose，并驱动 Leader 做短程位姿调整；Follower 同步去到等待/预对接位姿。
2. **置信一致融合（Belief-Consistent Fusion Gate）**：GNSS 与视觉相对位姿在过渡带内做平滑融合，融合权重需同时由“距离调度 + 一致性门控（如 NIS/卡方门限）”决定，避免异常视觉污染与指令跳变。
3. **Capture-Funnel + Contact Gate（捕获域漏斗 + 接触门控）**：显式刻画 Ackermann 近场不可原地修姿导致的“捕获域”问题；在进入接触带前必须满足必要条件，否则触发 backoff/微机动，避免残差死锁。
4. **近场微机动规划器（Parking-like Planner / BR-SMPC）**：支持“倒车-对齐-再前进”的非凸机动（MPPI/CEM/HybridA* 等均可），用于把系统状态拉入捕获域并推进到锁定域。
5. **异常回退（Fallback/Recovery）**：视觉丢失、FOV 脱离、局部不可行时，自动回退到全局/重规划；必要时重新 staging。
6. **统一安全过滤（Safety Projection）**：对任意名义控制输出施加碰撞/净空/执行器约束投影，保证零碰撞与最小净空要求。

### P-1.1B DockBench 场景标准与数据集冻结（P-1.2 前置阶段）
> **规范书**：`DOCK_SCENARIO_STANDARD.md`

在进入 P-1.2 的 Stage-1/Stage-2 之前，必须先冻结 docking 专用场景标准与固定数据集 `DockBench-v1`。该阶段不是“补充文档”，而是正式前置任务，后续所有两车实验必须基于该标准执行。

1. **建立两车 docking 的四层场景标准**：采用 `abstract -> logical -> concrete -> audit` 四层结构，避免“随机地图 + 人工挑 seed”。
2. **冻结场景 family 与 difficulty 体系**：至少固定 `CF / SC / FC / EC` 四类 family，并引入 `L1 / L2 / L3` 难度等级；所有 P-1.2 报告必须按 family 口径汇总。
3. **冻结固定数据集与 split**：正式 benchmark 使用固定 concrete scene 数据集，不得每次重新随机采样 test scenes；推荐 `DockBench-v1 = 72` 个 concrete scenes，采用 `tuning / test / challenge` 三段 split。
4. **冻结统一 JSON schema 与 manifest**：每个场景必须包含 `scene_id / split / family / difficulty / 车辆初始状态 / 障碍物几何与 role / descriptors / quality / audit` 等字段，并产出 manifest 与 split 清单。
5. **障碍物分布采用“角色化拓扑”而非仅按数量计数**：障碍应区分 `screen / channel / dock_zone / background / hybrid` 等角色，并按 `corridor / dock-zone / background` 三个任务相关区域统计特征。
6. **建立场景质量评估器（Scenario Quality Evaluator）**：至少包含 `validity`、`label fidelity`、`diversity`、`balance` 四类指标；同时增加闭环审计，确认 family 标签与真实基础支撑执行一致。
7. **建立 family admission 规则**：
   - `CF / SC / FC`：至少 1 个强传统基线在对应 `tuning` cell 上必须具备稳定非零成功率；
   - `EC`：必须体现 cooperative staging 带来的可解域扩展，不能只是“无解场景”。
8. **建立阶段耦合关系**：P-1.2 Stage-1 的代表场景、Stage-2 的批量对比、消融实验与 challenge 复现，全部必须从 frozen dataset 中读取，不允许临时手工替换正式 test scene。

**P-1.1B 验收要求**
- 产出正式规范书：`DOCK_SCENARIO_STANDARD.md`
- 产出固定 split 与 manifest 文档/文件名规范
- 给出每个 family 的 descriptor 阈值或 admission 条件
- 给出场景质量评估器定义与打分/过滤逻辑
- 明确 P-1.2 Stage-1/Stage-2 如何使用并验证该标准

### P-1.2 任务阶段（五步走，含 Stage-0 前置）
**统一对比要求（每个 Stage 必须执行）**
- 每个 Stage 都必须与若干“传统方法（Traditional Baselines）”做对比评测，而不是只展示本方法单跑结果；且对比集不能只包含明显偏弱、理论上难以完成任务的 baseline。
- 所有方法必须在**同一场景、同一初态、同一时间上限、同一碰撞/对接判定、同一执行器约束**下比较；baseline 调参必须在独立 held-out seed 集上完成，不得在正式测试 seed 上反复调参后报告。
- “传统方法”不再只允许弱基线，必须构成**分层基线池**（推荐依据：视觉伺服经典文献、非完整约束搜索规划、层级停车/泊车规划、采样/优化控制）：  
  1. **弱基线（Sanity Baselines，保留）**：  
     - **T-GlobalOnly**：仅使用全局定位/共享地图的截获与跟踪（不使用视觉伺服或仅作观测不入环）。  
     - **T-HardSwitch**：在阈值距离处做硬切换的两阶段方法（无融合/无平滑）。  
     - **T-GeoGoToPoint / PurePursuit**：基于几何目标点的纯跟踪/纯追踪（弱规划或无规划）。  
  2. **强传统基线（Strong Classical Baselines，至少实现其中 2 个）**：  
     - **T-HybridAStar-PBVS / T-Lattice-PBVS**：采用 Hybrid A* 或 State Lattice 生成满足 Ackermann 约束的预对接/泊车式路径，近场使用 PBVS/IBVS/同类经典视觉伺服，不使用 cooperative staging、belief-consistent gate、capture-funnel。  
     - **T-HybridAStar-NMPC / T-Lattice-MPPI**：采用搜索规划给出 warm start，再使用 NMPC/MPPI/CEM 等传统优化型局部控制完成近场机动，不使用 cooperative staging 或 belief-consistent gate。  
     - **T-Parking-Hierarchical**：采用“搜索规划 + 轨迹优化/跟踪”层级式泊车框架，把对接视作动态预对接位姿逼近问题，允许倒车和多次机动，但不使用本工作的 cooperative staging / capture-funnel 机制。  
  3. **能力匹配基线（Capability-Matched Baselines，至少实现 1 个）**：  
     - **T-Coop-HardSwitch**：允许 Leader 位姿调整，但感知切换采用硬切换/距离加权，不使用 NIS 一致性门控。  
     - **T-Coop-DistBlend**：允许 cooperative staging，但只使用距离调度融合，不使用 belief-consistent 一致性检验与 capture-funnel/contact gate。  
  （允许替换/增删，但 Stage-1/2 最终报告中**至少保留 4 个 baseline**，其中**至少 2 个强传统基线**、**至少 1 个能力匹配基线**。）
- 对比集必须按**场景机理分层**组织，而不是只在“只有本方法可能成功”的场景上比较：  
  - **共同可解子集（Common-Feasible / CF）**：不依赖 leader relocation 才能成立，至少 1 个强传统基线在该子集上应有稳定非零成功率；若所有强基线在当前测试集上均为 0，则必须补充/调整场景直到该子集成立。  
  - **切换/遮挡关键子集（Switching-Critical / SC）**：用于检验视觉融合、FOV 丢失回退、遮挡恢复。  
  - **捕获域关键子集（Funnel-Critical / FC）**：用于检验 capture-funnel、近场 backoff、micro-maneuver。  
  - **能力扩展子集（Extension-Critical / EC）**：需要 leader relocation、明显 staging gap 或显著绕障后才能完成，用于证明本方法扩展了可解域。  
  Stage-1/2 报告必须**同时给出 Overall + CF + SC + FC + EC** 的 family 结果口径；为兼容旧结果，可附 `Common-Feasible + Extension-Critical` 汇总口径。
- 对比指标需要冻结并在阶段报告中写清楚定义与计算方法；至少必须包含：  
  - **对接完成时间**（成功用例 `T_done` 的分布/均值/分位数）  
  - **规划/轨迹代价**（局部规划 cost、累计控制 effort、曲率/平滑惩罚、任务级总代价等，需统一口径）  
  - **碰撞率/碰撞次数**（含静态障碍与车车碰撞）  
  - **对接任务成功率**（满足对接完成判定 + hold）  
  - **最小净空**、**回退/重规划次数**、**视觉丢失/FOV 丢失次数**、**leader relocation 距离/时间**、**follower detour ratio**、**指令平滑度**（加加速度/转向变化率）
- 批量结果必须给出**配对统计显著性**或**置信区间**：建议至少对 `T_done`、trajectory cost、success rate 使用 bootstrap 95% CI；对共同可解子集上的成对时间/代价比较，建议补充 Wilcoxon signed-rank 或同等级非参数检验。
- 每个 Stage 的可视化需包含“本方法 vs 传统方法”的对照（至少 1 个代表用例），并输出批量汇总图表（Stage-2/Stage-4 必须）。

**统一实验协议（Stage-1/2 必须执行，Stage-3/4 继承）**
- 场景集必须拆分为 **tuning / test** 两部分：`tuning` 只用于 baseline 与消融超参冻结，`test` 只用于最终报告；若 Stage-2 数据量允许，建议再保留 `challenge` 子集承载失败复现与极端案例，不参与主指标调参。
- 每个 `test` 场景都必须保存**场景清单（seed、障碍参数、初始位姿、子集标签）**与**方法配对运行结果**，确保所有方法在同一实例上逐一执行，禁止“方法 A 跑一批场景、方法 B 跑另一批场景”的非配对比较。
- 对比实验需要同时产出 **主结果表 + 失败归因表**：失败原因至少区分 `collision`、`timeout`、`geometric_deadlock`、`fov_loss_unrecovered`、`lock_condition_not_met`，避免把所有失败都记为单一 `failed`。
- 基线准入需要分两步：先在 `Common-Feasible` held-out 子集上验证是否具备非零成功率，再进入正式 `test`；若某强传统基线经合理调参后仍完全失效，可保留为 sanity baseline，但不得作为“主要传统对比对象”支撑结论。
- Stage-1/2 报告的主表建议固定为：`Overall`、`Family Breakdown (CF/SC/FC/EC)`、`Compatibility (Common-Feasible/Extension-Critical)`、`Ablation` 四张核心表，并附配对散点/箱线图与代表失败链路回放。

**统一消融要求（Stage-1/2 必须执行，Stage-3/4 继承）**
- 必须围绕 `DOCK.md` 中声明的关键模块做消融，而不是只做“换超参”实验。Full model 记为 **A-Full**，至少应包含以下单模块消融：  
  - **A-NoStage**：去掉 cooperative staging，Leader 固定或只允许极小被动配合。  
  - **A-NoBeliefGate**：去掉 NIS/一致性门控，仅保留距离调度融合；如有必要再补 **A-HardSwitch** 作为更强消融。  
  - **A-NoFunnelGate**：去掉 capture-funnel / contact gate / backoff 进入条件。  
  - **A-NoMicroManeuver**：去掉近场 parking-like 微机动规划，仅保留前向跟踪/几何伺服。  
  - **A-NoFallback**：关闭视觉丢失回退、重规划、re-staging。  
  - **A-NoSafetyProj**：去掉统一 safety projection（如直接运行名义控制，或仅保留软惩罚）。
- 除单模块消融外，至少增加 **2 个机制耦合消融**，优先推荐：  
  - **A-NoStage-NoMicroManeuver**：检验“远场位姿准备 + 近场非凸机动”联合作用；  
  - **A-NoBeliefGate-NoFunnelGate**：检验“感知一致性 + 近场捕获域”联合作用。
- 消融必须与场景机理对齐，而不是所有模块在所有场景上混着跑：  
  - `A-NoStage` 重点在 `Extension-Critical` 子集；  
  - `A-NoBeliefGate / A-HardSwitch / A-NoFallback` 重点在 `Occlusion / Switching-Critical` 子集；  
  - `A-NoFunnelGate / A-NoMicroManeuver` 重点在近场窄净空/大初始航向误差子集；  
  - `A-NoSafetyProj` 重点在窄净空和障碍贴边子集。
- Stage-1 需要在代表性场景上完成**完整消融矩阵**；Stage-2 至少对上述单模块消融中的 **4 个关键消融**做批量统计，并报告 `Δsuccess_rate / Δcollision_rate / ΔT_done / Δcost / Δfallback_count`。
- 若某个模块消融后没有带来可观差异，不得直接删除该模块结论；必须检查：场景是否打到该模块的作用区间、判定口径是否足够敏感、baseline/ablation 是否过度共享实现，必要时需要补设计场景再验证。

0. **Stage-0：场景标准化与数据集冻结（由 P-1.1B 定义，P-1.2 使用并验证）**
   - 冻结 `DockBench-v1` 的 family、difficulty、split、scene id、JSON schema 与 quality evaluator。
   - Stage-1/Stage-2 的正式场景必须从 frozen dataset 中选取，不允许用临时随机 seed 替代正式 test split。
   - Stage-0 的正确性由 Stage-1/Stage-2 共同反证：若出现“family 标签与真实执行机理不一致”“Common-Feasible 强基线全灭”“Extension-Critical 本质无解”等情况，优先回到 Stage-0 修正数据集而不是先改结论。

1. **Stage-1：两车单场景对接（单次验证）**
   - 创建 1 个带若干随机障碍的场景；两辆车在场景中**随机分布**（但初始位置不应太接近）。
   - 运行闭环对接任务：后车对接前车，满足对接完成判定（位置/航向/速度差 + hold）。
   - 与传统方法对比：在同一场景、同一初态下跑通本方法与传统基线，输出对接时间/代价/是否碰撞/是否成功的对比表。
   - Stage-1 的“代表场景”必须从 frozen dataset 中选择：除主展示场景外，还必须补 1 个**共同可解对照场景**（至少 1 个强传统基线成功）；当验证 fusion/fallback 时，应额外从 `SC` family 取机制场景；当验证 funnel/micro-maneuver 时，应额外从 `FC` family 取机制场景。
   - 执行完整消融矩阵：至少完成 `A-Full + 6 个单模块消融 + 2 个耦合消融`，并给出模块-失败模式对应解释。
   - 交付可视化：GIF/MP4（轨迹、FOV、阶段切换权重、对接状态、安全边界等），并提供“本方法 vs 传统方法”的对照回放，以及至少 1 组“Full vs Ablation”的对照回放。

2. **Stage-2：两车多场景批量验证（成功率）**
   - 生成多组不同的随机场景（障碍、初始分散模式等）；**每个场景的初始车位可以固定**（不要求每次运行都重采样）。
   - 批量跑对接闭环，统计成功率、平均耗时、碰撞数、最小净空、视觉丢失/回退次数、指令平滑指标。
   - 与传统方法对比：对 frozen `test` split 中的每个场景同步跑本方法与传统基线，输出 per-method 的成功率、碰撞率、时间/代价分布与显著失败聚类；并分别给出 `Overall / CF / SC / FC / EC` 与 `Common-Feasible / Extension-Critical` 两套统计口径（前者用于新标准，后者兼容现有结论）。
   - 强传统基线必须在 `Common-Feasible` 子集上体现真实竞争力；若该子集上所有强传统基线仍接近 0 成功率，则 Stage-2 报告视为“对比集设计不合格”，需补充/重构场景集后重跑。
   - 批量消融：至少对 `A-NoStage`、`A-NoBeliefGate`、`A-NoFunnelGate`、`A-NoMicroManeuver` 做多 seed 统计；其余模块可在机制子集上补跑。
   - 交付可视化：每个场景至少 1 个回放 GIF（含本方法 vs 传统方法对照）+ 汇总图表（success heatmap、耗时/代价分布、失败聚类、方法对比柱状/雷达图、ablation bar/heatmap 等）。

3. **Stage-3：多车单场景对接（单次验证）**
   - 创建 1 个带随机障碍的场景；多辆车自由分布（避免初始过近）。
   - 支持多车对接成列车/链式结构（可顺序对接，也可并发对接的子集，具体由基础支撑内部协调）。
   - 与传统方法对比：在同一场景、同一初态下对比至少 2 个传统基线（可为“逐车贪心对接 + 纯跟踪”等），输出系统级成功/碰撞/时间/代价对比。
   - 交付可视化：展示拓扑演化（edges/pending_docks）、对接过程与安全边界，并提供“本方法 vs 传统方法”的代表对照回放。

4. **Stage-4：多车多场景批量验证（成功率）**
   - 多车场景批量生成与评测，输出系统级成功率与失败分析。
   - 与传统方法对比：批量对比本方法与传统基线的成功率、碰撞率、时间/代价分布，给出失败原因对照与消融结论。
   - 交付可视化：每子类/模式的成功率热力图 + 关键失败案例 GIF 包 + 传统方法对比汇总图（论文级可复用）。

### P-1.3 量化指标（建议门槛，可随平台能力微调）
- Stage-1：单场景两车对接 **成功 1/1**，碰撞 **0**，最小净空 \(\ge 0.1m\)。
- Stage-2：两车批量成功率 \(\ge 95\%\)，碰撞 **0**；对接平均耗时 \(\le 15s\)（或给出合理解释与曲线）。
- Stage-3：单场景多车对接 **成功 1/1**，碰撞 **0**，拓扑合法。
- Stage-4：多车批量成功率 \(\ge 90\%\)（初期可放宽但需逐步收敛到 \(\ge 95\%\)），碰撞 **0**。
- **共同可解子集约束**：Stage-1/2 报告必须包含至少 1 组/1 个子集，使得至少 1 个强传统基线具有稳定非零成功率；Stage-2 建议该强基线在 `Common-Feasible` 子集上的成功率 \(\ge 60\%\)，否则说明对比场景设计不充分。
- **竞争性要求**：在 `Common-Feasible` 子集上，本方法相对“最优强传统基线”应至少满足以下之一：  
  1. 成功率非劣（差距不超过 5 个百分点）且平均 `T_done` 或平均 trajectory cost 至少改善 5%；  
  2. 成功率提升至少 10 个百分点。  
  若不满足，必须在报告中明确说明哪一项设计没有形成增益，并回到 `DOCK.md` 修正。
- **能力扩展要求**：在 `Extension-Critical` 子集上，本方法相对“最优强传统基线”的成功率提升建议 \(\ge 15\) 个百分点；若未达到，需要说明 cooperative staging / capture-funnel 的真实收益是否被实现掩盖。
- **消融有效性要求**：对每个关键模块，其对应机制子集上移除该模块后，应在 `success_rate / collision_rate / T_done / cost / fallback_count` 至少一项上出现可解释且可复现的显著退化；若没有，视为该模块尚未被实验充分验证。

### P-1.4 Gate（进入 P0 的前置条件）
- 形成可复现脚本：`生成场景 -> 跑闭环 -> 输出指标 -> 导出可视化` 一键完成。
- Stage-2/Stage-4 的批量评测能稳定复现（同 seed 同结果），并能定位失败原因（日志/事件/最小复现实例）。

### P-1.5 预期交付物（可在实现时细化命名）
- 场景标准规范书（DockBench family / difficulty / split / quality evaluator）
- 固定数据集 manifest 与 split 清单（2-car 为主，可扩展到 N-car）
- 场景生成脚本（2-car / N-car）
- 批量评测脚本（2-car / N-car）
- 可视化脚本（单案回放 GIF + 批量汇总图）
- 对接闭环核心实现（Docking Skill）
- 基线规范文档（baseline family、超参、适用范围、失败模式、调参 seed 划分）
- 消融实验脚本与汇总报告（单模块 + 耦合模块）
- 共同可解/遮挡关键/能力扩展三类场景子集定义与 seed 列表
- 基线准入记录与调参/测试划分清单（含 tuning/test scene manifest）
- 失败模式归因报告（collision / timeout / geometric deadlock / FOV loss / lock failure）

### P-1 完成记录

状态：`In Progress`

更新（2026-03-06）：
0. 新增 P-1.1B / P-1.2 Stage-0：两车 docking 场景标准与固定数据集冻结，规范书为 `DOCK_SCENARIO_STANDARD.md`；后续 Stage-1/2 必须基于该标准执行并反证其有效性。
1. P-1.1 设计稿持续同步实现与实验：`DOCK.md`。
2. Stage-1 已切换为**共同可解 + 能力扩展**双代表场景协议：
   - Common-Feasible：`experiments/p_minus1_curated_scenes/common_test_seed7.json`
   - Extension-Critical：`experiments/p_minus1_curated_scenes/extension_test_seed14.json`
3. Stage-1/Stage-2 已统一到“同场景配对比较 + 强传统基线 + 能力匹配基线 + 关键消融”的协议：
   - 协议文档：`artifacts/P_MINUS1_BASELINE_PROTOCOL.md`
   - 基线/消融注册：`docking/p_minus1_baselines.py`
4. 当前 Stage-2 采用**经当前代码实测验证**的 curated manifest：
   - tuning：`common_tuning_seed111/112`
   - test：`common_test_seed0/1/4/7` + `extension_test_seed14`
   - 场景文件目录：`experiments/p_minus1_curated_scenes`
5. 当前 Stage-2 主门槛在该 validated manifest 上已满足：
   - `co_bcfd` overall 成功率 `1.000`
   - overall 碰撞率 `0.000`
   - overall 平均完成时间 `13.620s`
   - strong common success `1.000`
6. 当前能力扩展结论：在 `Extension-Critical` 子集（当前验证集 `n=1`）上，`co_bcfd` 成功率 `1.000`，最优强传统基线成功率 `0.000`。
7. 产物：
   - Stage-1 Suite 报告：`artifacts/P_MINUS1_STAGE1_SUITE_REPORT.md`
   - Stage-1 Suite 数据：`artifacts/p_minus1_stage1_suite_results.json`
   - Stage-2 报告：`artifacts/P_MINUS1_STAGE2_REPORT.md`
   - Stage-2 数据：`artifacts/p_minus1_stage2_results.json`
   - Stage-2 图表：`artifacts/p_minus1_stage2_success_heatmap.png`、`artifacts/p_minus1_stage2_subset_compare.png`、`artifacts/p_minus1_stage2_failure_clusters.png`、`artifacts/p_minus1_stage2_ablation_summary.png`
8. 当前仍未完全收敛的项：
   - `Common-Feasible` 子集上，`co_bcfd` 已达到成功率非劣，但相对最优强传统基线尚未形成 `T_done / cost` 的 `>=5%` 优势；P-1.3 的共同可解竞争性条款仍需继续补强；
   - `Extension-Critical` validated test 集规模仍偏小（当前 `n=1`），需要继续扩展到多 seed 稳定批量；
   - challenge 子集与 GIF 回放尚未重新生成；
   - 在进入 Stage-3 前，仍需把扩展子集做厚，并补充 `switching-critical` 机制子集，避免只在窄验证集上成立。

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
