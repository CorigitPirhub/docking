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
7. **统一证书场（Certificate Field）**：P-1.1/论文口径中，不再把 `CGFL / TVT / VPCR / PTCC` 叙述成四个并列工程模块；它们必须被统一为同一 Docking Skill 的证书化实现。推荐将其收敛为三个证书坐标：`进展证书`（吸收 `CGFL + PTCC`）、`终端证书`（吸收 `TVT + terminal capture boost + CTP`）、`可见性证书`（吸收 `VPCR`），并由同一个 belief-consistent capture-funnel 策略按证书值自适应切换/投影。

### P-1.1B DockBench 场景标准与数据集冻结（P-1.2 前置阶段）
> **规范书**：`DOCK_SCENARIO_STANDARD.md`

在进入 P-1.2 的 Stage-1/Stage-2 之前，必须先冻结 docking 专用场景标准与固定数据集 `DockBench-v1`。该阶段不是“补充文档”，而是正式前置任务，后续所有两车实验必须基于该标准执行。

1. **建立两车 docking 的四层场景标准**：采用 `abstract -> logical -> concrete -> audit` 四层结构，避免“随机地图 + 人工挑 seed”。
2. **冻结场景 family 与 difficulty 体系**：固定 `CF / SC / FC / EC / LC` 五类 family，并引入 `L1 / L2 / L3` 难度等级；所有 P-1.2 报告必须按 family 口径汇总。
3. **冻结固定数据集与 split**：正式 benchmark 使用固定 concrete scene 数据集，不得每次重新随机采样 test scenes；正式版 `DockBench-v1 = 90` 个 concrete scenes，采用 `tuning / test / challenge` 三段 split（`15 / 60 / 15`）。
4. **冻结统一 JSON schema 与 manifest**：每个场景必须包含 `scene_id / split / family / difficulty / 车辆初始状态 / 障碍物几何与 role / descriptors / quality / audit` 等字段，并产出 manifest 与 split 清单。
5. **障碍物分布采用“角色化拓扑”而非仅按数量计数**：障碍应区分 `screen / channel / dock_zone / background / hybrid` 等角色，并按 `corridor / dock-zone / background` 三个任务相关区域统计特征。
6. **新增独立场景类别 `LC (Lane-Constrained)`**：`lane-constrained` 不是所有场景都附带的一条正交轴，而是必须单独设计的一类正式场景类别。此类场景的核心不是“两车之间被障碍物简单阻隔”，而是**车辆的可行规划空间被车道/走廊宽度严格限制**，后车不能通过大幅度绕行在整张地图上任意找路。
7. **明确 `LC` 类别的核心研究目的**：`LC` 的设计必须突出“**初始航向角差**在通道约束下对对接难度的显著影响”。
   - 当两车初始航向角差较小时，对接应显著容易；
   - 当两车初始航向角差较大时，由于 follower 不能通过巨大转弯和自由绕行来修正几何关系，对接应显著困难；
   - 因而 `LC` 场景的关键不是 follower 的大范围绕障，而是 **Leader 必须学会倒车转弯、前后切换、带转角微调等协同微机动** 来主动创造 docking-friendly pose。
8. **冻结 `LC` 场景的车道拓扑与可行域 schema**：对 `LC` concrete scene，必须在 JSON schema 中显式给出 `drivable corridor` 的结构化表征（如 `corridor polygons / centerline / lane graph / bottleneck segments / passing bay / merge-diverge` 之一或组合），并给出“可行驶区域”与“非可行驶区域”的明确定义；scene audit 必须能够据此判断车辆轨迹是否发生 `off-lane shortcut`。
9. **冻结 `LC` 场景的关键 descriptors**：除现有 descriptors 外，`LC` 至少必须显式冻结：`corridor_width_min_m / width_profile / corridor_turn_deg_max / bottleneck_ratio / heading_diff_deg / leader_reverse_turn_required / leader_micro_adjust_budget_m / off_lane_shortcut_gap_m`。其中 `heading_diff_deg` 与 `leader_reverse_turn_required` 是 `LC` 难度分级的核心描述量。
10. **建立 `LC` 的难度分级与 admission 条件**：`LC-L1/L2/L3` 必须围绕“航向差 + 通道宽度 + Leader 微调需求”来定义，而不是简单复用原 family 的 admission 规则。至少要求：
   - `LC-L1`：航向差较小，Leader 仅需小幅位姿调整；
   - `LC-L2`：航向差中等，Leader 需要显式前进/后退切换或带转角微调；
   - `LC-L3`：航向差大，Leader 需要倒车转弯等更复杂协同微机动才能把系统送入可对接盆地。
11. **建立车道忠实度审计（Lane Fidelity Audit）**：对于声明为 `LC` 的场景，必须增加专门审计：
   - 审计对象不是“整张图的 polygon 自由空间”，而是 `corridor geodesic / lane graph` 所定义的合法 lane-follow 路径；
   - follower 若脱离车道约束便可显著缩短路径，则该场景才算“车道约束有效”；
   - `lane_fidelity` 必须以 `off_lane_shortcut_gap_m` 为主量，结合 `outside_ratio` 或 `geodesic inflation ratio` 判定，而不是只靠单一 polygon 内外测试；
   - 若存在明显自由空间捷径使 follower 可绕开车道瓶颈，则该场景必须被判为 `lane_fidelity = false` 并剔除。
12. **建立场景质量评估器（Scenario Quality Evaluator）**：至少包含 `validity`、`label fidelity`、`diversity`、`balance` 四类指标；同时增加闭环审计，确认 family 标签与真实基础支撑执行一致。
13. **建立 family / category admission 规则**：
   - `CF / SC / FC`：至少 1 个强传统基线在对应 `tuning` cell 上必须具备稳定非零成功率；
   - `EC`：必须体现 cooperative staging 带来的可解域扩展，不能只是“无解场景”；
   - `LC`：必须体现“航向差增大 → follower 自由绕行受限 → 对 Leader 倒车转弯微调需求显著上升”的机制，不能退化成普通自由空间场景。
14. **建立阶段耦合关系**：P-1.2 Stage-1 的代表场景、Stage-2 的批量对比、消融实验与 challenge 复现，全部必须从 frozen dataset 中读取，不允许临时手工替换正式 test scene。

**P-1.1B 验收要求**
- 产出正式规范书：`DOCK_SCENARIO_STANDARD.md`，其中新增独立 `LC (Lane-Constrained)` 场景类别的定义、schema 字段、descriptor 与 audit 规则
- 产出固定 split 与 manifest 文档/文件名规范，并在 manifest 中显式标注场景是否属于 `LC`
- 给出每个 family 的 descriptor 阈值或 admission 条件，并补充 `LC` 的 heading-diff / leader-reverse-turn 相关 admission 条件
- 给出场景质量评估器定义与打分/过滤逻辑，新增 `lane_fidelity`、`off_lane_shortcut` 与 `leader_reverse_turn_required` 审计结果
- 明确 P-1.2 Stage-1/Stage-2 如何使用并验证该标准，且报告中需单独给出 `LC` 类别结果
- 给出 `LC` 数据集覆盖目标：正式 frozen dataset 中该类样本必须形成稳定、可复现的非零占比，并覆盖“航向差小 / 中 / 大”三个层次，且至少包含需要 Leader 倒车转弯微调的代表样本

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
   - 当前仓库的标准落地输出目录固定为 `data/dockbench_v1/`；生成、split 冻结、Stage-0 审计分别由 `scripts/generate_dockbench_v1.py`、`scripts/freeze_dockbench_v1_splits.py`、`scripts/validate_dockbench_stage0.py` 负责。
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

2.5 **Stage-2.5：`co_bcfd` 全指标 SOTA 冲击计划（进入 Stage-3 前强制 Gate）**
   - **总体原则**：在进入 Stage-3 多车实验之前，必须先把 `co_bcfd` 在 frozen 双车 benchmark 上尽可能收敛到“**高成功率 + 零碰撞 + 低耗时 + 强共同可解竞争性 + 机制证据充分**”的状态；不允许通过修改 `test/challenge` split、放松协议或引入 data leakage 来“刷分”。
   - **输入协议固定**：只允许使用 `data/dockbench_v1/` 的 `tuning / test / challenge`；所有阈值、超参、模式切换逻辑与新增模块的标定只能在 `tuning` 上完成，`test/challenge` 只做一次性评估与复现。
   - **短板清单（由当前 Stage-2 直接驱动）**：
     1. `FC-L3` 仍存在近场碰撞，说明 `Capture-Funnel + Safety Projection` 在极窄净空终端段尚未完全闭合；
     2. `EC-L1` 仍存在 `fov_loss_unrecovered`，说明 `Cooperative Staging + fallback` 还没有把“可对接”与“可持续可见”统一优化；
     3. `Common-Feasible` 子集上相对最优强传统基线仍缺少 `T_done / trajectory_cost` 优势，说明当前 Full model 在 easy/common case 上仍偏保守；
     4. `SC / FC` 上已有 response-regime 证据，但 `Belief Gate / Funnel Gate / Micro Maneuver` 的增益还没有全部形成更强的统计显著性。
   - **执行版路线（按 4 条主线组织）**：

   **主线 1：安全性主线 —— `FC-L3` 清零碰撞**
   - **模块改进清单**：
     1. `Terminal Sweep Guard`：把单步安全投影扩展为短时域 swept-volume 守护；
     2. `Contact Gate`：把接触带进入条件改成“距离 + 横向偏差 + 航向误差 + 扫掠净空证书”的联合门；
     3. `Backoff / Reverse-Arc Library`：补接触前微机动库，使失败优先转化为 backoff / replan 而不是碰撞；
     4. `Terminal Speed Clamp`：对 `FC-L2/L3` 单独校准 terminal speed envelope。
   - **优先修改文件**：`docking/dock_skill.py`、`docking/coop_docking.py`、必要时 `docking/bcfd.py`。
   - **对应实验脚本**：
     1. 单案复现：`scripts/run_p_minus1_stage1_docking.py --scenario-json data/dockbench_v1/scenes/DBv1-FC-L3-052.json ...`
     2. family 回归：`scripts/run_p_minus1_stage2_batch.py --dataset-root data/dockbench_v1 ...`，重点看 `FC` 与 `Overall`；
     3. 机制对照：Stage-2 报告中比较 `co_bcfd` vs `A-NoFunnelGate` vs `A-NoMicroManeuver`。
   - **预期验收指标**：
     1. `DBv1-FC-L3-052` 从 `collision` 变为 `success` 或至少变为非碰撞失败；
     2. frozen `test` split 上 `collision_rate = 0`；
     3. `FC` family success `>= 0.95`；
     4. `FC` 上 `A-NoFunnelGate` 的退化证据仍需保留，不能靠“抹平机制差异”换成功率。

   **主线 2：可见性主线 —— `EC-L1` 清零 `fov_loss_unrecovered`**
   - **模块改进清单**：
     1. `RCS`（Revisibility-Constrained Staging）：在 staging 目标评估中显式加入 `LOS persistence / reacquisition distance / visibility margin`；
     2. `Fallback Tube`：把视觉丢失后的回退目标从“继续全局逼近”改成“回到最近可重见 tube / pose”；
     3. `Visibility-Aware Replan Trigger`：当 `visual_lost_time`、`reacquisition margin`、`occlusion forecast` 超阈值时提前重规划；
     4. `Leader Cooperation Constraint`：Leader staging 必须优先选择“可持续可见”的姿态，而不是仅最短路径姿态。
   - **优先修改文件**：`docking/coop_docking.py`、`docking/dock_skill.py`，必要时 `docking/sensors.py`（仅可见性判据，不得改 benchmark 协议）。
   - **对应实验脚本**：
     1. 单案复现：`scripts/run_p_minus1_stage1_docking.py --scenario-json data/dockbench_v1/scenes/DBv1-EC-L1-059.json ...`
     2. family 回归：`scripts/run_p_minus1_stage2_batch.py --dataset-root data/dockbench_v1 ...`，重点看 `EC` 与 `SC`；
     3. 审计回归：`scripts/validate_dockbench_stage0.py --dataset-root data/dockbench_v1`，确保 `EC` tuning / representative 未被打穿。
   - **预期验收指标**：
     1. `DBv1-EC-L1-059` 从 `fov_loss_unrecovered` 变为 `success`；
     2. frozen `test` split 上 `fov_loss_unrecovered = 0`；
     3. `EC` family success `>= 0.95`；
     4. `visual_loss_mean / fallback_mean` 长尾显著缩短，且不能靠超长 leader relocation “换”成功率。

   **主线 3：效率主线 —— `CF` 上真正赢过强传统基线**
   - **模块改进清单**：
     1. `CGFL`（Certificate-Gated Fast Lane）：只在满足证书时提高 approach speed、放宽 contact gate、推迟 lock-assist 接管；
     2. `No-Unnecessary-Relocation Rule`：在无遮挡、高净空、低航向差场景中抑制无意义的 leader relocation；
     3. `Common-Case Predock Simplification`：减少 predock 路径上的冗余拐点和低速段；
     4. `Cost-Aware Mode Scheduling`：对 `CF` 使用效率优先的 mode scheduling，而非困难场景同款保守阈值。
   - **优先修改文件**：`docking/dock_skill.py`，必要时补 `docking/coop_docking.py` 的 common-case 评分项。
   - **对应实验脚本**：
     1. 代表场景：`scripts/run_p_minus1_stage1_suite.py --dataset-root data/dockbench_v1 ...`，重点看 `CF_L2`；
     2. 批量回归：`scripts/run_p_minus1_stage2_batch.py --dataset-root data/dockbench_v1 ...`；
     3. 可选 smoke：允许增加 `co-only` 回归脚本或 summary，但正式结论仍以 Stage-2 协议为准。
   - **预期验收指标**：
     1. `CF` / `Common-Feasible` 子集上达到“成功率非劣 + 平均 `T_done` 或平均 trajectory cost 至少改善 `5%`”；
     2. overall 平均 `T_done` 先压到 `<=17s`，继续冲 `<=15s`；
     3. fast lane 不得降低 `SC / FC / EC` 的 success，不得新增碰撞。

   **主线 4：证据主线 —— 从描述性优势到统计性优势**
   - **模块改进清单**：
     1. `Paired Statistics`：对 success / time / cost 做 paired bootstrap / paired CI；
     2. `Failure-Mode Stratification`：把失败按 `collision / fov_loss_unrecovered / geometric_deadlock / lock_condition_not_met` 分层统计；
     3. `Mechanism-Targeted Ablation`：把 `A-NoStage`、`A-NoFunnelGate`、`A-NoMicroManeuver` 分别绑定到 `EC / FC / SC+EC` 子集；
     4. `Belief-Gate Evidence Audit`：若 `A-NoBeliefGate` 仍无明显退化，必须新增 family 机制子集或明确下调该模块论断。
   - **优先修改文件/脚本**：以 `scripts/run_p_minus1_stage2_batch.py` 为主；必要时补充 Stage-2.5 统计脚本，但正式结果口径仍需回写到 Stage-2 报告格式。
   - **对应实验脚本**：
     1. `scripts/run_p_minus1_stage2_batch.py --dataset-root data/dockbench_v1 ...`；
     2. 必要时增加 `Stage-2.5 stats appendix` 脚本，但不得替代主报告。
   - **预期验收指标**：
     1. `EC` 扩展收益 CI 明确大于 `0`；
     2. `A-NoStage` 在 `EC`、`A-NoFunnelGate` 在 `FC`、`A-NoMicroManeuver` 在 `SC/EC` 至少一项关键指标上出现稳定退化；
     3. `Belief Gate` 若仍无显著性，必须在报告中明确降级为“当前证据不足”，并记录补 family 机制子集计划。

   - **统一实验矩阵（四条主线共用）**：
     1. `single-case repro`：只跑对应残余失败 / 代表场景，快速定位；
     2. `family regression`：Stage-2 报告中重点看被修改 family 的 success / collision / time / fail_reason；
     3. `full frozen test`：每一轮都必须跑完整 `test` split；
     4. `challenge pressure test`：每两轮至少跑一次 `challenge`，防止只在 `test` 上过拟合；
     5. `stage0 re-audit`：如果改动触及 staging / family 机制判据，必须重跑 `scripts/validate_dockbench_stage0.py`。
   - **脚本约束**：
     1. 允许增加 Stage-2.5 专用脚本，但正式结论文件必须继续产出到 Stage-2 兼容格式；
     2. 所有新增脚本都必须显式区分 `tuning / test / challenge`，并把 seed / manifest 固定写入输出；
     3. 不允许使用 `test` 上反复调参后的最好一次结果作为最终结论。
   - **里程碑顺序（执行版）**：
     1. `M1`：主线 1，先消灭碰撞；
     2. `M2`：主线 2，消灭 `fov_loss_unrecovered`；
     3. `M3`：主线 3，补 `CF` 竞争性；
     4. `M4`：主线 4，补统计显著性与机制证据；
     5. 只有在 `M1~M4` 全部满足 Stage-2.5 gate 后，才允许进入 Stage-3。
   - **回归要求**：每完成一轮改动，都必须对 frozen `test` split 全量重跑，且对 `challenge` 子集做单独压力评估；若某一改动提升单个 family 但伤害总体安全/成功率，则视为未通过，不得带入 Stage-3。
   - **Stage-3 前硬门槛**：未完成 Stage-2.5 的 base gate，不进入 Stage-3 多车实验。

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
- **Stage-2.5（Stage-3 前 base gate）**：
  1. frozen `test` split 上 overall 成功率建议 \(\ge 97.5\%\)，stretch goal 为 `48/48`；
  2. overall 碰撞率必须回到 `0`；
  3. `fov_loss_unrecovered` 在 `test` split 上必须压到 `0`；
  4. overall 平均 `T_done` 建议先压到 \(\le 17s\)，最终冲击任务书原门槛 \(\le 15s\)；
  5. `CF` / `Common-Feasible` 子集上，本方法相对最优强传统基线必须达到“成功率非劣 + 平均 `T_done` 或平均 trajectory cost 改善至少 `5%`”；stretch goal 为时间与代价双改善；
  6. `FC` 与 `EC` family 的成功率建议都收敛到 \(\ge 95\%\)，且其主要失败模式不再是 `collision / fov_loss_unrecovered` 这类不可接受失效；
  7. `challenge` 子集的 `co_bcfd` 成功率建议 \(\ge 50\%\)，stretch goal 为 \(\ge 2/3\)；
  8. 关键结论必须补充 paired bootstrap / paired CI：至少覆盖 `EC` 扩展收益、`CF` 竞争性、`A-NoStage` 与 `A-NoFunnelGate / A-NoMicroManeuver` 的 targeted ablation 结论。
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

更新（2026-03-09）：
0. `P-1.1B` 的 `LC (Lane-Constrained)` 新需求已经正式并入 `DockBench-v1` 的 frozen benchmark：
   - 数据集根目录：`data/dockbench_v1/`
   - 当前 frozen manifest 为 `90` 个 concrete scenes（`15 tuning / 60 test / 15 challenge`），`CF / SC / FC / EC / LC` 五类 family 各 `18` 个；
   - 生成脚本：`scripts/generate_dockbench_v1.py`
   - split / audit 工具：`scripts/freeze_dockbench_v1_splits.py`、`scripts/validate_dockbench_stage0.py`
   - 规范书：`DOCK_SCENARIO_STANDARD.md`
1. `P-1.1` 设计稿已持续同步到当前实现与实验：`DOCK.md`。
2. Stage-1 representative 验证已完成：
   - `CF_L2 = DBv1-CF-L2-008`
   - `SC_L2 = DBv1-SC-L2-026`
   - `FC_L2 = DBv1-FC-L2-044`
   - `EC_L2 = DBv1-EC-L2-062`
   - `LC_L2 = DBv1-LC-L2-080`
   - representative 报告：`artifacts/p_minus1_stage1_lane_refresh/P_MINUS1_STAGE1_SUITE_REPORT.md`
3. 围绕 `LC` 已完成一轮新的方法级设计：`LC-CRBS (Lane-Constrained Corridor Reciprocal Basin Shaping)`。
   - 设计书位置：`DOCK.md`
   - 代码落点：`docking/lc_corridor.py`、`docking/coop_docking.py`、`docking/dock_skill.py`、`docking/planner.py`、`docking/controllers.py`
   - 单案复现：`artifacts/lc_round3_single/`
- challenge 说明：当前主指标对应的 `test=60` 已完整回收；challenge 汇总当前来自 `14/15` 场景，缺失样本为 `DBv1-LC-L1-078`。
4. 基于 `LC-CRBS` 的 frozen `Stage-2`（main gate 仍按 `test` split）已重新汇总：
   - 报告：`artifacts/dockbench_stage2_lc_crbs_round1_manual/P_MINUS1_STAGE2_REPORT.md`
   - 数据：`artifacts/dockbench_stage2_lc_crbs_round1_manual/p_minus1_stage2_results.json`
   - overall：`success_rate = 0.7000 (42/60)`，`collision_rate = 0.0167`，成功样本 `avg_T_done = 16.63s`；
   - family success：`CF / SC / FC / EC / LC = 1.000 / 0.6667 / 0.9167 / 0.9167 / 0.0000`；
   - `strong_cf_nonzero = True`，`p13_extension_gain_ge_15pt = True`，但 `co_success_ge_095 / co_collision_zero / co_avg_time_le_15` 仍全部为 `False`。
5. 与引入 `LC` 后但未修复前的 `Stage-2` 相比：
   - overall success 未提升（仍为 `0.7000`）；
   - 但 overall `collision_rate` 已从 `0.1833` 降到 `0.0167`；
   - `LC` family 已从 `collision` 主导转为 `lc_geometric_deadlock` 主导，说明新方法首先修复了**安全性**，但尚未完成**可完成性翻转**。
6. 当前已满足/已成立的条款：
   - `P-1.1B` 的正式规范、schema、family × difficulty × split 冻结已完成；
   - `Stage-1` 的 frozen representative protocol 已完成并可复现；
   - `Common-Feasible` 子集上强传统基线稳定非零成功仍成立；
   - `Extension-Critical` 子集上 `co_bcfd` 对最优强基线的扩展收益门槛仍成立；
   - 新增 `LC-CRBS` 模块已有清晰消融证据：`A_no_corridor_reciprocity` 不改变 success，但显著恶化 collision。
7. 当前未完成/需继续迭代的条款：
   - `Stage-0` 的 full audit 尚未在新 benchmark 上重新闭环；
   - `Stage-2` 的整体成功率与平均时间门槛仍未满足；
   - `LC` family 的 success 仍为 `0.0000`，当前的主要失败模式已收敛为 **corridor basin 未形成导致的 deadlock**；
   - 因此，下一轮的唯一主攻方向应是：把 `LC-CRBS` 从“collision suppression”推进到“corridor-compatible basin completion”。
   - 本轮还尝试了更激进的 `bidir basin descent` 原型（单案见 `artifacts/lc_round4_single/`），但其在 `LC-L1/L2` 上重新出现 collision，故不保留到主线。
   - 还尝试了 `closed-loop anchor rollout + basin-ready search` 原型：离线诊断可找到局部可行 anchor，但在真实 Stage-1 接线下未能稳定复现，部分场景直接退回 generic collision，因此也不保留到主线。
8. `LC` 的最新 round-3 迭代（`HybridMotionPrimitiveLibrary + \sigma_C + semantic reference`）已实现并完成当前子集验证：
   - 代码：`docking/lc_corridor.py`、`docking/coop_docking.py`、`docking/dock_skill.py`、`docking/p_minus1_baselines.py`；
   - 单案：`DBv1-LC-L1-074` 仍为 `lc_geometric_deadlock@16.0s`，`DBv1-LC-L2-081` 仍为 `lc_geometric_deadlock@24.0s`，但后者已经进入 `DOCKING:FUNNEL_ALIGN`，且保持 `collision=False`；
   - LC targeted regression：见 `artifacts/lc_target_regress_round5_ablation/lc_target_regress_ablation_summary.json`，当前 `co_bcfd` 为 `0/12 success, 0/12 collision, 11 deadlock + 1 timeout`；
   - 对应消融 `A_no_lc_hybrid_search` 同样是 `0/12 success`，但失败结构更差：`2 collision + 6 timeout + 4 deadlock`。
9. 因此，当前 round-3 的结论是：
   - 新设计**改善了安全性与 phase completion**，但尚未达到 `LC` 的 basin completion gate；
   - 在严格 protocol 下，由于 `DBv1-LC-L1-074 / DBv1-LC-L2-081` 的单案 gate 仍未过，本轮**不升级为新的 frozen full Stage-2 主报告**；
   - 下一轮必须继续专攻 `LC` 的 terminal handoff / final capture closure，而不是再回到纯安全修补。
10. round-4 诊断已完成，诊断报告见 `artifacts/lc_terminal_capture_round4/LC_TERMINAL_CAPTURE_DIAGNOSIS.md`：
   - `DBv1-LC-L1-074`：actual leader pose 诱导的 dynamic predock 直接 collision，说明当前 `sigma_C` 对 leader execution closure 过于乐观；
   - `DBv1-LC-L2-081`：曾进入 `DOCKING:FUNNEL_ALIGN` 的 terminal-shortfall 运行中，actual dynamic predock 同样 collision，因此 follower oscillation 的根因是 release 时 basin 已失效；
   - `DBv1-LC-L2-081` 的离线 witness 表明：两步 leader micro-reshape 可以把 `sigma_C^{exec}` 恢复到非零可行水平，说明后续正确方向应是 **execution-time basin certificate + leader terminal reshaping**。
11. 本轮已把上述诊断反映回设计书（`DOCK.md` 的 `5.3C` 与 `10.0B`），并完成了 `sigma_C^{exec}` / leader terminal reshaping 原型接线；当前实现状态见 `artifacts/lc_terminal_capture_round4/ROUND4_IMPLEMENTATION_STATUS.md`。但由于当前代码原型仍未通过关键单案 gate，因此这条线**仍属未完成的 round-4 研究分支**，不能回写为已达成的 Stage-2 结果。
12. round-5 已在 round-4 诊断基础上进一步落地 execution-time release gate 与 leader reshaping 闭环；当前状态汇总见 `artifacts/lc_terminal_capture_round4/ROUND5_EXECGATE_STATUS.md`。
   - 当前最好的 retained 单案为 `DBv1-LC-L2-081 @ artifacts/lc_round5_execgate3_L2_081/p_minus1_stage1_results_seed20670585.json`，结果为 `collision=False`、`failure=lc_geometric_deadlock`、`final_pos_error=0.831m`、`final_yaw_error=8.36deg`；
   - 更激进的 boost 版本虽然更接近 closure，但重新引入了 collision，见 `artifacts/lc_round5_execgate4_L2_081/p_minus1_stage1_results_seed20670585.json`，因此被拒绝；
   - 由于关键单案 gate 仍未通过，`LC_TARGET_REGRESSION` 仍未升级重跑为新的 retained Stage-2 结果。
13. round-6 已进一步把 `LEADER_SETTLE` 改写成主动的 certificate-ascent 本地锚点搜索；状态见 `artifacts/lc_terminal_capture_round4/ROUND6_CERT_ASCENT_STATUS.md`。
   - `DBv1-LC-L1-074`：`artifacts/lc_round6_certascent_L1_074/p_minus1_stage1_results_seed20660454.json`，结果仍为 `collision=False + lc_geometric_deadlock`；
   - `DBv1-LC-L2-081`：`artifacts/lc_round6_certascent_L2_081/p_minus1_stage1_results_seed20670585.json`，结果仍为 `collision=False + lc_geometric_deadlock`；
   - 两例在 `LEADER_SETTLE` 阶段都停在 `sigma_C^{exec}=0 / robust=0`，说明当前真正的剩余瓶颈已进一步收敛为：**`LEADER_SHAPE` 结束位姿本身仍落在 local viable basin 之外，导致局部 certificate-ascent search 无可用上升方向。**
14. 因此，当前主线判断更新为：
   - `sigma_C^{exec}` 与主动 certificate-ascent settlement 的方向是成立的；
   - 但仅靠 `LEADER_SETTLE` 的局部搜索仍不足以翻转 `LC`，下一轮若继续，应把 `LEADER_SHAPE` 的终态生成也纳入 execution-valid basin 的目标，而不是只在 settle 末端补局部搜索。
15. round-7 已按上述判断把 execution-valid basin 目标前置到 `LEADER_SHAPE` 规划层；状态见 `artifacts/lc_terminal_capture_round4/ROUND7_EXEC_SHAPE_STATUS.md`。
   - `DBv1-LC-L1-074`: `artifacts/lc_round7_execshape_L1_074/p_minus1_stage1_results_seed20660454.json`，当前已进入 docking，但重新出现 `collision_obstacle`；
   - `DBv1-LC-L2-081`: `artifacts/lc_round7_execshape_L2_081/p_minus1_stage1_results_seed20670585.json`，当前已进入 `DOCKING:FUNNEL_ALIGN / LOCK_ASSIST`，但仍以 `collision_obstacle` 失败；
   - 因此，前置 execution-valid basin 目标是正确的结构性修改，但当前 anchor 选择仍不够鲁棒，尚未满足“通过关键单案 gate 后再跑 LC 全量”的协议要求。
16. round-8 已进一步把“Follower 预测点到 contact 区间的扫掠净空”写成终端安全证书，并将其与 execution certificate 联合并入 `LEADER_SHAPE` 的优化目标；状态见 `artifacts/lc_terminal_capture_round4/ROUND8_TERMINAL_TUBE_STATUS.md`。
   - `DBv1-LC-L1-074`: `artifacts/lc_round8_terminalsafe_L1_074/p_minus1_stage1_results_seed20660454.json`，当前仍为 `collision=False + lc_geometric_deadlock`；当前候选集内尚未找到 terminal-safe anchor。
   - `DBv1-LC-L2-081`: 在 `artifacts/lc_round8_terminalsafe2_L2_081/p_minus1_stage1_results_seed20670585.json` 中，terminal-safe anchor 已被正确选出，但 leader 运行时未能到达该 anchor；在修正 leader signed path execution 的中间分支 `artifacts/lc_round8_terminalsafe4_L2_081/p_minus1_stage1_results_seed20670585.json` 中，该单案已出现 `success=True + collision=False` 的局部突破。
   - 因此，本轮新的剩余瓶颈已进一步收敛为：**terminal-safe anchor 的评分已经开始起作用，但双关键单案尚未同时翻转，尤其 `DBv1-LC-L1-074` 仍缺少可执行的 terminal-safe basin。**
17. round-9 已加入定向 `escape-shape` 搜索与 Any-time 风格的预算框架；状态见 `artifacts/lc_terminal_capture_round4/ROUND9_ESCAPE_BUDGET_STATUS.md`。
   - `DBv1-LC-L2-081`：当前代码快照上仍保持成功，见 `artifacts/lc_current_check_L2/p_minus1_stage1_results_seed20670585.json`；
   - `DBv1-LC-L1-074`：当前仍未翻转成功，保留结果仍为 `artifacts/lc_round8_terminalsafe_L1_074/p_minus1_stage1_results_seed20660454.json`；
   - 规划耗时方面，`L1` 的 targeted escape search 已从约 `5.7s` 降到约 `0.6s`，但仍未满足 `100ms` 硬预算，因此实时性问题只算部分缓解、未完全解决。
18. round-10 已进一步把 `L1` 的 small-gap escape 分支压缩为单模板、近似评估与硬预算路径；状态见 `artifacts/lc_terminal_capture_round4/ROUND10_ANYTIME_PERF_STATUS.md`。
   - `DBv1-LC-L1-074`: `artifacts/lc_round9_perfcheck_L1_074/p_minus1_stage1_results_seed20660454.json`，当前仍为 `collision=False + lc_geometric_deadlock`，但 `planner_elapsed_s ≈ 0.0169s`，已满足 `<100ms` 的实时性门槛；
   - `DBv1-LC-L2-081`: `artifacts/lc_round9_perfcheck_L2_081/p_minus1_stage1_results_seed20670585.json`，当前继续保持 `success=True + collision=False`；
   - 因此，本轮的阶段性结论是：**实时性问题已经解决到可接受范围，剩余瓶颈纯粹收敛为 `L1` 的覆盖度 / execution-closure 问题。**
19. round-11 已把 `L1` / `L2` 的 terminal closure 重写为 `\sigma_M`（Terminal-Manifold Closure Certificate）+ `FOLLOWER_TERMINAL_CLOSURE` runtime 近场闭环；状态见 `artifacts/lc_round11_terminal_manifold/ROUND11_TERMINAL_MANIFOLD_STATUS.md`。
   - `DBv1-LC-L1-074`: `artifacts/lc_round11o_lockhold_L1_074/p_minus1_stage1_results_seed20660454.json`，当前已不再是 `lc_geometric_deadlock`，而是进入 terminal-manifold 后以 `collision_obstacle` 失败；最低净空为 `0.0990m`，说明 remaining gap 已收敛到 generic `LOCK_ASSIST` 的 tight-corridor safety shortfall；
   - `DBv1-LC-L2-081`: `artifacts/lc_round11o_lockhold_L2_081/p_minus1_stage1_results_seed20670585.json`，当前继续保持 `success=True + collision=False`；
   - 按严格 protocol，由于 `L1` 单案 gate 仍未通过，本轮**不升级**为新的 `LC_TARGET_REGRESSION` 或 frozen full `Stage-2` retained 结果；round-11 只保留“deadlock 已消除、残差已前移到 final lock-assist safety mismatch”这一负结论。
20. round-12 已把 `LOCK_ASSIST` 改写为 certificate-consistent `LC_LOCK_HOLD`，并在 runtime evaluator 中加入 terminal-hold invariant projection；状态见 `artifacts/lc_round12_lock_hold/ROUND12_LOCK_HOLD_STATUS.md`。
   - `DBv1-LC-L1-074`: `artifacts/lc_round12g_gate_L1_074/p_minus1_stage1_results_seed20660454.json`，当前已达到 `success=True + collision=False + min_clearance=0.1031m`；
   - `DBv1-LC-L2-081`: `artifacts/lc_round12g_gate_L2_081/p_minus1_stage1_results_seed20670585.json`，当前继续保持 `success=True + collision=False + min_clearance=0.1486m`；
   - `LC` targeted regression：`artifacts/lc_round12_target_regression/LC_TARGET_REGRESSION.json`，当前达到 `success_rate=0.167 (2/12)`，但 `collision_rate=0.083 (1/12)`，残余失败以 `lc_geometric_deadlock` 为主；
   - 因此，round-12 的 retained 结论是：**关键单案 gate 已通过，L1 的 final lock-assist safety mismatch 已被闭合，但 broader LC family 仍未达到 Stage-2 通过线**；本轮不升级为 frozen full `Stage-2` 主报告。
21. round-13 已把 `FOLLOWER_READY / Release` 重写为 certificate-consistent handoff gate（短时域 downstream docking safety + witness gain）；状态见 `artifacts/lc_round13_release_gate/ROUND13_RELEASE_GATE_STATUS.md`。
   - `DBv1-LC-L2-082`: `artifacts/lc_round13d_handoff_L2_082/p_minus1_stage1_results_seed20670716.json`，当前已由 `collision_obstacle` 翻转为 `lc_geometric_deadlock`；
   - `DBv1-LC-L1-074`: `artifacts/lc_round13c_handoff_L1_074/p_minus1_stage1_results_seed20660454.json`，保持 `success=True + collision=False`；
   - `DBv1-LC-L2-081`: `artifacts/lc_round13d_handoff_L2_081/p_minus1_stage1_results_seed20670585.json`，保持 `success=True + collision=False`；
   - 最新 `LC` targeted regression：`artifacts/lc_round13_target_regression/LC_TARGET_REGRESSION.json`，当前达到 `success_rate=0.250 (3/12)`、`collision_rate=0.000 (0/12)`；
   - 因此，round-13 的 retained 结论是：**LC family 的安全红线已恢复为 0 collision，剩余问题已经纯化为 9 个 `lc_geometric_deadlock` 的 basin coverage**；本轮仍不升级为 frozen full `Stage-2` 主报告。

7. Stage-2.5 历史完成情况（pre-LC 4-family split，仅供参考）：
7. Stage-2.5 历史完成情况（pre-LC 4-family split，仅供参考）：
   - **范围说明**：以下 `48/48`、`collision=0` 与 `avg_T_done` 结论均基于引入 `LC` 之前的 `4-family / 48-test` frozen split，当前尚未在 `5-family / 60-test` benchmark 上重跑。
   - **独立分支已落地**：`TVT / VPCR` 已从“散落在主逻辑里的阈值修补”重构为真正独立的 mode / subskill 分支；代码位于 `docking/stage25_subskills.py`，运行时接线在 `docking/dock_skill.py`，冻结回归脚本为 `scripts/run_stage25_branch_eval.py`。
   - **当前主线口径**：从设计/论文叙事上，`co_bcfd` 不再被视为“`CGFL + TVT + VPCR + PTCC` 四个补丁模块”的叠加，而被统一表述为：`belief-consistent capture-funnel + 证书场约束策略`。其中 `CGFL + PTCC` 共同收敛到 `进展证书`，`TVT + terminal capture boost + CTP` 共同收敛到 `终端证书`，`VPCR` 收敛到 `可见性证书`；代码实现上仍分别落在 `docking/dock_skill.py`、`docking/stage25_subskills.py`、`docking/time_compression.py`。
   - **主线 frozen `test` 回归结果**：见 `artifacts/stage25_round10_timecert_test/co_bcfd_summary.json`；overall 成功率保持 `1.0000 (48/48)`，collision rate 维持 `0.0000`，overall 平均完成时间进一步压到 `19.64s`。
   - **failure-mode 分层统计**：见 `artifacts/stage25_round10_timecert_test/co_bcfd_failure_mode_summary.json`；当前主线在 frozen `test` split 上已无失败样本。
   - **TVT 主线进展**：`DBv1-FC-L3-052` 单案已被稳定修复；对应对比见 `artifacts/tmp_stage25_singlecases/fc052_compare/p_minus1_stage1_results_seed20480716.json`。在 frozen `test` 上，`FC` family success 已从 `0.9167` 提升到 `1.0000`，且 `collision_rate` 从 `0.0833` 下降到 `0.0000`。
   - **VPCR 主线进展**：`VPCR` 已进一步演化为 response-reachable pair-capture 设计，并在本轮补上了 `terminal capture boost + certified terminal projection (CTP)`。对应代码在 `docking/stage25_subskills.py` 与 `scripts/run_p_minus1_stage1_docking.py`。在 `DBv1-EC-L1-059` 的 benchmark-aligned 单案中，主线 `co_bcfd` 已被稳定翻转为成功；见 `artifacts/tmp_stage25_singlecases/ec059_promoted_mainline_32/p_minus1_stage1_results_seed20560847.json`。
   - **本轮 `contact-yaw closure` 结果**：通过 `PAIR → CAPTURE → terminal basin handoff → LOCK_ASSIST + CTP`，`EC-L1-059` 的 terminal pose closure 已在 `32s` budget 下闭合；对应中间诊断链路可见 `artifacts/tmp_stage25_singlecases/ec059_response_contact_vpcr_v2/p_minus1_stage1_results_seed20560847.json`、`artifacts/tmp_stage25_singlecases/ec059_lockassist_morefast_32/p_minus1_stage1_results_seed20560847.json`、`artifacts/tmp_stage25_singlecases/ec059_ctp_speed_32/p_minus1_stage1_results_seed20560847.json`。
   - **VPCR 全量回归结论**：
     1. 本轮 frozen `test` 结果见 `artifacts/stage25_round6_mainline_test/co_bcfd_summary.json`，主线 `co_bcfd` 已达到 `1.0000 / 0.0000 / 20.01s`；
     2. 对应 failure-mode 分层见 `artifacts/stage25_round6_mainline_test/co_bcfd_failure_mode_summary.json`，当前已无失败样本；
     3. 与旧主线相比，共同成功子集的平均完成时间保持 `19.7553s` 不变，因此新增 `VPCR` 主线并未伤害原有成功样本的时间表现；overall 平均时间上升仅因原先失败的 `EC-L1-059` 以 `31.75s` 被纳入成功统计。
   - **avg_T_done 冲击尝试结论**：本轮继续专攻“**不伤 `48/48` 的全局时间压缩**”。较激进的 corridor certificate / progress compression 原型在代表场景上可显著降时，例如 `DBv1-SC-L2-025 = 18.55s`、`DBv1-EC-L1-059 = 28.15s`、`DBv1-EC-L2-063 = 31.00s`；对应证据见 `artifacts/tmp_stage25_singlecases/sc025_timeenv/p_minus1_stage1_results_seed20370323.json`、`artifacts/tmp_stage25_singlecases/ec059_timeenv_32/p_minus1_stage1_results_seed20560847.json`、`artifacts/tmp_stage25_singlecases/ec063_timeenv/p_minus1_stage1_results_seed20570585.json`。
   - 这些激进原型无法同时保住全量 frozen `test` 的 `48/48`：例如 `DBv1-EC-L3-071` 会退化为 `timeout/fov_loss_unrecovered`，见 `artifacts/tmp_stage25_singlecases/ec071_timeenv/p_minus1_stage1_results_seed20580847.json`；对应 frozen `test` 的最好不安全版本可把均时压到 `17.37s`，但会引入新的回退，见 `artifacts/stage25_round3_test/co_bcfd_summary.json` 与 `artifacts/stage25_round3c_test/co_bcfd_summary.json`。
   - 本轮最终合入主线的是更保守的 `PTCC = SC-blocked-mid + EC-short` 证书，代码位于 `docking/time_compression.py`。其 frozen `test` 结果见 `artifacts/stage25_round10_timecert_test/co_bcfd_summary.json`：在保持 `48/48 + collision=0` 不变的同时，把 overall `avg_T_done` 从 `20.01s` 压到 `19.64s`，其中 `SC` family 均时从 `22.26s` 压到 `22.12s`，`EC` family 均时从 `30.13s` 压到 `29.47s`。
7. 当前仍未完全收敛的项：
   - overall `avg_T_done` 仍高于 `15s`，尤其 `SC / EC` family 耗时偏大；
   - `Common-Feasible` 子集上，`co_bcfd` 已通过 `CGFL` + `TVT/VPCR` 清除了 `FC / EC` 残余失败，但相对最优强传统基线的 `trajectory_cost >= 5%` 优势仍未形成；
8. 结论：
   - 以“为上层策略层提供可复现、可批量、可 family 分解的双车 docking 基准”为目标，当前 `P-1.1B + P-1.2 Stage-0/1/2` 仍保持可用状态；
   - 以“Stage-3 前先把 `co_bcfd` 冲到全指标更优”为目标，当前已完成 **第四轮可控增益落地（`CGFL` → `gated TVT/VPCR + PTCC`）**，并把 frozen `test` split 的残余 hard case 压到 `0` 个；
   - 在进入更高置信度的 Stage-3 多车扩展前，当前 Stage-2.5 的主剩余问题已收敛为：如何在不伤 `48/48` 成功率的前提下，继续把 `avg_T_done` 从 `19.64s` 压向 `15s`。

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
