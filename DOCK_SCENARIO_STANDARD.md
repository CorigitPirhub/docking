# DockBench 场景标准规范（P-1 前置基准）

> 适用范围：`CORE_REQUIREMENTS_TASKBOOK.md` 的 P-1 两车 docking 基础支撑验证。  
> 角色定位：该规范先冻结**场景构建标准 / 数据集格式 / split / 质量评估器**，再由 P-1.2 Stage-1/Stage-2 使用并验证。  
> 与 `SCENARIO_CLASSIFICATION.md` 的关系：后者服务于 P2-P6 的多车重构/通行任务；本文档服务于 P-1 的**两车 docking 闭环技能基准**。

---

## 0. 为什么必须先标准化场景

如果 P-1 继续沿用“随机障碍 + 人工挑 seed”的做法，会出现 4 个问题：
1. **类别不清**：同样都叫“困难场景”，可能一个是视觉切换难，一个是捕获域难，一个是 leader relocation 难，实验结论会混淆。
2. **对比失真**：若不冻结 `Common-Feasible` 与 `Extension-Critical` 的比例，可能出现“强基线全灭”或“只有 easy case”两种极端。
3. **数据泄漏**：baseline / ablation / full model 若在同一批 seed 上反复调参，P-1.2 的结论不可信。
4. **难以论文化**：没有标准化场景 grammar、固定 split、质量评估器，就很难把实验体系本身上升为可复用 benchmark。

因此，P-1 需要先冻结一个 docking 专用 benchmark：**DockBench-v1**。

---

## 1. 文献与标准依据（用于设计原则）

本文档采用“抽象场景 → 逻辑场景 → 具体场景 → 闭环审计”的结构，主要参考以下一手资料：
- ASAM OpenSCENARIO：强调场景应区分抽象/逻辑/具体层级，并给出可交换的场景描述标准。  
  Link: https://www.asam.net/standards/detail/openscenario/
- Scenic：强调用**可组合的生成语法**描述场景，而不是手写单个 seed。  
  Link: https://arxiv.org/abs/2306.14847
- VerifAI：强调场景生成不仅要能采样，还要服务于**覆盖、应力测试与 falsification**。  
  Link: https://arxiv.org/abs/1905.06950
- CommonRoad：强调 benchmark 需要固定实例集、可复现实例 ID 与统一评测协议。  
  Link: https://commonroad.in.tum.de/
- nuPlan：强调闭环规划 benchmark 应采用固定 split、统一指标和 scenario-level evaluation。  
  Link: https://www.nuplan.org/
- ComOpT：强调场景测试需要同时关注**组合覆盖**与**场景质量/代表性**。  
  Link: https://arxiv.org/abs/2108.08253
- TSC2CARLA：强调场景集不应只看数量，还要显式度量**多样性、均匀性、边界性与典型性**。  
  Link: https://arxiv.org/abs/2307.15084

### 1.1 从这些工作提炼出的 5 条硬原则

1. **层级化**：必须区分抽象家族、逻辑参数范围、具体实例与执行审计结果。  
2. **机制对齐**：场景类别必须对齐 P-1 真实模块，而不是只按“难/易”拍脑袋分。  
3. **固定 split**：必须冻结 `tuning / test / challenge`，禁止在 test 上来回调参。  
4. **质量评估器**：场景必须经过 validity + label fidelity + diversity + balance 的综合筛选。  
5. **执行闭环验证**：场景标签不应只靠几何推断，还要经由基础支撑的短程审计确认其确实打到了对应机制。

---

## 2. DockBench-v1 的总体目标

DockBench-v1 不是“随便凑若干障碍图”，而是一个**机制完备（mechanism-complete）**的两车 docking benchmark。它要同时覆盖 4 类能力：
1. **共同可解竞争性**：至少一类场景中强传统基线具备真实竞争力；
2. **感知切换与回退**：视觉噪声、遮挡、FOV 丢失、GNSS/视觉融合会真正影响结果；
3. **捕获域与近场微机动**：Ackermann 近场不能原地修姿，必须验证 funnel / backoff / micro-maneuver；
4. **协同 staging / leader relocation**：前车不应固定等待，必须验证“主动调整位姿”带来的可解域扩展。

因此，DockBench-v1 的类别必须围绕上述 4 种机制来定义。

---

## 3. 统一场景描述：抽象层 / 逻辑层 / 具体层 / 审计层

### 3.1 抽象层（Abstract Family）

抽象层只描述“这个场景想打哪种机制”，不放具体坐标。DockBench-v1 固定 4 个 family：
- `CF`：Common-Feasible（共同可解）
- `SC`：Switching-Critical（切换/遮挡关键）
- `FC`：Funnel-Critical（捕获域/近场关键）
- `EC`：Extension-Critical（协同 relocation 扩展关键）

### 3.2 逻辑层（Logical Parameters）

逻辑层描述参数范围，不给出唯一实例。必须至少包含：
- 车辆相关：
  - 初始 hitch 距离 `d0_m`
  - 初始航向差 `heading_diff_deg`
  - follower 相对 leader 的初始方位角 `bearing_deg`
  - 出生点到边界最小距离 `boundary_margin_m`
- 环境相关：
  - 全局障碍面积占比 `clutter_ratio`
  - 视线遮挡强度 `occlusion_index`
  - 最短路绕行系数 `detour_factor`
  - 不同同伦通路数量 `homotopy_count`
  - leader 周围 docking 区局部净空 `dock_zone_clearance_m`
  - leader 初始位姿到最佳 staging pose 的位移 `staging_shift_required_m`
  - 捕获域约束指数 `capture_constraint_index`

### 3.3 具体层（Concrete Instance）

具体层才给出：地图大小、障碍物坐标/尺寸/角色、leader/follower 初始位姿、固定 seed、推荐超时上限。

### 3.4 审计层（Audit Record）

每个场景在冻结入库前，必须附带审计结果：
- validity 是否通过；
- 标签是否与几何/执行一致；
- 是否属于某个固定 split；
- 是否满足 family 的 admission 条件；
- 与同 cell 已入库场景的最小描述子距离；
- 场景质量评分。

---

## 4. 不应只按“障碍数量”分场景，而应按“障碍角色”分

对于 docking，障碍不是越多越难，也不是越大越难。更关键的是障碍**相对于任务几何关系的位置与作用**。  
因此 DockBench-v1 采用**role-conditioned obstacle grammar**：

### 4.1 障碍角色（Obstacle Roles）

每个障碍物除几何属性外，还应打上 `role` 标签：
- `screen`：主要作用是阻断 follower 对 leader 尾部的 LOS / FOV；
- `channel`：主要作用是压缩 follower 到 predock/staging 区的通路宽度，提高绕行系数；
- `dock_zone`：位于 leader 周围 `r <= 2.0m` 邻域，直接约束 predock / capture maneuver；
- `background`：仅增加全局 clutter，不直接决定关键机制；
- `hybrid`：同时承担上述两种以上角色（如既遮挡又形成 pocket）。

### 4.2 环境不再用单一“密集/稀疏”描述，而是用 3 个任务相关子区

将地图拆成 3 个与任务对齐的分析区：
1. **Corridor 区**：follower 到 best predock/staging pose 的主要通路；
2. **Dock Zone 区**：leader 及其候选 staging pose 周围的近场区域；
3. **Background 区**：除上述区域外的剩余自由空间。

每个子区分别统计：
- 障碍面积占比；
- 最小净空；
- LOS 阻断率；
- 通路数 / 同伦数；
- 曲率与 maneuverability。

### 4.3 这比“障碍数量”更合理的原因

同样是 6 个障碍：
- 若 4 个都在 background，可能仍是 easy case；
- 若 1 个大障碍恰好横在 LOS 锥前方，就会成为典型 `SC`；
- 若 2 个小障碍卡在 leader 两侧，会形成 `FC`；
- 若 leader 初始位姿在 pocket 中且后方被 screen 封住，则会形成 `EC`。

因此，DockBench-v1 的类别划分依据是**障碍拓扑角色 + 车辆相对几何 + 基础支撑可达性分析**，而不是单一 obstacle count。

---

## 5. 车辆相关因子：必须固定进入 descriptor space

### 5.1 核心车辆因子

1. **初始距离**  
   \[
   d_0 = \|p_{fh,F}(0) - p_{rh,L}(0)\|_2
   \]
   取值范围固定为 `4.5m ~ 12.0m`，避免 trivial near-contact 与超远无意义样本。

2. **初始航向差**  
   \[
   \Delta \psi_0 = |\mathrm{wrap}(\psi_F(0) - \psi_L(0))|
   \]
   分层建议：
   - `H1`: `0° ~ 10°`
   - `H2`: `10° ~ 30°`
   - `H3`: `30° ~ 60°`

3. **初始相对方位角**（比单看航向差更关键）  
   记 leader 朝向单位向量为 `h_L=[cos ψ_L, sin ψ_L]`，leader rear-hitch 到 follower front-hitch 的向量为 `r_FL`，则
   \[
   \beta_0 = \angle(r_{FL}, -h_L)
   \]
   分层建议：
   - `B1`: rear-centered（`0° ~ 15°`）
   - `B2`: off-axis rear（`15° ~ 40°`）
   - `B3`: side-rear（`40° ~ 75°`）

### 5.2 为什么必须把它们固定进 benchmark

- `d0_m` 主要决定 intercept 与 planning horizon；
- `heading_diff_deg` 主要影响近场姿态收敛；
- `bearing_deg` 决定 follower 是否处在“天然后方”还是需要显著绕行；
- 三者共同决定单车规划难度与 capture 难度，不能在数据集里隐式漂移。

---

## 6. 4 类 family：DockBench-v1 的正式场景家族

## 6.1 `CF` — Common-Feasible（共同可解）

**定义**：不依赖 leader relocation，至少 1 个强传统基线在 admission 阶段应有稳定非零成功率。  
**目标**：验证“我们的方案不是只会做极端难例”，而是在 common cases 上也具备竞争力。  
**几何/环境条件建议**：
- `staging_shift_required_m < 0.35`
- `occlusion_index < 0.20`
- `detour_factor < 1.10`
- `dock_zone_clearance_m >= 0.80`
- `capture_constraint_index < 0.40`

## 6.2 `SC` — Switching-Critical（切换/遮挡关键）

**定义**：LOS / FOV / 视觉可用窗口会显著影响闭环，但 leader relocation 不是主要难点。  
**目标**：验证 belief-consistent fusion、hard switch 对比、视觉丢失回退。  
**条件建议**：
- `0.20 <= occlusion_index < 0.75`
- `staging_shift_required_m < 0.35`
- `dock_zone_clearance_m >= 0.65`
- `capture_constraint_index < 0.45`
- `detour_factor < 1.15`

## 6.3 `FC` — Funnel-Critical（捕获域/近场关键）

**定义**：leader 初始位姿本身可 dock，但进入接触带前的捕获域很窄，若无 funnel/backoff/micro-maneuver 容易近场死锁。  
**目标**：验证 capture-funnel、近场 backoff、micro-maneuver。  
**条件建议**：
- `staging_shift_required_m < 0.35`
- `occlusion_index < 0.35`
- `0.35 <= dock_zone_clearance_m < 0.80`
- `capture_constraint_index >= 0.55`

## 6.4 `EC` — Extension-Critical（协同 relocation 扩展关键）

**定义**：leader 初始位姿不可直接 dock 或代价极高，只有通过 cooperative staging 才能进入可解域。  
**目标**：验证“leader 主动调整位姿”不是工程修补，而是真正扩展了可解域。  
**条件建议**：
- `staging_shift_required_m >= 0.80` 或 `initial_dockability = false`
- `detour_factor >= 1.15` 或 `occlusion_index >= 0.35`
- 通常伴随 `screen`/`dock_zone`/`hybrid` 障碍角色

---

## 7. 难度分级：L1 / L2 / L3

为避免 family 内部难度失控，DockBench-v1 再引入统一难度指标：

\[
D(s)=0.15\tilde d_0 + 0.15\widetilde{|\Delta \psi_0|} + 0.20\tilde O_{occ} + 0.20\tilde F_{detour} + 0.15\tilde R_{stage} + 0.15\tilde C_{capture}
\]

其中所有项都归一化到 `[0,1]`。建议分级：
- `L1`：`0.00 ~ 0.33`
- `L2`：`0.33 ~ 0.66`
- `L3`：`0.66 ~ 1.00`

### 7.1 为什么要 family × difficulty 双索引

因为：
- `CF-L3` 与 `EC-L1` 的难点机理完全不同；
- 只按 family 分会掩盖 easy/hard 波动；
- 只按 difficulty 分又会混淆模块归因。

因此 DockBench-v1 的最小覆盖单元固定为：
**`4 families × 3 difficulty levels = 12 scenario cells`**。

---

## 8. 固定数据集规模、比例与 split

## 8.1 官方推荐规模：`DockBench-v1 = 72` 个 concrete scenes

采用**cell-balanced** 设计：每个 cell 固定 `6` 个场景：
- `1` 个 `tuning`
- `4` 个 `test`
- `1` 个 `challenge`

因此：
- `12 cells × 6 scenes = 72 scenes`
- split 比例：
  - `tuning = 12`（`16.7%`）
  - `test = 48`（`66.7%`）
  - `challenge = 12`（`16.7%`）
- family 比例：四类各 `25%`
- difficulty 比例：三档各 `33.3%`

## 8.2 为什么选 72 而不是更少/更多

- 少于 `48` 个 test scenes：paired comparison 方差过大，很难支撑 P-1.3 的竞争性判断；
- 多于 `100` 个 test scenes：在当前 foundation 闭环 + 多 baseline + 多 ablation 的算力预算下，P-1.2 批量运行成本过高；
- `72 = 12 cells × 6` 在**覆盖、平衡、算力**之间是当前最合理的折中。

## 8.3 P-1.2 如何使用它

- `Stage-1`：必须从 frozen dataset 中挑代表场景，而不是临时手搓 seed；
  - 强制展示：`CF-L2` 1 个、`EC-L2` 1 个；
  - 做 fusion/fallback 消融时，再额外取 `SC-L2`；
  - 做 funnel/micro 消融时，再额外取 `FC-L2`。
- `Stage-2`：必须使用 `48` 个 test scenes 全量批跑，并按 `CF / SC / FC / EC` 分 family 统计。

---

## 9. 固定文件格式：建议 JSON schema

每个 concrete scene 建议使用统一 JSON：

```json
{
  "schema_version": "dockbench-v1.0",
  "scene_id": "DBv1-EC-L2-003",
  "split": "test",
  "family": "EC",
  "difficulty": "L2",
  "generator_version": "dock_sgen_1.0",
  "map": {"width_m": 40.0, "height_m": 20.0, "resolution_m": 0.05},
  "leader": {"x": 3.2, "y": -1.5, "yaw": 0.0},
  "follower": {"x": -4.8, "y": 1.1, "yaw": 0.32},
  "obstacles": [
    {"x": 1.2, "y": -0.2, "width": 3.0, "height": 1.5, "yaw": 0.0, "role": "screen"},
    {"x": 3.0, "y": -2.2, "width": 1.2, "height": 0.8, "yaw": 0.0, "role": "dock_zone"}
  ],
  "descriptors": {
    "d0_m": 8.4,
    "heading_diff_deg": 18.3,
    "bearing_deg": 31.0,
    "occlusion_index": 0.62,
    "detour_factor": 1.21,
    "dock_zone_clearance_m": 0.46,
    "staging_shift_required_m": 1.32,
    "capture_constraint_index": 0.41,
    "homotopy_count": 2
  },
  "quality": {
    "valid": true,
    "label_fidelity": true,
    "diversity_score": 0.41,
    "balance_score": 1.0,
    "scene_quality_score": 0.86
  }
}
```

### 9.1 配套清单文件

除单场景 JSON 外，必须冻结：
- `dockbench_v1_manifest.json`
- `dockbench_v1_split_tuning.json`
- `dockbench_v1_split_test.json`
- `dockbench_v1_split_challenge.json`
- `dockbench_v1_quality_report.json`

---

## 10. 是否需要场景质量评估器？答案：必须需要

仅靠“生成器成功产出场景”远远不够。DockBench-v1 必须有一个**场景质量评估器（Scenario Quality Evaluator）**。

## 10.1 质量评估器分两层

### A. 结构质量评估（method-agnostic）

不依赖具体 docking 控制器，只检查场景本身是否合理：
- `G_valid`：出生合法、边界合法、基础可达、无明显无解；
- `G_label`：几何标签是否满足 family 阈值；
- `Q_div`：与同 cell 已入库场景的最小描述子距离是否足够大；
- `Q_bal`：当前 cell 是否缺样本，防止数据集分布失衡。

推荐形式：
\[
Q_{struct}(s)=G_{valid}(s)G_{label}(s)\Big(\lambda_{div}Q_{div}(s)+\lambda_{bal}Q_{bal}(s)\Big)
\]

其中可取：`λ_div = 0.7`，`λ_bal = 0.3`。

### B. 闭环审计评估（support-aware audit）

不是用来“挑对自己有利的 test scene”，而是用来确认 family 标签真的打到了对应机制：
- `CF / SC / FC`：至少 1 个强传统基线在对应 tuning cell 中应有非零成功率；
- `EC`：leader 初始位姿应确实存在明显 dockability gap，且 cooperative staging 可显著降低 gap；
- `SC`：应观测到视觉有效率/回退频率显著非零；
- `FC`：应观测到近场停滞/回退/微机动需求显著高于 `CF`。

## 10.2 推荐的质量描述子向量

定义场景描述子：
\[
\chi(s)=
[d_0,\ |\Delta\psi_0|,\ \beta_0,\ O_{occ},\ F_{detour},\ R_{stage},\ C_{capture},\ C_{dock},\ H_{homo}]^\top
\]

并在归一化后使用加权欧氏距离或 Gower 距离：
\[
Q_{div}(s)=\min_{s'\in \mathcal D_{cell}}\|W(\chi(s)-\chi(s'))\|_2
\]

若 `Q_div < τ_div`（建议 `τ_div = 0.18`），则认为该场景与已有样本过近，不入库。

---

## 11. DockBench-v1 的生成策略：不是纯随机，而是“生成 + 筛选 + 冻结”

推荐流程：
1. 先按 family/difficulty 的逻辑参数范围生成候选场景；
2. 用结构质量评估器筛掉无效/重复/越界/标签不稳的场景；
3. 对通过者做短程 foundation 审计；
4. 仅把通过审计者写入 frozen split；
5. 一旦 split 冻结，P-1.2 只能读取，不得再改。

这意味着：
- 生成器可以是随机的；
- **数据集本身必须是固定的**。

因此，问题 3 的答案是：**必须构建固定数据集，而不是每次实验重新随机出 test scenes。**

---

## 12. 这件事能否提炼成独立创新点？

**可以，但前提是做成“benchmark methodology”，而不是“整理若干 JSON”。**

### 12.1 可以论道的创新点是什么

可以提炼为：
**Mechanism-Complete Docking Benchmark with Support-Aware Scenario Concretization**

其核心不是“我们也做了个数据集”，而是：
1. **机制完备 taxonomy**：`CF / SC / FC / EC` 不是语义命名，而是与 docking 内部机制一一对应；
2. **support-aware quality evaluator**：不是只看几何，还用基础支撑短程审计验证场景标签；
3. **abstract → logical → concrete → audit** 的四层闭环；
4. **family-balanced, cell-balanced** 的固定 split，使 benchmark 既能验证竞争性，也能验证可解域扩展；
5. **标签与失效模式对齐**：后续可直接研究“某方法在哪个 family 失效，为什么失效”。

### 12.2 它能否单独成为论文主创新

- 作为**强 benchmark / dataset / methodology 论文**：有潜力；
- 作为顶会主线算法论文的唯一创新：通常还不够；
- 作为本项目 P-1/P2 的**独立子贡献或强第二贡献**：完全值得做。

我的判断：
- 若未来把 DockBench-v1 开源，并给出 formal coverage、quality evaluator、multi-method correlation analysis，它可以独立成篇；
- 在当前项目里，更建议把它作为**P-1 的制度化前置贡献 + 后续论文的 benchmark 贡献**。

---

## 13. 对用户提出的 6 个问题的直接答案

1. **是否需要显式纳入两车初始距离与航向角差？**  
   需要，而且必须进入 descriptor space；同时还要补一个 `bearing_deg`，否则信息不完整。

2. **障碍物分布应怎么分？**  
   不应按“数量/密度”粗分，而应按 `screen / channel / dock_zone / background / hybrid` 的**角色化拓扑**来分，并分别统计 corridor 与 dock zone 指标。

3. **是否应该构建固定数据集？**  
   必须。生成器可以随机，但 benchmark 的 `test/challenge` split 必须冻结。

4. **整个数据集应有多少测试场景？**  
   当前推荐官方版为 `72` 个 concrete scenes，其中 `48` 个 test；如需 CI，可额外维护 mini split，但正式结论只认官方 split。

5. **每类场景占比应是多少？**  
   采用 cell-balanced：`4 families × 3 difficulty` 完全均衡；family 各 `25%`，difficulty 各 `33.3%`。

6. **是否需要场景质量评估器？**  
   必须需要，而且应分为**结构质量评估**与**闭环审计评估**两层。

---

## 14. 对 P-1.2 的直接要求

P-1.2 不得再直接使用“临时随机 seed 集”作为正式实验基准，而必须：
1. 从 `DockBench-v1` 读取 frozen split；
2. Stage-1 从 frozen family representative 中选图；
3. Stage-2 对 frozen `test` split 全量批跑；
4. 报告必须按 `CF / SC / FC / EC` 四个 family 分别统计；
5. 若 family 标签与真实执行不符，优先回到本规范修订数据集，而不是先改模型结论。

这一步做完后，再进入 Stage-3 多车实验，逻辑会清晰很多。
