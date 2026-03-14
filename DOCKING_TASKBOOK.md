# DOCKING_TASKBOOK.md

> 作用：把 `DOCKING.md` 的下一版理论重构拆成**可执行、可验证、可抛弃**的任务路线。  
> 规则：本任务书只定义“设计—验证—接受/抛弃”的 protocol；在全部 gate 通过前，不允许直接覆盖 `DOCK.md`。

# 0. 任务总目标

目标不是继续堆叠 LC 专用补丁，而是完成以下三件事：

1. 用 `DOCKING.md` 中的**最小完备证书集**替换当前局部拼接式证书；
2. 用**混合系统合同 + 分层多目标优化**替换当前固定加权和 + 经验阈值门控；
3. 通过实验和证明，把系统从“局部有理性”推进到“端到端可解释 + safe reject / success-certified 可证明”。

最终 acceptance gate：
- `LC` 成功率 `>= 80%`；
- `LC` collision rate `= 0`；
- `CF / SC / FC / EC` 成功率不低于当前冻结基线；
- failure-mode 归因与证书日志一致；
- `DOCKING.md` 中的核心命题有可审计的证明草稿与实验支撑。

---

# 1. 当前冻结基线与差距

## 1.1 当前冻结基线

以当前恢复后的 Round-18 风格实现作为理论重构前的冻结基线：
- 报告：`output/round18_restore2/round18_restore_conclusion.md`
- 逐案表：`output/round18_restore2/round18_restore_summary.csv`

基线主指标：
- `LC success = 7/18 = 38.9%`
- `LC collision = 1/18 = 5.6%`
- 成功 LC 的平均航向误差约 `32.65°`
- `CF / SC / FC / EC` 代表守护场景维持成功

## 1.2 当前主要结构性缺口

1. `LC` 仍以 `lc_geometric_deadlock` 为主，说明 basin / handover completeness 不足；
2. `DBv1-LC-L1-073` 仍发生碰撞，说明 safety 尚未真正独立于软评分；
3. 成功个例的最终姿态误差仍大，说明 post-docking mobility 尚未成为可证明对象；
4. release / lock / post-align 仍主要靠局部补救，没有统一 contract；
5. 现有统计能说明“哪些 case 成败了”，但还不能严格回答“为什么对应证书应当充分且必要”。

---

# 2. 执行总协议

每一轮方法设计必须严格遵循：

1. **分析问题**：从 failure mode 与证书缺口出发，不得直接从代码 patch 出发；
2. **调研资料**：补充本轮问题对应的主文献，并写出“借鉴点 / 不采用点”；
3. **设计方法**：先写数学对象、合同条件、证明目标，再写代码结构；
4. **实验验证**：必须按“单案复现 → frozen 全量 test → 消融验证 → failure-mode 分层统计”执行；
5. **分析结果**：明确该设计解决了什么、未解决什么、引入了什么新副作用；
6. **接受/抛弃**：如果没有通过 gate，不允许模糊地“部分保留”；必须明确进入下一轮迭代或整体抛弃；
7. **整合方法**：只有被接受的方法，才允许进入 `DOCKING.md` 主体并准备未来写回 `DOCK.md`。

---

# 3. 研究与实现阶段划分

本任务书把下一轮工作拆为 6 个阶段，每个阶段都必须有独立输出物与 gate。

## Phase-A：Failure Taxonomy 与最小完备证书集冻结

### A.1 目标
- 冻结任务级 failure family `\mathfrak F`；
- 为每个 failure 指定责任证书与可审计日志信号；
- 形成 `DOCKING.md` 第 4–5 章的正式版本。

### A.2 输出物
- `artifacts/docking_research/failure_taxonomy.md`
- `artifacts/docking_research/failure_certificate_matrix.csv`
- `artifacts/docking_research/literature_gap_review.md`

### A.3 单案复现集
- `DBv1-LC-L1-073`：collision
- `DBv1-LC-L1-074`：terminal closure / tight corridor
- `DBv1-LC-L2-080`：right-bay reachability
- `DBv1-LC-L2-081`：success but poor post-dock pose
- `DBv1-LC-L2-082`：handover / release-sensitive
- `DBv1-LC-L2-083`：macro-witness-sensitive
- `DBv1-LC-L3-085`：deep deadlock / exec-shape-sensitive
- `DBv1-CF-L2-010`, `DBv1-SC-L2-026`, `DBv1-FC-L2-046`, `DBv1-EC-L2-065`：非 LC 守护集

### A.4 验收标准
- 每个 failure mode 都能映射到至少一张责任证书；
- 不允许存在“该 case 失败了，但无法说明是哪个证书在负值区间”；
- 若存在无法归因的 failure，则 Phase-A 不通过。

## Phase-B：保守证书实现（从 heuristic 到 conservative certificates）

### B.1 目标
按证书类别依次替换启发式逻辑：
1. `\sigma_S`：安全 / lane fidelity / swept-volume；
2. `\sigma_V`：可见性 / reacquisition；
3. `\sigma_B`：basin / reach；
4. `\sigma_E`：execution tube；
5. `\sigma_H`：handover；
6. `\sigma_L`：terminal closure；
7. `\sigma_{post}`：post-docking mobility。

### B.2 代码组织要求
允许新增但不限于：
- `docking/certificates/*.py`
- `docking/analysis/certificate_audit.py`
- `scripts/run_docking_certificate_audit.py`

禁止：
- 继续把新证书直接堆在 `docking/lc_corridor.py` 的分支里；
- 以“只对某一个 scene id 生效”的 if-else 代替几何/合同定义。

### B.3 逐证书验证协议
每加入一张证书，都必须完成：
1. **单案复现**：选择与该证书最相关的 1–2 个代表案例；
2. **frozen 全量 test**：全场景测试，至少跑 `LC-18 + 非 LC 守护集`；
3. **消融验证**：移除该证书，再跑相同集；
4. **failure-mode 统计**：比较 removal 前后 failure 结构是否按预期转移。

### B.4 逐证书接受标准
- `\sigma_S`：引入后 `LC collision rate` 必须压到 `0`；
- `\sigma_V`：可见性类 failure 必须被日志提前捕获；
- `\sigma_B`：deadlock 类 failure 必须与 basin margin / witness empty 明确一致；
- `\sigma_E`：执行偏差导致的失败必须能和 plan certificate 失配区分开；
- `\sigma_H`：错误 release 必须转化为延迟释放或 safe reject，优先于碰撞；
- `\sigma_L`：docking 末端 failure 必须能解释为 closure witness 不存在，而不是黑盒 timeout；
- `\sigma_{post}`：成功 case 的后对接姿态/前向机动性必须有显式统计指标。

## Phase-C：分层多目标优化器替换固定加权和

### C.1 目标
- 用 `Level-1 hard constraints + Level-2 lexicographic feasibility + Level-3 adaptive performance` 替换固定权重和；
- 实现 dual-induced adaptive weights 或等价的 state-dependent performance scheduler；
- 确保 `CGFL / PTCC` 被降级为 Level-3 performance policy，而不是可行性逻辑。

### C.2 代码候选位置
- `docking/optim/certificate_scheduler.py`
- `docking/optim/staging_optimizer.py`
- `docking/optim/witness_library.py`

### C.3 对比实验
必须固定 3 个求解器版本：
1. `Baseline-WeightedSum`：旧固定权重；
2. `Lexico-NoAdaptive`：仅 lexicographic，无 adaptive performance weights；
3. `Lexico-Adaptive`：完整新方案。

### C.4 验收标准
- `Lexico-Adaptive` 不能因为追求后对接姿态而破坏 `\sigma_S / \sigma_H / \sigma_L`；
- 相比 `Baseline-WeightedSum`，`LC` 成功率应显著提升或至少 deadlock 率显著下降；
- 若 `Lexico-Adaptive` 在 `CF / SC / FC / EC` 引入回退，则 Phase-C 不通过。

## Phase-D：混合合同、切换律与 safe-reject

### D.1 目标
- 把模式切换重写为正式的 guard / reset / dwell-time 合同；
- 证明无错误 release、无 post-align 死循环、无 mode 抖振；
- 明确 `SAFE_REJECT` 是合法系统终点，而不是“失败但说不清为什么”。

### D.2 需要冻结的对象
- mode 集合
- 每个 mode 的 active certificates
- enter / exit hysteresis
- reset compatibility 条件
- average dwell-time 或 minimum dwell-time

### D.3 验证重点
- `DBv1-LC-L2-082`：release contract
- `DBv1-LC-L1-074`：terminal closure contract
- `DBv1-LC-L2-081`：post-align 不得导致 success 回退
- `DBv1-SC-L2-026` / `DBv1-EC-L2-065`：visibility / fallback 切换不得抖振

### D.4 验收标准
- 任意切换必须能在日志中给出“哪几张证书满足了 guard”；
- 若发生 safe reject，必须给出责任证书链；
- 不允许再次出现“LEADER_POST_ALIGN 被触发但没有明确合同含义”的状态。

## Phase-E：组合定理与证明草稿

### E.1 目标
为以下命题产出证明草稿：
1. mode 内 soundness；
2. failure-mode completeness；
3. reset compatibility 下的 hybrid composition；
4. success-certified / safe-certified 端到端性质。

### E.2 输出物
- `artifacts/docking_proofs/mode_soundness.md`
- `artifacts/docking_proofs/failure_completeness.md`
- `artifacts/docking_proofs/hybrid_composition.md`
- `artifacts/docking_proofs/end_to_end_guarantee.md`

### E.3 验收标准
- 证明中引用的每一个集合 / 证书 / guard / reset 必须能在代码中找到对应实现；
- 若证明依赖了代码里不存在的对象，则 Phase-E 不通过。

## Phase-F：全量冻结、消融矩阵、准备回写 `DOCK.md`

### F.1 目标
- 跑完整 frozen dataset；
- 输出 full report、ablation report、failure-mode report、post-dock mobility report；
- 若全部 gate 通过，才允许把 `DOCKING.md` 回写为新的 `DOCK.md`。

### F.2 最终输出物
- `artifacts/docking_final/full_report.md`
- `artifacts/docking_final/ablation_report.md`
- `artifacts/docking_final/failure_mode_report.md`
- `artifacts/docking_final/postdock_report.md`
- `artifacts/docking_final/design_diff_vs_dock.md`

---

# 4. 实验 protocol（每轮都必须执行）

## 4.1 单案复现

### 必跑 LC 核心集
- `DBv1-LC-L1-073`
- `DBv1-LC-L1-074`
- `DBv1-LC-L2-080`
- `DBv1-LC-L2-081`
- `DBv1-LC-L2-082`
- `DBv1-LC-L2-083`
- `DBv1-LC-L3-085`

### 必跑 Non-LC 守护集
- `DBv1-CF-L2-010`
- `DBv1-SC-L2-026`
- `DBv1-FC-L2-046`
- `DBv1-EC-L2-065`

### 单案输出要求
每个 case 至少输出：
- 原始结果 JSON
- mode / certificate log
- failure attribution JSON
- 可视化 GIF / MP4
- case note（说明本轮证书对该 case 的作用）

## 4.2 frozen 全量 test

每轮接受评估时，至少执行：
1. `LC-18`
2. 全 family frozen test split
3. challenge split（若本轮目标已接近冻结）

输出指标：
- success rate / collision rate / min clearance
- mode-switch statistics
- certificate violation histogram
- handover false-positive / false-negative
- terminal closure success ratio
- post-docking mobility statistics

## 4.3 消融矩阵

最少需要以下消融：
- `A-NoSigmaS`
- `A-NoSigmaV`
- `A-NoSigmaB`
- `A-NoSigmaE`
- `A-NoSigmaH`
- `A-NoSigmaL`
- `A-NoSigmaP`
- `A-WeightedSum`
- `A-NoHybridContracts`
- `A-NoWitnessUpgrade`

要求：
- 每个消融都要说明预期退化的 family / failure mode；
- 若消融后没有任何可解释退化，则要么该模块无效，要么该实验设计无效，两者必须择一说明。

## 4.4 failure-mode 分层统计

冻结 failure labels：
- `collision_obstacle`
- `collision_pair`
- `off_lane_or_unsafe_clearance`
- `visibility_unrecovered`
- `lc_geometric_deadlock`
- `exec_mismatch`
- `handover_denied`
- `handover_unsafe`
- `terminal_closure_fail`
- `postdock_mobility_fail`
- `safe_reject`
- `timeout_unknown`（该类必须逐轮压缩；若长期存在，说明证书集仍不完备）

每轮必须输出：
- 整体 failure histogram
- family × failure matrix
- certificate violation → failure correlation matrix

---

# 5. 接受 / 抛弃规则

## 5.1 接受规则
一个新方法 / 新证书 / 新合同只有在同时满足下列条件时才允许接受：
1. 单案复现至少解决了它声称解决的问题；
2. frozen 全量 test 没有破坏 Non-LC 守护集；
3. 消融结果能证明其必要性；
4. failure-mode 分层统计与理论职责一致；
5. 代码组织符合 `DOCKING.md` 的模块化要求。

## 5.2 抛弃规则
若满足任一条，必须明确抛弃或回退：
1. 仅在个别 scene id 上通过硬编码生效；
2. 无法解释到哪张证书或哪个 guard；
3. 解决一个 failure，却在另一 family 大量引入 regression；
4. 不能通过消融证明自身必要性；
5. 依赖人工 magic weight 才成立。

## 5.3 保留但降级规则
有些方法不作为理论主体，但可以保留为 witness 工具：
- macro primitives
- semantic anchor seeds
- post-align 微机动
- speed fast-lane

前提是：它们被明确降级为 `witness library` 或 `Level-3 performance policy`，而不是系统正确性的前提。

---

# 6. 文档同步要求

每当某个 Phase 通过，必须同步更新：
- `DOCKING.md`：理论对象、命题、代码组织；
- `DOCKING_TASKBOOK.md`：阶段状态、通过条件、下一步任务；
- 相关实验汇总 `.md`：写明接受/抛弃的依据；
- 若涉及代码结构重整，还需同步更新 `DESIGN.md` 对应映射。

在所有 gate 通过之前：
- `DOCK.md` 只保留当前稳定基线，不混入未经验证的新理论；
- 不允许“先改 DOCK.md，后补验证”。

---

# 7. 本轮立即执行的设计任务

本轮不直接改算法，而是先完成以下文档层任务：

1. 冻结 `DOCKING.md` 的核心理论结构：
   - failure taxonomy
   - 最小完备证书集
   - 分层多目标优化
   - 混合合同
   - 端到端组合定理
2. 冻结 `DOCKING_TASKBOOK.md` 的执行 protocol：
   - Phase-A 到 Phase-F
   - 单案 / 全量 / 消融 / failure 统计流程
   - 接受/抛弃规则
3. 明确代码未来的组织边界，防止下一轮继续补丁化实现。

本轮完成标准：
- `DOCKING.md` 成为完整、独立、可执行的设计稿；
- `DOCKING_TASKBOOK.md` 成为后续每轮实验必须遵守的任务协议；
- 二者内容与当前 `DOCK.md`、`DESIGN.md`、当前基线工件不冲突。
