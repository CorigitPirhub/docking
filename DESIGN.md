# DESIGN.md

本文档把 `DOCK.md` 中的设计逐条映射到当前代码实现，并且按照**从最顶层运行入口到最底层几何/控制细节**的顺序展开。  
当前代码口径以“**手动恢复到 Round-18 基线行为**”为准；对应验证结果见：

- `output/round18_restore2/round18_restore_summary.csv`
- `output/round18_restore2/round18_restore_conclusion.md`
- `output/round18_restore2/round18_restore_vs_artifact.md`

该恢复口径与 `artifacts/lc_round18_lc12_regression/lc_round18_lc12_regression.md` 的关键行为基本一致：  
`DBv1-LC-L1-073` 重新回到 `collision_obstacle`；`DBv1-LC-L2-081 / DBv1-LC-L2-083` 重新回到 `success=True`。

---

## 1. 顶层运行路径

### 1.1 单例闭环入口

最顶层入口是：

- `scripts/run_p_minus1_stage1_docking.py`

运行顺序：

1. `main()` 解析命令行、读取配置与场景 JSON。
2. 通过 `docking/p_minus1_baselines.py:build_method_runners()` 创建方法实例。
3. 对于 `co_bcfd`，`SkillRunner` 会包装 `docking/dock_skill.py:CooperativeDockingSkill`。
4. `_run_one(...)` 做离散时间仿真：
   - 每个控制周期调用 `controller.compute_commands(...)`；
   - 更新 `leader/follower` 状态；
   - 记录 `history`；
   - 用 `DockingLockEvaluator` 做最终 `locked` 判定；
   - 导出 JSON / Markdown / GIF。

这条链路对应 `DOCK.md` 的：

- 第 3 节：`Co-BCFD` 总体框架
- 第 5 节：控制与规划闭环
- 第 7 节：Stage-1 验证方式

### 1.2 批量数据集入口

批量基准入口是：

- `scripts/run_p_minus1_stage2_batch.py`
- `scripts/run_p_minus1_stage1_suite.py`

两者都不会绕开单例逻辑，而是**批量调用 `run_p_minus1_stage1_docking.py`**。

因此：

- 所有 Stage-2 / frozen test / family breakdown 结果；
- 所有 baseline / ablation 对比；
- 所有代表场景 GIF；

本质上都共享同一套 `CooperativeDockingSkill + CooperativeStagingPlanner + CorridorReciprocityPlanner/Executor` 实现。

这对应 `DOCK.md` 的：

- 第 6 节：传统方法基线
- 第 7 节：Stage-1 验证方式
- 第 8 节：实现映射
- 第 9、10 节：代表场景与批量结论

---

## 2. 分层调用图（从上到下）

### Layer-0：实验脚本 / 报告脚本

#### A. 单例执行

- `scripts/run_p_minus1_stage1_docking.py`
  - `main()`：实验入口
  - `_run_one(...)`：统一仿真主循环
  - `_project_follower_locked(...)` / `_project_terminal_hold_pair(...)`：末端投影

#### B. 批量评测

- `scripts/run_p_minus1_stage2_batch.py`
  - `_run_stage1_once(...)`：批量时调用单例脚本
  - `_aggregate(...)`：方法/家族聚合
  - `_paired_*`：配对 bootstrap 对比

#### C. 代表场景套件

- `scripts/run_p_minus1_stage1_suite.py`

#### D. 可视化

- `scripts/visualize_docking_diagnostics.py`
- `docking/visualization.py`

---

### Layer-1：方法工厂与对比协议

文件：

- `docking/p_minus1_baselines.py`

关键符号：

- `build_method_runners(...)`
- `SkillRunner`
- `StaticLeaderRunner`
- `CoopBaselineRunner`

它完成两件事：

1. 把 `DOCK.md` 中“主方法 / 强基线 / 能力匹配基线 / 消融”落实为具体 `method_id`；
2. 保证所有方法共享相同场景输入和统一评测口径。

这直接对应 `DOCK.md` 第 6 节“传统方法基线（P-1.2 强制对比）”。

常用方法映射：

- `co_bcfd`：完整主方法
- `T_hard_switch` / `T_lattice_pbvs` / `T_parking_hierarchical`：传统强基线
- `T_coop_dist_blend`：能力匹配基线
- `A_no_stage` / `A_no_belief_gate` / `A_no_funnel_gate` / `A_no_micro_maneuver` / `A_no_corridor_reciprocity`：机制消融

---

### Layer-2：Docking Skill 总状态机

文件：

- `docking/dock_skill.py`

核心类：

- `CooperativeDockingSkill`

这是整个实现的**总状态机和聚合器**。  
`DOCK.md` 中所有“belief / capture funnel / cooperative staging / TVT / VPCR / PTCC / LC corridor reciprocity”的设计，最后都在这里被调度。

主阶段：

- `STAGING`
- `DOCKING`

主内部状态：

- `_active_branch`：`TVT` / `VPCR`
- `_semantic_lane_plan`：是否是 lane/corridor 约束计划
- `_leader_goal` / `_follower_goal`
- `_leader_path_xy` / `_follower_path_xy`

与 `DOCK.md` 的映射：

- 第 3 节：`Co-BCFD` 总体创新
- 第 4.4 节：统一证书场在运行时的聚合
- 第 5.2 节：Safety Projection
- 第 5.3 节：协同 staging
- 第 5.5 节：TVT
- 第 5.6 节：VPCR
- 第 5.8 节：PTCC

---

### Layer-3：全局 / 半全局 staging 规划

文件：

- `docking/coop_docking.py`
- `docking/lc_corridor.py`

#### A. 一般场景 staging

类：

- `CooperativeStagingPlanner`
- `StagingTracker`

职责：

- 在非 LC 场景中生成 `leader_goal / follower_goal / leader_path / follower_path`
- 决定 leader 是否需要 relocation
- 对 `semantic_anchor` 做 lightweight 语义锚点优先

对应 `DOCK.md`：

- 第 5.3 节：协同 staging

#### B. LC 场景专用 planning / execution

类：

- `CorridorReciprocityPlanner`
- `CorridorReciprocityExecutor`

职责：

- 生成 corridor-compatible cooperative staging plan
- 评估 `sigma_C`、`sigma_C^{exec}`、`sigma_M`
- 在 `LEADER_SHAPE / LEADER_SETTLE / FOLLOWER_INGRESS / FOLLOWER_TERMINAL_CLOSURE` 中执行

对应 `DOCK.md`：

- 第 4.5 节：`\sigma_C`
- 第 5.3B 节：`LC-CRBS`
- 第 5.3C 节：`\sigma_C^{exec}`
- 第 5.3D 节：`\sigma_M`

---

### Layer-4：子技能与辅助证书

文件：

- `docking/stage25_subskills.py`
- `docking/time_compression.py`

类：

- `TerminalViabilityTubeSkill`
- `VisibilityPersistentRestagingSkill`
- `PlanCompressionCertifier`

对应 `DOCK.md`：

- 第 5.5 节：TVT
- 第 5.6 节：VPCR
- 第 5.8 节：PTCC

---

### Layer-5：底层控制 / 感知 / 几何 / 安全

文件：

- `docking/bcfd.py`
- `docking/controllers.py`
- `docking/sensors.py`
- `docking/collision.py`
- `docking/kinematics.py`

这些文件承接 `DOCK.md`：

- 第 1 节：车辆、几何、感知、过程约束
- 第 4.1 节：相对误差
- 第 4.2 节：Belief
- 第 4.3 节：Capture-Funnel
- 第 5.2 节：Safety Projection

---

## 3. DOCK.md → 代码映射总表

说明：

- `DOCK.md` 第 2 节“顶刊/顶会相关研究脉络”是**方法来源与差异化论证**，不是直接代码模块，因此它在本表中只通过后续实现章节间接落地。
- `DOCK.md` 第 9、10 节是**实验结论章节**，其实现载体是实验脚本与产物，而不是新的算法类。

| DOCK 节 | 设计含义 | 主要实现文件 | 主要符号 |
|---|---|---|---|
| `1.1` 车辆与几何 | Ackermann 约束、hitch 几何、尺寸 | `docking/kinematics.py`, `docking/types.py`, `docking/config.py` | `AckermannModel`, `VehicleGeometry`, `VehicleState` |
| `1.2` 感知 | GNSS / Vision 相对观测 | `docking/sensors.py`, `docking/dock_skill.py`, `docking/bcfd.py` | `GlobalPoseSensor`, `VisionSensor`, `_fusion()`, `_robust_fusion()` |
| `1.3` 过程约束 | bounded map / collision / clearance / lock rules | `docking/collision.py`, `scripts/run_p_minus1_stage1_docking.py` | `CollisionEngine`, `_run_one()` |
| `3` Co-BCFD 总体 | 主方法组合体 | `docking/dock_skill.py`, `docking/p_minus1_baselines.py` | `CooperativeDockingSkill`, `build_method_runners()` |
| `4.1` 相对误差 | leader/follower frame relative state | `docking/lc_corridor.py`, `docking/dock_skill.py`, `docking/bcfd.py` | `_leader_frame_relative_state()`, `_bcfd_near_field()` |
| `4.2` Belief | GNSS↔Vision consistency fusion | `docking/dock_skill.py`, `docking/bcfd.py` | `_fusion()`, `_robust_fusion()` |
| `4.3` Capture-Funnel | near-field longitudinal/lateral/yaw regulation | `docking/dock_skill.py`, `docking/bcfd.py` | `_bcfd_near_field()`, `BeliefConsistentFunnelDockingController` |
| `4.4` 统一证书场 | `sigma_P/T/V/C/post` 统一评分对象 | `docking/lc_corridor.py` | `UnifiedCertificateField`, `StagingCertificateOptimizer` |
| `4.5` `sigma_C` | corridor reciprocity / basin quality | `docking/lc_corridor.py` | `_certificate_from_anchor()`, `_robust_execution_certificate()` |
| `5.1` BR-SMPC 近场捕获 | 采样式/原语式近场控制近似 | `docking/lc_corridor.py`, `docking/dock_skill.py` | `terminal_capture_command()`, `_terminal_closure_primitives()` |
| `5.2` Safety Projection | 单步安全投影 | `docking/dock_skill.py`, `scripts/run_p_minus1_stage1_docking.py` | `_project_safe()`, `_project_follower_locked()` |
| `5.3` 协同 staging | leader/follower joint staging | `docking/coop_docking.py` | `CooperativeStagingPlanner.plan()` |
| `5.3B` LC-CRBS | LC corridor planning + execution | `docking/lc_corridor.py` | `CorridorReciprocityPlanner.plan()`, `CorridorReciprocityExecutor.command()` |
| `5.3C` `sigma_C^{exec}` | 运行时 basin 有效性 | `docking/lc_corridor.py` | `compute_execution_certificate()`, `_search_certificate_ascent_anchor()` |
| `5.3D` `sigma_M` | terminal manifold closure | `docking/lc_corridor.py` | `compute_terminal_closure_certificate()`, `_terminal_closure_ready()` |
| `5.4` CGFL | certificate-gated fast lane | `docking/time_compression.py`, `docking/dock_skill.py` | `PlanCompressionCertifier`, `_fast_lane_certified` |
| `5.5` TVT | terminal viability tube branch | `docking/stage25_subskills.py`, `docking/dock_skill.py` | `TerminalViabilityTubeSkill`, `_maybe_activate_tvt()` |
| `5.6` VPCR | visibility-persistent restaging branch | `docking/stage25_subskills.py`, `docking/dock_skill.py` | `VisibilityPersistentRestagingSkill`, `_maybe_activate_vpcr()` |
| `5.8` PTCC | plan-triggered corridor compression | `docking/time_compression.py`, `docking/dock_skill.py` | `PlanCompressionCertifier.certify()` |
| `6` 基线/消融协议 | baseline pool / ablation ids | `docking/p_minus1_baselines.py` | `METHOD_SPECS`, `build_method_runners()` |
| `7` 验证方式 | 单例、suite、batch、artifact | `scripts/run_p_minus1_stage1_docking.py`, `scripts/run_p_minus1_stage1_suite.py`, `scripts/run_p_minus1_stage2_batch.py` | `main()`, `_run_one()` |

---

## 4. 关键公式与代码实现推导

### 4.1 相对对接误差（对应 DOCK 4.1）

`DOCK.md` 给出的 leader/follower frame 相对状态，本质是刚体坐标变换：

\[
\mathbf{r}_W = \mathbf{p}_{target} - \mathbf{p}_{ref}
\]

对参考车体朝向 \(\psi\) 做逆旋转：

\[
\begin{bmatrix}
x_r \\
y_r
\end{bmatrix}

=
R(-\psi)
\begin{bmatrix}
\Delta x \\
\Delta y
\end{bmatrix}
=
\begin{bmatrix}
\cos\psi & \sin\psi \\
-\sin\psi & \cos\psi
\end{bmatrix}
\begin{bmatrix}
\Delta x \\
\Delta y
\end{bmatrix}
\]

航向误差：

\[
\psi_r = \operatorname{wrap}(\psi_{target} - \psi_{ref})
\]

代码落点：

- `docking/lc_corridor.py:_leader_frame_relative_state()`
- `docking/dock_skill.py:_bcfd_near_field()`
- `docking/bcfd.py:BeliefConsistentFunnelDockingController._bcfd_near_field()`

这些函数都在做同一件事：  
把 `rear_hitch - front_hitch` 的世界坐标误差投到 leader frame，再用 `x_l / y_l / yaw_diff` 驱动纵向与横向控制。

---

### 4.2 双源观测与 belief 一致性（对应 DOCK 4.2）

代码实现不是完整的 Kalman filter，而是一个**belief-consistent blending gate**：

1. 先分别得到：
   - global rear-hitch 估计 \((\hat p_g, \hat\psi_g)\)
   - vision rear-hitch 估计 \((\hat p_v, \hat\psi_v)\)

2. 用协方差和创新定义 NIS：

\[
\nu = \hat p_v - \hat p_g,\quad
S = R_g + R_v,\quad
\mathrm{NIS} = \nu^\top S^{-1}\nu
\]

3. 再定义两级门控：

\[
\gamma_{dist} = \sigma\big(k_d (s(d)-c_d)\big)
\]
\[
\gamma_{nis} = \sigma\big(k_n (\tau_{nis} - \mathrm{NIS})\big)
\]
\[
\gamma = \operatorname{clip}(\gamma_{dist}\gamma_{nis}, 0, 1)
\]

4. 视觉权重：

\[
w_{vis} = \gamma \cdot \frac{\operatorname{tr}(R_g)}{\operatorname{tr}(R_g)+\operatorname{tr}(R_v)}
\]

5. 最终融合：

\[
\hat p = (1-w_{vis})\hat p_g + w_{vis}\hat p_v
\]
\[
\hat\psi = \operatorname{wrap}\left(\hat\psi_g + w_{vis}\operatorname{angle\_diff}(\hat\psi_v,\hat\psi_g)\right)
\]

代码落点：

- `docking/dock_skill.py:_fusion()`
- `docking/bcfd.py:BeliefConsistentFunnelDockingController._robust_fusion()`

---

### 4.3 Capture-Funnel 近场控制（对应 DOCK 4.3 / 5.1）

当前实现用的是“**距离调度 + leader-frame Stanley-like regulation**”这一离散化近似，而不是显式求解 MPC：

#### 纵向参考

\[
v_{ref} = v_L + k_x (x_l - x^\star(d))
\]

其中 \(x^\star(d)\) 按距离分段变化：

- 远场：保持 standoff
- 中场：对齐 standoff
- 近场：收敛到 0

#### 横向/航向参考

\[
\delta_{ref} = \psi_r + \arctan\left(\frac{k_{cte} y_l}{|v|+\epsilon}\right)
\]

#### 近接触 gate

若 \(d\) 很小但 \(|y_l|\) 或 \(|\psi_r|\) 过大，则切成 backoff / speed clamp / lock assist。

代码落点：

- `docking/dock_skill.py:_bcfd_near_field()`
- `docking/dock_skill.py:compute_commands()` 中 `FUNNEL_ALIGN / LOCK_ASSIST`
- baseline 对照版本：`docking/bcfd.py`

因此，`DOCK.md` 中的 `Capture-Funnel` 理论，在当前代码里对应的是“**分阶段 leader-frame funnel control + lock-assist + soft-capture projection**”。

---

### 4.4 统一证书场（对应 DOCK 4.4）

统一证书场在代码里由：

- `docking/lc_corridor.py:UnifiedCertificateField`

给出。

主要分量：

- `sigma_post(...)`：后对接行进便利性
- `sigma_c_offline(...)`：离线 corridor execution 证书
- `sigma_t_offline(...)`：离线 terminal safety 证书
- `sigma_c_runtime(...)`：运行时 corridor 证书
- `sigma_t_runtime(...)`：运行时 terminal closure 证书
- `sigma_r_runtime(...)`：handoff/release readiness 证书

统一评分：

\[
S = \sum_k w_k \sigma_k
\]

代码中通过 `unified_staging_score(...)` 实现；  
在当前恢复基线里，`sigma_post` 仍然是**soft bias**，不是硬门控。

---

### 4.5 Corridor Reciprocity Certificate：`\sigma_C`（对应 DOCK 4.5）

`\sigma_C` 的离线主体由：

- `docking/lc_corridor.py:_certificate_from_anchor()`
- `docking/lc_corridor.py:_robust_execution_certificate()`
- `docking/lc_corridor.py:_robust_terminal_safety_certificate()`

构成。

其核心逻辑是：给定候选 leader anchor，检查

1. anchor 本身是否在 corridor 内且不碰撞；
2. 由 anchor 诱导的 follower predock 是否在 corridor 内；
3. follower 到 predock 的 corridor geodesic 是否有足够净空；
4. dock-zone / handoff / ingress alignment 是否良好。

这正是 `DOCK.md` 第 4.5 节写的：

\[
\sigma_C \sim
f(\text{anchor feasibility}, \text{predock feasibility}, \text{corridor ingress}, \text{handoff clearance}, \text{alignment})
\]

---

### 4.6 `\sigma_C^{exec}` 与 `\sigma_M`（对应 DOCK 5.3C / 5.3D）

运行时版本由 `CorridorReciprocityExecutor` 承担：

- `_robust_execution_certificate(...)`
- `compute_execution_certificate(...)`
- `compute_terminal_closure_certificate(...)`
- `_terminal_closure_ready(...)`
- `_docking_handoff_ready(...)`

其中：

- `sigma_C^{exec}`：release 前 basin 是否仍可行；
- `sigma_M`：pair 是否已进入 terminal manifold，并可完成末端闭合；

这就是 `DOCK.md` 第 5.3C / 5.3D 中“execution-time basin certificate + terminal manifold closure”的代码对应物。

---

## 5. LC-CRBS / LC corridor planning 的顶层映射

### 5.1 规划器入口

- `docking/lc_corridor.py:CorridorReciprocityPlanner.plan()`

这是 `DOCK.md` 第 5.3B “LC-CRBS” 的总入口。

其分支优先级（当前恢复基线）大致为：

1. `smallgap_escape`
2. `macro_escape`
3. `semantic_cert_anchor`
4. `escape_shape`
5. `exec_shape`
6. `semantic_reference`
7. `semantic_projection`
8. fallback `hybrid_search`
9. fallback `legacy`

与 Round-18 对应的 retained 主分支是：

- `lc_corridor_smallgap_escape`
- `lc_corridor_macro_escape`
- `lc_corridor_semantic_cert_anchor`
- `lc_corridor_exec_shape`
- `lc_corridor_semantic_reference`

### 5.2 运行时执行器入口

- `docking/lc_corridor.py:CorridorReciprocityExecutor.command()`

阶段机：

- `LEADER_SHAPE`
- `LEADER_SETTLE`
- `FOLLOWER_INGRESS`
- `FOLLOWER_TERMINAL_CLOSURE`
- `LEADER_RESHAPE`

这对应 `DOCK.md` 中：

- leader shaping / settling
- follower ingress
- terminal manifold closure
- runtime reshaping

---

## 6. Stage-2.5 子技能映射

### 6.1 TVT（对应 DOCK 5.5）

文件：

- `docking/stage25_subskills.py`

类：

- `TerminalViabilityTubeSkill`

入口：

- `docking/dock_skill.py:_maybe_activate_tvt()`

作用：

- 当 nominal terminal closure 不可靠但仍存在 tube-feasible corridor 时，切到 tube branch。

### 6.2 VPCR（对应 DOCK 5.6）

文件：

- `docking/stage25_subskills.py`

类：

- `VisibilityPersistentRestagingSkill`

入口：

- `docking/dock_skill.py:_maybe_activate_vpcr()`

作用：

- 近场视觉丢失后，做 visibility-persistent restaging，而不是继续沿 nominal funnel 顶。

### 6.3 PTCC（对应 DOCK 5.8）

文件：

- `docking/time_compression.py`

类：

- `PlanCompressionCertifier`

入口：

- `docking/dock_skill.py:_apply_plan()`
- `docking/dock_skill.py:compute_commands()` 内部的 `progress_compress / approach_compress`

作用：

- 对 `SC / EC` 代表场景做计划条件触发的 corridor speed compression。

---

## 7. 当前恢复实现与 DOCK.md 的对应关系

### 7.1 当前恢复的是哪个版本？

当前代码已手动恢复到 `artifacts/lc_round18_lc12_regression/lc_round18_lc12_regression.md` 对应的主行为口径，具体见：

- `DOCK.md:814`

恢复含义：

- 保留 `UnifiedCertificateField + StagingCertificateOptimizer`
- `sigma_post` 仍在 planning 侧作为 soft bias
- 不再让 `LEADER_POST_ALIGN` 成为成功锁定的强门槛
- 恢复 Round-18 的 LC 行为：能成功的场景继续成功，即使终态姿态仍不完美

### 7.2 恢复验证结果

输出：

- `output/round18_restore2/round18_restore_summary.csv`
- `output/round18_restore2/round18_restore_conclusion.md`
- `output/round18_restore2/round18_restore_vs_artifact.md`

关键场景对齐：

- `DBv1-LC-L1-073`：恢复为 `collision_obstacle`
- `DBv1-LC-L2-081`：恢复为 `success=True`
- `DBv1-LC-L2-083`：恢复为 `success=True`

---

## 8. 单例测试 / 全量测试 / 对比实验 / 消融实验 / 可视化

### 8.1 单例测试

单场景最直接的命令：

```bash
python scripts/run_p_minus1_stage1_docking.py \
  --scenario-json data/dockbench_v1/scenes/DBv1-LC-L2-081.json \
  --methods co_bcfd \
  --max-time-s 32 \
  --out-dir output/design_single/DBv1-LC-L2-081 \
  --skip-gifs
```

若要同时跑主方法和几个基线：

```bash
python scripts/run_p_minus1_stage1_docking.py \
  --scenario-json data/dockbench_v1/scenes/DBv1-LC-L2-081.json \
  --methods co_bcfd,T_hard_switch,T_lattice_pbvs,T_parking_hierarchical,T_coop_dist_blend \
  --max-time-s 32 \
  --out-dir output/design_compare_single/DBv1-LC-L2-081 \
  --skip-gifs
```

### 8.2 全量数据集测试

批量 full benchmark：

```bash
python scripts/run_p_minus1_stage2_batch.py \
  --dataset-root data/dockbench_v1 \
  --out-dir output/design_batch \
  --max-time-s 32 \
  --skip-gifs \
  --workers 4
```

产物包括：

- 成功率/碰撞率聚合
- family breakdown
- bootstrap 配对对比
- heatmap / subset bars / failure clusters / ablation bars

### 8.3 全量数据集对比实验

最常见的方式是：分别跑两个不同实现目录，再比较 CSV/JSON。

例如：

```bash
python scripts/run_p_minus1_stage2_batch.py --dataset-root data/dockbench_v1 --out-dir output/exp_A --max-time-s 32 --skip-gifs --workers 4
python scripts/run_p_minus1_stage2_batch.py --dataset-root data/dockbench_v1 --out-dir output/exp_B --max-time-s 32 --skip-gifs --workers 4
```

再对比：

- `output/exp_A/...summary...`
- `output/exp_B/...summary...`

若只看代表场景，可用：

```bash
python scripts/run_p_minus1_stage1_suite.py \
  --dataset-root data/dockbench_v1 \
  --out-dir output/design_suite \
  --max-time-s 32 \
  --skip-gifs
```

### 8.4 消融实验

单例消融：

```bash
python scripts/run_p_minus1_stage1_docking.py \
  --scenario-json data/dockbench_v1/scenes/DBv1-LC-L2-081.json \
  --methods co_bcfd,A_no_stage,A_no_belief_gate,A_no_funnel_gate,A_no_micro_maneuver,A_no_corridor_reciprocity,A_no_lc_hybrid_search \
  --max-time-s 32 \
  --out-dir output/design_ablation/DBv1-LC-L2-081 \
  --skip-gifs
```

全量批量时，`scripts/run_p_minus1_stage2_batch.py` 会自动把：

- `BATCH_FULL_METHOD_IDS`
- `CORE_ABLATION_METHOD_IDS`

一起聚合进去。

### 8.5 特定场景案例可视化

最直接的方式是不加 `--skip-gifs`：

```bash
python scripts/run_p_minus1_stage1_docking.py \
  --scenario-json data/dockbench_v1/scenes/DBv1-LC-L2-083.json \
  --methods co_bcfd \
  --max-time-s 32 \
  --out-dir output/design_viz/DBv1-LC-L2-083
```

这会在输出目录生成：

- `p_minus1_stage1_results_seed....json`
- `P_MINUS1_STAGE1_REPORT_seed....md`
- GIF（通过 `docking/visualization.py:save_docking_animation()`）

若要查看更通用的诊断图：

```bash
python scripts/visualize_docking_diagnostics.py
```

它会导出：

- `artifacts/docking_diagnostics.png`
- `artifacts/docking_process.gif`

### 8.6 如何做 failure-mode 分层统计

最直接的数据源是：

- 单例 JSON：`p_minus1_stage1_results_seed*.json`
- 批量 CSV/JSON：如 `output/round18_restore2/round18_restore_summary.csv`

按字段：

- `success`
- `reason`
- `failure_category`
- `collision`

即可得到：

- `collision_obstacle`
- `collision_vehicle`
- `lc_geometric_deadlock`
- `timeout`
- `locked`

这些字段正是 `DOCK.md` 第 6、7、9、10 节中 failure-mode 分层统计所要求的证据。

---

## 9. 设计阅读建议

如果想从“代码”理解 `DOCK.md`，建议按下面顺序读：

1. `scripts/run_p_minus1_stage1_docking.py`
2. `docking/p_minus1_baselines.py`
3. `docking/dock_skill.py`
4. `docking/coop_docking.py`
5. `docking/lc_corridor.py`
6. `docking/stage25_subskills.py`
7. `docking/time_compression.py`
8. `docking/bcfd.py`
9. `docking/controllers.py` / `docking/collision.py` / `docking/sensors.py`

这条顺序与 `DOCK.md` 的理论组织是一致的：

- 先看闭环怎么跑；
- 再看方法工厂怎么组；
- 再看总状态机怎么调度；
- 最后再看每一张证书、每一个子技能和每一个底层控制器。
