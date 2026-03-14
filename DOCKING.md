# DOCKING.md

> 状态：**改进设计稿 / 未冻结**。  
> 作用：在不直接改写 `DOCK.md` 的前提下，先把下一版 Docking Skill 的理论与工程框架完整重写，再按 `DOCKING_TASKBOOK.md` 逐项实验验证；只有通过验证后，才允许回写并固化为新的 `DOCK.md`。

# 1. 设计目标与当前缺口

当前 `DOCK.md` 已经把 `belief-consistent fusion + capture funnel + cooperative staging` 组织成一个可运行框架，但仍存在 5 个结构性缺口：

1. **固定加权和依赖人工调参**：当前 `\sigma` 场本质上仍是手工权重的经验组合，难以证明在不同 family、不同 mode、不同风险水平下始终给出正确权衡。
2. **证书集不完备**：现有 `\sigma_P / \sigma_T / \sigma_V / \sigma_C / \sigma_{post}` 能解释部分失效，但尚未形成一组对主要 failure modes 具有覆盖性的最小完备证书集。
3. **证书大多仍是近似几何判据**：许多运行时分数是“看起来合理”的启发式指标，而不是由保守可达集、控制不变集、robust tube 或 barrier 条件直接诱导出来的形式化证书。
4. **模式切换仍偏阈值门控**：切换守卫缺少统一的 guard / reset / dwell-time 定义，难以证明无抖振、无错误放行、无 mode-level feasibility break。
5. **缺少端到端组合定理**：当前更多是局部模块各自合理，还没有“从初始 certified set 出发，闭环系统最终要么锁定成功、要么安全拒绝”的系统级保证。

本设计稿的目标不是在现有代码上继续增补 patch，而是把整个 Docking Skill 重写为一个**可证明的混合证书系统**：

- 证书层：从 failure mode 出发，构造**最小完备证书集**；
- 求解层：从固定加权和升级为**证书约束下的分层多目标优化**；
- 切换层：把经验阈值改写为**混合系统 guard / reset / dwell-time**；
- 证明层：给出**模块级保证 → mode 级拼接 → 系统级组合定理**；
- 工程层：把历史外挂模块全部解释为统一证书系统中的实现子件，而不再保留理论上彼此割裂的命名。

---

# 2. 设计总纲：从 Co-BCFD 升级到 CCHD

本稿将下一版 Docking Skill 暂命名为：

**CCHD — Certificate-Complete Hybrid Docking**

它保留 `DOCK.md` 中已经成立的三条主线：

1. **Belief-consistent**：GNSS / vision 融合必须受一致性门控；
2. **Capture-aware**：对接不能只看末端位置误差，必须显式建模 capture basin / terminal manifold；
3. **Cooperative**：Leader 配合动作是 skill 内部动作，而不是上层临时补丁。

但在理论对象上，CCHD 不再把 `CGFL / TVT / VPCR / PTCC / sigma_C / sigma_M / sigma_R / macro primitives` 当作并列模块，而是统一为三层对象：

1. **证书对象（certificate objects）**：定义系统在哪些集合内“安全 / 可见 / 可达 / 可交接 / 可闭合 / 可后续行进”；
2. **模式合同（mode contracts）**：定义每个 mode 的 active certificate bundle、允许流动的 flow set、允许切换的 jump guard、切换后的 reset condition；
3. **证书优化器（certificate optimizer）**：在当前 mode 的证书约束下，选择最优控制、最优 staging、最优 witness primitive。

因此，历史模块将被重解释为：

- `CGFL / PTCC`：属于**性能层**，只影响二级/三级目标，不再决定系统可行性；
- `TVT / sigma_M`：属于**终端闭合证书**的不同实现器；
- `VPCR`：属于**可见性证书**触发下的恢复分支；
- `sigma_C / LC-CRBS / macro primitives`：属于**可达/交接证书**下的 witness 生成器；
- `Release Gate / Lock Hold`：属于**handover / terminal closure 合同**的一部分。

---

# 3. 研究启发与理论参考

本设计不直接复用现成方法，而是吸收以下研究脉络中的可证明结构：

## 3.1 安全集与不变性
- Blanchini, “Set Invariance in Control,” *Automatica*, 1999.  
  启发：把“安全”从单步几何检测提升为控制不变集问题。  
  链接：<https://doi.org/10.1016/S0005-1098(99)00113-2>
- Ames et al., “Control Barrier Function Based Quadratic Programs for Safety Critical Systems,” *IEEE TAC*, 2017.  
  启发：把运行时安全约束写成 barrier inequality，而不是碰撞后再拒绝动作。  
  链接：<https://authors.library.caltech.edu/records/jnhr0-1ww05>

## 3.2 鲁棒可行性与误差管
- Mayne, Seron, Raković, “Robust Model Predictive Control of Constrained Linear Systems with Bounded Disturbance,” *Automatica*, 2005.  
  启发：执行偏差应通过 tube / disturbance set 纳入证书，而不是事后以 heuristic retry 补救。  
  链接：<https://doi.org/10.1016/j.automatica.2004.08.019>
- Majumdar, Tedrake, “Funnel Libraries for Real-Time Robust Feedback Motion Planning,” *IJRR*, 2017.  
  启发：用 funnel / robust reachable tube 表达 mode 内可闭合区域，比纯采样评分更适合做理论对象。  
  链接：<https://doi.org/10.1177/0278364917712421>

## 3.3 可达性与 viability
- Aubin, *Viability Theory*, 1991 / 2009 edition.  
  启发：任务“可解”应被写成 viability kernel 问题，证书是 kernel 的保守内逼近，而不是经验打分。  
  链接：<https://link.springer.com/book/10.1007/978-3-642-16684-6>
- Mitchell, Bayen, Tomlin, “A Time-Dependent Hamilton–Jacobi Formulation of Reachable Sets for Continuous Dynamic Games,” *IEEE TAC*, 2005.  
  启发：reach / avoid 证书可由 backward reachable tube 诱导。  
  链接：<https://doi.org/10.1109/TAC.2005.851439>

## 3.4 混合系统与切换合同
- Goebel, Sanfelice, Teel, *Hybrid Dynamical Systems*, Princeton University Press, 2012.  
  启发：mode 切换应写成 hybrid inclusion，必须显式定义 flow set、jump set、reset map。  
  链接：<https://press.princeton.edu/books/hardcover/9780691153890/hybrid-dynamical-systems>
- Hespanha, Morse, “Stability of Switched Systems with Average Dwell-Time,” *CDC*, 1999.  
  启发：抖振抑制不能只靠经验 hysteresis，应有 dwell-time / average dwell-time 条件。  
  链接：<https://doi.org/10.1109/CDC.1999.831330>

## 3.5 组合式保证
- Nilsson, Ozay, Liu, “Incremental Synthesis of Switching Protocols via Barrier Functions,” *HSCC*, 2021.  
  启发：分模式 barrier / contract 可以组合成系统级 switching 保证。  
  链接：<https://doi.org/10.1145/3447928.3456646>
- Ghasemi et al., “Compositional Verification and Synthesis of Stochastic Hybrid Systems using Barrier Certificates,” *IEEE TAC*, 2024.  
  启发：组合式 barrier 思想可用于把多个局部证书拼成系统级结论。  
  链接：<https://doi.org/10.1109/TAC.2023.3346868>

这些工作只提供理论抓手；本设计中的证书集、mode 合同和多证书优化器均针对当前 docking / LC 问题重新构造，不直接照搬现成算法。

---

# 4. 问题形式化与失败模式全集

## 4.1 状态、模式与环境参数

定义连续状态：
\[
\chi = [x_F, x_L, b, e, q, \tau]^\top
\]
其中：
- \(x_F, x_L\)：Follower / Leader 车辆状态；
- \(b\)：belief state（含 leader hitch belief 与协方差）；
- \(e\)：planning-tracking mismatch；
- \(q\)：离散 mode；
- \(\tau\)：当前 mode 已停留时间。

环境上下文记为 \(\gamma\)，包括：
- 障碍几何；
- lane / corridor 几何；
- dominant direction；
- 可视域、LOS、dock-zone、post-dock exit direction；
- 允许的 leader cooperation budget。

## 4.2 任务级失败模式全集

我们把当前 docking 的关键 failure modes 冻结为：

\[
\mathfrak F = \{f_{col}, f_{lane}, f_{vis}, f_{dead}, f_{exec}, f_{handover}, f_{lock}, f_{post}\}
\]

分别表示：
1. \(f_{col}\)：障碍/车车碰撞；
2. \(f_{lane}\)：越出可行 corridor / 违反 lane fidelity；
3. \(f_{vis}\)：视觉长期失效且不可恢复；
4. \(f_{dead}\)：几何死锁 / basin coverage 不足；
5. \(f_{exec}\)：规划可行但执行偏差使证书失效；
6. \(f_{handover}\)：错误释放 / follower 接管后立即失稳；
7. \(f_{lock}\)：进入 docking 后无法完成 terminal closure；
8. \(f_{post}\)：对接成功但对接后前方机动空间不足、姿态不利、系统被困。

**设计要求**：下一版证书集必须对 \(\mathfrak F\) 完全覆盖。所谓“完全覆盖”不是指所有场景都必须成功，而是指：
- 只要 failure 属于 \(\mathfrak F\)，就必须能被某张证书提前预警或解释；
- 如果某次运行既没有成功、又没有触发任何证书缺口，则证书集视为不完备。

---

# 5. 最小完备证书集

## 5.1 证书对象的统一定义

对每个 mode \(q\) 与每类任务属性 \(i\)，定义证书对象
\[
\mathcal C_{q,i} = (\underline{\mathcal V}_{q,i}, h_{q,i}, \rho_{q,i}, \mathcal W_{q,i})
\]
其中：
- \(\underline{\mathcal V}_{q,i}\)：保守可证集合（inner approximation）；
- \(h_{q,i}(\chi)\)：barrier / Lyapunov / value margin；
- \(\rho_{q,i}(\chi)\)：误差膨胀项（belief、tracking、离散化余量）；
- \(\mathcal W_{q,i}\)：用于证明可收缩/可交接/可恢复的 witness 库。

统一运行时 margin 定义为
\[
\sigma_{q,i}(\chi) = h_{q,i}(\chi) - \rho_{q,i}(\chi).
\]
若 \(\sigma_{q,i}(\chi) \ge 0\)，则表示当前状态位于对应证书的保守可证集内。

这一定义有两个关键含义：
1. **证书不再等于 heuristic score**，而是等于“保守 margin”；
2. **近似证书合法化**：只要 \(\underline{\mathcal V}_{q,i}\) 是 exact viable set 的保守内逼近，则 \(\sigma_{q,i}\ge 0\) 仍是 sound 的。

## 5.2 七张最小完备证书

### 5.2.1 安全证书 \(\sigma_S\)

作用：覆盖 \(f_{col}\) 与 \(f_{lane}\)。

定义安全可证集：
\[
\underline{\mathcal V}_{S} = \left\{\chi:\ \min_{t\in[0,H_S]} d_{obs}(\chi_t) \ge d_{min}+\Delta_{trk}(t),\ \chi_t\in\Gamma_{drive},\ \forall t\in[0,H_S] \right\}
\]
其中：
- \(d_{obs}\)：对障碍/对车的 swept-volume 净空；
- \(\Delta_{trk}\)：跟踪误差管与离散化余量；
- \(\Gamma_{drive}\)：可行驶区域 / corridor / lane graph 诱导的 driving set。

运行时 margin：
\[
\sigma_S(\chi)=\min_{t\in[0,H_S]}\Big(d_{obs}(\chi_t)-d_{min}-\Delta_{trk}(t),\ d_{lane}(\chi_t)\Big).
\]

解释：
- 若 \(\sigma_S<0\)，则安全动作不存在或当前预测已越出保守安全集；
- 安全层只能在 \(\sigma_S\ge 0\) 的控制集合内做优化。

### 5.2.2 可见性证书 \(\sigma_V\)

作用：覆盖 \(f_{vis}\)。

定义观测可证集：
\[
\underline{\mathcal V}_{V} = \left\{\chi:\ \mathrm{LOS}=1,\ |\beta|\le \theta_{fov}/2-\Delta_\beta,\ d\le d_{vis}-\Delta_d,\ P_{reacq}\ge \rho_{reacq}\right\}
\]

运行时 margin：
\[
\sigma_V(\chi)=\min\Big( h_{los}(\chi),\ h_{fov}(\chi),\ h_{dist}(\chi),\ h_{reacq}(\chi) \Big).
\]

其中 \(h_{reacq}\) 不是简单当前可见，而是“若此刻丢失，是否仍可在限定时间内通过 `VPCR`-style witness 重获可见”。

### 5.2.3 Basin / Reach 证书 \(\sigma_B\)

作用：覆盖 \(f_{dead}\)，即几何死锁、capture basin 不可达、corridor basin coverage 不足。

定义 reduced task state \(\zeta\)：
\[
\zeta = [r,\ \psi_F,\ \psi_L,\ s_\Gamma,\ \delta_\Gamma,\ \kappa_\Gamma]^\top
\]
其中包含相对误差、corridor 坐标、dominant direction / curvature 信息。

对每个 mode \(q\) 定义目标集 \(\mathcal T_q\)（例如 predock manifold、handover manifold、lock manifold），并构造 backward reachable inner tube
\[
\underline{\mathcal R}_{q}^{-}(\mathcal T_q).
\]

则 basin margin 定义为
\[
\sigma_B(\chi)=V_B^{thr}(q,\gamma)-\underline V_B(\zeta; q,\gamma),
\]
其中 \(\underline V_B\) 是 conservative reachability value function；\(\sigma_B\ge 0\) 表示存在一条保守 witness 能把当前状态送入下一 mode 的目标集。

### 5.2.4 执行一致证书 \(\sigma_E\)

作用：覆盖 \(f_{exec}\)。

定义 planning-tracking tube：
\[
\mathcal E_q = \{e:\ \|e\|_{P_q}\le \bar e_q\}
\]

执行一致 margin：
\[
\sigma_E(\chi)=\bar e_q - \|e\|_{P_q}.
\]

但仅这样还不够；需要把 tube lift 到 task certificate：
\[
\chi \in \underline{\mathcal V}_{q,i}^{lift}
\iff
\zeta \in \underline{\mathcal V}_{q,i}^{red} \ominus \Pi_\zeta \mathcal E_q.
\]

含义：规划证书只对“考虑了执行误差膨胀后仍为正”的状态有效；否则必须在 runtime reshaping / replanning 中重新求证，而不能沿原计划强推。

### 5.2.5 交接证书 \(\sigma_H\)

作用：覆盖 \(f_{handover}\)。

交接不是当前静态可行就行，而必须验证 release 之后 follower 在短时域内的动态安全与继续闭合能力。定义：
\[
\sigma_H(\chi)=\sup_{\pi_F\in\mathcal W_H(\chi)}\ \min_{t\in[0,H_H]}\Big( \sigma_S(\chi_t^{\pi_F}),\ \sigma_B(\chi_t^{\pi_F}),\ \sigma_E(\chi_t^{\pi_F}) \Big).
\]

解释：
- witness 库 \(\mathcal W_H\) 由 follower takeover primitives 与短时域 terminal plans 构成；
- 若 \(\sigma_H<0\)，即使当前看起来“站得住”，也不允许 Leader 释放。

这一步把历史上的 `release gate / early release / LC_LOCK_HOLD` 统一为同一证书对象。

### 5.2.6 终端闭合证书 \(\sigma_L\)

作用：覆盖 \(f_{lock}\)。

当前项目在 LC 中最大的难点不是“到了 predock 没”，而是“进入 docking 后能否完成 closure”。因此终端证书必须明确包含**收缩性**而不只是瞬时几何误差。

定义 terminal manifold 误差：
\[
\eta = [x_r, y_r, \psi_r, v_r]^\top.
\]

定义 contractive terminal Lyapunov function：
\[
V_L(\eta)=\eta^\top Q_L \eta.
\]

终端闭合证书为
\[
\sigma_L(\chi)=\sup_{\pi\in\mathcal W_L(\chi)}\ \min\Big(
\sigma_S(\chi^{\pi}),
\sigma_E(\chi^{\pi}),
V_L^{thr}-V_L(\eta_H^{\pi}),
V_L(\eta_0)-V_L(\eta_H^{\pi})-\delta_L
\Big).
\]

含义：必须存在一个短时域 witness，使 terminal Lyapunov 严格下降 \(\delta_L\)，同时不破坏安全/执行证书。

### 5.2.7 后对接机动证书 \(\sigma_{post}\)（新设计中保留 `\sigma_{post}` 记号，避免与旧 `\sigma_P` 的进展证书混淆）

作用：覆盖 \(f_{post}\)。

旧 `\sigma_{post}` 只看航向是否接近平行，过于弱；新证书必须直接回答“对接完成后是否还能沿任务方向继续运动”。

定义 post-lock exit tube 目标集 \(\mathcal T_{exit}(\gamma)\)，其含义是：锁定后的 pair 在限定控制预算内能沿环境主导方向离开当前 narrow region。

保守 exit-viability 集合记为
\[
\underline{\mathcal V}_{post}(\gamma)=\mathrm{BRT}^{-}(\mathcal T_{exit}(\gamma)).
\]

运行时 margin：
\[
\sigma_{post}(\chi)=h_{post}(\chi;\gamma)-\rho_{post}(\chi).
\]

在 `LC` 中可用以下可计算 surrogate 落地：
\[
\sigma_{post}^{LC}=\min\Big(
\cos(\psi_L-\psi_\Gamma)-\cos \bar\psi_{exit},
\frac{d_{front}-d_{front}^{min}}{d_{front}^{ref}},
\frac{\ell_{exit}-\ell_{exit}^{min}}{\ell_{exit}^{ref}}
\Big).
\]

因此，后对接机动证书不再只是“姿态更漂亮”，而是“后续任务可继续”。

## 5.3 最小完备性定义

定义责任映射 \(\mathfrak R\)：

| Failure mode | 责任证书 |
|---|---|
| \(f_{col}, f_{lane}\) | \(\sigma_S\) |
| \(f_{vis}\) | \(\sigma_V\) |
| \(f_{dead}\) | \(\sigma_B\) |
| \(f_{exec}\) | \(\sigma_E\) |
| \(f_{handover}\) | \(\sigma_H\) |
| \(f_{lock}\) | \(\sigma_L\) |
| \(f_{post}\) | \(\sigma_{post}\) |

称证书集 \(\Sigma=\{\sigma_S,\sigma_V,\sigma_B,\sigma_E,\sigma_H,\sigma_L,\sigma_{post}\}\) 对 failure family \(\mathfrak F\) **最小完备**，若满足：

1. **Soundness**：对任意 \(f\in\mathfrak F\)，若所有责任证书均保持非负，则 \(f\) 不可能发生；
2. **Anticipation**：若 \(f\) 发生，则在其发生前有限时间内，至少一张责任证书变为负或其 witness 集为空；
3. **Minimality**：删除任意一张证书后，至少存在一个 \(f\in\mathfrak F\) 不再被上述两条覆盖。

## 5.4 完备性命题（设计目标）

在假设：
- 静态障碍地图正确；
- 车辆动力学、tracking error bound、感知噪声界已知；
- leader cooperation budget 有界；
- corridor / lane geometry 已审计；
- witness primitive 库覆盖每个 mode 的本征短时域机动；

则目标证明如下：

**命题 A（任务失败覆盖）**：证书集 \(\Sigma\) 对 \(\mathfrak F\) 最小完备。  
**命题 B（certified solvability）**：若初始状态属于 success-certified set \(\mathcal X_0^{succ}\)，则闭环系统在有限混合时间内到达 `LOCKED`；若不属于 \(\mathcal X_0^{succ}\) 但属于 safe-certified set \(\mathcal X_0^{safe}\)，则系统最终进入 `SAFE_REJECT`，且全过程无碰撞、无错误释放。

这两个命题是新设计取代旧 `DOCK.md` 的核心门槛。

---

# 6. 从固定权重到分层多目标优化

## 6.1 旧问题

旧证书场本质是：
\[
\max_x \sum_i w_i \sigma_i(x)
\]
这会带来三个问题：
- 权重 \(w_i\) 很难跨 family / mode 复用；
- 安全、可见、交接等并不应与性能项放在同一层做 trade-off；
- 即使调出一个好结果，也难以解释为何在另一个场景失效。

## 6.2 新解法：三级优化结构

下一版优化器采用**三级证书优化**：

### Level-1：硬约束层（不可妥协）
\[
\sigma_i(\chi,u) \ge 0,
\quad i\in\mathcal I_{hard}(q)=\{S,V,E\} \cup \text{mode-required}\{H,L\}
\]

### Level-2：可解性层（优先保证存在下一步）
在硬约束可行集内，最大化 basin / handover / closure 的 slack：
\[
\max_{u,\pi}\ \Big( \sigma_B,\ \sigma_H,\ \sigma_L \Big)_{lex}
\]
这里使用 **lexicographic order**，而不是固定加权和。

### Level-3：性能层（只有在前两层满足后才优化）
\[
\min_{u,\pi}\ J_{perf}=J_{time}+J_{energy}+J_{smooth}+J_{post}
\]
其中
\[
J_{post} = -\sigma_{post}.
\]

因此：
- 安全与执行一致性不会被“航向更平行”之类的软目标牺牲；
- post-docking mobility 只有在可行前提下才参与优选；
- `CGFL / PTCC` 等快车道逻辑只能出现在 Level-3，不能干扰 Level-1/2。

## 6.3 状态依赖自适应权重

在 Level-3 内仍可能存在多个性能子目标。这里不使用固定常数权重，而使用**对偶变量诱导的在线权重**：

首先求解 slack feasibility problem：
\[
\min_{u,s\ge 0} \ \mathbf 1^\top s + \eta\|u-u_{ref}\|^2
\]
\[
\text{s.t.}\quad \dot h_i(\chi,u)+\alpha_i(h_i(\chi))\ge -s_i,
\quad i\in\mathcal I_{soft-active}(q).
\]

记最优对偶变量为 \(\lambda_i^\star\)，则定义性能层自适应权重：
\[
\omega_i(\chi)=\frac{\exp(\beta\lambda_i^\star)}{\sum_{j}\exp(\beta\lambda_j^\star)}.
\]

含义：哪一类证书当前更“紧”，其性能相关目标就自动获得更高权重。这样做的本质是：
- **不是人手给常数权重**；
- 权重由当前状态、当前 mode、当前证书张力在线决定；
- 同一个策略在 LC / CF / SC / EC 下可闭环自调度。

---

# 7. 混合系统切换律

## 7.1 Mode 集合

定义离散模式
\[
\mathcal Q = \{\text{APPROACH},\ \text{STAGE},\ \text{PREDOCK},\ \text{DOCKING},\ \text{LOCK\_ASSIST},\ \text{POST\_ALIGN},\ \text{LOCKED},\ \text{SAFE\_REJECT}\}.
\]

`LC` 不是额外理论分支，而是通过 \(\gamma\) 与 active witness 库影响上述 mode 中的证书与优化器；因此不再允许 `if scenario == LC` 直接决定理论流程。

## 7.2 每个 mode 的合同

对每个 mode \(q\)，定义：
- flow set \(C_q\)：证书持续为正且未满足 jump 条件的状态；
- jump set \(D_{q\to q'}\)：满足切换守卫的状态；
- reset map \(G_{q\to q'}\)：切换时更新参考目标、belief tube、witness index；
- dwell-time \(\tau_q^{min}\)：最短停留时间；
- hysteresis margin \((\epsilon_i^{enter},\epsilon_i^{exit})\)。

形式化写为：
\[
\mathcal H:
\begin{cases}
\dot \chi \in F_q(\chi), & \chi\in C_q \\
\chi^+ \in G_{q\to q'}(\chi), & \chi\in D_{q\to q'}
\end{cases}
\]

## 7.3 守卫条件

切换 `q -> q'` 的充要守卫写成：
\[
D_{q\to q'} = \left\{ \chi:\ \sigma_i(\chi)\ge \epsilon_i^{q\to q'},\ \forall i\in\mathcal G_{q\to q'},\ \tau_q\ge \tau_q^{min} \right\}
\]

例如：
- `STAGE -> PREDOCK`：要求 \(\sigma_S,\sigma_V,\sigma_B,\sigma_E\) 均为正；
- `LOCK_ASSIST -> LOCKED`：要求 \(\sigma_S,\sigma_E,\sigma_L\) 均为正；
- `LEADER_RELEASE -> FOLLOWER_TAKEOVER`：必须 \(\sigma_H\ge 0\)；
- 任意 mode -> `SAFE_REJECT`：当硬证书连续失效且无恢复 witness 时触发。

## 7.4 reset 相容性条件

为了使 mode 级拼接成立，必须证明：
\[
G_{q\to q'}(D_{q\to q'}) \subseteq \underline{\mathcal V}_{q',0}
\]

即：一旦某个 jump guard 满足，reset 后的状态必然落在新 mode 的初始 certified set 内。否则该切换不可被接受，即使经验上“好像能跑通”。

## 7.5 无抖振条件

采用双阈值 + dwell-time：
\[
\epsilon_i^{enter} > \epsilon_i^{exit},
\qquad
\tau_q \ge \tau_q^{min}.
\]

目标证明：
- mode 不会因观测小抖动来回切换；
- 不会出现“刚 release 又立即反悔”的 contract violation；
- 在 `POST_ALIGN`、`LOCK_ASSIST` 这类局部 mode 中，不会因小幅误差震荡导致死循环。

---

# 8. 端到端闭环性质

## 8.1 两级初始集合

定义：
\[
\mathcal X_0^{safe}=\{\chi_0:\ \sigma_S(\chi_0),\sigma_E(\chi_0)\ge 0\}
\]
\[
\mathcal X_0^{succ}=\{\chi_0\in\mathcal X_0^{safe}:\ \exists\ \text{guard-compatible mode sequence to }\text{LOCKED}\}
\]

二者区分了两类保证：
- `safe-certified`: 无法承诺必然成功，但可承诺安全拒绝；
- `success-certified`: 可承诺最终闭合成功。

## 8.2 目标定理

### 定理 1：Mode 内安全与可证流动
若 \(\chi(0)\in C_q\cap \underline{\mathcal V}_{q}\)，且优化器始终输出满足 Level-1 约束的控制，则在离开 \(C_q\) 之前：
- \(\sigma_S, \sigma_E\) 保持非负；
- 与当前 mode 相关的必要证书不为负；
- 若无法继续保持，则必先进入某个 jump set 或 `SAFE_REJECT`。

### 定理 2：Failure-mode 覆盖
若某次运行发生 \(f\in\mathfrak F\)，则在 \(f\) 发生前，责任证书 \(\mathfrak R(f)\) 中至少一张证书变负或 witness 为空。不存在“所有证书都为正但 failure 仍无征兆发生”的 silent failure。

### 定理 3：混合拼接安全性
若所有 jump 均满足 reset 相容性，且系统 obeys dwell-time / hysteresis，则最大解不存在 Zeno 切换；系统要么前进到下一 certified mode，要么进入 `SAFE_REJECT`。

### 定理 4：端到端闭环结论
若 \(\chi_0\in\mathcal X_0^{succ}\)，则系统在有限混合时间内到达 `LOCKED`；若 \(\chi_0\in\mathcal X_0^{safe}\setminus\mathcal X_0^{succ}\)，则系统有限时间内进入 `SAFE_REJECT`，且全过程保持 \(\sigma_S\ge 0\)、无错误 release、无 terminal collision。

这一定理是本设计最终取代 `DOCK.md` 的必要条件。

---

# 9. 历史模块的统一吸收方式

下表给出新理论对象如何吸收历史命名：

| 历史对象 | 新理论归属 | 保留方式 |
|---|---|---|
| `CGFL` | Level-3 性能优化 | 只保留为 fast-lane policy 的实现标签 |
| `PTCC` | Level-3 性能优化 | 只作为 time-compression 实现，不单独占理论章节 |
| `TVT` | `\sigma_L` / `\sigma_B` 的 witness 生成器 | 作为 terminal tube library |
| `VPCR` | `\sigma_V` 的恢复 witness | 作为 visibility recovery 分支 |
| `sigma_C` | 吸收到 `\sigma_B + \sigma_H` | corridor reciprocity 属于 reach/handover 的几何上下文 |
| `sigma_M` | 吸收到 `\sigma_L` | terminal manifold closure 的 contractive 证书 |
| `sigma_R` | 吸收到 `\sigma_H` | release 安全性是 handover 证书的一部分 |
| macro primitives | witness 库 `\mathcal W_{q,i}` | 不再单独叙述成“理论模块” |
| `LEADER_POST_ALIGN` | `\sigma_{post}` 的执行层改善尝试 | 只能作为低优先级后处理，不可反向破坏 `\sigma_L` |

因此，新文档不再接受“某 case 失败 → 新增一个命名模块”的迭代方式；任何新增实现都必须先声明它属于哪张证书、哪个 mode 合同、哪个 witness 库。

---

# 10. 代码组织重构建议

为了避免继续在 `docking/lc_corridor.py` 和 `docking/dock_skill.py` 中堆叠 patch，下一版建议按以下结构组织：

## 10.1 证书层
- `docking/certificates/base.py`：证书对象基类、margin 接口、normalization、contract registry。
- `docking/certificates/safety.py`：`\sigma_S`，含 swept-volume、lane fidelity、robust clearance。
- `docking/certificates/visibility.py`：`\sigma_V`，含 LOS/FOV/reacquisition。
- `docking/certificates/reachability.py`：`\sigma_B`，含 reduced kernel / basin witness。
- `docking/certificates/execution.py`：`\sigma_E`，含 tube mismatch、plan-lift margin。
- `docking/certificates/handover.py`：`\sigma_H`。
- `docking/certificates/terminal.py`：`\sigma_L`。
- `docking/certificates/postdock.py`：`\sigma_{post}`。

## 10.2 mode 合同层
- `docking/hybrid/contracts.py`：mode 定义、guard、reset、dwell-time。
- `docking/hybrid/runtime.py`：hybrid supervisor，负责 flow/jump 执行。

## 10.3 求解层
- `docking/optim/certificate_scheduler.py`：三级优化器、lexicographic solver、dual-induced weight。
- `docking/optim/witness_library.py`：primitive / terminal tube / vpcr witness / corridor witness 的统一接口。
- `docking/optim/staging_optimizer.py`：只作为 `\sigma_B / \sigma_H / \sigma_{post}` 的求解器，不再出现“按场景 if-else”。

## 10.4 技能层
- `docking/dock_skill.py`：只保留技能入口、hybrid runtime 装配、日志输出；不再承载大量几何细节。
- `docking/lc_corridor.py`：退化为 `LC` 几何上下文与 witness provider，而非整套 LC 特化状态机。

## 10.5 分析层
- `docking/analysis/failure_taxonomy.py`：failure mode 归因。
- `docking/analysis/certificate_audit.py`：证书日志、coverage 验证、completeness 审计。
- `docking/analysis/postdock_metrics.py`：后对接 mobility 统计。

---

# 11. 实验与证明的最低验收口径

本设计在没有通过以下口径前，不允许替换当前 `DOCK.md`：

1. **单案复现**：每张证书都要有对应代表案例，能证明“证书缺失时失败、加入后 failure 转移或消失”；
2. **frozen 全量测试**：所有 family 都要回归，`LC` 是主战场，`CF/SC/FC/EC` 是守护集；
3. **消融验证**：每张证书、每个 guard、每类 witness 都要有独立 ablation；
4. **failure-mode 分层统计**：必须证明 failure 不只是数量变化，而是结构上被正确重分布；
5. **证明闭环**：至少给出 mode 内 soundness、切换相容性、failure coverage、safe-reject / success-certified 四类定理与证明草稿。

具体执行顺序见 `DOCKING_TASKBOOK.md`。

---

# 12. 与当前 `DOCK.md` 的关系

- `DOCK.md`：当前稳定实现的设计基线；
- `DOCKING.md`：下一版理论重构草案；
- `DOCKING_TASKBOOK.md`：把 `DOCKING.md` 拆解成可逐轮验证、可接受/可抛弃的实验任务。

**规则**：只有当 `DOCKING_TASKBOOK.md` 中的理论与实验 gate 全部通过后，才允许把 `DOCKING.md` 的内容回写并冻结到新的 `DOCK.md`。
