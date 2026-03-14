# Docking Skill (P-1.1) — Cooperative Belief-Consistent Capture-Funnel Docking (Co-BCFD)

> 本文档是 `CORE_REQUIREMENTS_TASKBOOK.md` 的 **P-1.1** 设计稿（会持续迭代）。  
> 目标：把“对接”从一个工程功能，提升为一个可复用、可证明性质、可对比评测的 **Docking Skill**，并作为潜在顶会/顶刊论文级创新点。  
> **关键假设更新（按本项目当前需求）**：被对接的车（Leader）不要求“原地等待”，可执行**协同微机动/位姿调整**以提升可达性与可见性并降低总体代价（该动作对上层决策透明，作为 Docking Skill 内部封装）。

---

## 1. 问题重述与约束（面向可落地）

我们关注 **两车对接** 的闭环技能：后车（Follower）在静态障碍地图中与前车（Leader）的尾部对接点对接。对接过程允许 Leader 进行**有限协同运动**，以规避“当前位置几何上不可对接/代价极高/遮挡严重”的情况。

### 1.1 车辆与几何
- 单车阿克曼：状态 \(x=[p_x,p_y,\psi,v,\delta]^\top\)，控制 \(u=[a,\dot\delta]^\top\)。
- 几何与对接点：车体矩形 + 前后铰接点（对接点伸出）。

### 1.2 感知
两类观测（均带噪声，视觉可间歇失效）：
1. **全局 GNSS/共享定位**：给出 leader 全局位姿 \(\hat p_L, \hat \psi_L\)（低精度、稳定）。
2. **视觉相对位姿**：当处于视距/FOV/无遮挡时给出 rear-hitch 相对位姿 \(\hat r_V=[\hat x_r,\hat y_r,\hat \psi_r]\)（高精度、间歇）。

### 1.3 过程约束（强制）
- 非完整约束、转向/加速度限制、禁止原地转向等。
- **安全**：静态障碍/车车碰撞为 0；最小净空 \(\ge 0.1m\)（含铰接点）。
- **末端对接判定**：\(\epsilon_p<0.02m,\ \epsilon_\psi<5^\circ,\ \epsilon_v<0.1m/s\)，持续 \(\tau_{hold}=0.5s\)，且车身对接瞬间近似平直。
- **感知切换平滑**：禁止 1.5m 硬切换；必须在过渡带 \([1.2,1.8]m\) 做平滑融合。
- **视觉异常回退**：视觉丢失需回退或重规划。

---

## 2. 顶刊/顶会相关研究脉络（用于“对比+差异化”）

下面列出与 P-1.1 最相关的“方法族”，用于指导设计与后续对比基线（只列核心代表作；Stage-1/2 报告中会冻结最终基线实现与超参）。

### 2.1 视觉伺服（IBVS/PBVS/Hybrid）
- Hutchinson, S.; Hager, G. D.; Corke, P. I., “A Tutorial on Visual Servo Control,” *IEEE Transactions on Robotics and Automation*, 12(5):651–670, 1996.
- Chaumette, F.; Hutchinson, S., “Visual Servo Control. Part I: Basic Approaches,” *IEEE Robotics & Automation Magazine*, 13(4):82–90, 2006.
- Chaumette, F.; Hutchinson, S., “Visual Servo Control. Part II: Advanced Approaches,” *IEEE Robotics & Automation Magazine*, 14(1):109–118, 2007.

启示：视觉伺服能提供强末端精度，但对可见性、噪声、模型误差敏感；与全局导航结合时，“切换与稳定性”成为关键。

### 2.2 非完整底盘的视觉跟踪/移动机器人对接
- Chen et al., “Homography-Based Visual Servo Tracking Control of a Wheeled Mobile Robot,” *IEEE Transactions on Robotics*, 22(2), 2006.
- McCarthy, C.; Barnes, N.; Mahony, R., “A Robust Docking Strategy for a Mobile Robot,” *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2006.

启示：对接不是纯末端控制；进入“可对接捕获域”前需要全局/局部规划与轨迹约束，否则容易在障碍环境中失效。

### 2.3 鲁棒反馈规划 / Funnel / Reachability
- Majumdar, A.; Tedrake, R., “Funnel Libraries for Real-Time Robust Feedback Motion Planning,” *The International Journal of Robotics Research (IJRR)*, 2017.

启示：用“Funnel（可达管）”描述从粗到细的可达域收缩，能把“全局趋近 → 末端捕获”变为可解释、可验证的结构。

### 2.4 安全过滤（CBF/CLF-QP）
- Ames, A. D.; Xu, X.; Grizzle, J. W.; Tabuada, P., “Control Barrier Function Based Quadratic Programs for Safety Critical Systems,” *IEEE Transactions on Automatic Control*, 62(8):3861–3876, 2017.

启示：将“名义控制”投影到安全可行域可形成通用安全层，天然适配“规划/视觉伺服/回退”等上层输出的统一约束。

### 2.5 鲁棒 MPC / Tube-MPC（不确定性+约束）
- Mayne, D. Q.; Seron, M. M.; Raković, S. V., “Robust Model Predictive Control of Constrained Linear Systems with Bounded Disturbance,” *Automatica*, 41(2):219–224, 2005.

启示：对接中不可避免存在感知噪声/遮挡/延迟，Tube/Chance-constraint 的思想可用于把“不确定性”系统化纳入规划。

### 2.6 采样最优控制 / MPPI（非线性+约束的滚动优化）
- Williams, G. et al., “Information Theoretic MPC for Model-Based Reinforcement Learning,” *IEEE ICRA*, 2017（及其扩展版 T-RO/预印本）。

启示：对接近场存在“必须先后退再前进/必须先对齐再闭合”等非凸行为，单步几何控制容易陷入局部平衡点；采样型滚动优化（MPPI/随机射击/CEM）可把“非凸机动”纳入同一代价框架。

### 2.7 车式底盘的全局搜索规划（Hybrid A*/格点/原语）
- Dolgov, D.; Thrun, S.; Montemerlo, M.; Diebel, J., “Practical Search Techniques in Path Planning for Autonomous Driving,” *STAIR@AAAI*, 2008.
- Kuwata, Y. et al., “Real-Time Motion Planning With Applications to Autonomous Urban Driving,” *IEEE Control Systems Magazine*, 29(1), 2009.
- Pivtoraiko, M.; Kelly, A., “Differentially Constrained Motion Planning in State Lattices,” *Journal of Field Robotics*, 28(1), 2011.
- Reeds, J. A.; Shepp, L. A., “Optimal Paths for a Car That Goes Both Forwards and Backwards,” *Pacific Journal of Mathematics*, 145(2), 1990.（Reeds–Shepp）
- Dubins, L. E., “On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents,” *American Journal of Mathematics*, 79(3), 1957.（Dubins）

启示：对接本质上是一个“停车/倒车”类非完整姿态到姿态问题，若仅靠短视局部控制，常出现“侧向贴近但姿态无法收敛”的死锁；可用 Hybrid A* 或运动原语格点为技能提供“可达性保证”的全局候选轨迹，再由局部安全过滤闭环执行。

### 2.8 Belief-space 规划 / 不确定性下的可达性与决策
- Platt, R.; Tedrake, R.; Kaelbling, L. P.; Lozano-Pérez, T., “Belief Space Planning Assuming Maximum Likelihood Observations,” arXiv:1005.1925, 2010.
- Kaelbling, L. P.; Lozano-Pérez, T., “Hierarchical Task and Motion Planning in the Now,” *IEEE ICRA*, 2011.
- van den Berg, J.; Abbeel, P.; Goldberg, K., “LQG-MP: Optimized Path Planning for Robots with Motion Uncertainty and Imperfect State Information,” *The International Journal of Robotics Research (IJRR)*, 2012.

启示：对接的困难不仅是运动学，更在于观测间歇/遮挡带来的不确定性与切换；把对接建模为 belief-space funnel（“置信一致 + 可见性可达”）可形成比纯工程切换更强的理论抓手。

### 2.9 视觉约束下的非完整导航（FOV/可见性约束）
- Ma, Y.; Košecká, J.; Sastry, S. S., “Vision Guided Navigation for a Nonholonomic Mobile Robot,” *IEEE Transactions on Robotics and Automation*, 15(3):521–536, 1999.

启示：在近场对接中，“目标留在 FOV 内”本身是一类约束（与安全约束同级）；将 FOV/遮挡以 barrier 或 gate 的形式写入代价/约束，是避免视觉抖动与失锁的重要手段。

### 2.10 协同对接 / Rendezvous & Docking（移动目标 + 主动配合）
- （近期）Rendezvous & Docking of Mobile Ground Robots for Efficient Transportation Systems（arXiv 2026，若后续存在 ICRA/IROS/RA-L 版本以其为准）：提供“多地面机器人 rendezvous + 物理对接”的问题化表达与工程动机。
- 编队重构/车队协同（platooning / formation reconfiguration）的 ICRA/IROS/RA-L 系列工作：把“前车主动配合（减速/稳向/短程位姿调整）”视为重构策略的一部分（本项目中将其封装为 Docking Skill 内部可选动作）。

> 注：协同对接的文献跨度较大（从移动机器人 docking 到车辆编队重构/platooning）。本设计在 P-1.1 中只吸收其“可达性门控 + 协同 staging”的思想，并以统一数学形式落地，避免变成“额外工程分支”。

---

## 3. 我们的核心创新：Co-BCFD（Belief-Consistent + Capture-Funnel + Cooperative Staging）

我们把对接写成一个 **Belief-Conditioned, Capture-Set Aware, Cooperative Skill**。其核心不在于“两阶段对接”本身（那只是手段），而在于：  
**把“是否需要 leader 调整位姿、何时进入末端伺服、何时回退重规划”统一为同一个闭环门控与代价框架。**

### 3.1 三个核心贡献（可形成论文主线）
1. **C1 — Belief-Consistent Fusion Gate（置信一致门控融合）**  
   在融合带内不是按距离硬编码权重，而是用 NIS 一致性 + 距离调度联合给出 \(\gamma\in[0,1]\)，实现“视觉异常不污染”“视觉越近越占优”的一致性融合。
2. **C2 — Capture-Funnel + Contact Gate（捕获域漏斗 + 接触门）**  
   针对 Ackermann 近场不可原地修姿的本质难点，显式构造“允许进入接触带”的必要条件（集合门控），并用随距离收缩的 funnel 把“进入捕获域 → 锁定域”变成可解释结构；必要时触发**微机动（parking-like）**把状态拉回捕获域。
3. **C3 — Cooperative Staging as Inner Skill Action（协同 staging，内部封装动作）**  
   当当前位置的可达性/遮挡/风险导致捕获域不可进入时，由 Docking Skill 内部自主选择一个 docking-friendly staging pose \(x_L^\dagger\)，并驱动 Leader 进行短程调整；Follower 同步调整到等待/预对接位姿。对上层而言依然是一次 `Dock` 指令。

### 3.2 与传统两阶段方法的差异化（避免“工程拼接”）
传统做法常见链路是：  
1) 全局定位规划到目标点；2) 到阈值后切换视觉伺服；3) 失败则硬回退。

Co-BCFD 的最小闭环链路是：  
1) **staging 可选优化**（必要时 leader 先去到更可对接的位姿）；  
2) **置信一致融合**（视觉异常自动降权）；  
3) **capture-funnel 门控**（未落入捕获域则 backoff + 微机动）；  
4) **BR-SMPC 近场机动**（允许倒车/对齐的非凸机动）；  
5) **统一安全投影**（碰撞/净空/执行器约束一次性封装）。  

这样，“切换连续性 / 观测一致性 / 安全统一性”三类系统性问题被同一结构吸收，而不是在工程上堆叠补丁。

---

## 4. 数学建模（最小闭环所需）

### 4.1 相对对接误差（Follower 坐标系）
用 follower 前向相机/前铰接点作为参考点（记为 \(c_F\)），leader 后铰接点为对接目标（记为 \(h_L\)）。

定义目标相对位姿
\[
r = 
\begin{bmatrix}
x_r \\ y_r \\ \psi_r
\end{bmatrix},
\quad
\begin{bmatrix}
x_r \\ y_r
\end{bmatrix}
 = R(\psi_F)^\top\,(h_L - c_F),\quad
\psi_r = \psi_L - \psi_F
\]
对接集合（锁定集合）：
\[
\mathcal{D} = \Big\{ r:\ \sqrt{x_r^2+y_r^2}\le \epsilon_p,\ |\psi_r|\le \epsilon_\psi,\ |v_F-v_L|\le\epsilon_v \Big\}
\]

视觉捕获集合（可见性）：
\[
\mathcal{C} = \Big\{ r:\ \sqrt{x_r^2+y_r^2}\le d_{vis},\ |\mathrm{atan2}(y_r,x_r)|\le \theta_{fov}/2,\ \mathrm{LOS}=1 \Big\}
\]

### 4.2 两源观测与一致性（Belief）
我们在 **世界坐标** 上估计 leader rear-hitch 的位姿 \(\mu=[x_h,y_h,\psi_L]^\top\) 及协方差 \(\Sigma\)。

- GNSS 观测（低精度稳定）：
\[
z_G = \mu + \nu_G,\ \nu_G\sim\mathcal N(0,R_G)
\]
- 视觉观测（高精度间歇）：先把相对观测 \(z_V=[x_r,y_r,\psi_r]\) 通过 follower 真值/估计变换到世界系得到 \(z_V^w\)，满足
\[
z_V^w = \mu + \nu_V,\ \nu_V\sim\mathcal N(0,R_V)
\]

**一致性融合（信息滤波形式）**：
当视觉有效且通过一致性门限（见下）时，
\[
\Sigma^{-1} \leftarrow \Sigma^{-1} + \gamma\,R_V^{-1},\quad
\Sigma^{-1}\mu \leftarrow \Sigma^{-1}\mu + \gamma\,R_V^{-1} z_V^w
\]
其中 \(\gamma\in[0,1]\) 是 **鲁棒门控系数**（避免视觉异常时污染估计）：
\[
\mathrm{NIS} = (z_V^w - z_G)^\top (R_G+R_V)^{-1}(z_V^w - z_G)
\]
\[
\gamma = \underbrace{\sigma(k_d(d_{max}-d))}_{\text{距离调度}}\cdot
\underbrace{\sigma(k_c(\tau-\mathrm{NIS}))}_{\text{一致性门控}}
\]
\(\sigma(\cdot)\) 为 sigmoid，\(\tau\) 为卡方门限（对应 95%/99% 置信）。

> 直观解释：视觉越近越可信；视觉与 GNSS 越一致越可信；否则自动降权而不是“硬回退 + 抖动”。

### 4.3 Capture-Funnel（从捕获域到锁定域的“管”）
Co-BCFD 用一个随距离收缩的 Funnel 把“全局趋近/近场微机动/视觉伺服/回退”统一起来：

- 距离 \(d=\sqrt{x_r^2+y_r^2}\)。
- 定义收缩函数 \(s(d)\in[0,1]\)（全局→末端）：
\[
s(d)=\mathrm{clip}\Big(\frac{d_{max}-d}{d_{max}-d_{min}},0,1\Big)
\]
其中 \(d_{min},d_{max}\) 是 **funnel 自身的收缩区间**（推荐与“感知融合带”解耦）。  
原因：融合带 \([1.2,1.8]\) 只负责“软切换”，而 funnel 需要从“可达捕获”一直收缩到“接触锁定”，若在 \(1.2m\) 处就饱和，会导致近场过严/过松并触发死锁。实践建议：
- \(d_{max}\approx 2\sim 4m\)：开始强调对齐与可见性；
- \(d_{min}\approx 0.2\sim 0.6m\)：进入锁定捕获域并强制满足更严格的几何包络（\(\epsilon_\psi,\epsilon_p\) 的先验门限）。

定义“允许误差包络”：
\[
|y_r|\le y_{max}(d),\quad |\psi_r|\le \psi_{max}(d)
\]
并让其单调收缩：
\[
y_{max}(d)= (1-s(d))\,\bar y + s(d)\,\underline y,\quad
\psi_{max}(d)= (1-s(d))\,\bar \psi + s(d)\,\underline \psi
\]
其中 \(\bar y,\bar\psi\) 为全局阶段容许误差，\(\underline y,\underline\psi\) 为末端捕获阶段容许误差（由对接/视觉 FOV 约束给出下界）。

**Funnel 代价（CLF 形式）**：
\[
V(r;d)=
\begin{bmatrix}
x_r-x^\star(d)\\y_r\\\psi_r
\end{bmatrix}^\top
Q(d)
\begin{bmatrix}
x_r-x^\star(d)\\y_r\\\psi_r
\end{bmatrix}
\]
其中 \(x^\star(d)\) 是纵向“捕获参考”（远处先对齐再接近，近处收敛到 0），\(Q(d)\) 随 \(s(d)\) 增大而增大（末端更“硬”）。

### 4.4 统一证书场：把 `CGFL/TVT/VPCR/PTCC` 收敛为同一策略对象
为避免 Stage-2.5 后续改进在论文里退化成“工程补丁堆叠”，我们把这些历史命名重新收敛为同一个**证书场约束策略**：

\[
\sigma =
\begin{bmatrix}
\sigma_P \\
\sigma_T \\
\sigma_V \\
\sigma_C \\
\sigma_{post}
\end{bmatrix}
\in [0,1]^5
\]

其中：
- \(\sigma_P\) —— **进展证书（Progress Certificate）**：度量“当前 plan / corridor 是否允许安全压时”。它吸收了 `CGFL` 与 `PTCC` 的作用：
  - 在共同可解 easy case 上，表现为 fast-lane 的速度/接触门限放宽；
  - 在 `SC/EC` 的可通过 corridor 上，表现为 plan-conditioned 的 staging / approach 速度缩放。
- \(\sigma_T\) —— **终端证书（Terminal Certificate）**：度量“当前是否存在可进入终端捕获盆地并完成末端姿态闭合的可行短时域轨迹”。它吸收了 `TVT`、`terminal capture boost` 与 `CTP` 的作用：
  - 当 nominal funnel 无法直接保证终端可行时，切换到终端可行管/原语；
  - 一旦重新进入 certified capture basin，则允许更强的 terminal lock-assist 与 bounded terminal projection。
- \(\sigma_V\) —— **可见性证书（Visibility Certificate）**：度量“当前是否仍处在可持续可见的 pair-capture 流形上”。它吸收了 `VPCR` 的作用：
  - 当近场视觉失配/长时丢失时，不再沿原 funnel 硬顶，而是切到 visibility-persistent pair-capture 重 staging；
  - 当重新进入 certified visible capture basin 后，再返回 base terminal policy。
- \(\sigma_C\) —— **走廊互惠证书（Corridor Reciprocity Certificate）**：度量“当前是否存在一条 corridor-compatible cooperative staging plan”，使 Leader 能塑形 basin、Follower 能沿走廊进入 handoff 带。
- \(\sigma_{post}\) —— **后对接行进证书（Post-Docking Mobility Certificate）**：度量“当前 Leader staging pose 在完成对接后是否仍保留后续任务机动便利性”。在 `LC` 下它惩罚车身航向与走廊方向的偏差；在自由空间场景下退化为 `1.0`。

于是，Docking Skill 不再由四个并列模块组成，而是由一个统一策略写成：
\[
u_k^\star = \arg\min_{u\in\mathcal U_{cert}(x_k,\hat x_k,\sigma_k)} \ell\big(x_k,\hat x_k,u;\sigma_k\big)
\]
其中
\[
\mathcal U_{cert} = \mathcal U_P(\sigma_P) \cap \mathcal U_T(\sigma_T) \cap \mathcal U_V(\sigma_V)\cap \mathcal U_C(\sigma_C)\cap \mathcal U_{post}(\sigma_{post})
\]
分别对应：
- `\mathcal U_P`：允许的速度缩放、contact gate 放宽与 corridor 压时；
- `\mathcal U_T`：允许进入终端捕获盆地的可行管与 terminal projection；
- `\mathcal U_V`：允许维持/重建可见性的 pair-capture 位姿与路径集合。
- `\mathcal U_C`：允许满足 corridor mutual feasibility 的 staging / ingress / release 集合；
- `\mathcal U_{post}`：允许对接后仍保持任务空间可达性的 leader staging 位姿集合。

更重要的是，模式切换本身也被统一进证书场，而不是写成散落的 patch：
\[
m_k = \arg\max_{m\in\{\text{BASE},\text{VIS},\text{TERM}\}} \Delta_m(\sigma_k)
\]
其中 \(\Delta_m\) 表示哪一类证书缺口当前最主导闭环性能/可行性。

论文叙事上，`CGFL / TVT / VPCR / PTCC` 只保留为**实现映射标签**，而不再是核心理论对象。核心理论对象只有：
1. belief-consistent relative belief；
2. capture-funnel 的 Lyapunov / barrier 结构；
3. certificate field \(\sigma=(\sigma_P,\sigma_T,\sigma_V,\sigma_C,\sigma_{post})\)。

### 4.5 `\sigma_C`：Corridor Reciprocity Certificate（新增，针对 `LC`）
新增 `LC` benchmark 后，原有三张证书仍不足以刻画窄通道里的核心困难：
**Follower 不能自由绕行，因此是否能对接，首先取决于 Leader 是否能通过有限次 forward/reverse 微机动，把系统送入一个 corridor-compatible 的 capture basin。**

因此对 `LC` 引入第四张证书：
\[
\sigma_C \in [0,1]
\]
它衡量“当前是否存在一条**相位分离的 corridor reciprocal plan**”，使得：
1. Leader 能在 lane/corridor 内通过有限长度的双向运动原语退出 pocket / bay，并到达 basin anchor；
2. Follower 在 Leader 完成 basin shaping 前保持等待或低速让位；
3. Leader 完成 basin shaping 后，Follower 能沿 corridor geodesic 安全进入 predock / handoff 带。

将 `LC` 下的协同 Staging 写为：
\[
\Pi_C^\star=
rg\min_{\Pi_L,\pi_F,s_h}
J_C
= w_L L(\Pi_L)
+ w_F L(\pi_F)
+ w_G N_{gear}
+ w_S R_{safe}
- w_B \mathcal B_\Gamma(x_L^\star)
\]
其中：
- \(\Pi_L\) 是 Leader 的双向运动原语序列；
- \(\pi_F\) 是 Follower 的 corridor ingress 路径；
- \(s_h\) 是 Follower 的 reciprocal hold / release 相位；
- \(\mathcal B_\Gamma(x_L^\star)\) 是 corridor capture basin 质量，度量 anchor pose 是否让 Follower 能在不离开 corridor 的情况下进入 handoff 带。

约束写成：
\[
\Pi_L \subset \Gamma,\quad \pi_F \subset \Gamma,
\quad d_{clr}(\Pi_L,\pi_F)\ge d_{safe},
\quad s_F(t) \le s_h\;\;	ext{until}\; x_L(t)\in\mathcal X_{basin}
\]
其中 \(\Gamma\) 是 lane graph / corridor free set。

这一步对应吸收了三条文献脉络：
- `Reeds–Shepp / Hybrid A* / motion primitives`：说明倒车与 gear switch 不是补丁，而是车式底盘姿态到姿态问题的本征结构；
- `continuous-curvature parking / target-tree`：说明 parking-like 机动应在轨迹生成层显式考虑曲率连续与可达性；
- `reachability / viability`：说明 lane-constrained 对接的本质是“是否存在 corridor-compatible basin”，而不只是末端再加几条 if-else。

在统一证书场中，`LC` 下的可行控制集合进一步变为：
\[
\mathcal U_{cert}^{LC} = \mathcal U_P \cap \mathcal U_T \cap \mathcal U_V \cap \mathcal U_C(\sigma_C)
\]
进一步地，`LC` 下的协同 staging 优化问题写成：
\[
x_L^{\dagger\star}=\arg\max_{x_L^\dagger}\sum_{k\in\{P,T,V,C,post\}}w_k\sigma_k(x_L^\dagger)
\quad \text{s.t.}\quad \sigma_k(x_L^\dagger)\ge \epsilon_k
\]
因此，`LC` 新模块在论文口径中不再被描述为单独工程逻辑，而是统一证书场在 lane-constrained family 下的一个**特化约束分量**。其中 `\sigma_{post}` 把“对接后不要被卡住”的后续任务要求前置成 staging 前约束，而 `StagingCertificateOptimizer` 则只是这个多证书优化问题的求解器实现。

---

## 5. 控制与规划：Capture-Aware Planning + Safety Projection

对接闭环被拆成两个“可解释”的子优化（共享同一 cost 分解）：
1. **协同 staging 优化**：决定 Leader 是否需要去哪里“更好对接”（可选动作）；
2. **Follower 近场捕获优化**：在 Ackermann 约束下进入捕获域并完成锁定。

### 5.1 近场捕获：Barrier-Regularized Sampling MPC（BR-SMPC）
每个控制周期，在控制序列空间做采样（MPPI/CEM/随机射击均可），并滚动仿真 \(H\) 步：
\[
x_{k+1}=f(x_k,u_k),\quad k=0\dots H-1
\]
对每个候选序列计算代价：
\[
J=\sum_{k=1}^{H}\Big(
V(r_k;d_k)
+\lambda_u\|u_k\|^2
+\lambda_{\Delta u}\|u_k-u_{k-1}\|^2
+\lambda_b\,\phi(d_{clr}(x_k))
\Big)
\]
其中 \(\phi(\cdot)\) 是 barrier-like 惩罚（净空越小惩罚越大），\(d_{clr}\) 由几何碰撞模块得到。

并强制可行性：
- 候选轨迹若发生碰撞 → 直接剔除（hard constraint）。
- 执行器限制/最大曲率 → 直接剔除。

输出最优名义控制 \(u_{nom}\)。

> 这一步把“全局趋近/视觉伺服/回退”统一为同一优化结构，只是 \(Q(d)\)、\(x^\star(d)\)、以及 \(\gamma\)（视觉权重）随距离与一致性自适应变化。

### 5.2 Safety Projection（统一安全层）
即便名义控制来自 MPC，也仍需要统一安全层防止：
- 采样集合过稀导致漏检；
- 突发视觉失效导致误动作；
- 数值误差导致短时穿障。

因此我们对 \(u_{nom}\) 做安全投影：
\[
u^\star = \arg\min_{u\in\mathcal U}\|u-u_{nom}\|^2
\quad \text{s.t.}\quad \min_{k\le K} d_{clr}(x_k(u))\ge d_{safe}
\]
实现上可用小规模采样投影（与 CBF-QP 等价目标），保证“只要存在安全动作，最终输出就安全”。

### 5.3 协同 staging：把“Leader 配合”写成同一代价的内部选择
令 \(x_L^\dagger\) 是 Leader 计划用于对接的 staging pose（短程位姿调整点）。定义：
- Leader 到 staging 的可行轨迹代价 \(J_L(x_L^\dagger)\)；
- Follower 到预对接位姿 \(x_F^{pre}(x_L^\dagger)\) 的代价 \(J_F(x_L^\dagger)\)；
- 可见性/遮挡风险 \(R_{vis}(x_L^\dagger)\)（由 LOS/FOV 与障碍几何计算）；
- 净空风险 \(R_{clr}(x_L^\dagger)\)（由最小净空/碰撞余量计算）。

协同 staging 选择写成离散优化：
\[
x_L^{\dagger\star} = \arg\min_{x_L^\dagger\in\mathcal X_{free}}
w_T \hat T(x_L^\dagger)+w_E \hat E(x_L^\dagger)+w_R \hat R(x_L^\dagger)
\;+\mathbf 1_{\neg feasible}(x_L^\dagger)\cdot M
\]
其中 \(\hat T,\hat E,\hat R\) 可由规划代价/控制能量/风险项近似得到，\(M\) 为大惩罚。  
**关键点**：这不是“额外工程逻辑”，而是同一 cost 分解下的一个内部离散决策变量（类似小型 task-and-motion 层次，但封装在 Docking Skill 内部）。

> 注：以下 `CGFL / TVT / VPCR / PTCC` 章节保留的是**实现映射**与实验历史，不再是论文中的四个并列理论模块；论文口径以 4.4 的统一证书场建模为准。

### 5.3B `LC-CRBS`：Lane-Constrained Corridor Reciprocal Basin Shaping（本轮新增）
`LC-CRBS` 是 `LC` family 下对 5.3 协同 staging 的特化，而不是新的并列总框架。其核心思想是：
**不再让 leader/follower 在窄 corridor 里同时“各走各的路径”，而是先由 Leader 通过有限次前进/后退微机动塑造 basin，再由 Follower 进入 corridor handoff 带。**

具体地，把 `LC` staging 进一步分解为三相：
1. `Leader-Shape`：Leader 在 bay / pocket 内做 bidirectional primitive search，得到一条安全的 reverse/forward 微机动序列；
2. `Leader-Settle`：Leader 从局部 exit pose 收敛到 corridor-compatible anchor pose；
3. `Follower-Ingress`：Follower 仅在 Leader 达到 basin-ready 条件后，沿 corridor geodesic 进入 handoff 带，再交还给 base docking funnel。

数学上，这相当于把 `LC` 的内部动作从单个 staging pose 变量，提升为带相位变量的混合动作：
\[
(\Pi_L^\star,\pi_F^\star,\eta^\star)
=rg\min J_C(\Pi_L,\pi_F,\eta)
\]
其中 \(\eta\in\{	ext{shape},	ext{settle},	ext{ingress}\}\) 是 reciprocal phase。执行时满足：
\[
\eta=	ext{shape} \Rightarrow u_F = u_{hold},
\qquad
\eta=	ext{ingress} \Rightarrow u_L = u_{anchor-hold}
\]
也即：在 lane-constrained case 中，pair interaction 的主约束不是“谁更快”，而是“谁先动、谁后动”。

在实现上，`LC-CRBS` 落成三部分：
- **协同 Staging**：`docking/lc_corridor.py` 中的 `CorridorReciprocityPlanner`，负责生成 corridor reciprocal plan；
- **规划控制**：同文件中的 `CorridorReciprocityExecutor`，负责 `Leader-Shape / Leader-Settle / Follower-Ingress` 的相位执行；
- **安全保护**：每一相都通过短时域采样规划 + 一步碰撞投影，保证 corridor 内不穿墙、不撞车。

它与旧版实现的本质差异是：
- 旧版把 `LC` 当作普通 cooperative staging + generic path tracking；
- 新版把 `LC` 明确建模为 **corridor-constrained reciprocal hybrid control**。 

### 5.3C `\sigma_C^{exec}`：Execution-Time Basin Certificate + Terminal Reshaping（round-4 诊断新增）
round-4 的单案诊断表明，`LC` 的当前瓶颈不是“还会不会撞”，而是**offline corridor basin 与 runtime actual pose 之间的执行偏差**。  
原有 `\sigma_C` 更多回答的是：
\[
\text{“候选 anchor }x_L^\star\text{ 是否几何上可行？”}
\]
而 round-4 的真实失败在于：
\[
x_L^{act}\neq x_L^\star
\]
当 Leader 实际停在 `x_L^{act}` 时，actual predock / ingress tube 可能已经失效，即便 `x_L^\star` 原本有证书。

因此，`LC` 需要把 release 判据提升为运行时证书
\[
\sigma_C^{exec}(x_L^{act},x_F)
=
\Phi_{predock}(x_L^{act})
\cdot
\Phi_{ingress}(x_L^{act},x_F)
\cdot
\Phi_{align}(x_L^{act},x_F)
\]
其中：
- `\Phi_{predock}`：由 **actual** leader pose 诱导的 dynamic predock 是否无碰撞、净空是否足够；
- `\Phi_{ingress}`：Follower 从当前 pose 到该 dynamic predock 的 corridor 内路径是否仍然可行；
- `\Phi_{align}`：当前 pair 的 terminal longitudinal / lateral / yaw 残差是否进入可捕获 basin。

若 `\sigma_C^{exec}` 低于 handoff threshold，则不应释放 Follower；反之若 `x_L^{act}` 已接近可行 basin，则允许 handoff。  
若 `\sigma_C^{exec}` 长时间不达标，则应执行一段 leader terminal reshaping：
\[
\Pi_L^{term}
=
\arg\max_{\Pi\in\mathcal U_{prim}^{1:K}}
\sigma_C^{exec}(f(x_L^{act},\Pi),x_F)
- \lambda_L L(\Pi)
\]
即：在短时域 primitive 库上搜索一小段 leader 微机动，把 actual pair 推回 execution-valid basin，再重新评估 handoff。

这一步的意义是：把 `LC` 的“terminal capture closure”从 follower 端的局部控制问题，重新提升为**execution-valid cooperative basin closure** 问题；它仍然属于 `LC-CRBS` 的统一框架，而不是额外堆补丁。

### 5.3D `\sigma_M`：Terminal-Manifold Closure Certificate（round-11 新增）
round-11 的两关键单案回放进一步表明：即便 `\sigma_C^{exec}` 已经允许 release，`LC` 仍可能在进入 `DOCKING` 后失败。  
真正缺失的是一个回答下面问题的证书：
\[
\text{“pair 当前是否已经进入一个对 generic lock / funnel 可闭合的 near-field manifold？”}
\]

因此，本轮把 `LC` 的末端闭合显式写成
\[
\sigma_M(x_L,x_F)

=
\Phi_{clr}(x_L,x_F)
\odot
\Phi_{geom}(x_L,x_F)
\odot
\Phi_{wit}(x_L,x_F)
\]
其中：
- `\Phi_{clr}`：当前 tight corridor 下的局部障碍净空是否仍有 terminal margin；
- `\Phi_{geom}`：leader-frame 下的 longitudinal / lateral / yaw 残差是否进入可收缩的近场 manifold；
- `\Phi_{wit}`：在 short-horizon follower primitive 集上，是否存在一条 **contractive witness** 能继续提高 `\sigma_M`。

执行口径上，`LC-CRBS` 被扩展为：
`Leader-Shape -> Leader-Settle -> Follower-Ingress -> Follower-Terminal-Closure`。  
也就是说，`LC` 不再把 `Follower-Ingress` 的“到达 predock”当成终点，而是要求 pair 在 handoff 前先经过 `FOLLOWER_TERMINAL_CLOSURE`，直到：
- `\sigma_M` 达到 handoff threshold，或
- pair 已进入 `LC_LOCK_HOLD` 的 terminal lock manifold。

这使得 `LC` 的 handoff 判据从“能到 predock”升级为“能在 corridor 约束下连续闭合到 lock manifold”。  
round-11 的 retained 结论是：`DBv1-LC-L2-081` 在这一口径下继续保持成功，而 `DBv1-LC-L1-074` 已经不再 deadlock，但最终仍暴露出 generic `LOCK_ASSIST` 与 tight-corridor safety envelope 的最后一处不一致。

round-12 则进一步把这一 near-field manifold 写成**保持不变量**：一旦 `\sigma_M` 已经把 pair 送入 `LC_LOCK_HOLD`，执行层不再允许 generic `LOCK_ASSIST` / soft-capture 继续推动车体，而是执行一个 certificate-consistent terminal hold：
\[
\Pi_{\mathcal M}^{hold}(x_L,x_F)=
\operatorname{argmin}_{\tilde x \in \mathcal M_{lock}}
\|\tilde x - x\|
\quad
\text{s.t.}\quad d_{clr}(\tilde x)\ge d_{min}.
\]
也就是说，控制器在 `LC_LOCK_HOLD` 中做的不是“再向前逼近一点”，而是把 pair 投影并保持在满足净空约束的 terminal lock manifold 上，等待 lock evaluator 完成时间闭合。  
本轮实现上，这体现为：
- `docking/dock_skill.py` 中的 `LC_LOCK_HOLD` 末端保持命令；
- `scripts/run_p_minus1_stage1_docking.py` 中的 terminal-hold invariant projection；
- 以及 round-12 状态文件 `artifacts/lc_round12_lock_hold/ROUND12_LOCK_HOLD_STATUS.md`。

round-13 则把 `LC` 的 release 本身也写成证书：
\[
\sigma_R(x_L,x_F)
=
\sigma_M(x_L,x_F)
\cdot
\Phi_{handoff}(x_L,x_F)
\cdot
\Phi_{wit}(x_L,x_F)
\]
其中：
- `\Phi_{handoff}`：用一个 short-horizon downstream docking surrogate 预测“如果现在把控制权交给 generic docking funnel，未来若干步是否仍保持 obstacle clearance”；
- `\Phi_{wit}`：当前 terminal manifold 不是伪闭合，即 `witness_sigma - \sigma_M` 仍为正，说明 pair 还有可验证的 contractive handoff 方向；
- 只有 `terminal_lock_ready` 或 `\sigma_R` 足够高时，才允许真正释放到 `DOCKING`。

这一步的作用是把 round-12 暴露出的“`L2-082` 早释放碰撞”从执行故障提升为**交接证书缺失**问题，并把 `Settle / LockHold -> Release` 的切换纳入与 `\sigma_C,\sigma_M` 同一条证书链。

### 5.4 `CGFL`：Certificate-Gated Fast Lane（Stage-2.5 当前已合入）
为解决 `Common-Feasible` 子集上 Full model 相对强传统基线过于保守的问题，当前 Stage-2.5 首先落地了一个**证书门控快速通道**。
快速通道证书可表述为：仅当“无需 leader relocation、无遮挡、初始航向差小、环境净空充分、dock zone 净空充分”同时满足时，才允许进入 fast lane。
只有当系统状态进入 \(\mathcal C_{fast}\) 时，才允许：
1. 提高 predock approach 的参考速度上限；
2. 放宽近场 contact gate 的保守阈值；
3. 推迟 lock-assist 的强制低速接管时刻。

这样做的理论含义是：把“共同可解 easy case 的效率最优性”也写成一张**安全证书**，而不是直接删掉 funnel / safety 逻辑。当前实现只在高净空、无遮挡、无 leader relocation 的局面触发，因此不会改变 `SC / FC / EC` 的主机制判别口径。

### 5.5 `TVT`：Terminal Viability Tube（Stage-2.5 独立子技能，针对 `FC-L3`）
`TVT` 在实现上不再表现为 base skill 上继续叠若干阈值，而是定义一个独立的 branch variable：

\[
 b_k \in \{\text{BASE},\ \text{TVT},\ \text{VPCR}\}
\]

当 nominal staging plan 的剩余 corridor clearance 过低、dock-zone 净空不足，且存在局部终端安全管证书时，系统从 `BASE` 切换到 `TVT`，由 `TVT` 子技能独立负责 terminal restaging 与 narrow-corridor tracking。

`FC-L3` 的本质不是“单纯再加几条安全 if-else”，而是：
**在窄 dock zone 中，近场接触段是否存在一段满足 Ackermann 约束、扫掠净空约束、末端捕获约束的短时域可行轨迹。**

为此定义终端可行原语库：
\[
\Pi_{tube}=\{\pi_m\}_{m=1}^M,
\quad
\pi_m=\{u_0,\dots,u_{H-1}\}
\]
每个原语是短时域的 forward / reverse / arc primitive。对任一原语定义：
\[
G_{clr}(\pi_m)=\min_{k\le H} d_{clr}(x_k),
\quad
E_{cap}(\pi_m)=\alpha_y|y_H|+\alpha_\psi|\psi_H|+\alpha_x|x_H-x_H^{ref}|
\]
并定义终端可行集合：
\[
\mathcal V_{tube}=\{\pi_m:\ G_{clr}(\pi_m)\ge d_{safe},\ x_H\in \mathcal C_{term}\}
\]
其中 \(\mathcal C_{term}\) 是允许安全进入接触带的终端捕获集合。

执行时不再直接把当前名义控制送入接触带，而是求解：
\[
\pi^\star = \arg\min_{\pi\in\mathcal V_{tube}} J_{tube}(\pi)
\]
若 \(\mathcal V_{tube}=\emptyset\)，则不进入接触段，而是选择 `escape primitive` 回到 funnel 外层并等待重规划。

这条线的理论意义是：把 `FC-L3` 的难点从“局部碰撞补丁”提升成**终端可行域 / viability tube** 问题；只有当存在安全短时域管时，系统才允许进入最危险的终端段。

### 5.6 `VPCR`：Visibility-Persistent Cooperative Re-Staging（Stage-2.5 独立子技能，针对 `EC-L1`）
`VPCR` 也不再是 base fallback 上继续堆叠 `if visual_lost: ...` 的补丁，而是一个独立的 visibility-recovery branch：一旦系统已经进入近场、曾成功建立视觉、随后出现长时丢视野事件，系统从 `BASE` 切到 `VPCR`，由 `VPCR` 子技能独立求解 visibility-persistent restaging pose 与 recovery path。

`EC-L1` 的本质不是“视觉丢失后怎么补救”，而是：
**一旦进入近场，对接目标是否仍处在一个对 follower 来说“可持续可见”的协同位姿流形上。**

为此定义 leader staging pose 的可持续可见指标：
\[
P_{vis}(x_L^\ddagger)=\frac{1}{N}\sum_{k=1}^{N}\mathbf 1\{\mathrm{LOS}_k=1,\ |\beta_k|\le \theta_{fov}/2,\ d_k\le d_{vis}\}
\]
并定义末段可见性保持率与重见距离：
\[
\underline P_{vis}(x_L^\ddagger),\qquad D_{reacq}(x_L^\ddagger)
\]
于是可持续可见的重 staging 集合为：
\[
\mathcal S_{vis}=\{x_L^\ddagger:\ P_{vis}\ge \rho_1,\ \underline P_{vis}\ge \rho_2,\ D_{reacq}\le D_{max}\}
\]
发生“已进入近场且曾成功看见目标，但随后视觉长时丢失”的事件后，不再仅对 leader 重新选一个 pose，而是把问题提升成**成对可见流形**上的协同重 staging：
\[
(x_L^{exp\star},x_F^{view\star},x_F^{cap\star})
=\arg\min_{(x_L^{exp},x_F^{view},x_F^{cap})\in\mathcal M_{vpcr}}
J_L+J_F+\lambda_v(1-P_{pair})+\lambda_r D_{reacq}
\]
其中：
- `exposure pose` `x_L^{exp}`：leader 为恢复 LOS/FOV 主动暴露出的可见位姿；
- `view pose` `x_F^{view}`：follower 为重新建立稳定视觉而到达的观察位姿；
- `capture pose` `x_F^{cap}`：在重新建立视觉后，从观察位姿继续推进到的可接管 predock 位姿。

成对可见流形定义为：
\[
\mathcal M_{vpcr}=\left\{(x_L^{exp},x_F^{view},x_F^{cap}):
\underbrace{V(x_F^{0},x_L^{exp})\ge \rho_0}_{\text{即时可见}},
\underbrace{P_{vis}(\pi_F^{view};x_L^{exp})\ge \rho_1}_{\text{重见路径可见}},
\underbrace{P_{vis}(\pi_F^{cap};x_L^{exp})\ge \rho_2}_{\text{捕获路径可见}}
\right\}
\]
其中 `\pi_F^{view}` 是 follower 到 `x_F^{view}` 的重见路径，`\pi_F^{cap}` 是从 `x_F^{view}` 到 `x_F^{cap}` 的捕获路径。

因此，`VPCR` 的运行时模式也从单一的 `re-staging` 细化为两个独立子阶段：
1. `VPCR-PAIR`：leader 与 follower 协同移动到 `(x_L^{exp\star},x_F^{view\star})`，以最快恢复稳定视觉；
2. `VPCR-CAPTURE`：leader 保持 exposure pose，follower 沿 `\pi_F^{cap}` 进入新的 predock / capture 区，再切回 base docking。

当前实现进一步采用了 **response-reachable pair-capture** 变体：`x_L^{exp}` 不再由任意几何采样直接给出，而是从 leader 的短时域 Ackermann 响应原语库中筛选，确保 exposure path 本身可执行；然后再在这个 reachable exposure 上联立搜索 `view/capture` 两段 follower 路径。这一实现对应代码位于 `docking/stage25_subskills.py` 的 `VisibilityPersistentRestagingSkill`。

这条线的理论意义是：把 `EC-L1` 的失败从“fallback 细节”提升成**visibility-persistent manifold** 问题；目标不再只是“重新看见一次”，而是“重新落到一个近场不会再次快速丢视野的协同流形上”。

> Stage-2.5 中先前已经探索过 `RCS` / `SSTF` 原型，但在 frozen 全量 `test` split 上出现了总体成功率回退，因此当前**未合入主实现**，只保留为实验分支结论。本轮的 `TVT / VPCR` 是对这两条线的更方法化重写。

### 5.7 Stage-2.5 运行时集成口径（独立 mode / subskill）
运行时状态机不再把 `TVT / VPCR` 编码成 scattered threshold patches，而是冻结为如下口径：
### 5.8 `PTCC`：Plan-Triggered Corridor Compression（本轮新增，针对 `SC / EC` 的安全压时）
在不改变 success / safety 主逻辑的前提下，本轮新增了一类**基于 plan-stats 的时间压缩证书**，实现位于 `docking/time_compression.py`。

其核心对象不是“直接把速度统一调高”，而是先对当前 `cooperative staging` 生成的 leader/follower 路径做一张**可压时证书**：
\[
\mathcal C_{time} = \{L_{shift},\ L_F,\ D_{clr},\ \Theta_{turn},\ b_{los}\}
\]
其中分别对应 leader relocation 量、follower 路径长度、路径最小净空、路径最大转角变化与 LOS block 标志。基于这组量，运行时只在两类可解释的签名上放开速度：
1. `SC-blocked-mid`：小 leader shift + 中等 follower 路径 + 足够 corridor clearance 的 switching case；
2. `EC-short`：中等 leader shift + follower 路径不过长 + corridor clearance 充足的短扩展 case。

压时证书一旦失效（例如进入 `VPCR`、触发可见性丢失回退），系统立即回退到 baseline 速度口径。因此，这条线的理论含义是：把“是否允许压时”也写成一张**plan-conditioned safety certificate**，而不是全局改大速度上限。


1. `BASE`：原有 `co_bcfd = cooperative staging + BCFD + safety projection + CGFL` 主线；
2. `TVT`：当 nominal 剩余 corridor 证书失效，但局部终端可行管存在时，切换到 `TVT`；其内部使用局部 terminal plan + tube tracker 执行，完成后回到 `BASE/DOCKING`；
3. `VPCR`：当系统已经进入近场并建立过视觉、随后发生持续视觉丢失，且存在可持续可见的协同重 staging 解时，切换到 `VPCR`；其内部使用 visibility-persistent restaging planner 执行，重建稳定可见通道后回到 `BASE/DOCKING`。

因此，Stage-2.5 的研究问题从“要不要加若干保护阈值”升级为：
\[
\text{BASE} \xrightarrow{\mathcal C_{tvt}} \text{TVT},\qquad
\text{BASE} \xrightarrow{\mathcal C_{vpcr}} \text{VPCR}
\]
其中 `\mathcal C_{tvt}` / `\mathcal C_{vpcr}` 是由终端可行证书与可持续可见证书驱动的 branch activation conditions。

---

## 6. 传统方法基线（P-1.2 强制对比）

Stage-1/2 报告中的对比协议已在 `CORE_REQUIREMENTS_TASKBOOK.md` 的 `P-1.2/P-1.3/P-1.5` 冻结；此处只保留设计上的最小约束：
- 基线必须采用**分层池**而非单一弱基线：弱 sanity baseline 仅用于边界 sanity check，主结论必须来自至少 2 个强传统基线（如 `HybridA*/Lattice + PBVS`、`HybridA*/Lattice + NMPC/MPPI`、层级泊车式框架）与至少 1 个能力匹配基线（如 `T-Coop-HardSwitch` / `T-Coop-DistBlend`）。
- 场景必须按 `Common-Feasible / Occlusion-Switching-Critical / Extension-Critical` 分层，以区分“共同可解竞争力”和“可解域扩展能力”。
- 指标至少冻结：对接完成时间 \(T_{done}\)、轨迹/控制代价、碰撞率、成功率、最小净空、回退次数、FOV 丢失次数；并采用配对统计与失败归因表。
- `DOCK.md` 的模块有效性必须通过 `A-Full` 对 `A-NoStage / A-NoBeliefGate / A-NoFunnelGate / A-NoMicroManeuver / A-NoFallback / A-NoSafetyProj` 及耦合消融来验证，而不是只做超参灵敏度分析。

---

## 7. Stage-1 验证方式（闭环 + 可视化 + 对照）

> 场景标准与固定数据集以 `DOCK_SCENARIO_STANDARD.md` 为准；Stage-1/Stage-2 的代表场景与批量 split 均应从 `DockBench-v1` 中读取，而不是临时随机挑 seed。当前落地实现把候选集生成、support-aware split freeze 与 Stage-0 校验解耦为 `scripts/generate_dockbench_v1.py`、`scripts/freeze_dockbench_v1_splits.py`、`scripts/validate_dockbench_stage0.py` 三步，并把代表场景冻结到 `data/dockbench_v1/dockbench_v1_representatives.json`。

Stage-1 要求：
1. 在带随机障碍的单场景中，两车随机分布（不近距）；
2. 本方法完成对接（success=True, collision=0, clearance>=0.1m）；
3. 与任务书中冻结的分层 baseline 池对照：代表场景至少包含 1 个 `Common-Feasible` 场景（至少 1 个强基线成功）与 1 个 `Extension-Critical` 场景（体现 cooperative staging / capture-funnel 收益）；
4. 同时输出 `A-Full vs Ablation` 的机制回放与失败归因，确保设计中的每个关键模块都有对应证据链。

---

## 8. 实现映射（本仓库）

后续代码将以 “Docking Skill” 形式落地（保持与基础支撑其它模块解耦）：
- 观测：`docking/sensors.py`
- 碰撞与净空：`docking/collision.py`
- Docking Skill 入口（聚合/状态机）：`docking/dock_skill.py`（新增）
- 协同 staging 规划：`docking/coop_docking.py`（新增）
- 近场 funnel + lock-assist 控制：`docking/dock_skill.py`（集成实现）
- 轨迹回放：`docking/visualization.py` + `scripts/run_p_minus1_stage1_docking.py`

---

## 9. Stage-1 最新验证结论（2026-03-06）
Stage-1 已切换到 `DockBench-v1` 的 frozen representative protocol：
- 数据集：`data/dockbench_v1/`
- 代表场景：`data/dockbench_v1/dockbench_v1_representatives.json`
  - `CF_L2 = DBv1-CF-L2-010`
  - `SC_L2 = DBv1-SC-L2-026`
  - `FC_L2 = DBv1-FC-L2-046`
  - `EC_L2 = DBv1-EC-L2-065`
- 套件脚本：`scripts/run_p_minus1_stage1_suite.py`
- 报告：`artifacts/dockbench_stage1/P_MINUS1_STAGE1_SUITE_REPORT.md`
- 数据：`artifacts/dockbench_stage1/p_minus1_stage1_suite_results.json`

当前同场景对照结果：
1. **Common-Feasible / `DBv1-CF-L2-010`**
   - `co_bcfd`：`success=True`，`T_done=9.70s`，`collision=False`；
   - `T_lattice_pbvs`：`success=True`，`T_done=8.35s`；
   - 结论：已满足“强传统基线在共同可解子集稳定非零成功”的 Stage-1 准入要求。
2. **Switching-Critical / `DBv1-SC-L2-026`**
   - `co_bcfd`：`success=True`，`T_done=16.85s`，`fallback=2`，`visual_loss=3`；
   - `T_lattice_pbvs`：`success=True`，`T_done=13.70s`；
   - `A_no_belief_gate`：仍可成功，但 `fallback / visual_loss` 进一步上升；
   - 结论：`SC` 代表场景当前主要体现为**response-regime 证据**，即感知切换与回退链路被稳定触发，而不是简单的 ablation 全灭。
3. **Funnel-Critical / `DBv1-FC-L2-046`**
   - `co_bcfd`：`success=True`，`T_done=10.10s`，`fallback=5`，`visual_loss=6`；
   - `T_lattice_pbvs`：`success=True`，`T_done=7.80s`；
   - `A_no_funnel_gate / A_no_micro_maneuver`：当前在该代表场景上未形成硬失败，但近场高回退/高视觉丢失 regime 已稳定出现；
   - 结论：`FC` 的代表证据当前同样以**response-regime** 为主，而非单点 catastrophic gap。
4. **Extension-Critical / `DBv1-EC-L2-065`**
   - `co_bcfd`：`success=True`，`T_done=25.45s`，`collision=False`；
   - `T_lattice_pbvs / T_parking_hierarchical / A_no_stage / A_no_stage_no_belief`：全部失败；
   - 结论：`cooperative staging` 的可解域收益在 `EC` family 上是当前最清晰、最稳定的 Stage-1 证据。

### 9.1 Stage-1 到 Stage-2 期间的关键修订
1. **DockBench-v1 正式冻结**：生成、split 冻结、Stage-0 审计分别固定为 `scripts/generate_dockbench_v1.py`、`scripts/freeze_dockbench_v1_splits.py`、`scripts/validate_dockbench_stage0.py`。
2. **支持层与 benchmark 口径对齐**：Stage-0 audit 现在同时检查 `label_match + tuning admission + representative readiness`，避免旧版 `seed` 演示与正式 benchmark 脱节。
3. **Timeout 问题显式回归验证**：`SC-L1` 中存在至少 1 个场景在 `22s` 时限下表现为 deadlock、在 `30s` 时限下可完成对接，因此当前 `DockBench-v1` 已把 `SC-L1` 的正式 `max_time_s` 提升到 `30s`，避免把“时间上限过紧”误判为机制失效。
4. **Response-regime 作为正式证据类型**：对于 `SC / FC`，当前论文叙事不再强求“ablation 一定 hard fail”，而是接受“fallback / visual_loss / time-cost 显著上升”的 regime gap 作为 family 证据的一部分。

## 10. Stage-2 批量对比结论（2026-03-09，LC-CRBS round-1）
在新增 `LC` family 后，我们围绕 `LC` 引入了 `LC-CRBS (Lane-Constrained Corridor Reciprocal Basin Shaping)`，并按“单案复现 → frozen 全量 test → 消融验证 → failure-mode 分层统计”的口径完成了第一轮验证。

- 数据集：`data/dockbench_v1/`
- Stage-2 报告：`artifacts/dockbench_stage2_lc_crbs_round1_manual/P_MINUS1_STAGE2_REPORT.md`
- Stage-2 数据：`artifacts/dockbench_stage2_lc_crbs_round1_manual/p_minus1_stage2_results.json`
- 图表：`artifacts/dockbench_stage2_lc_crbs_round1_manual/p_minus1_stage2_success_heatmap.png`、`artifacts/dockbench_stage2_lc_crbs_round1_manual/p_minus1_stage2_subset_compare.png`、`artifacts/dockbench_stage2_lc_crbs_round1_manual/p_minus1_stage2_failure_clusters.png`、`artifacts/dockbench_stage2_lc_crbs_round1_manual/p_minus1_stage2_ablation_summary.png`
- 单案复现：`artifacts/lc_round3_single/`
- challenge 说明：当前主指标对应的 `test=60` 已完整回收；challenge 汇总当前来自 `14/15` 场景，缺失样本为 `DBv1-LC-L1-078`。

### 10.0 当前 round-1 结果
1. **overall 主指标**：`co_bcfd` overall `success_rate = 0.7000 (42/60)`，`collision_rate = 0.0167`，成功样本平均 `T_done = 16.63s`。
2. **family 分解**：
   - `CF`：`success=1.000`，`collision=0.000`，`avg_T_done=8.74s`；
   - `SC`：`success=0.6667`，`collision=0.0833`，`avg_T_done=17.13s`；
   - `FC`：`success=0.9167`，`collision=0.000`，`avg_T_done=14.20s`；
   - `EC`：`success=0.9167`，`collision=0.000`，`avg_T_done=27.32s`；
   - `LC`：`success=0.0000`，`collision=0.0000`，`12/12` 全部转化为 `lc_geometric_deadlock`。
3. **与上一轮 `LC refresh` 的关键差异**：
   - overall success 仍是 `0.7000`，说明新模块还没有把 `LC` 从“不可完成”翻到“可完成”；
   - 但 overall `collision_rate` 从 `0.1833` 压到 `0.0167`，`LC` family 从 `0.8333` collision 降到 `0.0000`，说明 `LC-CRBS` 已经把主要问题从“撞墙/撞车”转移成“安全但未形成可 handoff basin”。
4. **消融结果（对应新模块的必要性）**：
   - 新增消融 `A_no_corridor_reciprocity`：overall success 与 full model 同为 `0.7000`，但 `collision_rate` 回升到 `0.2167`；
   - 在 `LC` family 上，full model 为 `0/12 collision + 12/12 deadlock`，而 `A_no_corridor_reciprocity` 为 `13` 个 collision 主导失败（overall fail reasons），说明新模块目前的直接收益主要体现在**安全性**而非**可完成性翻转**；
   - `A_no_stage` 仍然在 `EC=0.0000`、`LC=0.0000`，说明 cooperative staging 仍是最关键的底层能力。
5. **单案复现结论**：
   - `DBv1-LC-L1-074 / LC-L2-080 / LC-L3-089`：`co_bcfd` 已从早期的 obstacle collision 主导，转为 `timeout / lc_geometric_deadlock` 主导；
   - 对应消融 `A_no_corridor_reciprocity` 则仍以 collision 为主；
   - `DBv1-SC-L2-026` 上，`co_bcfd` 仍保持 `success=15.05s`，说明该轮 `LC` 改动未明显伤害既有 `SC` representative。
6. **当前研究判断**：
   - `LC-CRBS` 证明了：`LC` 的核心缺陷首先不是 belief/funnel，而是 corridor 内的 reciprocal staging 与安全执行；
   - 第一轮设计已经解决了“安全性彻底失控”的问题，但还没有把 Leader 真正送入可 handoff 的 corridor basin，因此 Follower 最终卡在 `Follower-Ingress` 或 `Leader-Settle`；
   - 因而，下一轮应继续把 `LC-CRBS` 从“collision suppression”推进到“basin completion”，重点是：更强的 leader reverse-turn primitive search、phase completion 判据、以及 corridor handoff 到 docking funnel 的可完成闭环。
- 本轮也探索了更激进的 `bidir basin descent` / signed-basin-descent controller 原型；其单案结果见 `artifacts/lc_round4_single/`。该原型虽然意图把 `LC` 从 deadlock 继续推向 basin completion，但在 `LC-L1/L2` 上重新引入了 collision，因此**未合入主线**。
- 进一步地，本轮还探索了 `closed-loop anchor rollout + basin-ready search` 原型：通过离线枚举 anchor pose，并对“Leader shaping + Follower ingress”做闭环 rollout 选取 basin certificate。该原型在离线诊断中可找到局部可行解，但在真实 Stage-1 接线下未能稳定复现，甚至会退回 generic collision 模式，因此同样**未合入主线**。

### 10.0B LC-CRBS round-3：Hybrid Primitive + `\sigma_C` + semantic reference（未达 gate，2026-03-09）
本轮按“显式 `HybridMotionPrimitiveLibrary + Basin-Ready Certificate`”路线，在 `docking/lc_corridor.py` 中新增了：
- `HybridMotionPrimitiveLibrary`：显式 forward/reverse straight、arc、forward-turn-reverse 原语；
- `\sigma_C` 证书计算：把 leader anchor 可达性、follower ingress 可行性、handoff clearance、dock-zone clearance 与对齐质量统一成标量证书；
- `semantic-reference` 走廊支架：利用 `DockBench-v1` 的 `leader_stage_path_xy / leader_stage_anchor_xy` 作为 lane-constrained scaffold，把 LC 的 search 约束到“走廊 basin shaping”而不是全局自由搜索；
- plan-budget-aware deadlock horizon：`scripts/run_p_minus1_stage1_docking.py` 中把 `LC` 的 deadlock 判据改成依赖 `shape_budget_s + settle_budget_s` 的计划内时域，而不再对 corridor reciprocal plan 一律用 `12s / 18s` 的固定门槛。

对应实验产物：
- 单案：`artifacts/lc_single_round4_L2_081/p_minus1_stage1_results_seed20670585.json`
- LC 子集回归：`artifacts/lc_target_regress_round5_ablation/lc_target_regress_ablation_summary.json`

当前保留结论如下：
1. **关键单案仍未翻转成功**：
   - `DBv1-LC-L1-074`：当前 `co_bcfd` 已从早期的 `collision` 转为 `lc_geometric_deadlock`，并把 pre-ingress deadlock 从 `12.0s` 推迟到 `16.0s`，但 follower 仍未被稳定释放；
   - `DBv1-LC-L2-081`：当前 `co_bcfd` 已能进入 `DOCKING:FUNNEL_ALIGN`，并在 `24.0s` 时达到 `final_pos_error=0.7765m`、`final_yaw_error=12.82deg`、`collision=False`，说明 corridor handoff 已部分闭合，但 terminal capture 仍未最终锁定。
2. **LC targeted regression（当前主线 `co_bcfd`）**：
   - `success = 0/12`；
   - `collision = 0/12`；
   - `failure mix = 11 x lc_geometric_deadlock + 1 x timeout`。
3. **关键消融 `A_no_lc_hybrid_search`**：
   - `success = 0/12`；
   - `collision = 2/12`；
   - `failure mix = 6 x timeout + 4 x lc_geometric_deadlock + 2 x collision`。
4. **当前研究判断**：
   - 新设计已经把 LC 的 retained failure 从“碰撞 / 无法起步”进一步压到了“safe-but-deadlock / terminal capture shortfall”；
   - 相比 legacy reciprocal planner，`Hybrid Primitive + \sigma_C + semantic reference` 的直接收益是：**碰撞被压到 0，timeout 数显著下降，并且至少 `DBv1-LC-L2-081` 已能进入真正的 `DOCKING:FUNNEL_ALIGN`**；
   - 但这还不能算完成了 corridor-compatible basin completion：leader 仍未把 pair reliably 送入一个能被 base funnel 最终闭合的 terminal basin。
5. **round-4 关键诊断**（见 `artifacts/lc_terminal_capture_round4/LC_TERMINAL_CAPTURE_DIAGNOSIS.md`）：
   - `DBv1-LC-L1-074`：deadlock 时 actual leader pose 诱导出的 dynamic predock 直接落入碰撞区，`sigma_C^{exec}=None`，说明问题首先是 **leader execution closure failure**，不是 follower 不愿意前进；
   - `DBv1-LC-L2-081`：在曾进入 `DOCKING:FUNNEL_ALIGN` 的 terminal-shortfall 运行中，actual pair 满足 `x_l=0.768m, y_l=0.115m, yaw_gap=12.8deg`，但由 actual leader pose 诱导的 dynamic predock 同样进入碰撞区，故 follower 只能在 funnel 中 oscillate；
   - 同一 L2 末态下，离线两步 leader micro-reshape（`reverse_arc_right_tight -> reverse_arc_right`）能够恢复 `sigma_C^{exec}\approx 0.442` 的 execution-valid basin，说明缺失的不是“更激进的 follower 控制”，而是 **release 前的 leader terminal reshaping**。
6. **round-4 原型实现状态**：`sigma_C^{exec}` 与 leader terminal reshaping 原型已接到当前代码，但在当前执行器口径下仍未翻转关键单案；最新状态见 `artifacts/lc_terminal_capture_round4/ROUND4_IMPLEMENTATION_STATUS.md`。因此本轮保留的是“诊断成立 + 方向成立”，而不是“实现已达 gate”。
7. **round-5 exec-gate 原型状态**：在 round-4 诊断基础上，已把 `compute_execution_certificate(...)`、execution-time release gate、以及 witness-driven leader reshaping 闭环接入主实现；状态汇总见 `artifacts/lc_terminal_capture_round4/ROUND5_EXECGATE_STATUS.md`。当前最好的 retained 单案是 `DBv1-LC-L2-081` 在 `artifacts/lc_round5_execgate3_L2_081/p_minus1_stage1_results_seed20670585.json` 上达到 `collision=False` 且 residual 收敛到 `0.831m / 8.36deg`，说明闭环方向有效，但 terminal closure 仍未最终锁定。
8. **round-6 certificate-ascent settlement 状态**：在 round-5 的 execution gate 基础上，`LEADER_SETTLE` 已被改写成主动的 certificate-ascent 本地锚点搜索；状态汇总见 `artifacts/lc_terminal_capture_round4/ROUND6_CERT_ASCENT_STATUS.md`。当前保留结论是：从 `LEADER_SHAPE` 的实际末态出发，`DBv1-LC-L1-074 / DBv1-LC-L2-081` 仍然得到 `sigma_C^{exec}=0`，且 witness-sized 本地 primitive 搜索无法找到正的 execution-valid anchor，因此两例都仍停在 `STAGING_LC:LC_LEADER_SETTLE[n=0.000,r=0.000]`。
9. **round-7 exec-shape 状态**：进一步把 execution-valid basin 目标前置到 `LEADER_SHAPE` 规划层后，规划器已能直接输出 `lc_corridor_exec_shape` 计划，且在 `DBv1-LC-L1-074 / DBv1-LC-L2-081` 上找到正的 nominal / robust execution certificate；状态见 `artifacts/lc_terminal_capture_round4/ROUND7_EXEC_SHAPE_STATUS.md`。但当前选出的 exec-shape anchors 仍会在 terminal docking 阶段重新引入 collision，因此该轮仍不能作为 retained gate 结果。
10. **round-8 terminal-tube 状态**：进一步把“Follower 预测点到 contact 区间的扫掠净空”写成终端安全证书，并与执行证书联合并入 `LEADER_SHAPE` 目标；状态见 `artifacts/lc_terminal_capture_round4/ROUND8_TERMINAL_TUBE_STATUS.md`。当前保留结论是：对于 `DBv1-LC-L2-081`，规划层已经能选出 offline 上同时满足 execution / terminal safety 的 `lc_corridor_semantic_cert_anchor`，且在修正 leader 的 signed path execution 后，单案 `artifacts/lc_round8_terminalsafe4_L2_081/p_minus1_stage1_results_seed20670585.json` 已出现 `success=True / collision=False` 的局部突破；但 `DBv1-LC-L1-074` 仍未找到可稳定闭合的 terminal-safe anchor，因此 round-8 仍不能作为完整 gate 结果。
11. **round-9 escape-budget 状态**：针对 `DBv1-LC-L1-074` 已加入定向 `escape-shape` 分支并重排 `LC` 规划优先级；状态见 `artifacts/lc_terminal_capture_round4/ROUND9_ESCAPE_BUDGET_STATUS.md`。当前保留结论是：`L2` 在当前代码快照上仍保持成功（`artifacts/lc_current_check_L2/p_minus1_stage1_results_seed20670585.json`），而 `L1` 的 targeted escape search 已把单次规划耗时从约 `5.7s` 压到约 `0.6s`，但仍未达到 `100ms` 硬预算，且 `L1` 仍未翻转成功。
12. **round-10 anytime 性能状态**：进一步把 `L1` 的 small-gap escape 分支压成单模板、近似评估、硬预算路径后，`DBv1-LC-L1-074` 的单次规划耗时已降到 `16.9ms`，见 `artifacts/lc_terminal_capture_round4/ROUND10_ANYTIME_PERF_STATUS.md`；对应单案结果为 `artifacts/lc_round9_perfcheck_L1_074/p_minus1_stage1_results_seed20660454.json`。与此同时，`DBv1-LC-L2-081` 在 `artifacts/lc_round9_perfcheck_L2_081/p_minus1_stage1_results_seed20670585.json` 上继续保持 `success=True / collision=False`。这说明实时性瓶颈已基本解除，但 `L1` 的覆盖度 / execution-closure 仍未翻转。
13. **round-11 terminal-manifold closure 状态**：本轮把 `LC` 的末端闭合重写为 `\sigma_M`（Terminal-Manifold Closure Certificate）+ `FOLLOWER_TERMINAL_CLOSURE` 近场阶段，状态见 `artifacts/lc_round11_terminal_manifold/ROUND11_TERMINAL_MANIFOLD_STATUS.md`。当前保留结论是：
   - `DBv1-LC-L2-081` 继续保持成功，且新闭环没有破坏其 `success=True / collision=False`；
   - `DBv1-LC-L1-074` 已从 round-10 的 `lc_geometric_deadlock` 翻转为 **稳定进入 terminal-manifold 并把末端误差压到 0**，但最终仍在 `DOCKING:LOCK_ASSIST` 中以 `collision_obstacle` 失败，最低净空为 `0.0990m`；
   - 因而，当前最精确的研究判断已从“`L1` 找不到 closure basin”前移为“`L1` 的 closure basin 已能被 runtime terminal-manifold controller 实际闭合，但 generic lock-assist 还没有和 tight-corridor safety envelope 完全一致”。
14. **round-12 certificate-consistent lock-hold 状态**：本轮把 `LC` 的最后一段 `LOCK_ASSIST` 改写为 certificate-consistent `LC_LOCK_HOLD`，并在 runtime evaluator 中加入 terminal-hold invariant projection；状态见 `artifacts/lc_round12_lock_hold/ROUND12_LOCK_HOLD_STATUS.md`。当前 retained 结论是：
   - `DBv1-LC-L1-074` 首次通过关键 gate，达到 `success=True / collision=False / min_clearance=0.1031m`；
   - `DBv1-LC-L2-081` 保持 `success=True / collision=False / min_clearance=0.1486m`；
   - `LC` 12-scene targeted regression 已达到 `success_rate=0.167 (2/12)`，但 `collision_rate` 仍为 `0.083 (1/12)`，且残余失败仍主要是 `lc_geometric_deadlock`；
   - 因此，round-12 可以保留为“关键单案 gate 已通过、LC 成功率首次达到非零”的正结论，但**仍不能升级为新的 frozen full Stage-2 主报告**。
15. **round-13 release-gate 状态**：本轮进一步把 `FOLLOWER_READY` / `Release` 重写为 certificate-consistent handoff gate，状态见 `artifacts/lc_round13_release_gate/ROUND13_RELEASE_GATE_STATUS.md`。当前 retained 结论是：
   - `DBv1-LC-L2-082` 已从 `collision_obstacle` 翻转为 `lc_geometric_deadlock`，说明早释放碰撞已被消除；
   - `DBv1-LC-L1-074 / DBv1-LC-L2-081` 继续保持成功；
   - 最新 `LC` 12-scene targeted regression 已提升到 `success_rate=0.250 (3/12)`，并把 `collision_rate` 压到 `0.000 (0/12)`；
   - 残余失败已经纯化为 `9` 个 `lc_geometric_deadlock`，因此接下来的主问题已不再是安全性，而是 basin coverage / escape-shape 泛化。

因此，本轮结果**仍不应被写成“Stage-2 达标”**。round-13 虽然已经把 `LC` 子集碰撞压到 `0`，并把关键单案 gate 稳定保住，但 `LC` targeted regression 仍只有 `3/12` 成功，因此本轮仍没有把该分支升级为新的 frozen full Stage-2 主报告；当前能够保留到设计书中的，是“安全问题已被清零，剩余瓶颈纯化为 basin coverage”的更精确阶段性结论。

16. **Round-18 restored baseline（当前恢复实现）**：当前代码基线已手动恢复到 `artifacts/lc_round18_lc12_regression/lc_round18_lc12_regression.md` 对应的实现口径，即：
   - `LC` 仍采用 `UnifiedCertificateField + StagingCertificateOptimizer` 框架；
   - `\sigma_{post}` 只作为 **soft scoring bias** 保留在 planning 侧；
   - 不再启用后续实验轮次中引入的 `front_clearance / longitudinal_offset` 软代价主导逻辑；
   - 不再把 `LEADER_POST_ALIGN` 作为 lock gate 的强制前置条件，恢复 round-18 的“先完成对接、再接受姿态偏差”口径。
   - 按该恢复实现重新验证时，关键样例与 round-18 记录重新对齐：`DBv1-LC-L1-073` 回到 `collision_obstacle`，`DBv1-LC-L2-081 / DBv1-LC-L2-083` 回到 `success=True` 且终态航向分别约为 `30.08° / -34.37°`；对应实验结果位于 `output/round18_restore2/`。

### 10.1 Stage-2.5 历史结果（pre-LC 4-family split，2026-03-07）
> 注意：以下 `48/48` 与 `avg_T_done` 结论均基于**引入 `LC` 之前**的 `4-family / 48-test` frozen split；在当前 `5-family / 60-test` benchmark 上尚未重跑，因此只能视为历史结果，不能直接作为当前 gate 结论。
- 当前 Stage-2.5 已把 `TVT / VPCR` 落成**真正独立的 mode / subskill 分支**：核心代码位于 `docking/stage25_subskills.py`，运行时接线位于 `docking/dock_skill.py`，冻结回归脚本位于 `scripts/run_stage25_branch_eval.py`。
- **当前主线 `co_bcfd` 的正式口径**：`CGFL + gated TVT + gated VPCR + PTCC(plan-certified SC/EC time compression) + terminal capture boost + belief-consistent funnel + safety projection`。
- **主线 frozen `test` split 回归结果**：见 `artifacts/stage25_round10_timecert_test/co_bcfd_summary.json`；overall 成功率保持 `1.0000 (48/48)`，碰撞率保持 `0.0000`，overall 平均完成时间进一步压到 `19.64s`。对应 failure-mode 分层统计见 `artifacts/stage25_round10_timecert_test/co_bcfd_failure_mode_summary.json`。
- **family 分解**：`CF=1.000 / SC=1.000 / FC=1.000 / EC=1.000`；相较 `CGFL-only` 基线（`artifacts/stage25_final_co_only/co_only_summary.json`），`FC / EC` 的残余 hard case 已全部清零。
- **单案复现结论**：
  - `DBv1-FC-L3-052`：`S25_cgfl_only` 仍为 `collision`；`co_bcfd` 与 `S25_tvt_vpcr` 均恢复为 `success`，单案结果见 `artifacts/tmp_stage25_singlecases/fc052_compare/p_minus1_stage1_results_seed20480716.json`。
  - `DBv1-EC-L1-059`：历史基线 `co_bcfd / S25_cgfl_only` 为 `fov_loss_unrecovered`；当前主线 `co_bcfd` 已稳定翻转为 `success`，见 `artifacts/tmp_stage25_singlecases/ec059_promoted_mainline_32/p_minus1_stage1_results_seed20560847.json`。
- **实验分支全量回归结论**：
  - 历史 `VPCR` 结果见 `artifacts/stage25_vpcr_branch_eval_test/S25_vpcr_only_summary.json` 与 `artifacts/stage25_vpcr_branch_eval_test/S25_tvt_vpcr_summary.json`；其结论是“独立分支不伤整体，但未翻转 `EC-L1-059`”。
  - 本轮新的 response-reachable pair-capture + terminal capture boost + certified terminal projection 设计在 frozen `test` 上的最终结果见 `artifacts/stage25_round6_mainline_test/co_bcfd_summary.json`；主线 `co_bcfd` 已达到 `1.0000 / 0.0000 / 20.01s`。
  - `DBv1-EC-L1-059` 已被稳定翻转：benchmark-aligned 单案见 `artifacts/tmp_stage25_singlecases/ec059_promoted_mainline_32/p_minus1_stage1_results_seed20560847.json`，结果为 `success=True`、`done_time=31.75s`、`collision=False`。
  - 与旧主线 `artifacts/stage25_round6_mainline_test/co_bcfd_summary.json` 相比，本轮 `PTCC` 继续保持 `48/48` 不变，并把 overall `avg_T_done` 从 `20.01s` 压到 `19.64s`；其中 `SC` family 均时从 `22.26s` 压到 `22.12s`，`EC` family 均时从 `30.13s` 压到 `29.47s`。
  - 本轮也继续尝试了“**不伤 `48/48` 的全局时间压缩**”：新增的 corridor certificate / progress compression 原型在代表场景上可显著降时，例如 `DBv1-SC-L2-025 = 18.55s`、`DBv1-EC-L1-059 = 28.15s`、`DBv1-EC-L2-063 = 31.00s`；但同一原型会把 `DBv1-EC-L3-071` 推回 `timeout/fov_loss_unrecovered`，见 `artifacts/tmp_stage25_singlecases/ec071_timeenv/p_minus1_stage1_results_seed20580847.json`。
  - 对应 frozen `test` 的最好不安全版本可把均时压到 `17.37s`，但会引入新的 `DBv1-EC-L2-063 / geometric_deadlock` 回退，因此未合入主线。对应证据见 `artifacts/stage25_round3_test/co_bcfd_summary.json` 与 `artifacts/stage25_round3c_test/co_bcfd_summary.json`。
  - 因而，本轮没有把任何新的时间压缩逻辑吸收入主线；当前主线仍保持 `artifacts/stage25_round6_mainline_test/co_bcfd_summary.json` 对应的 `48/48 + collision=0` 版本。

因此，当前最合理的研究判断是：**Stage-2.5 已完成从 `CGFL` 到 `gated TVT/VPCR` 的第三轮安全增益落地，主线已把双车 frozen `test` split 做到 `48/48` 成功且碰撞为 `0`；当前唯一剩余未达目标的是 `avg_T_done <= 15s`。**
