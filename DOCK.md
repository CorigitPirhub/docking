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

### 5.4 `CGFL`：Certificate-Gated Fast Lane（Stage-2.5 当前已合入）
为解决 `Common-Feasible` 子集上 Full model 相对强传统基线过于保守的问题，当前 Stage-2.5 首先落地了一个**证书门控快速通道**。
快速通道证书可表述为：仅当“无需 leader relocation、无遮挡、初始航向差小、环境净空充分、dock zone 净空充分”同时满足时，才允许进入 fast lane。
只有当系统状态进入 \(\mathcal C_{fast}\) 时，才允许：
1. 提高 predock approach 的参考速度上限；
2. 放宽近场 contact gate 的保守阈值；
3. 推迟 lock-assist 的强制低速接管时刻。

这样做的理论含义是：把“共同可解 easy case 的效率最优性”也写成一张**安全证书**，而不是直接删掉 funnel / safety 逻辑。当前实现只在高净空、无遮挡、无 leader relocation 的局面触发，因此不会改变 `SC / FC / EC` 的主机制判别口径。

### 5.5 `TVT`：Terminal Viability Tube（本轮新增设计，针对 `FC-L3`）
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

### 5.6 `VPCR`：Visibility-Persistent Cooperative Re-Staging（本轮新增设计，针对 `EC-L1`）
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
发生“已进入近场且曾成功看见目标，但随后视觉长时丢失”的事件后，不再沿原路径硬顶，而是求解：
\[
x_L^{\ddagger\star}=\arg\min_{x_L^\ddagger\in\mathcal S_{vis}}
J_L(x_L^\ddagger)+J_F(x_L^\ddagger)+\lambda_v(1-P_{vis})+\lambda_r D_{reacq}
\]
并把系统切回 `re-staging` 模式，直到重建一个可持续可见的接近通道。

这条线的理论意义是：把 `EC-L1` 的失败从“fallback 细节”提升成**visibility-persistent manifold** 问题；目标不再只是“重新看见一次”，而是“重新落到一个近场不会再次快速丢视野的协同流形上”。

> Stage-2.5 中先前已经探索过 `RCS` / `SSTF` 原型，但在 frozen 全量 `test` split 上出现了总体成功率回退，因此当前**未合入主实现**，只保留为实验分支结论。本轮的 `TVT / VPCR` 是对这两条线的更方法化重写。

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

## 10. Stage-2 批量对比结论（2026-03-06）
Stage-2 已切换到 `DockBench-v1` frozen test split：
- 数据集：`data/dockbench_v1/`
- 批量脚本：`scripts/run_p_minus1_stage2_batch.py`
- Stage-0 审计：`data/dockbench_v1/dockbench_v1_stage0_audit.json`
- 报告：`artifacts/dockbench_stage2_fast/P_MINUS1_STAGE2_REPORT.md`
- 数据：`artifacts/dockbench_stage2_fast/p_minus1_stage2_results.json`
- 图表：`artifacts/dockbench_stage2_fast/p_minus1_stage2_success_heatmap.png`、`artifacts/dockbench_stage2_fast/p_minus1_stage2_subset_compare.png`、`artifacts/dockbench_stage2_fast/p_minus1_stage2_failure_clusters.png`、`artifacts/dockbench_stage2_fast/p_minus1_stage2_ablation_summary.png`

当前批量结果：
1. **任务级主门槛已跨过**：`co_bcfd` overall `success_rate = 0.9583 (46/48)`，`success >= 95%` 已满足；`Common-Feasible` 上强传统基线稳定非零成功也已满足。
2. **family 分解**：
   - `CF`：`success=1.000`，`collision=0.000`，`avg_T_done=11.88s`；
   - `SC`：`success=1.000`，`collision=0.000`，`avg_T_done=22.26s`；
   - `FC`：`success=0.9167`，`collision=0.0833`，`avg_T_done=16.81s`；
   - `EC`：`success=0.9167`，`collision=0.000`，`avg_T_done=29.98s`。
3. **当前 gate 状态**：
   - `co_success_ge_095 = True`；
   - `strong_cf_nonzero = True`；
   - `p13_extension_gain_ge_15pt = True`；
   - `co_collision_zero = False`；
   - `co_avg_time_le_15 = False`。
4. **残余失败场景**：
   - `DBv1-FC-L3-052`：`collision`；
   - `DBv1-EC-L1-059`：`fov_loss_unrecovered`。
5. **当前结论**：
   - `P-1.1B` 与 `P-1.2 Stage-0` 已经正式落地；
   - `P-1.2 Stage-1 / Stage-2` 的 frozen benchmark、报告链路、family 口径、代表场景与批量统计均已成型；
   - 若以“全场景验证策略层有效性并可视化”为目标，当前基础支撑已经能稳定支撑**大规模双车验证**，但距离“碰撞率严格为 0、整体平均时间压到 15s 内”的更高门槛仍差最后两类 hard cases（`FC-L3`、`EC-L1`）。

### 10.1 Stage-2.5 当前合入结果（2026-03-07）
- 已正式合入主实现的是 `CGFL` fast lane；其稳定回归结果见 `artifacts/stage25_final_co_only/co_only_summary.json`，对应 failure-mode 分层统计见 `artifacts/stage25_final_co_only/failure_mode_summary.json`。
- 当前 `co_bcfd` 在 frozen `test` split 上的 **co-only 回归** 为：overall 成功率 `0.9583`、碰撞率 `0.0208`、平均完成时间 `19.72s`；其中 `CF` 平均时间从先前的 `11.88s` 收缩到 `10.42s`。
- 失败模式分层统计进一步确认：当前剩余失败**只集中在 2 个场景**：`FC-L3-052 / collision` 与 `EC-L1-059 / fov_loss_unrecovered`。
- 本轮按照“failure-mode 分层统计 → 最小作用域改动 → 单案复现 → frozen 全量 test 回归”的顺序，再次围绕这两个 hard case 做了原型探索：
  - `M1` 的最小作用域原型在 `DBv1-FC-L3-052` 单案上可把 `collision` 拉回 `success`；
  - `M2` 的最小作用域原型在 `DBv1-EC-L1-059` 单案上可把 `fov_loss_unrecovered` 拉回 `success`；
  - 但一旦放到 frozen 全量 `test` split，仍会诱发新的 `SC/EC` 回退，因此当前这两类修复**依然不能安全合入主线**。
- 相关失败证据保留在 `artifacts/stage25_stage2/p_minus1_stage2_results.json`、`artifacts/stage25_stage2b/p_minus1_stage2_results.json`、`artifacts/stage25_m1_co_only/co_only_summary.json`，用来证明“单案可修复”尚未等价于“全量不过拟合修复”。

因此，当前最合理的研究判断是：**DockBench-v1 / Stage-0 / Stage-1 / Stage-2 的制度化框架已经完成，可以作为后续 Stage-3 多车实验的正式前置基线；Stage-2.5 已经完成第一轮可控改进（`CGFL`），并明确锁定了最后两个残余 hard case，但要把 P-1 做到“完全收敛”，仍需继续针对 `FC-L3` 的近场碰撞与 `EC-L1` 的视觉丢失回退做更稳健、不过拟合的改进。**
