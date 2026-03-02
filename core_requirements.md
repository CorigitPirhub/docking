# 核心需求

# 动态可重组多智能体系统的时空资源协同优化与自适应编队规划问题

## 1. 系统与符号

设车辆集合为 \( \mathcal{V}=\{1,\dots,N\} \)，离散时间 \(k=0,1,\dots,K\)，步长 \(\Delta t\)。

1. 单车连续状态  
\[
x_i^k=[p_{x,i}^k,\ p_{y,i}^k,\ \psi_i^k,\ v_i^k,\ \delta_i^k]^\top
\]

2. 编队拓扑（可重组）  
\[
G^k=(\mathcal{V},E^k),\quad (j,i)\in E^k \Rightarrow i\ \text{挂接在}\ j\ \text{后}
\]

3. 模式变量  
\[
m_i^k \in \{\text{FREE},\ \text{DOCKING},\ \text{TRAIN\_FOLLOW},\ \text{WAIT},\ \text{SPLIT}\}
\]

4. 场景  
静态障碍集合 \(\mathcal{O}\)，边界 \(\partial\Omega\)，起点任务区域 \(A\)，目标区域 \(B\)。

5. 观测  
\[
\hat x_i^k = x_i^k + \xi_i^k,\quad \xi_i^k \sim \mathcal{N}(0,\Sigma_i)
\]

## 2. 决策变量（动作）

联合动作 \(a^k=(u^k,z^k)\)。

1. 连续控制  
\[
u_i^k=[a_i^k,\ \dot\delta_i^k]^\top \ \text{或}\ [v_{ref,i}^k,\delta_{ref,i}^k]^\top
\]

2. 离散重组决策 \(z^k\)  
\[
z^k=\{y_{i\to j}^k,\ w_c^k,\ s_e^k,\ q_i^k\}
\]

其中：
- \(y_{i\to j}^k\in\{0,1\}\)：是否发起“\(i\) 对接 \(j\)”  
- \(w_c^k\in\{0,1\}\)：编队 \(c\) 是否等待（前车可停）  
- \(s_e^k\in\{0,1\}\)：是否在连接边 \(e\) 处解编  
- \(q_i^k\)：车辆 \(i\) 的目标对象/子任务分配

## 3. 混合动态系统

1. 单车阿克曼动力学（FREE/DOCKING）  
\[
x_i^{k+1}=f_{\text{ack}}(x_i^k,u_i^k)
\]

2. 列车态动力学（TRAIN\_FOLLOW）  
\[
x_{c}^{k+1}=f_{\text{train}}(x_c^k,u_{\text{head}(c)}^k,G^k)
\]

3. 拓扑转移（事件触发）  
\[
E^{k+1}=E^k \cup \{(j,i)\}\ \text{if}\ y_{i\to j}^k=1\ \land\ \mathcal{D}_{i,j}(x^k)=1
\]
\[
E^{k+1}=E^k \setminus \{e\}\ \text{if}\ s_e^k=1\ \land\ \mathcal{S}_{e}(x^k)=1
\]

其中 \(\mathcal{D}_{i,j}\) 是对接达成门限，\(\mathcal{S}_e\) 是安全解编门限。

## 4. 目标函数（时空资源协同优化）

定义任务完成时间 \(T_{\text{done}}\)：所有运输单元到达 \(B\) 且拓扑合法的最早时刻。  
定义总能耗 \(E_{\text{tot}}\)（含列车节能效应）：
\[
E_{\text{tot}}=\sum_{k}\sum_{i} P_i^k \Delta t,\quad
P_i^k=
\begin{cases}
P_i^{\text{single}}(x_i^k,u_i^k), & i\ \text{单车}\\
\eta(n_c,\kappa,v)\,P_i^{\text{single}}(x_i^k,u_i^k), & i\in c,\ 0<\eta<1
\end{cases}
\]

优化目标可写为：
\[
\min J=
w_T T_{\text{done}}
+w_E E_{\text{tot}}
+w_W T_{\text{wait}}
+w_R N_{\text{reconfig}}
+w_S R_{\text{safety}}
+w_F \mathbf{1}_{\text{fail}}
\]

其中 \(N_{\text{reconfig}}\) 统计对接/解编次数，\(R_{\text{safety}}\) 为最小净空与高风险行为惩罚项。

## 5. 约束条件

1. 运动学与执行器约束  
\[
|v_i^k|\le v_{\max},\ |a_i^k|\le a_{\max},\ |\delta_i^k|\le \delta_{\max},\ |\dot\delta_i^k|\le \dot\delta_{\max}
\]

2. 障碍与边界安全  
\[
\mathcal{B}_i(x_i^k)\cap(\mathcal{O}\cup\partial\Omega)=\emptyset
\]

3. 车车防碰撞  
\[
\mathcal{B}_i(x_i^k)\cap\mathcal{B}_j(x_j^k)=\emptyset,\ \forall i\neq j
\]
合法铰接接触仅在门限内例外。

4. 防折刀约束（列车态）  
\[
|\phi_e^k|\le \phi_{\max},\ \forall e\in E^k
\]

5. 对接约束  
\[
\|p_{h,i}-p_{t,j}\|\le \epsilon_p,\ |\psi_i-\psi_j|\le \epsilon_\psi,\ |v_i-v_j|\le \epsilon_v
\]
并持续 \(\tau_{\text{hold}}\)。

6. 通行性约束（是否需解编）  
\[
W_{\text{env}}(\pi_c)\ge W_{\text{req}}(n_c,\kappa)\ \text{若不满足则必须触发 split}
\]

7. 时间窗口/任务约束  
\[
T_{\text{done}}\le T_{\max}\ \text{(若设时限)}
\]

## 6. 可解框架（推荐）

这是一个带不确定性的 MINLP / Hybrid Optimal Control 问题，直接全局求解代价过高。推荐分层滚动优化：

1. 高层（0.5–2 Hz）：重组与任务协同  
输出 \(z^k\)：何时对接、何时等待、何时解编、谁对接谁。  
方法：MILP/MIQP（近似线性化）或 MCTS/分支定界。

2. 中层（5–10 Hz）：模式自适应轨迹规划  
单车用非完整约束局部规划；列车用编队整体规划与通过性检查；对接车用预测截获轨迹。

3. 低层（10–50 Hz）：跟踪与安全滤波  
跟踪控制 + 安全投影（CBF/QP 或采样投影）得到 \(U_{\text{safe}}\)，保障无碰撞与防折刀。

4. 事件触发重规划  
触发条件：预测失配、FOV 丢失、通道不可达、可完成性判定翻转、风险超阈值。

## 7. 可直接落地的“问题分解”

可作为研究子问题 \(P1\sim P6\)：

1. \(P1\)：重组时机优化（合/等/散切换策略）
2. \(P2\)：多车对接序列与并发调度优化
3. \(P3\)：列车节能收益模型 \(\eta(\cdot)\) 的在线估计与利用
4. \(P4\)：窄通道可通过性判定与解编决策
5. \(P5\)：混合模式下的安全可行域 \(U_{\text{safe}}\) 实时构造
6. \(P6\)：时间-能耗-安全多目标权重自适应调参与稳定性分析
