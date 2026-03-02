# P0 文献调研综述（策略层创新框架前置）

## 1. 调研目标与执行口径

本调研用于支撑 `CORE_REQUIREMENTS_TASKBOOK.md` 的 P0 Gate-0：

1. 建立可复用知识基线，覆盖“任务分配、重构控制、对接回退、多目标优化”四条主线。
2. 明确可直接迁移到本平台的机制，避免“工程堆叠式拼装”。
3. 为后续 `reviews/P0_FRAMEWORK_DESIGN.md` 提供证据链。

调研范围与约束：

1. 场景：静态障碍、全局共享状态、阿克曼与列车约束。
2. 方法：非学习 + 学习两类均覆盖，且包含可落地的安全约束方法。
3. 数据源：期刊/会议 DOI 条目 + arXiv 预印本（用于前沿趋势扫描）。

---

## 2. 覆盖统计（对照任务书量化指标）

### 2.1 数量统计

1. 文献总数：`53`（达标，要求 `>=30`）
2. 近5年文献（2021-2026）：`39`（达标，要求 `>=20`）
3. 高相关文献（直接对应 MRTA/重构/对接/列车控制）：`44`（达标，要求 `>=15`）

### 2.2 方法对比覆盖

基线/竞品方法族：`8` 套（达标，要求 `>=6`）

1. 非学习方法：`5` 套（CBBA/拍卖、MILP-MIQP、MAPF-TAPF、规则+可行性门控、DMPC）
2. 学习方法：`3` 套（MARL、Decision-Focused Learning、RL-Servo）

---

## 3. 方法族对比（用于后续策略层基线）

| 方法族 | 类别 | 代表文献 | 优势 | 风险/短板 | 对本题定位 |
|---|---|---|---|---|---|
| CBBA/拍卖式任务分配 | 非学习 | [L04], [L07], [L09] | 实时性好，分布式实现成熟 | 局部最优，复杂约束表达弱 | 作为 `P2` 高实时基线 |
| MILP/MIQP 滚动分配 | 非学习 | [L10], [L38], [L47] | 约束表达强，可解释 | 计算开销高，需时域裁剪 | 作为 `P2` 精确对照线 |
| MAPF/TAPF + 任务耦合 | 非学习 | [L15], [L16], [L17], [L18] | 路径-分配一体化，冲突显式 | 动态重构事件接入复杂 | 作为“可达性与冲突先验”模块 |
| DMPC/触发式协同控制 | 非学习 | [L20]-[L27] | 与多率执行契合，安全性强 | 参数调谐敏感，模型依赖高 | 作为规控层主力实现族 |
| 规则门控（可完成性+通行性） | 非学习 | [L12]-[L14], [L28] | 工程稳健，易验证 | 上界受限，策略保守 | 作为策略安全兜底 |
| MARL（CTDE/图结构） | 学习 | [L35], [L36], [L48], [L49], [L50] | 适配复杂耦合与长期收益 | 样本成本高，可解释性弱 | 作为中长期增强方向 |
| Decision-Focused Learning | 学习 | [L37] | 直接优化决策目标，减少代理误差 | 训练管线复杂 | 候选创新点之一 |
| RL-Servo 融合对接 | 学习 | [L34], [L53] | 感知误差鲁棒性潜力高 | 安全约束需外层投影 | 用于对接末端增强 |

---

## 4. 深入阅读后的关键共识（跨论文联想）

1. 仅靠“谁对接谁”的离散决策不足以稳定落地，必须联动规控层可行性反馈，形成闭环（[L04], [L09], [L20]-[L27]）。
2. 在多车重构场景里，路径冲突与任务分配是耦合的，先分后规或先规后分都容易失败，需弱耦合联合求解（[L15]-[L18], [L10]）。
3. 头车默认不等待是合理基线，但必须保留“短时等待权”作为优化动作，否则在瓶颈前后会出现次优序列（[L20], [L24], [L28]）。
4. 纯学习策略难以直接满足安全硬约束，必须叠加可行域投影/安全滤波层（[L26], [L27], [L35], [L36]）。
5. 对接应采用“预测截获路径点”而非纯尾随，且对接失败后要快速回退，避免任务时间窗被吞噬（[L29]-[L34], [L53]）。
6. 窄道和弯道约束使编组规模呈时空变化，策略层应显式处理 split/dock 窗口，而非静态全局规划（[L28], [L42], [L47]）。
7. 前沿 MRTA 正在从“离线最优”转向“在线可解释+可验证”的决策框架，决策聚焦学习是一个可行方向（[L37]-[L41], [L44], [L45]）。

---

## 5. 对本课题的直接设计约束提炼

1. 策略层输入必须包含：拓扑状态、可达性摘要、任务可完成性预算、场景约束标签。
2. 策略层输出必须是离散命令而非轨迹：`DockingCommand/SplitCommand/WaitCommand`。
3. 命令协议必须具备：版本、TTL、优先级、撤销、失败码。
4. 异步一致性必须具备：冲突仲裁与抖振抑制（禁止“刚对接立即解编”循环）。
5. 学习策略不得直接越权到底层执行器，必须经过规控层可行性过滤。

---

## 6. 文献清单（编号可追溯）

### 6.1 任务分配与重构（MRTA）

- [L01] Gerkey, B. P., & Matarić, M. J. (2004). *A Formal Analysis and Taxonomy of Task Allocation in Multi-Robot Systems*. IJRR. https://doi.org/10.1177/0278364904045564
- [L02] Gerkey, B. P., & Matarić, M. J. (2003). *Multi-Robot Task Allocation in Uncertain Environments*. Autonomous Robots. https://doi.org/10.1023/A:1022291921717
- [L03] Dias, M. B., et al. (2006). *Market-Based Multirobot Coordination: A Survey and Analysis*. Proceedings of the IEEE. https://doi.org/10.1109/JPROC.2006.876939
- [L04] Choi, H.-L., Brunet, L., & How, J. P. (2009). *Consensus-Based Decentralized Auctions for Robust Task Allocation*. IEEE TRO. https://doi.org/10.1109/TRO.2009.2022423
- [L05] Korsah, G. A., Stentz, A., & Dias, M. B. (2013). *A comprehensive taxonomy for multi-robot task allocation*. IJRR. https://doi.org/10.1177/0278364913496484
- [L06] Nunes, E., et al. (2015). *Multi-robot Task Allocation: A Review of the State-of-the-Art*. Springer. https://doi.org/10.1007/978-3-319-18299-5_2
- [L07] Kalra, N., et al. (2015). *Multi-Robot Auctions for Allocation of Tasks with Temporal Constraints*. AAAI. https://doi.org/10.1609/aaai.v29i1.9440
- [L08] Tang, J., et al. (2017). *Distributed Task Rescheduling With Time Constraints for the Optimization of Total Task Allocations in a Multirobot System*. IEEE TCYB. https://doi.org/10.1109/TCYB.2017.2743164
- [L09] Bai, X., et al. (2020). *A Distributed Approach to the Multi-Robot Task Allocation Problem Using the Consensus-Based Bundle Algorithm and Ant Colony System*. IEEE Access. https://doi.org/10.1109/ACCESS.2020.2971585
- [L10] Das, P., et al. (2023). *Optimization techniques for Multi-Robot Task Allocation problems: Review on the state-of-the-art*. Robotics and Autonomous Systems. https://doi.org/10.1016/j.robot.2023.104492
- [L11] Khamis, A., et al. (2020). *Review on state-of-the-art dynamic task allocation strategies for multiple-robot systems*. Industrial Robot. https://doi.org/10.1108/IR-04-2020-0073
- [L37] FORMICA (2026). *Decision-Focused Learning for Communication-Free Multi-Robot Task Allocation*. arXiv. https://arxiv.org/abs/2602.18622
- [L38] (2026). *Modeling and Optimizing the Provisioning of Exhaustible Capabilities for Simultaneous Task Allocation and Scheduling*. arXiv. https://arxiv.org/abs/2602.13866
- [L39] (2025). *Uncertainty-Aware Multi-Robot Task Allocation With Strongly Coupled Inter-Robot Rewards*. arXiv. https://arxiv.org/abs/2509.22469
- [L40] (2025). *Online Multi-Robot Coordination and Cooperation with Task Precedence Relationships*. arXiv. https://arxiv.org/abs/2509.15052
- [L41] (2025). *Efficient Human-Aware Task Allocation for Multi-Robot Systems in Shared Environments*. arXiv. https://arxiv.org/abs/2508.19731
- [L42] (2025). *Very Large-scale Multi-Robot Task Allocation in Challenging Environments via Robot Redistribution*. arXiv. https://arxiv.org/abs/2506.07293
- [L43] (2025). *Multi-Robot Task Allocation for Homogeneous Tasks with Collision Avoidance via Spatial Clustering*. arXiv. https://arxiv.org/abs/2505.10073
- [L44] (2025). *MRTA-Sim: A Modular Simulator for Multi-Robot Allocation, Planning, and Control in Open-World Environments*. arXiv. https://arxiv.org/abs/2504.15418
- [L45] (2025). *Automatic MILP Model Construction for Multi-Robot Task Allocation and Scheduling Based on Large Language Models*. arXiv. https://arxiv.org/abs/2503.13813
- [L46] (2025). *Energy-Aware Task Allocation for Teams of Multi-mode Robots*. arXiv. https://arxiv.org/abs/2503.12787
- [L47] (2025). *Scalable Multi-Robot Task Allocation and Coordination under Signal Temporal Logic Specifications*. arXiv. https://arxiv.org/abs/2503.02719

### 6.2 路径规划与任务-路径耦合

- [L12] Khorshid, M. M., et al. (2021). *Trajectory planning for multi-robot systems: Methods and applications*. Expert Systems with Applications. https://doi.org/10.1016/j.eswa.2021.114660
- [L13] Mia, M. M., et al. (2022). *A Critical Review of Communications in Multi-robot Systems*. Journal of Intelligent & Robotic Systems. https://doi.org/10.1007/s43154-022-00090-9
- [L14] Zornoza, C., et al. (2023). *Survey of Methods Applied in Cooperative Motion Planning of Multiple Robots*. IntechOpen. https://doi.org/10.5772/intechopen.1002428
- [L15] Sharon, G., et al. (2014). *Conflict-based search for optimal multi-agent pathfinding*. Artificial Intelligence. https://doi.org/10.1016/j.artint.2014.11.006
- [L16] Ma, H., & Koenig, S. (2016). *Optimal Target Assignment and Path Finding for Teams of Agents*. AAMAS. https://doi.org/10.5555/2936924.2937092
- [L17] Ma, H., et al. (2017). *Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks*. AAMAS. https://doi.org/10.5555/3091125.3091243
- [L18] Li, J., et al. (2021). *Lifelong Multi-Agent Path Finding in Large-Scale Warehouses*. AAAI. https://doi.org/10.1609/aaai.v35i13.17344

### 6.3 列车/编队控制、MPC 与安全

- [L20] Zhang, C., et al. (2021). *Model Predictive Control for Connected Vehicle Platoon Under Switching Communication Topology*. IEEE T-ITS. https://doi.org/10.1109/TITS.2021.3073012
- [L21] Jiang, H., et al. (2021). *Robust Platoon Control in Mixed Traffic Flow Based on Tube Model Predictive Control*. IEEE T-IV. https://doi.org/10.1109/TIV.2021.3060626
- [L22] Xie, Y., et al. (2022). *Data-Driven Modeling and Distributed Predictive Control of Mixed Vehicle Platoons*. IEEE T-IV. https://doi.org/10.1109/TIV.2022.3168591
- [L23] Dörfler, F., et al. (2022). *Robust Min-Max Model Predictive Vehicle Platooning With Causal Disturbance Feedback*. IEEE T-ITS. https://doi.org/10.1109/TITS.2022.3146149
- [L24] Wang, J., et al. (2022). *String Stable and Collision-Safe Model Predictive Platoon Control*. IEEE T-ITS. https://doi.org/10.1109/TITS.2022.3160236
- [L25] Hu, J., et al. (2022). *Distributed Model Predictive Control for Heterogeneous Vehicle Platoon With Inter-Vehicular Spacing Policy*. IEEE T-ITS. https://doi.org/10.1109/TITS.2022.3227465
- [L26] Zhou, X., et al. (2023). *Asynchronous Self-Triggered Stochastic Distributed MPC for Cooperative Vehicle Platooning over Vehicular Ad-Hoc Networks*. IEEE TVT. https://doi.org/10.1109/TVT.2023.3288210
- [L27] Li, K., et al. (2023). *Resilient Event-Triggered Model Predictive Control for Adaptive Cruise Control Under Sensor Attacks*. IEEE/CAA JAS. https://doi.org/10.1109/JAS.2023.123111
- [L28] (2025). *Internal-Stably Energy-Saving Cooperative Control of Articulated Wheeled Robot with Distributed Drive Units*. ICRA. https://doi.org/10.1109/ICRA55743.2025.11128843
- [L51] (2026). *Hybrid System Planning using a Mixed-Integer ADMM Heuristic and Hybrid Zonotopes*. arXiv. https://arxiv.org/abs/2602.17574
- [L52] (2025). *Sharing the Load: Distributed Model-Predictive Control for Precise Multi-Rover Cargo Transport*. arXiv. https://arxiv.org/abs/2510.18766

### 6.4 对接与视觉伺服

- [L29] Zhu, Z., et al. (2017). *Autonomous Rendezvous and Docking with Nonfull Field of View for Tethered Space Robot*. Mathematical Problems in Engineering. https://doi.org/10.1155/2017/3162349
- [L30] Kim, J., et al. (2021). *Autonomous Docking of Hybrid-Wheeled Modular Robots With an Integrated Active Genderless Docking Mechanism*. Journal of Mechanisms and Robotics. https://doi.org/10.1115/1.4051519
- [L31] Park, S., et al. (2021). *Pallet detection and docking strategy for autonomous pallet truck AGV operation*. IROS. https://doi.org/10.1109/IROS51168.2021.9636270
- [L32] Pinto, C., et al. (2021). *Quadrotor going through a window and landing: An image-based visual servo control approach*. Control Engineering Practice. https://doi.org/10.1016/j.conengprac.2021.104827
- [L33] Li, Y., et al. (2024). *Image-Based Visual Servoing With Collision-Free Path Planning for Monocular Vision-Guided Assembly*. IEEE TIM. https://doi.org/10.1109/TIM.2024.3463014
- [L34] (2025). *Position-based acoustic visual servo control for docking of autonomous underwater vehicle using deep reinforcement learning*. Robotics and Autonomous Systems. https://doi.org/10.1016/j.robot.2024.104914
- [L53] (2025). *Robust Docking Maneuvers for Autonomous Trolley Collection: An Optimization-Based Visual Servoing Scheme*. arXiv. https://arxiv.org/abs/2509.07413

### 6.5 学习型协同决策

- [L35] Gronauer, S., & Diepold, K. (2021). *Multi-Agent Reinforcement Learning: A Review of Challenges and Applications*. Applied Sciences. https://doi.org/10.3390/app11114948
- [L36] Nguyen, T. T., et al. (2022). *A review of cooperative multi-agent deep reinforcement learning*. Applied Intelligence. https://doi.org/10.1007/s10489-022-04105-y
- [L48] (2024). *Learning Policies for Dynamic Coalition Formation in Multi-Robot Task Allocation*. arXiv. https://arxiv.org/abs/2412.20397
- [L49] (2023). *Decentralized Multi-Robot Formation Control Using Reinforcement Learning*. arXiv. https://arxiv.org/abs/2306.14489
- [L50] (2025). *DCT-MARL: A Dynamic Communication Topology-Based MARL Algorithm for Connected Vehicle Platoon Control*. arXiv. https://arxiv.org/abs/2508.12633
- [L54] (2025). *DMPC-Swarm: Distributed Model Predictive Control on Nano UAV Swarms*. arXiv. https://arxiv.org/abs/2508.20553

---

## 7. P0 结论（用于 Gate-0）

1. 文献覆盖数量、近5年比例、方法对比数量均已达标。
2. 证据链支持“策略层-规控层解耦 + 事件驱动回退 + 多率异步一致性”作为主框架方向。
3. 下一步进入 `reviews/P0_FRAMEWORK_DESIGN.md`，给出统一接口、时序、仲裁与创新点定义。
