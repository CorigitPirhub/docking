# P0 创新框架设计（策略层-规控层解耦）

## 1. 设计目标与边界

### 1.1 目标

1. 将“何时对接/何时解编/是否等待”的离散策略决策与底层轨迹控制解耦。
2. 保持策略层平台无关，可迁移到其他异构多智能体系统。
3. 通过统一协议实现异步多率运行下的时空一致性。

### 1.2 非目标

1. P0 不做完整性能最优实现，不进入大规模工程化开发。
2. P0 不引入动态障碍，仍基于静态场景与全局共享状态。

---

## 2. 框架总览

框架命名：`STRC`（Strategy-Track Reconfiguration Coupling）。

### 2.1 分层结构

1. 策略层（平台无关，大脑）
2. 规控层（平台相关，小脑）
3. 命令总线（异步协议 + 事件反馈）

架构图（ASCII）：

```text
                +-------------------------------------------+
                |              Strategy Layer               |
                |  State Abstraction -> Reconfig Optimizer  |
 Global State ->|  -> Feasibility Governor -> Cmd Compiler |-> Unified Commands
 Scenario Tags  +-------------------------------------------+        |
                                                                 Command Bus
                +-------------------------------------------+        |
                |               Runtime Bus                  |<-------+
                |  TTL/Version/ACK/Arbitration/Event Log    |
                +--------------------------+-----------------+
                                           |
                                           v
                +-------------------------------------------+
                |             Control Layer                 |
                |  Mode Planner -> Tracker/Safety Filter    |
                |  -> Execution Monitor -> Event/Feedback   |
                +-------------------------------------------+
```

### 2.2 策略层组件

1. `State Abstraction`
将原始全局状态压缩为决策摘要：
- 编队拓扑摘要（头车、链长、边集合）
- 场景约束摘要（`N_max_pass_profile`、`split_mandatory_zones`、`dock_friendly_zones`）
- 任务可完成性摘要（`T_est`, `T_leader_remain`）
- 风险摘要（最小净空、冲突概率、FOV 丢失风险）

2. `Reconfiguration Optimizer`
在滚动时域内搜索离散动作 `z^k = {dock, split, wait, assign}`，输出候选动作序列。

3. `Feasibility Governor`
执行硬约束门控：
- 若 `T_est > T_leader_remain`，禁止继续对接并触发回退策略。
- 若进入 `split_mandatory_zones`，对接动作降级，解编动作升优先级。

4. `Command Compiler`
将离散决策转成标准指令对象（带版本号与 TTL）。

### 2.3 规控层组件

1. `Mode Planner`
- FREE：单车局部规划
- DOCKING：两阶段对接（全局趋近 + 视觉伺服）
- TRAIN_FOLLOW：列车整体规划与头车牵引跟踪

2. `Tracking + Safety Filter`
- 轨迹跟踪控制（Pure Pursuit/Stanley/LQR）
- 安全投影（碰撞约束、防折刀约束、动力学约束）

3. `Execution Monitor`
检测并上报：
- 执行成功/失败
- 失败码（不可达、FOV 丢失、姿态超限、命令过期等）
- 状态机事件（DOCKING_STARTED/DOCK_LOCKED/SPLIT_DONE/...）

---

## 3. 指令接口规范（P1 冻结草案）

本节定义策略层与规控层唯一交互面。

### 3.1 状态快照输入（策略层）

```text
StrategyStateSnapshot
- timestamp: float
- state_seq: int                 # 单调递增状态版本号
- vehicles: List[VehicleSummary] # pose/speed/mode
- topology: TopologySummary      # G^k
- scenario_tags: ScenarioTags    # 来自场景标签计算器
- feasibility: FeasibilityState  # T_est vs T_leader_remain
- risks: RiskSummary             # clearance, collision_risk, fov_risk
```

### 3.2 指令对象（策略层输出）

```text
CommandHeader
- command_id: str
- parent_state_seq: int          # 命令基于哪个状态版本生成
- issued_at: float
- valid_for_s: float             # TTL
- priority: int                  # 大值优先
- source: str                    # strategy_id
- can_preempt: bool
- rollback_on_reject: bool

DockingCommand
- follower_id: int
- leader_id: int
- intercept_path_s: float | None # 轨迹弧长坐标；None 表示规控层自主选择

SplitCommand
- parent_id: int
- child_id: int
- reason: str

WaitCommand
- vehicle_id: int
- duration_s: float
- reason: str
```

### 3.3 执行反馈对象（规控层输出）

```text
CommandFeedback
- command_id: str
- stage: submit | execute | done
- accepted: bool
- status: queued | running | success | failed
- reason_code: str
- event_time: float
- state_seq_after: int
```

---

## 4. 异步一致性与冲突仲裁

### 4.1 多率时序

1. 高层调度：`0.5~2 Hz`
2. 中层规划/命令执行：`5~10 Hz`
3. 低层控制：`10~50 Hz`

时序图（ASCII）：

```text
Time ---->
Strategy Tick (1Hz):   S0 -------- S1 -------- S2 -------- S3
Cmd Submit:              C0      C1   C2            C3
Mid Execute (8Hz):       e0 e1 e2 e3 e4 e5 e6 e7 e8 ...
Low Control (20Hz):      l0 l1 l2 l3 l4 l5 l6 l7 l8 ...
Feedback/Event:            ACK0  RUN0  DONE0  ACK1 FAIL1 ...
Rule: command.parent_state_seq must match latest confirmed snapshot window
```

### 4.2 一致性规则

1. `state_seq` 单调递增，策略只能基于已确认快照下发命令。
2. 若 `now > issued_at + valid_for_s`，命令自动失效拒绝。
3. 同一 `command_id` 只允许一次有效提交（去重）。
4. 指令执行后必须返回终态反馈（`success/failed`）。

### 4.3 冲突仲裁优先级（硬规则）

1. 安全硬约束（碰撞、防折刀、动力学可执行）
2. 通行硬约束（`split_mandatory_zones`）
3. 可完成性约束（`T_est > T_leader_remain`）
4. 时间目标（`T_done`）
5. 能耗目标（`E_tot`）

### 4.4 抖振抑制规则

1. `dock->split` 互斥切换需满足最小驻留时间 `tau_mode_min`。
2. 同目标边在 `cooldown_s` 内禁止重复对接/解编反复触发。
3. 若冲突持续，使用“高优先级动作锁”覆盖低优先级动作。

---

## 5. 策略层代价函数骨架（P2 输入）

在 P0 阶段仅冻结结构，不做最终参数最优。

\[
J_{seq} =
w_t \cdot T_{catch}
 + w_d \cdot T_{dock}
 + w_e \cdot \Delta E
 + w_r \cdot R_{risk}
 + w_s \cdot N_{switch}
\]

其中：

1. `T_catch`：追击/截获时间
2. `T_dock`：对接执行时间
3. `ΔE`：编组节能收益（P2 用 `eta_nominal=0.95` 标称模型）
4. `R_risk`：安全风险代价（净空、FOV、障碍邻近）
5. `N_switch`：对接/解编切换惩罚

截获点定义（无歧义）：

1. 截获点必须位于头车共享轨迹上，为未来路径点 `p_leader(s, t+Δt)`。
2. 不允许将截获点定义为任意静态空间点。

---

## 6. 与“常规工程堆叠”的差异（创新点）

1. 不是“先写策略再补接口”，而是先冻结命令协议与事件语义，再让策略与规控并行迭代。
2. 不是“策略直接管控制器”，而是通过可审计命令总线驱动规控层，保证平台可替换性。
3. 不是“单一收益最大化”，而是硬约束门控 + 软代价优化的双环结构，避免学习策略越权。
4. 不是“离线一次性规划”，而是事件触发滚动重规划（可完成性翻转/FOV 丢失/路径阻塞）。

---

## 7. 与现有代码基座映射（可实现性）

现有能力可直接复用：

1. `docking/runtime_support.py`
- 已有 `DockingCommand/SplitCommand/WaitCommand`
- 已有事件与 ACK 机制
- 已有可完成性监控与多率执行器

2. 需在 P1/P2 增补：
- `parent_state_seq/priority/preempt` 字段
- 冲突仲裁器（统一策略冲突分解）
- 截获点选择器（基于共享轨迹）

---

## 8. P0 输出结论

1. 框架已完成“可执行接口 + 异步一致性 + 仲裁规则”三级定义。
2. 已具备进入 P0 评审打分的输入条件。
3. 下一步：在 `reviews/P0_INNOVATION_FEASIBILITY_REVIEW.md` 做创新度与可实现性量化评估，给出 Go/No-Go。
