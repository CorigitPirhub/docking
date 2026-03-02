# P0 形式化接口契约与框架级保证（补强版）

## 1. 目的

本文件用于补足 `CORE_REQUIREMENTS_TASKBOOK.md` 当前目标中的两项缺口：

1. “接口契约缺少形式化定义”；
2. “缺少框架级可验证保证（安全优先、无抖振、可回退、活性）”。

---

## 2. 形式化系统模型

### 2.1 状态与命令

记离散时刻为 \(k\)，系统状态为 \(s_k\)，命令集合为 \(\mathcal{C}_k\)。

\[
s_k = (x_k, G_k, m_k, r_k, \sigma_k)
\]

其中：

1. \(x_k\)：车辆运动状态摘要；
2. \(G_k\)：拓扑图（父子边集合）；
3. \(m_k\)：车辆模式集合；
4. \(r_k\)：风险与可完成性摘要；
5. \(\sigma_k\)：状态版本号（`state_seq`）。

命令定义为：

\[
c = (\text{id}, \text{kind}, \sigma_{parent}, t_{issue}, \tau_{ttl}, p, \pi)
\]

其中 `kind ∈ {DOCK, SPLIT, WAIT}`，\(p\) 为优先级，\(\pi\) 为参数（车辆对/等待时长等）。

### 2.2 提交与执行语义

提交有效性：

\[
\text{SubmitValid}(c, t)=
[\tau_{ttl}>0]\land[\text{id}\notin \mathcal{I}_{seen}]
\]

执行有效性：

\[
\text{ExecValid}(c,s_k,t)=
[t\le t_{issue}+\tau_{ttl}]
\land[\sigma_{parent}\le \sigma_k]
\land \text{Pre}_{kind}(c,s_k)
\]

其中 \(\text{Pre}_{kind}\) 为命令前置条件（见第3节）。

---

## 3. 接口契约（前置/后置）

## 3.1 DockingCommand

前置条件：

1. follower 与 leader 均存在；
2. follower 当前为链头（`parent[follower]=None`）；
3. leader 尾部未被占用（`child[leader]=None` 且未被其他 pending dock 预占）；
4. 不形成环（无祖先冲突）；
5. follower 不处于 WAIT。

后置条件（成功执行）：

1. 进入 `DOCKING_STARTED`；
2. 若满足对接锁定门限，触发 `DOCK_LOCKED`；
3. 锁定后拓扑边新增 `(leader, follower)`，且拓扑不变式保持成立。

## 3.2 SplitCommand

前置条件：

1. 边 `(parent, child)` 存在；
2. 父子一致性成立（双向引用一致）。

后置条件（成功执行）：

1. 删除边 `(parent, child)`；
2. 触发 `SPLIT_DONE`；
3. 拓扑仍为无环链式结构。

## 3.3 WaitCommand

前置条件：

1. vehicle 存在；
2. `duration_s >= 0`。

后置条件：

1. 进入 WAIT 模式并触发 `WAIT_STARTED`；
2. 超时后触发 `WAIT_ENDED` 并恢复模式刷新。

---

## 4. 冲突仲裁与一致性语义

给定同时刻候选命令集 \(\mathcal{C}_k\)，仲裁器输出：

\[
c_k^\* = \arg\max_{c\in \mathcal{C}_k} \left( \text{Priority}(c) \right)
\]

但必须满足硬约束优先规则：

\[
\text{Safety} \succ \text{Passability} \succ \text{Feasibility} \succ \text{Time} \succ \text{Energy}
\]

即若安全/通行约束与代价最优冲突，必须覆盖后者。

一致性语义：

1. `command_id` 唯一，重复命令拒绝（exactly-once submit semantics）；
2. `state_seq` 单调递增；
3. 命令执行仅可基于不晚于当前的状态版本（禁止未来状态执行）。

---

## 5. 框架级性质与可验证断言

## 5.1 性质 P1：拓扑安全不变式

断言：

1. 任意时刻拓扑无环；
2. 任意节点入度/出度不超过1；
3. 父子引用一致。

验证方式：每个低层 tick 后执行不变式检查。  
验证证据：`invariant_ok=true`（见独立 demo 结果）。

## 5.2 性质 P2：安全优先可执行

断言：

若进入硬瓶颈区且当前头链规模 \(>1\)，系统必须触发 split 直至满足上限。

验证方式：按头车位置窗口统计 `max_leader_train_size_hard_bottleneck`。  
通过条件：该值 \(\le 1\)。

## 5.3 性质 P3：可回退（Abort-on-infeasible）

断言：

若可完成性由 true 翻转为 false 且存在 pending dock，则触发 `DOCKING_ABORTED`，系统可继续推进。

验证方式：检查 `FEASIBILITY_FLIP` 与 `DOCKING_ABORTED` 事件序列存在性。

## 5.4 性质 P4：进展性（Eventual Progress）

在以下假设下：

1. 有限命令注入速率；
2. 执行器可用；
3. 环境无不可恢复硬阻塞；

系统满足：在有限时间内产生至少一个终态事件（`DOCK_LOCKED` 或 `SPLIT_DONE`），并保持拓扑一致。

验证方式：统计终态事件计数是否 \(>0\) 且不变式持续成立。

---

## 6. 与可执行证据的映射

本节给出“形式化断言 -> 可执行检查”映射：

1. P1 -> `standalone_metrics.json: invariant_ok`
2. P2 -> `standalone_metrics.json: max_leader_train_size_hard_bottleneck`
3. P3 -> `event_count.FEASIBILITY_FLIP` 与 `event_count.DOCKING_ABORTED`
4. P4 -> `event_count.DOCK_LOCKED` 与 `event_count.SPLIT_DONE`

---

## 7. 结论

接口契约已从“工程描述”提升为“可检验前后置条件 + 一致性语义”，  
框架级性质已给出可验证断言与对应证据路径，可用于论文中“方法正确性与工程可证”部分。
