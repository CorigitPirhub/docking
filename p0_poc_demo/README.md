# P0 PoC Demo

## 目标

验证 P0 阶段最小闭环链路：

`策略命令 -> 运行时执行 -> 事件/ACK反馈 -> 拓扑状态更新`

PoC 不追求策略最优，仅验证框架可执行性与协议可用性。

## 运行

```bash
python p0_poc_demo/run_p0_poc.py
```

## 输出

1. `artifacts/p0_poc_metrics.json`
2. `artifacts/P0_POC_REPORT.md`
3. `artifacts/p0_poc_timeline.png`

## 行为设计

1. `0s~8s`：策略尝试构建列车（Dock）。
2. `8s~12s`：瓶颈前准备区，策略触发解编（Split）。
3. `12s~14s`：硬瓶颈区，要求列车规模不超过上限。
4. `14s` 后：策略尝试重组到目标规模（Re-Dock）。

## 通过条件

1. 拓扑不变量成立。
2. 发生至少一次 `DOCK_LOCKED` 与一次 `SPLIT_DONE`。
3. 硬瓶颈区（`12s~14s`）内最大列车规模满足设定上限。
