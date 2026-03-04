# Acceptance & Regression Runbook

本文档冻结一组“上层决策 → 基础支撑执行/代价 → 全场景评测/可视化”的验收入口，用于后续迭代的回归检查。

## 1) 最小闭环（P0 PoC）

- 目标：验证 `策略命令 -> runtime 执行 -> feedback` 闭环与拓扑不变量。
- 运行：`python p0_poc_demo/run_p0_poc.py`
- 产物：`artifacts/P0_POC_REPORT.md`、`artifacts/p0_poc_metrics.json`、`artifacts/p0_poc_timeline.png`

## 2) 协议一致性（P1 Command Bus）

- 目标：唯一协议 `runtime/command_bus.py` 的去重/过期/冲突仲裁行为稳定。
- 运行：`python scripts/validate_p1_command_bus.py`
- 产物：`artifacts/P1_COMMAND_BUS_REPORT.md`、`artifacts/p1_command_bus_validation.json`

## 3) 基础支撑能力回归（规划/对接/列车/碰撞）

- 目标：单车规划、列车跟随/安全守护、对接闭环与碰撞逻辑在基础场景下稳定。
- 运行：`python scripts/run_foundation_acceptance.py`
- 产物：`artifacts/FOUNDATION_ACCEPTANCE_REPORT.md`（及对应 json）

## 4) 场景生成与标签校验（静态几何）

- 目标：`docking/scenario_support.py` 生成的分类/标签在参数层面自洽且达标。
- 运行：`python scripts/validate_scenario_generation.py --seeds-per-subtype 30`
- 产物：`artifacts/SCENARIO_VALIDATION_REPORT.md`、`artifacts/scenario_validation_report.json`

## 5) 标签与真实执行一致性（带噪声闭环）

- 目标：在“真实基础支撑执行 + GNSS/视觉噪声（可选）”下，验证 `n_max_pass_profile` 等标签不会诱导策略产生硬违规（如超限编组通行）。
- 运行（推荐带噪声）：`python scripts/validate_scenario_tags_execution.py --enable-sensor-noise --seeds-per-subtype 2`
- 产物：`artifacts/SCENARIO_TAG_EXECUTION_REPORT.md`、`artifacts/scenario_tag_execution_report.json`

## 6) 全场景系统级评测与可视化（P6）

- 目标：在 A/B/C 全子类、多初始分散模式、多随机种子下对策略做可复现评测，并生成可审计可视化。
- 评测：
  - `python scripts/run_p6_system_evaluation.py --seeds-per-subtype 4 --inject-disturbance`
  - 可选：增加 `--enable-sensor-noise` 覆盖带噪声闭环鲁棒性评测
- 可视化：`python scripts/visualize_p6_results.py`
- 产物：`experiments/p6_system_evaluation_results.json`、`artifacts/P6_SYSTEM_EVALUATION_REPORT.md`、`artifacts/p6_*.png`

