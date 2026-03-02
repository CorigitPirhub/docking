# Standalone Framework Demo

This demo is intentionally independent from the existing multi-vehicle platform implementation.

## What It Validates

1. Decoupled architecture: strategy layer vs control layer.
2. Unified command protocol: `DockingCommand`, `SplitCommand`, `WaitCommand`.
3. Async execution consistency: high/mid/low rate ticks.
4. Runtime feedback loop: `ACK + event timeline`.
5. Bottleneck-constrained reconfiguration behavior:
   - open area: docking encouraged,
   - bottleneck: split enforced,
   - post-bottleneck: partial re-docking.

## Run

```bash
python standalone_framework_demo/run_demo.py
python standalone_framework_demo/run_transferability_suite.py
```

## Outputs

Generated under `artifacts/standalone_framework_demo/`:

1. `standalone_metrics.json`
2. `STANDALONE_FRAMEWORK_DEMO_REPORT.md`
3. `standalone_trajectories.png`
4. `standalone_topology_timeline.png`
5. `standalone_events_timeline.png`

Transferability suite outputs under `artifacts/standalone_framework_transferability/`:

1. `transferability_summary.json`
2. `TRANSFERABILITY_SUMMARY.md`
3. per-case plots and reports
