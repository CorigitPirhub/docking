#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.simulation import run_docking_case
from docking.visualization import save_docking_animation


def main() -> None:
    cfg = load_config()
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)

    seed = 52
    result = run_docking_case(cfg, leader_train_size=2, seed=seed, with_obstacles=True, moving_leader=True, max_time=30.0)

    # State continuity diagnostics for jump detection.
    x = np.array(result.history["follower_x"], dtype=float)
    y = np.array(result.history["follower_y"], dtype=float)
    yaw = np.unwrap(np.array(result.history["follower_yaw"], dtype=float))
    t = np.array(result.history["t"], dtype=float)

    ds = np.hypot(np.diff(x), np.diff(y)) if len(x) > 1 else np.array([0.0])
    dyaw = np.abs(np.diff(yaw)) if len(yaw) > 1 else np.array([0.0])
    dt = np.diff(t) if len(t) > 1 else np.array([cfg.control.dt])

    max_step_dist = float(np.max(ds))
    max_step_yaw_deg = float(np.degrees(np.max(dyaw)))
    max_equiv_speed = float(np.max(ds / np.maximum(dt, 1e-9)))

    # Plot diagnostics.
    fig, axs = plt.subplots(4, 1, figsize=(10, 8), dpi=150, sharex=True)
    axs[0].plot(t, result.history["pos_error"], label="pos err (m)")
    axs[0].axhline(cfg.docking.lock_position_tol, color="red", linestyle="--", linewidth=1.0)
    axs[0].legend(loc="upper right")

    axs[1].plot(t, result.history["yaw_error_deg"], label="yaw err (deg)")
    axs[1].axhline(10.0, color="red", linestyle="--", linewidth=1.0, label="10deg rule")
    axs[1].legend(loc="upper right")

    axs[2].plot(t, result.history["w_vis"], label="vision weight")
    axs[2].plot(t, result.history["stage_id"], label="stage id")
    axs[2].legend(loc="upper right")

    axs[3].plot(t[1:], ds / np.maximum(dt, 1e-9), label="step speed equiv")
    axs[3].axhline(cfg.vehicle.max_speed * 1.2, color="red", linestyle="--", linewidth=1.0)
    axs[3].legend(loc="upper right")
    axs[3].set_xlabel("time (s)")

    fig.suptitle(
        f"Docking Diagnostics | success={result.success} reason={result.reason} | "
        f"max_step_dist={max_step_dist:.3f}m max_step_yaw={max_step_yaw_deg:.2f}deg"
    )
    diag_png = out_dir / "docking_diagnostics.png"
    fig.savefig(diag_png)
    plt.close(fig)

    gif_path = out_dir / "docking_process.gif"
    save_docking_animation(cfg, result.history, result.obstacles, str(gif_path), fps=25)

    jump_flag = (max_equiv_speed > cfg.vehicle.max_speed * 1.2) or (max_step_yaw_deg > 8.0)

    print("docking_success", result.success)
    print("docking_reason", result.reason)
    print("docking_sim_time", float(result.sim_time))
    print("docking_max_step_dist", max_step_dist)
    print("docking_max_step_yaw_deg", max_step_yaw_deg)
    print("docking_max_equiv_speed", max_equiv_speed)
    print("docking_jump_detected", jump_flag)
    print("docking_angle_rule_final_deg", float(result.final_yaw_error_deg))
    print("docking_diag_png", diag_png)
    print("docking_gif", gif_path)


if __name__ == "__main__":
    main()
