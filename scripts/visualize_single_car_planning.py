#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.collision import CollisionEngine, obstacle_polygon
from docking.config import load_config
from docking.kinematics import AckermannModel, VehicleGeometry
from docking.planner import LocalPlanner
from docking.types import Obstacle, VehicleState


def main() -> None:
    cfg = load_config()
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)

    obstacles = [
        Obstacle(x=-6.0, y=-0.8, width=2.4, height=6.0, yaw=0.1),
        Obstacle(x=-1.0, y=3.2, width=2.2, height=4.0, yaw=-0.2),
        Obstacle(x=4.0, y=-2.8, width=2.2, height=4.2, yaw=0.15),
        Obstacle(x=9.0, y=1.5, width=2.2, height=5.0, yaw=-0.1),
    ]

    start = VehicleState(vehicle_id=1, x=-16.0, y=-6.0, yaw=0.15, v=0.0, delta=0.0)
    goal = np.array([16.0, 6.0], dtype=float)
    goal_yaw = 0.0

    model = AckermannModel(cfg.vehicle)
    geom = VehicleGeometry(cfg.vehicle)
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    planner = LocalPlanner(cfg.vehicle, cfg.planner, collision, cfg.control.dt)

    state = start
    states = [state.copy()]
    traj = [state.xy()]
    min_clearance = 1e9
    reached = False

    max_steps = int(40.0 / cfg.control.dt)
    for _ in range(max_steps):
        res = planner.plan_step(state, goal, goal_yaw, obstacles, [])
        if not res.feasible:
            break
        state = model.step(state, res.command, cfg.control.dt)
        states.append(state.copy())
        traj.append(state.xy())

        if collision.in_collision(state, obstacles, []):
            break

        min_clearance = min(min_clearance, collision.min_clearance_vehicle_obstacles(state, obstacles))

        if float(np.linalg.norm(state.xy() - goal)) < 0.8:
            reached = True
            break

    traj_np = np.array(traj)

    # Dynamics sanity metrics.
    max_step_speed = 0.0
    max_lateral = 0.0
    for i in range(len(states) - 1):
        s0, s1 = states[i], states[i + 1]
        dt = cfg.control.dt
        dx, dy = s1.x - s0.x, s1.y - s0.y
        step_speed = math.hypot(dx, dy) / dt
        max_step_speed = max(max_step_speed, step_speed)
        c = math.cos(-s0.yaw)
        s = math.sin(-s0.yaw)
        bx = c * dx - s * dy
        by = s * dx + c * dy
        lateral = abs(by) / max(dt, 1e-9)
        max_lateral = max(max_lateral, lateral)

    # Static plot.
    fig, ax = plt.subplots(figsize=(10, 5), dpi=150)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-20, 20)
    ax.set_ylim(-10, 10)
    for obs in obstacles:
        ax.add_patch(Polygon(obstacle_polygon(obs), closed=True, fill=False, edgecolor="black", linewidth=1.2))
    ax.plot(traj_np[:, 0], traj_np[:, 1], color="tab:blue", linewidth=2.0, label="executed path")
    ax.scatter([start.x, goal[0]], [start.y, goal[1]], c=["tab:green", "tab:red"], s=50)
    ax.legend(loc="upper left")
    ax.set_title(
        f"Single-Car Obstacle Planning | reached={reached} | min_clearance={min_clearance:.3f}m"
    )
    png_path = out_dir / "single_car_planning.png"
    fig.savefig(png_path)
    plt.close(fig)

    # Animation.
    fig, ax = plt.subplots(figsize=(10, 5), dpi=140)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-20, 20)
    ax.set_ylim(-10, 10)
    for obs in obstacles:
        ax.add_patch(Polygon(obstacle_polygon(obs), closed=True, fill=False, edgecolor="black", linewidth=1.0))
    line, = ax.plot([], [], color="tab:blue", linewidth=2.0)
    patch = Polygon(geom.body_polygon(states[0]), closed=True, fill=False, edgecolor="tab:orange", linewidth=2.0)
    ax.add_patch(patch)
    txt = ax.text(0.01, 0.99, "", transform=ax.transAxes, ha="left", va="top", fontsize=9)

    def update(i: int):
        line.set_data(traj_np[: i + 1, 0], traj_np[: i + 1, 1])
        patch.set_xy(geom.body_polygon(states[i]))
        txt.set_text(f"t={i*cfg.control.dt:.2f}s\\nstep={i}/{len(states)-1}")
        return line, patch, txt

    ani = FuncAnimation(fig, update, frames=len(states), interval=40, blit=True)
    gif_path = out_dir / "single_car_planning.gif"
    ani.save(gif_path, writer=PillowWriter(fps=25))
    plt.close(fig)

    print("single_car_reached", reached)
    print("single_car_steps", len(states) - 1)
    print("single_car_min_clearance", float(min_clearance))
    print("single_car_max_step_speed", float(max_step_speed))
    print("single_car_max_lateral_speed", float(max_lateral))
    print("single_car_png", png_path)
    print("single_car_gif", gif_path)


if __name__ == "__main__":
    main()
