#!/usr/bin/env python3
from __future__ import annotations

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
from docking.train import TrainController, TrainKinematics, TrainSafetyGuard
from docking.types import Obstacle, VehicleState


def polyline_min_dist(point: np.ndarray, line: np.ndarray) -> float:
    d = np.linalg.norm(line - point, axis=1)
    return float(np.min(d))


def make_train(geom: VehicleGeometry, n: int, x0: float, y0: float) -> list[VehicleState]:
    spacing = geom.front_hitch_x - geom.rear_hitch_x
    out = []
    for i in range(n):
        out.append(VehicleState(vehicle_id=100 + i, x=x0 - i * spacing, y=y0, yaw=0.0, v=0.0, delta=0.0))
    return out


def plan_head_path(
    planner: LocalPlanner,
    model: AckermannModel,
    collision: CollisionEngine,
    start: VehicleState,
    goal: np.ndarray,
    obstacles: list[Obstacle],
    dt: float,
) -> tuple[np.ndarray, bool]:
    s = start.copy()
    path = [s.xy()]
    reached = False
    for _ in range(int(55.0 / dt)):
        res = planner.plan_step(s, goal, 0.0, obstacles, [])
        if not res.feasible:
            break
        s = model.step(s, res.command, dt)
        if collision.in_collision(s, obstacles, []):
            break
        path.append(s.xy())
        if float(np.linalg.norm(s.xy() - goal)) < 1.0:
            reached = True
            break
    return np.array(path), reached


def main() -> None:
    cfg = load_config()
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)

    obstacles = [
        Obstacle(x=-4.0, y=2.2, width=2.8, height=4.6, yaw=0.06),
        Obstacle(x=4.6, y=-2.1, width=2.8, height=4.6, yaw=-0.05),
        Obstacle(x=12.0, y=2.2, width=2.8, height=4.6, yaw=0.05),
    ]

    geom = VehicleGeometry(cfg.vehicle)
    model = AckermannModel(cfg.vehicle)
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    planner = LocalPlanner(cfg.vehicle, cfg.planner, collision, cfg.control.dt)

    head_start = VehicleState(vehicle_id=100, x=-16.0, y=-4.0, yaw=0.0, v=0.0, delta=0.0)
    goal = np.array([16.0, 4.0], dtype=float)

    ref_path, path_ok = plan_head_path(planner, model, collision, head_start, goal, obstacles, cfg.control.dt)

    train = make_train(geom, n=2, x0=-16.0, y0=-4.0)
    train_ctrl = TrainController(cfg.vehicle, cfg.control)
    train_kin = TrainKinematics(cfg.vehicle)
    guard = TrainSafetyGuard(cfg.vehicle, cfg.safety, collision)

    hist = [[s.copy() for s in train]]
    emergency_count = 0
    min_clearance = 1e9
    reached = False

    if len(ref_path) < 2:
        ref_path = np.array([[-16.0, -4.0], [16.0, 4.0]], dtype=float)

    for _ in range(int(100.0 / cfg.control.dt)):
        prev = [s.copy() for s in train]
        cmd = train_ctrl.track_head_path(train, ref_path, target_speed=0.45, dt=cfg.control.dt)
        upd = train_kin.update(train, cmd, cfg.control.dt)
        rep = guard.check(prev, upd.states, upd.articulation_angles, obstacles)
        if rep.emergency_stop:
            emergency_count += 1
            train = prev
            for s in train:
                s.v = 0.0
                s.delta = 0.0
        else:
            train = upd.states

        bad = False
        for i, s in enumerate(train):
            if collision.in_collision(s, obstacles, []):
                bad = True
                break
            min_clearance = min(min_clearance, collision.min_clearance_vehicle_obstacles(s, obstacles))
            for j, o in enumerate(train):
                if i >= j:
                    continue
                if abs(i - j) == 1:
                    continue
                if collision.collide_vehicle_vehicle(s, o, include_clearance=False):
                    bad = True
                    break
            if bad:
                break

        hist.append([s.copy() for s in train])

        if bad:
            break

        if float(np.linalg.norm(train[0].xy() - goal)) < 1.0:
            reached = True
            break

    # Same trajectory control metric against planned head reference.
    follower_track_errs = []
    for k in range(1, len(train)):
        errs = []
        for frame in hist:
            errs.append(polyline_min_dist(frame[k].xy(), ref_path))
        follower_track_errs.append(float(np.mean(errs)))

    # Static figure.
    fig, ax = plt.subplots(figsize=(10, 5), dpi=150)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-20, 20)
    ax.set_ylim(-10, 10)
    for obs in obstacles:
        ax.add_patch(Polygon(obstacle_polygon(obs), closed=True, fill=False, edgecolor="black", linewidth=1.2))
    ax.plot(ref_path[:, 0], ref_path[:, 1], color="tab:green", linewidth=2.0, label="planned head path")
    for idx in range(1, len(train)):
        trail = np.array([frame[idx].xy() for frame in hist])
        ax.plot(trail[:, 0], trail[:, 1], linewidth=1.5, label=f"car{idx} trajectory")
    ax.legend(loc="upper left")
    ax.set_title(
        f"Train Planning + Same-Path Tracking | path_ok={path_ok} reached={reached} "
        f"| emerg={emergency_count} | min_clearance={min_clearance:.3f}m"
    )
    png_path = out_dir / "train_planning.png"
    fig.savefig(png_path)
    plt.close(fig)

    # Animation.
    fig, ax = plt.subplots(figsize=(10, 5), dpi=140)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-20, 20)
    ax.set_ylim(-10, 10)
    for obs in obstacles:
        ax.add_patch(Polygon(obstacle_polygon(obs), closed=True, fill=False, edgecolor="black", linewidth=1.0))
    ax.plot(ref_path[:, 0], ref_path[:, 1], color="tab:green", linewidth=1.3)

    colors = ["tab:green", "tab:blue", "tab:orange", "tab:red"]
    patches = []
    for i in range(len(train)):
        p = Polygon(geom.body_polygon(hist[0][i]), closed=True, fill=False, edgecolor=colors[i], linewidth=2.0)
        ax.add_patch(p)
        patches.append(p)
    txt = ax.text(0.01, 0.99, "", transform=ax.transAxes, ha="left", va="top", fontsize=9)

    def update(i: int):
        cur = hist[i]
        for j in range(len(train)):
            patches[j].set_xy(geom.body_polygon(cur[j]))
        txt.set_text(f"t={i*cfg.control.dt:.2f}s\\nstep={i}/{len(hist)-1}")
        return (*patches, txt)

    ani = FuncAnimation(fig, update, frames=len(hist), interval=40, blit=True)
    gif_path = out_dir / "train_planning.gif"
    ani.save(gif_path, writer=PillowWriter(fps=25))
    plt.close(fig)

    print("train_path_planning_ok", path_ok)
    print("train_reached", reached)
    print("train_steps", len(hist) - 1)
    print("train_min_clearance", float(min_clearance))
    print("train_emergency_count", int(emergency_count))
    print("train_follower_mean_track_error", follower_track_errs)
    print("train_png", png_path)
    print("train_gif", gif_path)


if __name__ == "__main__":
    main()
