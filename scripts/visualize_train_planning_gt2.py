#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import replace
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
    max_time: float = 90.0,
) -> tuple[np.ndarray, bool]:
    s = start.copy()
    path = [s.xy()]
    reached = False
    for _ in range(int(max_time / dt)):
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


def run_case(train_n: int) -> dict:
    cfg = load_config()
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)

    obstacles = [
        Obstacle(x=-2.0, y=3.0, width=2.0, height=2.0, yaw=0.05),
        Obstacle(x=7.0, y=-3.0, width=2.0, height=2.0, yaw=-0.05),
        Obstacle(x=12.0, y=3.0, width=2.0, height=2.0, yaw=0.04),
    ]

    geom = VehicleGeometry(cfg.vehicle)
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    train_kin = TrainKinematics(cfg.vehicle)

    # Train-aware planner: increase min turning radius according to train length.
    train_rmin = train_kin.min_turn_radius_train(train_n)
    vehicle_plan_cfg = replace(cfg.vehicle, min_turn_radius_single=train_rmin)
    planner = LocalPlanner(vehicle_plan_cfg, cfg.planner, collision, cfg.control.dt)
    model = AckermannModel(vehicle_plan_cfg)

    head_start = VehicleState(vehicle_id=100, x=-16.0, y=-1.5, yaw=0.0, v=0.0, delta=0.0)
    goal = np.array([16.0, 1.5], dtype=float)
    ref_path, path_ok = plan_head_path(planner, model, collision, head_start, goal, obstacles, cfg.control.dt)

    if len(ref_path) < 2:
        ref_path = np.array([[-16.0, -4.0], [16.0, 4.0]], dtype=float)

    train = make_train(geom, n=train_n, x0=-16.0, y0=-1.5)
    ctrl = TrainController(cfg.vehicle, cfg.control)
    guard = TrainSafetyGuard(cfg.vehicle, cfg.safety, collision)

    hist = [[s.copy() for s in train]]
    emergency_count = 0
    min_clearance = 1e9
    reached = False
    max_phi = 0.0

    for _ in range(int(120.0 / cfg.control.dt)):
        prev = [s.copy() for s in train]
        target_speed = 0.30 if train_n <= 5 else 0.35
        cmd = ctrl.track_head_path(train, ref_path, target_speed=target_speed, dt=cfg.control.dt)
        upd = train_kin.update(train, cmd, cfg.control.dt)
        if upd.articulation_angles:
            max_phi = max(max_phi, max(abs(a) for a in upd.articulation_angles))

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
            for j in range(i + 1, len(train)):
                if abs(i - j) <= 1:
                    continue
                if collision.collide_vehicle_vehicle(s, train[j], include_clearance=False):
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

    follower_track_errs = []
    for k in range(1, train_n):
        errs = []
        for frame in hist:
            errs.append(polyline_min_dist(frame[k].xy(), ref_path))
        follower_track_errs.append(float(np.mean(errs)))

    # Static
    fig, ax = plt.subplots(figsize=(10, 5), dpi=150)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-20, 20)
    ax.set_ylim(-10, 10)
    for obs in obstacles:
        ax.add_patch(Polygon(obstacle_polygon(obs), closed=True, fill=False, edgecolor="black", linewidth=1.2))
    ax.plot(ref_path[:, 0], ref_path[:, 1], color="tab:green", linewidth=2.0, label="planned head path")
    for idx in range(1, train_n):
        trail = np.array([frame[idx].xy() for frame in hist])
        ax.plot(trail[:, 0], trail[:, 1], linewidth=1.3, label=f"car{idx} traj")
    ax.legend(loc="upper left")
    ax.set_title(
        f"Train n={train_n} | path_ok={path_ok} reached={reached} emerg={emergency_count} "
        f"| min_clear={min_clearance:.3f}m max_phi={np.degrees(max_phi):.1f}deg"
    )
    png_path = out_dir / f"train_gt2_n{train_n}.png"
    fig.savefig(png_path)
    plt.close(fig)

    # GIF
    fig, ax = plt.subplots(figsize=(10, 5), dpi=140)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-20, 20)
    ax.set_ylim(-10, 10)
    for obs in obstacles:
        ax.add_patch(Polygon(obstacle_polygon(obs), closed=True, fill=False, edgecolor="black", linewidth=1.0))
    ax.plot(ref_path[:, 0], ref_path[:, 1], color="tab:green", linewidth=1.1)

    palette = ["tab:green", "tab:blue", "tab:orange", "tab:red", "tab:brown", "tab:purple"]
    patches = []
    for i in range(train_n):
        p = Polygon(geom.body_polygon(hist[0][i]), closed=True, fill=False, edgecolor=palette[i % len(palette)], linewidth=1.8)
        ax.add_patch(p)
        patches.append(p)
    txt = ax.text(0.01, 0.99, "", transform=ax.transAxes, ha="left", va="top", fontsize=9)

    def update(i: int):
        cur = hist[i]
        for j in range(train_n):
            patches[j].set_xy(geom.body_polygon(cur[j]))
        txt.set_text(f"n={train_n} t={i*cfg.control.dt:.2f}s step={i}/{len(hist)-1}")
        return (*patches, txt)

    frame_count = min(len(hist), 500)
    frame_idx = np.linspace(0, len(hist) - 1, frame_count, dtype=int)
    ani = FuncAnimation(fig, lambda k: update(int(frame_idx[k])), frames=frame_count, interval=35, blit=True)
    gif_path = out_dir / f"train_gt2_n{train_n}.gif"
    ani.save(gif_path, writer=PillowWriter(fps=25))
    plt.close(fig)

    return {
        "train_n": train_n,
        "path_ok": path_ok,
        "reached": reached,
        "steps": len(hist) - 1,
        "emergency_count": emergency_count,
        "min_clearance": float(min_clearance),
        "max_phi_deg": float(np.degrees(max_phi)),
        "follower_mean_track_error": follower_track_errs,
        "rmin_train": float(train_rmin),
        "png": str(png_path),
        "gif": str(gif_path),
    }


def main() -> None:
    for n in [3, 4, 5, 6]:
        res = run_case(n)
        print(res)


if __name__ == "__main__":
    main()
