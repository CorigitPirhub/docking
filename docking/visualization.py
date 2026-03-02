from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

from .collision import obstacle_polygon
from .config import Config
from .kinematics import VehicleGeometry
from .types import Obstacle, VehicleState


def _vehicle_patch(state: VehicleState, geom: VehicleGeometry, color: str) -> Polygon:
    poly = geom.body_polygon(state)
    return Polygon(poly, closed=True, fill=False, edgecolor=color, linewidth=1.7)


def _fov_points(state: VehicleState, geom: VehicleGeometry, fov_deg: float, max_range: float) -> np.ndarray:
    cam = geom.front_hitch(state)
    half = math.radians(fov_deg) * 0.5
    angles = np.linspace(-half, half, 25)
    pts = [cam]
    for a in angles:
        th = state.yaw + a
        pts.append(cam + np.array([math.cos(th), math.sin(th)]) * max_range)
    return np.array(pts)


def save_docking_animation(
    cfg: Config,
    history: dict[str, list[float]],
    obstacles: list[Obstacle],
    output_path: str,
    fps: int = 25,
) -> None:
    geom = VehicleGeometry(cfg.vehicle)
    fig, ax = plt.subplots(figsize=(10, 5), dpi=160)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-cfg.environment.width * 0.5, cfg.environment.width * 0.5)
    ax.set_ylim(-cfg.environment.height * 0.5, cfg.environment.height * 0.5)
    ax.set_title("Docking Replay")

    for obs in obstacles:
        ax.add_patch(Polygon(obstacle_polygon(obs), closed=True, fill=False, edgecolor="black", linewidth=1.0))

    follower_patch = _vehicle_patch(VehicleState(-1, 0, 0, 0, 0, 0), geom, "tab:blue")
    target_patch = _vehicle_patch(VehicleState(-2, 0, 0, 0, 0, 0), geom, "tab:orange")
    head_patch = _vehicle_patch(VehicleState(-3, 0, 0, 0, 0, 0), geom, "tab:green")
    ax.add_patch(follower_patch)
    ax.add_patch(target_patch)
    ax.add_patch(head_patch)
    fov_patch = Polygon(np.zeros((3, 2)), closed=True, fill=False, edgecolor="tab:cyan", linewidth=1.0, linestyle="--")
    ax.add_patch(fov_patch)

    txt = ax.text(0.01, 0.98, "", transform=ax.transAxes, va="top", ha="left", fontsize=9)

    n = len(history.get("t", []))

    def update(i: int):
        follower = VehicleState(
            vehicle_id=1,
            x=history["follower_x"][i],
            y=history["follower_y"][i],
            yaw=history["follower_yaw"][i],
            v=0.0,
            delta=0.0,
        )
        target = VehicleState(
            vehicle_id=2,
            x=history["target_x"][i],
            y=history["target_y"][i],
            yaw=history["target_yaw"][i],
            v=0.0,
            delta=0.0,
        )
        head = VehicleState(
            vehicle_id=3,
            x=history["leader_head_x"][i],
            y=history["leader_head_y"][i],
            yaw=history["leader_head_yaw"][i],
            v=0.0,
            delta=0.0,
        )

        follower_patch.set_xy(geom.body_polygon(follower))
        target_patch.set_xy(geom.body_polygon(target))
        head_patch.set_xy(geom.body_polygon(head))

        fov = _fov_points(follower, geom, cfg.sensors.vision.fov_deg, cfg.sensors.vision.max_distance)
        fov_patch.set_xy(fov)

        if "stage_name" in history and len(history["stage_name"]) > i:
            stage = history["stage_name"][i]
        else:
            stage = "GLOBAL" if history["stage_id"][i] < 0.5 else "VISION"
        txt.set_text(
            f"t={history['t'][i]:.2f}s\\n"
            f"stage={stage}  w_vis={history['w_vis'][i]:.2f}\\n"
            f"dist={history['distance'][i]:.2f}m pos_err={history['pos_error'][i]:.3f}m"
        )

        return follower_patch, target_patch, head_patch, fov_patch, txt

    ani = FuncAnimation(fig, update, frames=n, interval=1000 / fps, blit=True)
    ani.save(output_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
