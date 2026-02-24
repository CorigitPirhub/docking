from __future__ import annotations

import math
import os
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib

# Safe backend for headless runtime.
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Polygon, Rectangle
import numpy as np

from core.utils import heading
from core.vehicle import AckermannVehicle, VehicleSpec
from env.world import WorldMap
from sim.simulator import Frame


class SimulationVisualizer:
    def __init__(self, world: WorldMap, vehicle_specs: Dict[str, VehicleSpec]):
        self.world = world
        self.vehicle_specs = vehicle_specs

    def _draw_world(self, ax: plt.Axes) -> None:
        ax.set_xlim(0, self.world.size_m)
        ax.set_ylim(0, self.world.size_m)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(True, alpha=0.12)

        for obs in self.world.obstacles:
            ax.add_patch(
                Rectangle(
                    (obs.xmin, obs.ymin),
                    obs.w,
                    obs.h,
                    facecolor="#6a6d72",
                    edgecolor="#3c4044",
                    alpha=0.8,
                )
            )

    def _vehicle_polygon(self, x: float, y: float, theta: float, spec: VehicleSpec) -> np.ndarray:
        hl = 0.5 * spec.length
        hw = 0.5 * spec.width
        corners = np.array(
            [[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]],
            dtype=float,
        )
        c = math.cos(theta)
        s = math.sin(theta)
        rot = np.array([[c, -s], [s, c]], dtype=float)
        poly = corners @ rot.T + np.array([x, y], dtype=float)
        return poly

    def _draw_vehicle(self, ax: plt.Axes, vid: str, pose: Tuple[float, float, float], color: str) -> None:
        x, y, theta = pose
        spec = self.vehicle_specs[vid]
        poly = self._vehicle_polygon(x, y, theta, spec)
        ax.add_patch(Polygon(poly, closed=True, facecolor=color, edgecolor="black", linewidth=0.9, alpha=0.92))

        hv = heading(theta)
        front = np.array([x, y], dtype=float) + spec.front_port_offset * hv
        rear = np.array([x, y], dtype=float) - spec.rear_port_offset * hv
        ax.plot([rear[0]], [rear[1]], marker="s", color="black", markersize=3)
        ax.plot([front[0]], [front[1]], marker="o", color="white", markeredgecolor="black", markersize=3)
        ax.text(x, y, vid, fontsize=7, ha="center", va="center", color="black")

    def render_timeline(self, frames: Sequence[Frame], output_path: str) -> str:
        if not frames:
            raise ValueError("No frames to render")

        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        colors = {
            "R1": "#ff6b3d",
            "R2": "#3aaed8",
            "R3": "#47b86e",
            "R4": "#f6b042",
            "R5": "#ad6edb",
        }

        phase_list = [f.phase for f in frames]
        idx0 = 0
        idx1 = max(0, len(frames) // 3)
        idx2 = max(0, 2 * len(frames) // 3)
        idx3 = len(frames) - 1
        picks = [idx0, idx1, idx2, idx3]
        titles = [
            "Initial",
            "Planning / Early Docking",
            "Sequential Docking",
            "Final Articulated Motion",
        ]

        fig, axs = plt.subplots(2, 2, figsize=(13, 11), constrained_layout=True)
        axs = axs.ravel()

        for ax, idx, title in zip(axs, picks, titles):
            self._draw_world(ax)
            fr = frames[idx]
            for vid, pose in fr.poses.items():
                self._draw_vehicle(ax, vid, pose, colors.get(vid, "#4488aa"))
            ax.set_title(f"{title}\nphase={fr.phase}, t={fr.t:.1f}s")

        # Overlay trajectories on final panel.
        ax = axs[-1]
        for vid in colors.keys():
            traj = np.array([[f.poses[vid][0], f.poses[vid][1]] for f in frames], dtype=float)
            ax.plot(traj[:, 0], traj[:, 1], color=colors[vid], alpha=0.35, linewidth=1.1)

        fig.suptitle("Multi-Ackermann Dynamic Docking Simulation", fontsize=14)
        fig.savefig(output_path, dpi=160)
        plt.close(fig)
        return output_path

    def render_animation(self, frames: Sequence[Frame], output_path: str, stride: int = 4) -> str:
        if not frames:
            raise ValueError("No frames to render")
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        colors = {
            "R1": "#ff6b3d",
            "R2": "#3aaed8",
            "R3": "#47b86e",
            "R4": "#f6b042",
            "R5": "#ad6edb",
        }

        sampled = list(frames[::max(1, stride)])

        fig, ax = plt.subplots(figsize=(7.8, 7.6))

        def draw_frame(i: int) -> None:
            ax.clear()
            self._draw_world(ax)
            fr = sampled[i]
            for vid, pose in fr.poses.items():
                self._draw_vehicle(ax, vid, pose, colors.get(vid, "#4488aa"))
            ax.set_title(f"phase={fr.phase}, t={fr.t:.1f}s")

        ani = animation.FuncAnimation(fig, draw_frame, frames=len(sampled), interval=70)

        # Save gif when pillow writer is available.
        try:
            ani.save(output_path, writer="pillow", fps=12)
        finally:
            plt.close(fig)
        return output_path
