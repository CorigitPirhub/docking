#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, Polygon

ROOT = Path(__file__).resolve().parents[1]
import sys
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.collision import obstacle_polygon
from docking.config import load_config
from docking.dockbench import FAMILIES, dataset_root, load_manifest, load_representatives
from docking.kinematics import VehicleGeometry
from docking.types import Obstacle, VehicleState

ROLE_COLORS: dict[str, str] = {
    "background": "#8b949e",
    "screen": "#d62728",
    "dock_zone": "#ff7f0e",
    "channel": "#9467bd",
    "hybrid": "#2ca02c",
}

FAMILY_PREFIX: dict[str, str] = {
    "CF": "common_feasible",
    "SC": "switching_critical",
    "FC": "funnel_critical",
    "EC": "extension_critical",
}


@dataclass(frozen=True)
class SceneRenderSpec:
    family: str
    family_tag: str
    slot: str
    scene_id: str
    scene_path: Path


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Render DockBench static scene figures by family.")
    p.add_argument("--dataset-root", type=str, default=str(dataset_root()), help="DockBench dataset root.")
    p.add_argument("--out-dir", type=str, default=str(ROOT / "artifacts" / "tmp_scenes"), help="Output directory for PNG files.")
    return p.parse_args()


def _load_scene(scene_path: Path) -> dict[str, Any]:
    return json.loads(scene_path.read_text(encoding="utf-8"))


def _vehicle_from_payload(payload: dict[str, Any], *, vehicle_id: int) -> VehicleState:
    return VehicleState(
        vehicle_id=vehicle_id,
        x=float(payload["x"]),
        y=float(payload["y"]),
        yaw=float(payload["yaw"]),
        v=0.0,
        delta=0.0,
    )


def _obstacles_from_scene(scene: dict[str, Any]) -> list[Obstacle]:
    return [
        Obstacle(
            x=float(item["x"]),
            y=float(item["y"]),
            width=float(item["width"]),
            height=float(item["height"]),
            yaw=float(item.get("yaw", 0.0)),
        )
        for item in scene["obstacles"]
    ]


def _fov_points(state: VehicleState, geom: VehicleGeometry, *, fov_deg: float, max_range: float) -> np.ndarray:
    cam = geom.front_hitch(state)
    half = math.radians(float(fov_deg)) * 0.5
    pts = [cam]
    for angle in np.linspace(-half, half, 31):
        heading = float(state.yaw) + float(angle)
        pts.append(cam + np.array([math.cos(heading), math.sin(heading)], dtype=float) * float(max_range))
    return np.asarray(pts, dtype=float)


def _plot_vehicle(ax, geom: VehicleGeometry, state: VehicleState, *, color: str, label: str) -> None:
    body = geom.body_polygon(state)
    patch = Polygon(body, closed=True, facecolor=color, edgecolor="black", linewidth=1.2, alpha=0.86)
    ax.add_patch(patch)
    front = geom.front_hitch(state)
    rear = geom.rear_hitch(state)
    ax.plot([front[0], rear[0]], [front[1], rear[1]], color="white", linewidth=1.2, alpha=0.9)
    ax.scatter([front[0]], [front[1]], color="white", s=18, zorder=6)
    ax.text(float(state.x), float(state.y) + 0.42, label, ha="center", va="bottom", fontsize=9, weight="bold")


def _plot_stage_plan(ax, stage_plan: dict[str, Any]) -> None:
    leader_path = np.asarray(stage_plan.get("leader_path_xy", []), dtype=float)
    follower_path = np.asarray(stage_plan.get("follower_path_xy", []), dtype=float)
    if len(leader_path) >= 2:
        ax.plot(leader_path[:, 0], leader_path[:, 1], color="#2ca02c", linestyle="--", linewidth=1.5, alpha=0.9)
        ax.add_patch(FancyArrowPatch(posA=tuple(leader_path[-2]), posB=tuple(leader_path[-1]), arrowstyle="->", mutation_scale=10, linewidth=1.5, color="#2ca02c"))
    if len(follower_path) >= 2:
        ax.plot(follower_path[:, 0], follower_path[:, 1], color="#1f77b4", linestyle="--", linewidth=1.5, alpha=0.9)
        ax.add_patch(FancyArrowPatch(posA=tuple(follower_path[-2]), posB=tuple(follower_path[-1]), arrowstyle="->", mutation_scale=10, linewidth=1.5, color="#1f77b4"))


def _hardness(scene: dict[str, Any]) -> float:
    descriptor = scene["descriptors"]
    return float(
        descriptor["occlusion_index"]
        + descriptor["staging_shift_required_m"]
        + descriptor["detour_factor"]
        + max(0.0, 1.0 - descriptor["dock_zone_clearance_m"])
        + 0.02 * descriptor["heading_diff_deg"]
    )


def _select_scene_specs(ds_root: Path) -> list[SceneRenderSpec]:
    manifest = load_manifest(ds_root)
    reps = load_representatives(ds_root)
    scenes_by_id = {spec.scene_id: spec for spec in manifest}

    def scene(spec):
        return _load_scene(Path(spec.scenario_json))

    selections: list[SceneRenderSpec] = []
    for family in FAMILIES:
        family_specs = [spec for spec in manifest if spec.family == family]
        tuning_l1 = next(spec for spec in family_specs if spec.split == "tuning" and spec.difficulty == "L1")
        tuning_l3 = next(spec for spec in family_specs if spec.split == "tuning" and spec.difficulty == "L3")
        challenge = next(spec for spec in family_specs if spec.split == "challenge")
        representative = reps[f"{family}_L2"]

        selected_ids = {tuning_l1.scene_id, tuning_l3.scene_id, challenge.scene_id, representative.scene_id}
        test_candidates = [spec for spec in family_specs if spec.split == "test" and spec.scene_id not in selected_ids]
        hardest_test = max(test_candidates, key=lambda spec: (_hardness(scene(spec)), spec.scene_id))

        picks = [
            ("01_tuning_l1", tuning_l1),
            ("02_rep_l2", representative),
            ("03_tuning_l3", tuning_l3),
            ("04_test_hard", hardest_test),
            ("05_challenge", challenge),
        ]
        for slot, spec in picks:
            selections.append(
                SceneRenderSpec(
                    family=family,
                    family_tag=FAMILY_PREFIX[family],
                    slot=slot,
                    scene_id=spec.scene_id,
                    scene_path=Path(spec.scenario_json),
                )
            )
    return selections


def render_scene(scene_path: Path, out_path: Path, *, family: str, slot: str) -> None:
    cfg = load_config()
    geom = VehicleGeometry(cfg.vehicle)
    scene = _load_scene(scene_path)
    leader = _vehicle_from_payload(scene["leader"], vehicle_id=2)
    follower = _vehicle_from_payload(scene["follower"], vehicle_id=1)
    obstacles = _obstacles_from_scene(scene)
    roles = [str(item["role"]) for item in scene["obstacles"]]
    stage_plan = scene["audit"].get("stage_plan", {})

    fig, ax = plt.subplots(figsize=(12.8, 6.8), dpi=180)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-0.5 * float(scene["map"]["width_m"]), 0.5 * float(scene["map"]["width_m"]))
    ax.set_ylim(-0.5 * float(scene["map"]["height_m"]), 0.5 * float(scene["map"]["height_m"]))
    ax.grid(alpha=0.18, linestyle=":")

    for obstacle, role in zip(obstacles, roles):
        poly = obstacle_polygon(obstacle)
        color = ROLE_COLORS.get(role, "#7f7f7f")
        ax.add_patch(Polygon(poly, closed=True, facecolor=color, edgecolor="black", linewidth=0.9, alpha=0.34))
        center = np.mean(poly, axis=0)
        ax.text(float(center[0]), float(center[1]), role, ha="center", va="center", fontsize=7, color="black")

    _plot_stage_plan(ax, stage_plan)
    _plot_vehicle(ax, geom, leader, color="#ff7f0e", label="Leader")
    _plot_vehicle(ax, geom, follower, color="#1f77b4", label="Follower")

    fov_poly = _fov_points(follower, geom, fov_deg=cfg.sensors.vision.fov_deg, max_range=cfg.sensors.vision.max_distance)
    ax.add_patch(Polygon(fov_poly, closed=True, facecolor="#17becf", edgecolor="#17becf", linewidth=1.0, alpha=0.16))

    rear = geom.rear_hitch(leader)
    cam = geom.front_hitch(follower)
    los_color = "#d62728" if bool(scene["scenario"]["direct_los_blocked"]) else "#2ca02c"
    ax.plot([cam[0], rear[0]], [cam[1], rear[1]], color=los_color, linewidth=1.6, linestyle="-.", alpha=0.9)
    ax.scatter([rear[0]], [rear[1]], s=22, color="#ff7f0e", edgecolor="black", zorder=7)
    ax.scatter([cam[0]], [cam[1]], s=22, color="#1f77b4", edgecolor="black", zorder=7)

    descriptor = scene["descriptors"]
    title = f"{family} | {slot} | {scene['scene_id']} | {scene['difficulty']} | {scene['split']}"
    subtitle = (
        f"d0={descriptor['d0_m']:.2f}m, Δψ={descriptor['heading_diff_deg']:.1f}°, occ={descriptor['occlusion_index']:.2f}, "
        f"detour={descriptor['detour_factor']:.2f}, stage={descriptor['staging_shift_required_m']:.2f}m, dock_clear={descriptor['dock_zone_clearance_m']:.2f}m"
    )
    ax.set_title(title + "\n" + subtitle, fontsize=11)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    legend_handles = [
        Line2D([0], [0], color="#1f77b4", linewidth=8, label="Follower"),
        Line2D([0], [0], color="#ff7f0e", linewidth=8, label="Leader"),
        Line2D([0], [0], color="#1f77b4", linestyle="--", linewidth=1.5, label="Follower stage path"),
        Line2D([0], [0], color="#2ca02c", linestyle="--", linewidth=1.5, label="Leader stage path"),
        Line2D([0], [0], color=los_color, linestyle="-.", linewidth=1.6, label="Camera→dock LOS"),
    ]
    ax.legend(handles=legend_handles, loc="upper right", fontsize=8, framealpha=0.95)

    note = (
        f"subset={scene['scenario']['subset_tag']}\n"
        f"blocked={scene['scenario']['direct_los_blocked']}\n"
        f"large_obstacle={scene['scenario']['large_obstacle_present']}\n"
        f"planner_score={float(stage_plan.get('score', 0.0)):.2f}"
    )
    ax.text(
        0.01,
        0.01,
        note,
        transform=ax.transAxes,
        ha="left",
        va="bottom",
        fontsize=8,
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "alpha": 0.88, "edgecolor": "#cccccc"},
    )

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    ds_root = dataset_root(args.dataset_root)
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    for png in out_dir.glob("*.png"):
        png.unlink()
    manifest_path = out_dir / "scene_manifest.json"
    if manifest_path.exists():
        manifest_path.unlink()

    scene_specs = _select_scene_specs(ds_root)
    manifest: list[dict[str, str]] = []
    for spec in scene_specs:
        out_path = out_dir / f"{spec.family.lower()}_{spec.slot}_{spec.scene_id}.png"
        render_scene(spec.scene_path, out_path, family=spec.family, slot=spec.slot)
        manifest.append(
            {
                "family": spec.family,
                "family_tag": spec.family_tag,
                "slot": spec.slot,
                "scene_id": spec.scene_id,
                "scene_json": str(spec.scene_path),
                "image": str(out_path),
            }
        )
        print(f"saved {out_path}")

    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"manifest {manifest_path}")


if __name__ == "__main__":
    main()
