#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.collision import obstacle_polygon
from docking.config import load_config
from docking.scenario_support import ScenarioGenerator


def draw_case(case, cfg, out_path: Path, title: str) -> None:
    assert case.labels is not None
    fig, ax = plt.subplots(figsize=(10, 5), dpi=170)
    ax.set_aspect("equal", adjustable="box")
    half_w = 0.5 * cfg.environment.width
    half_h = 0.5 * cfg.environment.height
    ax.set_xlim(-half_w, half_w)
    ax.set_ylim(-half_h, half_h)

    ax.add_patch(Rectangle((-half_w, -half_h), 2 * half_w, 2 * half_h, fill=False, edgecolor="black", linewidth=1.2))

    for obs in case.obstacles:
        ax.add_patch(Polygon(obstacle_polygon(obs), closed=True, fill=False, edgecolor="dimgray", linewidth=1.2))

    ax.plot(case.path_xy[:, 0], case.path_xy[:, 1], color="tab:blue", linewidth=2.1, label="task-critical path")
    ax.scatter([case.start_xy[0]], [case.start_xy[1]], c="tab:green", s=70, marker="o", label="A")
    ax.scatter([case.goal_xy[0]], [case.goal_xy[1]], c="tab:red", s=70, marker="x", label="B")

    xy = np.array([[v.x, v.y] for v in case.vehicles_init], dtype=float)
    ax.scatter(xy[:, 0], xy[:, 1], c="tab:orange", s=32, label="initial vehicles")

    for z in case.labels.split_mandatory_zones:
        ax.plot([z["x0"], z["x1"]], [z["y0"], z["y1"]], color="tab:red", linewidth=4.0, alpha=0.6)
    for z in case.labels.dock_friendly_zones:
        ax.plot([z["x0"], z["x1"]], [z["y0"], z["y1"]], color="tab:green", linewidth=3.0, alpha=0.55)

    txt = (
        f"type={case.labels.predicted_type} (expected {case.labels.expected_type})\n"
        f"subtype={case.subtype}  n_max_global={case.labels.n_max_pass_global}\n"
        f"bottleneck_count={case.labels.bottleneck_count}  occlusion={case.labels.occlusion_level}\n"
        f"decision_horizon={case.labels.decision_horizon}  coordination={case.labels.coordination_intensity}\n"
        f"initial_dispersion={case.labels.initial_dispersion_mode}"
    )
    ax.text(0.01, 0.99, txt, transform=ax.transAxes, va="top", ha="left", fontsize=8.5)
    ax.set_title(title)
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main() -> None:
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)

    reps = [
        ("A1", "Clustered_At_A", {}, "A1 Open-Plain"),
        ("A2", "Clustered_At_A", {}, "A2 Open-Sparse"),
        ("A3", "Clustered_At_A", {}, "A3 Open-WideS"),
        ("B1", "Clustered_At_A", {}, "B1 Bottleneck-SingleOnly"),
        ("B2", "Clustered_At_A", {"B2_K": 2}, "B2 Bottleneck-K2"),
        ("B3", "Clustered_At_A", {}, "B3 Bottleneck-Serial"),
        ("C1", "Random_Scattered", {}, "C1 Open-Narrow-Open"),
        ("C2", "Random_Scattered", {}, "C2 Hybrid-MultiPocket"),
        ("C3", "Random_Scattered", {}, "C3 Hybrid-OcclusionDock"),
    ]

    outputs = []
    for subtype, mode, ov, title in reps:
        case = gen.generate(
            subtype,
            seed=cfg.testing.random_seed + 7 * len(outputs),
            n_vehicles=cfg.scenario.representative_vehicle_count,
            initial_dispersion_mode=mode,
            overrides=ov,
        )
        path = out_dir / f"scenario_{subtype}.png"
        draw_case(case, cfg, path, title)
        outputs.append(path)

    # Keep type-level representative aliases for compatibility.
    type_alias = {
        "A": out_dir / "scenario_A1.png",
        "B": out_dir / "scenario_B2.png",
        "C": out_dir / "scenario_C1.png",
    }
    for t, src in type_alias.items():
        dst = out_dir / f"scenario_type_{t}.png"
        if src.exists():
            dst.write_bytes(src.read_bytes())
            outputs.append(dst)

    for p in outputs:
        print("scenario_rep_png", p)


if __name__ == "__main__":
    main()
