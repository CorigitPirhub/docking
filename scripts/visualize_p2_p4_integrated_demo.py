#!/usr/bin/env python3
from __future__ import annotations

import io
import json
import re
import sys
from pathlib import Path
from typing import Any

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.collision import obstacle_polygon
from docking.config import load_config
from docking.kinematics import VehicleGeometry
from docking.scenario_support import ScenarioGenerator
from docking.types import VehicleState


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _parse_scenario_id(scenario_id: str) -> tuple[str, int, int, str]:
    m = re.match(
        r"^(A1|A2|A3|B1|B2|B3|C1|C2|C3)_seed(\d+)_n(\d+)_(Clustered_At_A|Random_Scattered|Uniform_Spread)$",
        scenario_id,
    )
    if not m:
        raise ValueError(f"cannot parse scenario_id: {scenario_id}")
    return m.group(1), int(m.group(2)), int(m.group(3)), m.group(4)


def _recover_case(meta: dict[str, Any]):
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    subtype = str(meta["subtype"])
    seed = int(meta["seed"])
    n = int(meta["n_vehicles"])
    mode = str(meta["mode"])
    overrides = meta.get("overrides", None)
    case = gen.generate(subtype, n_vehicles=n, seed=seed, initial_dispersion_mode=mode, overrides=overrides)
    return cfg, case


def _vehicle_poly(geom: VehicleGeometry, d: dict[str, float], vid: int) -> np.ndarray:
    st = VehicleState(vehicle_id=vid, x=float(d["x"]), y=float(d["y"]), yaw=float(d["yaw"]), v=float(d["v"]), delta=0.0)
    return geom.body_polygon(st)


def _mode_color(mode: str) -> str:
    m = str(mode)
    if m == "DOCKING":
        return "#f4a261"
    if m == "TRAIN_FOLLOW":
        return "#2a9d8f"
    if m == "WAIT":
        return "#457b9d"
    return "#8d99ae"


def _events_until(events: list[dict[str, Any]], t: float, max_items: int = 4) -> list[str]:
    rows = [e for e in events if float(e["t"]) <= t + 1e-9]
    rows = rows[-max_items:]
    return [f"{x['t']:.1f}s {x['source']}:{x['event']} {x['detail']}" for x in rows]


def _render_gif(case, run_data: dict[str, Any], out_path: Path, fps: int = 14) -> None:
    cfg = load_config()
    geom = VehicleGeometry(cfg.vehicle)
    hist = run_data["history"]
    events = run_data["events"]
    if not hist:
        return
    sample_idx = np.linspace(0, len(hist) - 1, 170, dtype=int)
    sample_idx = np.unique(sample_idx)

    frames = []
    for idx in sample_idx:
        h = hist[int(idx)]
        t = float(h["t"])
        fig, ax = plt.subplots(figsize=(10.2, 5.6), dpi=130)
        ax.set_aspect("equal", adjustable="box")
        hw = 0.5 * cfg.environment.width
        hh = 0.5 * cfg.environment.height
        ax.set_xlim(-hw, hw)
        ax.set_ylim(-hh, hh)
        ax.add_patch(Rectangle((-hw, -hh), 2 * hw, 2 * hh, fill=False, edgecolor="black", linewidth=1.2))
        for obs in case.obstacles:
            ax.add_patch(Polygon(obstacle_polygon(obs), closed=True, fill=False, edgecolor="dimgray", linewidth=1.0))

        ax.plot(case.path_xy[:, 0], case.path_xy[:, 1], color="#1d3557", linewidth=2.0, label="shared path")
        ax.scatter([case.start_xy[0]], [case.start_xy[1]], c="#2a9d8f", s=65, marker="o", label="A")
        ax.scatter([case.goal_xy[0]], [case.goal_xy[1]], c="#e63946", s=65, marker="x", label="B")

        states = h["states"]
        for edge in h["edges"]:
            a = str(edge[0])
            b = str(edge[1])
            pa = states[a]
            pb = states[b]
            ax.plot([pa["x"], pb["x"]], [pa["y"], pb["y"]], color="#264653", linewidth=1.2, alpha=0.9)

        pending = h["pending_docks"]
        for f, l in pending.items():
            if f in states and str(l) in states:
                sf = states[f]
                sl = states[str(l)]
                ax.plot([sf["x"], sl["x"]], [sf["y"], sl["y"]], color="#ff006e", linewidth=1.0, linestyle="--", alpha=0.8)

        for k, v in states.items():
            vid = int(k)
            poly = _vehicle_poly(geom, v, vid)
            color = _mode_color(str(v["mode"]))
            ax.add_patch(Polygon(poly, closed=True, facecolor=color, edgecolor="black", linewidth=0.8, alpha=0.88))
            ax.text(float(v["x"]) + 0.10, float(v["y"]) + 0.10, f"{vid}", fontsize=7)

        evt_text = _events_until(events, t, max_items=4)
        ax.text(
            0.01,
            0.99,
            f"t={t:.2f}s  leader_s={float(h['leader_s']):.2f}  pending={len(pending)}",
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=8.2,
        )
        ax.text(
            0.01,
            0.92,
            "\n".join(evt_text) if evt_text else "events: none",
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=7.3,
        )
        ax.set_title(
            f"P2-P4 Integrated Demo | {run_data['scenario_id']} | docks={run_data['dock_success_count']} "
            f"splits={run_data['split_count']} interrupts={run_data['p4_interruption_count']}"
        )
        ax.grid(alpha=0.20)
        ax.legend(loc="lower right", fontsize=7)
        fig.tight_layout()
        buf = io.BytesIO()
        fig.savefig(buf, format="png")
        plt.close(fig)
        buf.seek(0)
        frames.append(imageio.imread(buf))

    imageio.mimsave(out_path, frames, duration=1.0 / max(1, fps))


def _plot_compare_summary(data: dict[str, Any], out_path: Path) -> None:
    runs = data["runs"]
    labels = [r["case_name"] for r in runs]
    t_gain = [100.0 * float(r["compare"]["time_improve_ratio"]) for r in runs]
    e_gain = [100.0 * float(r["compare"]["energy_improve_ratio"]) for r in runs]
    docks = [int(r["integrated"]["dock_success_count"]) for r in runs]
    splits = [int(r["integrated"]["split_count"]) for r in runs]
    interrupts = [int(r["integrated"]["p4_interruption_count"]) for r in runs]

    x = np.arange(len(labels), dtype=float)
    w = 0.38
    fig, axes = plt.subplots(2, 1, figsize=(10.2, 7.2), dpi=170)
    axes[0].bar(x - 0.5 * w, t_gain, width=w, color="#2a9d8f", label="Time gain vs independent [%]")
    axes[0].bar(x + 0.5 * w, e_gain, width=w, color="#ef476f", label="Energy gain vs independent [%]")
    axes[0].set_xticks(x, labels)
    axes[0].set_ylabel("Gain [%]")
    axes[0].set_title("Integrated P2-P4 Effectiveness")
    axes[0].grid(alpha=0.24, axis="y")
    axes[0].legend(loc="upper right", fontsize=8)

    axes[1].plot(x, docks, marker="o", color="#264653", linewidth=1.8, label="dock_success")
    axes[1].plot(x, splits, marker="s", color="#e76f51", linewidth=1.8, label="split_count")
    axes[1].plot(x, interrupts, marker="^", color="#ff006e", linewidth=1.8, label="p4_interruptions")
    axes[1].set_xticks(x, labels)
    axes[1].set_ylabel("Count")
    axes[1].set_title("Reconfiguration / Recovery Events")
    axes[1].grid(alpha=0.24, axis="y")
    axes[1].legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main() -> None:
    import argparse

    p = argparse.ArgumentParser(description="Visualize P2-P4 integrated demo results (GIF + summary plot).")
    p.add_argument(
        "--input",
        type=str,
        default=str(ROOT / "experiments" / "p2_p4_integrated_demo_results.json"),
        help="Input JSON path.",
    )
    p.add_argument(
        "--tag",
        type=str,
        default="",
        help="Optional tag appended to output filenames (e.g. 'noise').",
    )
    args = p.parse_args()

    in_path = Path(args.input)
    data = _load(in_path)
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)
    tag = str(args.tag).strip()
    prefix = f"p2_p4_demo_{tag}_" if tag else "p2_p4_demo_"

    for row in data["runs"]:
        name = row["case_name"]
        cfg, case = _recover_case(row["case_meta"])
        _ = cfg
        integ = row["integrated"]
        out_gif = out_dir / f"{prefix}{name}.gif"
        _render_gif(case, integ, out_gif)
        print("p2_p4_demo_gif", out_gif)

    p_sum = out_dir / (f"p2_p4_demo_summary_{tag}.png" if tag else "p2_p4_demo_summary.png")
    _plot_compare_summary(data, p_sum)
    print("p2_p4_demo_summary", p_sum)

    # Keep compatibility alias for user-facing P4 demo.
    if not tag:
        c_gif = out_dir / "p2_p4_demo_C.gif"
        alias = out_dir / "p4_demo_recovery.gif"
        if c_gif.exists():
            alias.write_bytes(c_gif.read_bytes())
            print("p2_p4_demo_alias", alias)


if __name__ == "__main__":
    main()
