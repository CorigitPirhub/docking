#!/usr/bin/env python3
from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.collision import obstacle_polygon
from docking.config import load_config
from docking.scenario_support import ScenarioGenerator


def _load_results(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _mean_by_policy(cases: list[dict[str, Any]]) -> dict[str, dict[str, float]]:
    out: dict[str, dict[str, float]] = {}
    for p in ("independent", "fixed_sequence", "adaptive"):
        t = np.array([float(c[p]["total_time_s"]) for c in cases], dtype=float)
        e = np.array([float(c[p]["total_energy"]) for c in cases], dtype=float)
        out[p] = {
            "mean_time": float(np.mean(t)),
            "mean_energy": float(np.mean(e)),
            "std_time": float(np.std(t)),
            "std_energy": float(np.std(e)),
        }
    return out


def _plot_policy_compare_bar(cases: list[dict[str, Any]], out_path: Path) -> None:
    m = _mean_by_policy(cases)
    policies = ["independent", "fixed_sequence", "adaptive"]
    x = np.arange(len(policies), dtype=float)
    times = np.array([m[p]["mean_time"] for p in policies], dtype=float)
    energies = np.array([m[p]["mean_energy"] for p in policies], dtype=float)
    tstd = np.array([m[p]["std_time"] for p in policies], dtype=float)
    estd = np.array([m[p]["std_energy"] for p in policies], dtype=float)

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2), dpi=170)
    axes[0].bar(x, times, yerr=tstd, color=["#7a8fa6", "#98c1d9", "#2a9d8f"], capsize=3)
    axes[0].set_xticks(x, policies, rotation=8)
    axes[0].set_ylabel("Total Time [s]")
    axes[0].set_title("Policy Time Comparison")
    axes[0].grid(alpha=0.25, axis="y")

    axes[1].bar(x, energies, yerr=estd, color=["#6c757d", "#adb5bd", "#ef476f"], capsize=3)
    axes[1].set_xticks(x, policies, rotation=8)
    axes[1].set_ylabel("Total Energy [arb]")
    axes[1].set_title("Policy Energy Comparison")
    axes[1].grid(alpha=0.25, axis="y")

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _plot_energy_time_scatter(cases: list[dict[str, Any]], out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.6, 5.6), dpi=170)
    style = {
        "independent": dict(color="#7a8fa6", marker="o", alpha=0.70),
        "fixed_sequence": dict(color="#f4a261", marker="s", alpha=0.70),
        "adaptive": dict(color="#2a9d8f", marker="^", alpha=0.78),
    }
    for p in ("independent", "fixed_sequence", "adaptive"):
        e = np.array([float(c[p]["total_energy"]) for c in cases], dtype=float)
        t = np.array([float(c[p]["total_time_s"]) for c in cases], dtype=float)
        ax.scatter(e, t, label=p, s=24, linewidths=0.3, edgecolors="black", **style[p])

    ax.set_xlabel("Total Energy [arb]")
    ax.set_ylabel("Total Time [s]")
    ax.set_title("Energy-Time Distribution by Policy")
    ax.grid(alpha=0.28)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _plot_subtype_summary(cases: list[dict[str, Any]], out_path: Path) -> None:
    subtypes = sorted(set(str(c["subtype"]) for c in cases))
    e_gain = []
    t_gain = []
    for st in subtypes:
        rows = [c for c in cases if c["subtype"] == st]
        e = []
        t = []
        for r in rows:
            eind = float(r["independent"]["total_energy"])
            eadp = float(r["adaptive"]["total_energy"])
            tfix = float(r["fixed_sequence"]["total_time_s"])
            tadp = float(r["adaptive"]["total_time_s"])
            if eind > 1e-9:
                e.append((eind - eadp) / eind)
            if tfix > 1e-9:
                t.append((tfix - tadp) / tfix)
        e_gain.append(float(np.mean(e) if e else 0.0))
        t_gain.append(float(np.mean(t) if t else 0.0))

    x = np.arange(len(subtypes), dtype=float)
    w = 0.38
    fig, ax = plt.subplots(figsize=(11.2, 4.4), dpi=170)
    ax.bar(x - 0.5 * w, 100.0 * np.array(e_gain), width=w, color="#ef476f", label="Energy Reduction vs Independent [%]")
    ax.bar(x + 0.5 * w, 100.0 * np.array(t_gain), width=w, color="#2a9d8f", label="Time Reduction vs Fixed [%]")
    ax.axhline(2.5, color="#ef476f", linestyle="--", linewidth=1.1, alpha=0.55)
    ax.axhline(8.0, color="#2a9d8f", linestyle="--", linewidth=1.1, alpha=0.55)
    ax.set_xticks(x, subtypes)
    ax.set_ylabel("Improvement [%]")
    ax.set_title("Adaptive Policy Gain by Scenario Subtype")
    ax.grid(alpha=0.24, axis="y")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _score_case(record: dict[str, Any]) -> float:
    ind = record["independent"]
    fix = record["fixed_sequence"]
    adp = record["adaptive"]
    eind = float(ind["total_energy"])
    eadp = float(adp["total_energy"])
    tfix = float(fix["total_time_s"])
    tadp = float(adp["total_time_s"])
    e_gain = 0.0 if eind <= 1e-9 else (eind - eadp) / eind
    t_gain = 0.0 if tfix <= 1e-9 else (tfix - tadp) / tfix
    n_dock = int(adp["docked_followers"])
    sched = adp["schedule"]
    penalty = 0.0 if n_dock >= 2 else -0.2
    if not sched:
        return -999.0
    return 1.8 * e_gain + 1.2 * t_gain + 0.06 * n_dock + penalty


def _pick_representative_case(cases: list[dict[str, Any]]) -> dict[str, Any]:
    rows = [c for c in cases if len(c["adaptive"]["schedule"]) > 0]
    if not rows:
        return cases[0]
    rows.sort(key=_score_case, reverse=True)
    return rows[0]


def _parse_scenario_id(scenario_id: str) -> tuple[str, int, int, str]:
    m = re.match(r"^(A1|A2|A3|B1|B2|B3|C1|C2|C3)_seed(\d+)_n(\d+)_(Clustered_At_A|Random_Scattered|Uniform_Spread)$", scenario_id)
    if not m:
        raise ValueError(f"Cannot parse scenario_id: {scenario_id}")
    subtype = m.group(1)
    seed = int(m.group(2))
    n = int(m.group(3))
    mode = m.group(4)
    return subtype, seed, n, mode


def _recover_case(record: dict[str, Any]):
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    sid = str(record["scenario_id"])
    subtype, seed, n, mode = _parse_scenario_id(sid)
    ov = None
    if subtype == "B2":
        k = int(record["labels"]["n_max_pass_global"])
        ov = {"B2_K": k}
    case = gen.generate(subtype, n_vehicles=n, seed=seed, initial_dispersion_mode=mode, overrides=ov)
    return cfg, case


def _plot_intercept_overlay(record: dict[str, Any], out_path: Path) -> None:
    cfg, case = _recover_case(record)
    adp = record["adaptive"]
    sched = adp["schedule"]

    fig, ax = plt.subplots(figsize=(10.5, 5.2), dpi=170)
    ax.set_aspect("equal", adjustable="box")
    half_w = 0.5 * cfg.environment.width
    half_h = 0.5 * cfg.environment.height
    ax.set_xlim(-half_w, half_w)
    ax.set_ylim(-half_h, half_h)

    ax.add_patch(Rectangle((-half_w, -half_h), 2 * half_w, 2 * half_h, fill=False, edgecolor="black", linewidth=1.2))
    for obs in case.obstacles:
        ax.add_patch(Polygon(obstacle_polygon(obs), closed=True, fill=False, edgecolor="dimgray", linewidth=1.0))

    ax.plot(case.path_xy[:, 0], case.path_xy[:, 1], color="#1d3557", linewidth=2.0, label="leader shared path")
    ax.scatter([case.start_xy[0]], [case.start_xy[1]], c="#2a9d8f", s=70, marker="o", label="A")
    ax.scatter([case.goal_xy[0]], [case.goal_xy[1]], c="#e63946", s=70, marker="x", label="B")

    xy0 = np.array([[v.x, v.y] for v in case.vehicles_init], dtype=float)
    ids = [int(v.vehicle_id) for v in case.vehicles_init]
    ax.scatter(xy0[:, 0], xy0[:, 1], c="#8d99ae", s=28, label="initial vehicles")
    for i, v in enumerate(ids):
        ax.text(xy0[i, 0] + 0.15, xy0[i, 1] + 0.10, f"{v}", fontsize=7)

    for k, c in enumerate(sched):
        px, py = float(c["point_xy"][0]), float(c["point_xy"][1])
        follower = int(c["follower_id"])
        src_idx = ids.index(follower)
        sx, sy = float(xy0[src_idx, 0]), float(xy0[src_idx, 1])
        ax.plot([sx, px], [sy, py], color="#f4a261", linewidth=1.0, alpha=0.75)
        ax.scatter([px], [py], c="#ef476f", s=30 + 2 * k, marker="D", alpha=0.86)
        ax.text(px + 0.10, py + 0.08, f"{k+1}:{follower}", fontsize=7, color="#4d194d")

    txt = (
        f"scenario={record['scenario_id']}\n"
        f"subtype={record['subtype']}  docked={adp['docked_followers']}  final_train={adp['final_train_size']}\n"
        f"energy={adp['total_energy']:.2f}  time={adp['total_time_s']:.2f}s"
    )
    ax.text(0.01, 0.99, txt, transform=ax.transAxes, va="top", ha="left", fontsize=8.2)
    ax.set_title("Adaptive Intercept Plan (Path-Point Intercept)")
    ax.grid(alpha=0.20)
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _plot_docking_timeline(record: dict[str, Any], out_path: Path) -> None:
    adp = record["adaptive"]
    sched = adp["schedule"]
    sid = str(record["scenario_id"])

    if not sched:
        fig, ax = plt.subplots(figsize=(9.0, 2.4), dpi=170)
        ax.text(0.5, 0.5, f"No docking schedule in {sid}", ha="center", va="center")
        ax.set_axis_off()
        fig.tight_layout()
        fig.savefig(out_path)
        plt.close(fig)
        return

    followers = [int(c["follower_id"]) for c in sched]
    y = np.arange(len(followers), dtype=float)
    catch_l = np.array([float(c["t_follower"]) for c in sched], dtype=float)
    dock_s = np.array([float(c["dock_start"]) for c in sched], dtype=float)
    dock_f = np.array([float(c["dock_finish"]) for c in sched], dtype=float)
    leader_t = np.array([float(c["t_leader"]) for c in sched], dtype=float)

    fig, ax = plt.subplots(figsize=(10.8, 4.6), dpi=170)
    h = 0.34
    ax.barh(y, np.maximum(0.0, dock_s - catch_l), left=catch_l, height=h, color="#98c1d9", label="catch/alignment")
    ax.barh(y, np.maximum(0.0, dock_f - dock_s), left=dock_s, height=h, color="#2a9d8f", label="docking/lock")
    ax.scatter(leader_t, y, marker="|", s=200, color="#e76f51", label="leader at intercept")

    for i, c in enumerate(sched):
        ax.text(float(dock_f[i]) + 0.08, float(y[i]), f"#{i+1}", va="center", fontsize=7)

    ax.set_yticks(y, [f"veh-{v}" for v in followers])
    ax.set_xlabel("Time [s]")
    ax.set_title(f"Adaptive Docking Timeline: {sid}")
    ax.grid(alpha=0.25, axis="x")
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main() -> None:
    in_path = ROOT / "experiments" / "p2_sequence_results.json"
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)
    data = _load_results(in_path)
    cases = data["cases"]

    p1 = out_dir / "p2_policy_compare_bar.png"
    p2 = out_dir / "p2_energy_time_scatter.png"
    p3 = out_dir / "p2_subtype_summary.png"

    _plot_policy_compare_bar(cases, p1)
    _plot_energy_time_scatter(cases, p2)
    _plot_subtype_summary(cases, p3)

    rep = _pick_representative_case(cases)
    sid = str(rep["scenario_id"])
    p4 = out_dir / f"p2_intercept_overlay_{sid}.png"
    p5 = out_dir / f"p2_docking_timeline_{sid}.png"
    _plot_intercept_overlay(rep, p4)
    _plot_docking_timeline(rep, p5)

    # Stable aliases.
    alias4 = out_dir / "p2_intercept_overlay_representative.png"
    alias5 = out_dir / "p2_docking_timeline_representative.png"
    alias4.write_bytes(p4.read_bytes())
    alias5.write_bytes(p5.read_bytes())

    print("p2_viz_file", p1)
    print("p2_viz_file", p2)
    print("p2_viz_file", p3)
    print("p2_viz_file", p4)
    print("p2_viz_file", p5)
    print("p2_viz_file", alias4)
    print("p2_viz_file", alias5)


if __name__ == "__main__":
    main()
