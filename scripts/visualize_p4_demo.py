#!/usr/bin/env python3
from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from typing import Any

import matplotlib.animation as animation
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


def _parse_scenario_id(scenario_id: str) -> tuple[str, int, int, str]:
    m = re.match(
        r"^(A1|A2|A3|B1|B2|B3|C1|C2|C3)_seed(\d+)_n(\d+)_(Clustered_At_A|Random_Scattered|Uniform_Spread)$",
        scenario_id,
    )
    if not m:
        raise ValueError(f"cannot parse scenario_id: {scenario_id}")
    return m.group(1), int(m.group(2)), int(m.group(3)), m.group(4)


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


def _pick_representative_case(cases: list[dict[str, Any]]) -> dict[str, Any]:
    def score(c: dict[str, Any]) -> float:
        inj = c["injected"]
        traces = inj["traces"]
        interrupted = sum(1 for t in traces if bool(t["interrupted"]))
        redock = sum(1 for t in traces if str(t["fallback_action"]) == "REPLAN_REDOCK")
        within3 = sum(1 for t in traces if bool(t["interrupted"]) and bool(t["reentered_within_3s"]))
        return 3.0 * interrupted + 1.2 * redock + 0.8 * within3

    rows = [c for c in cases if c["injected"]["interruption_count"] > 0]
    if not rows:
        return cases[0]
    rows.sort(key=score, reverse=True)
    return rows[0]


def _polyline_s(path_xy: np.ndarray) -> np.ndarray:
    if len(path_xy) <= 1:
        return np.zeros(len(path_xy), dtype=float)
    ds = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(ds)])


def _interp_path(path_xy: np.ndarray, s_samples: np.ndarray, s_query: float) -> np.ndarray:
    s = float(np.clip(s_query, s_samples[0], s_samples[-1]))
    if s <= s_samples[0] + 1e-9:
        return path_xy[0].copy()
    if s >= s_samples[-1] - 1e-9:
        return path_xy[-1].copy()
    idx = int(np.searchsorted(s_samples, s, side="right") - 1)
    idx = max(0, min(idx, len(s_samples) - 2))
    ds = max(1e-9, float(s_samples[idx + 1] - s_samples[idx]))
    u = (s - float(s_samples[idx])) / ds
    return (1.0 - u) * path_xy[idx] + u * path_xy[idx + 1]


def _plot_timeline(record: dict[str, Any], out_path: Path) -> None:
    inj = record["injected"]
    traces = inj["traces"]
    if not traces:
        fig, ax = plt.subplots(figsize=(8.0, 2.6), dpi=170)
        ax.text(0.5, 0.5, "no trace", ha="center", va="center")
        ax.set_axis_off()
        fig.tight_layout()
        fig.savefig(out_path)
        plt.close(fig)
        return

    y = np.arange(len(traces), dtype=float)
    fig, ax = plt.subplots(figsize=(11.0, 4.8), dpi=170)
    for i, tr in enumerate(traces):
        init = tr["initial_candidate"]
        if init is None:
            continue
        t0 = float(init["t_follower"])
        t1 = float(tr["interruption_time_s"]) if bool(tr["interrupted"]) else float(t0 + 0.8)
        t2 = float(t1 + tr["recovery_latency_s"]) if bool(tr["interrupted"]) else t1
        ax.barh(i, max(0.0, t1 - 0.0), left=0.0, height=0.32, color="#98c1d9", alpha=0.95)
        if bool(tr["interrupted"]):
            ax.barh(i, max(0.0, t2 - t1), left=t1, height=0.32, color="#ffd166", alpha=0.95)
            ax.scatter([t1], [i], c="#e63946", s=22, marker="x")
        if tr["fallback_action"] == "REPLAN_REDOCK":
            ax.barh(i, 0.9, left=t2, height=0.32, color="#2a9d8f", alpha=0.95)
        elif tr["fallback_action"] in {"SWITCH_INDEPENDENT", "ABORT_TO_INDEPENDENT"}:
            ax.barh(i, 0.9, left=t2, height=0.32, color="#457b9d", alpha=0.95)
        else:
            ax.barh(i, 0.9, left=t1, height=0.32, color="#2a9d8f", alpha=0.95)

    ax.set_yticks(y, [f"veh-{int(t['follower_id'])}" for t in traces])
    ax.set_xlabel("time [s]")
    ax.set_title(f"P4 recovery timeline: {record['scenario_id']}")
    ax.grid(alpha=0.22, axis="x")
    txt = (
        f"interruptions={inj['interruption_count']}  "
        f"recoveries={inj['recovered_count']}  "
        f"within3s={inj['reenter_3s_count']}  "
        f"failures={inj['failure_count']}"
    )
    ax.text(0.01, 0.99, txt, transform=ax.transAxes, va="top", ha="left", fontsize=8.2)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _build_demo_gif(record: dict[str, Any], out_path: Path) -> None:
    cfg, case = _recover_case(record)
    s_samples = _polyline_s(case.path_xy)
    path_len = float(s_samples[-1]) if len(s_samples) else 0.0
    leader_speed = 1.55
    leader_goal_t = path_len / max(leader_speed, 1e-6)

    traces = record["injected"]["traces"]
    tr_list = [t for t in traces if bool(t["interrupted"]) and t["initial_candidate"] is not None]
    tr = tr_list[0] if tr_list else next((t for t in traces if t["initial_candidate"] is not None), None)
    if tr is None:
        return
    follower_id = int(tr["follower_id"])
    f0 = next(v for v in case.vehicles_init if int(v.vehicle_id) == follower_id)
    p_start = np.array([float(f0.x), float(f0.y)], dtype=float)
    p_goal = case.goal_xy.copy()
    init = tr["initial_candidate"]
    p_int = np.array([float(init["point_xy"][0]), float(init["point_xy"][1])], dtype=float)
    t_catch = float(init["t_follower"])
    t_event = float(tr["interruption_time_s"]) if bool(tr["interrupted"]) else max(0.6, t_catch * 0.8)
    t_recover = t_event + float(tr["recovery_latency_s"])
    rc = tr["recovery_candidate"]
    p_rec = None if rc is None else np.array([float(rc["point_xy"][0]), float(rc["point_xy"][1])], dtype=float)
    t_end = max(leader_goal_t, t_recover + 4.0, 8.0)

    def lerp(a: np.ndarray, b: np.ndarray, u: float) -> np.ndarray:
        uu = float(np.clip(u, 0.0, 1.0))
        return (1.0 - uu) * a + uu * b

    def leader_pos(t: float) -> np.ndarray:
        s = min(path_len, max(0.0, t * leader_speed))
        return _interp_path(case.path_xy, s_samples, s)

    def follower_pos(t: float) -> np.ndarray:
        if t <= t_catch:
            return lerp(p_start, p_int, t / max(t_catch, 1e-6))
        if not bool(tr["interrupted"]):
            return lerp(p_int, p_goal, (t - t_catch) / max(t_end - t_catch, 1e-6))
        p_evt = leader_pos(t_event)
        if t <= t_event:
            return lerp(p_int, p_evt, (t - t_catch) / max(t_event - t_catch, 1e-6))
        if t <= t_recover:
            return p_evt.copy()
        if p_rec is not None:
            t_mid = min(t_end, t_recover + 2.0)
            if t <= t_mid:
                return lerp(p_evt, p_rec, (t - t_recover) / max(t_mid - t_recover, 1e-6))
            return lerp(p_rec, p_goal, (t - t_mid) / max(t_end - t_mid, 1e-6))
        return lerp(p_evt, p_goal, (t - t_recover) / max(t_end - t_recover, 1e-6))

    fig, ax = plt.subplots(figsize=(9.8, 5.2), dpi=150)
    ax.set_aspect("equal", adjustable="box")
    hw = 0.5 * cfg.environment.width
    hh = 0.5 * cfg.environment.height
    ax.set_xlim(-hw, hw)
    ax.set_ylim(-hh, hh)
    ax.add_patch(Rectangle((-hw, -hh), 2 * hw, 2 * hh, fill=False, edgecolor="black", linewidth=1.2))
    for obs in case.obstacles:
        ax.add_patch(Polygon(obstacle_polygon(obs), closed=True, fill=False, edgecolor="dimgray", linewidth=1.0))

    ax.plot(case.path_xy[:, 0], case.path_xy[:, 1], color="#1d3557", linewidth=2.0, label="leader shared path")
    ax.scatter([case.start_xy[0]], [case.start_xy[1]], c="#2a9d8f", s=70, marker="o", label="A")
    ax.scatter([case.goal_xy[0]], [case.goal_xy[1]], c="#e63946", s=70, marker="x", label="B")
    ax.scatter([p_start[0]], [p_start[1]], c="#8d99ae", s=38, marker="s", label=f"follower-{follower_id} start")
    ax.scatter([p_int[0]], [p_int[1]], c="#f4a261", s=44, marker="D", label="initial intercept")
    if p_rec is not None:
        ax.scatter([p_rec[0]], [p_rec[1]], c="#2a9d8f", s=44, marker="D", label="replanned intercept")

    leader_dot = ax.scatter([], [], c="#d90429", s=55, marker="o", zorder=5, label="leader")
    follower_dot = ax.scatter([], [], c="#264653", s=55, marker="o", zorder=5, label=f"follower-{follower_id}")
    time_text = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", ha="left", fontsize=8.3)
    info_text = ax.text(
        0.01,
        0.93,
        f"event={tr['interruption_type']}  action={tr['fallback_action']}  latency={tr['recovery_latency_s']:.2f}s",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=8.1,
    )
    _ = info_text
    ax.grid(alpha=0.20)
    ax.legend(loc="lower right", fontsize=7)
    ax.set_title(f"P4 demo: interruption + fallback + recovery ({record['scenario_id']})")

    times = np.linspace(0.0, t_end, 120)

    def _update(frame_id: int):
        t = float(times[frame_id])
        lp = leader_pos(t)
        fp = follower_pos(t)
        leader_dot.set_offsets([[lp[0], lp[1]]])
        follower_dot.set_offsets([[fp[0], fp[1]]])
        phase = "nominal"
        if bool(tr["interrupted"]) and t >= t_event:
            phase = "fallback_recovery" if t < t_recover else "post_recovery"
        time_text.set_text(f"t={t:.2f}s  phase={phase}")
        return leader_dot, follower_dot, time_text

    ani = animation.FuncAnimation(fig, _update, frames=len(times), interval=70, blit=True)
    writer = animation.PillowWriter(fps=14)
    ani.save(out_path, writer=writer)
    plt.close(fig)


def main() -> None:
    in_path = ROOT / "experiments" / "p4_recovery_results.json"
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)
    data = _load_results(in_path)
    cases = data["summary"]["cases"]
    rep = _pick_representative_case(cases)

    p1 = out_dir / "p4_demo_timeline.png"
    p2 = out_dir / "p4_demo_recovery.gif"
    _plot_timeline(rep, p1)
    _build_demo_gif(rep, p2)
    print("p4_demo_file", p1)
    print("p4_demo_file", p2)


if __name__ == "__main__":
    main()
