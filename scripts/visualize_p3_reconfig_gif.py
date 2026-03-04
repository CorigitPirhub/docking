#!/usr/bin/env python3
from __future__ import annotations

import io
import json
import sys
from pathlib import Path
from typing import Any

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _pick_rep(results: list[dict[str, Any]], prefix: str) -> dict[str, Any]:
    rows = [r for r in results if str(r["subtype"]).startswith(prefix)]
    if not rows:
        raise ValueError(f"no result for prefix={prefix}")
    rows.sort(key=lambda r: len(r["events"]), reverse=True)
    return rows[0]


def _render_frame(trace: list[dict[str, float]], events: list[dict[str, Any]], i_end: int, title: str) -> np.ndarray:
    s = np.array([float(x["s"]) for x in trace[:i_end]], dtype=float)
    n = np.array([float(x["train_size"]) for x in trace[:i_end]], dtype=float)
    nmax = np.array([float(x["nmax"]) for x in trace[:i_end]], dtype=float)
    s_last = float(s[-1]) if len(s) else 0.0

    fig, ax = plt.subplots(figsize=(8.8, 3.8), dpi=120)
    ax.plot(s, nmax, color="#6c757d", linewidth=2.0, label="n_max_pass")
    ax.plot(s, n, color="#2a9d8f", linewidth=2.2, label="planned_train_size")
    ax.fill_between(s, n, nmax, where=(n > nmax), color="#ef233c", alpha=0.22)
    for ev in events:
        se = float(ev["s"])
        if se > s_last + 1e-9:
            continue
        color = "#e63946" if ev["event_type"] == "SPLIT" else "#3a86ff"
        ax.axvline(se, color=color, alpha=0.30, linewidth=1.0)
    ax.set_xlim(0.0, max(1.0, float(trace[-1]["s"])))
    y_hi = max(2.0, float(np.max(nmax) + 0.8))
    ax.set_ylim(0.5, y_hi)
    ax.set_xlabel("Path s [m]")
    ax.set_ylabel("Train Size")
    ax.set_title(f"{title}  (s={s_last:.2f}m)")
    ax.grid(alpha=0.24)
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()

    buf = io.BytesIO()
    fig.savefig(buf, format="png")
    plt.close(fig)
    buf.seek(0)
    frame = imageio.imread(buf)
    return frame


def _make_gif(result: dict[str, Any], out_path: Path, title: str) -> None:
    trace = result["size_profile"]
    events = result["events"]
    n = len(trace)
    if n <= 1:
        return
    frame_idx = np.linspace(2, n, 90, dtype=int)
    frames = [_render_frame(trace, events, int(k), title) for k in frame_idx]
    imageio.mimsave(out_path, frames, duration=0.07)


def main() -> None:
    data = _load(ROOT / "experiments" / "p3_reconfig_results.json")
    results = data["summary"]["results"]
    rb = _pick_rep(results, "B")
    rc = _pick_rep(results, "C")

    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)
    p_b = out_dir / "p3_reconfig_B_representative.gif"
    p_c = out_dir / "p3_reconfig_C_representative.gif"
    _make_gif(rb, p_b, title=f"P3 Reconfig B Rep: {rb['scenario_id']}")
    _make_gif(rc, p_c, title=f"P3 Reconfig C Rep: {rc['scenario_id']}")
    print("p3_gif_file", p_b)
    print("p3_gif_file", p_c)


if __name__ == "__main__":
    main()
