#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Summarize Stage-2/Stage-2.5 failure modes by family/difficulty/category.")
    p.add_argument("input", type=str, help="Path to `co_only_summary.json` or `p_minus1_stage2_results.json`.")
    p.add_argument("--out", type=str, default="", help="Optional output json path.")
    return p.parse_args()


def _load_rows(payload: dict[str, Any]) -> list[dict[str, Any]]:
    if "rows" in payload and payload["rows"] and "scene_id" in payload["rows"][0]:
        return list(payload["rows"])
    raise ValueError("Unsupported input payload: expected rows[].")


def main() -> None:
    args = parse_args()
    payload = json.loads(Path(args.input).read_text(encoding="utf-8"))
    rows = _load_rows(payload)
    fails = [row for row in rows if not bool(row.get("success", False))]

    by_family: dict[str, Counter[str]] = defaultdict(Counter)
    by_family_difficulty: dict[str, Counter[str]] = defaultdict(Counter)
    by_scene: dict[str, dict[str, Any]] = {}
    for row in fails:
        family = str(row.get("family") or row.get("family_code") or "?")
        difficulty = str(row.get("difficulty") or "?")
        failure = str(row.get("failure_category") or row.get("reason") or "unknown")
        by_family[family][failure] += 1
        by_family_difficulty[f"{family}-{difficulty}"][failure] += 1
        by_scene[str(row["scene_id"])] = {
            "family": family,
            "difficulty": difficulty,
            "failure_category": failure,
            "collision": bool(row.get("collision", False)),
            "done_time_s": float(row.get("done_time_s", 0.0)),
            "trajectory_cost": float(row.get("trajectory_cost", 0.0)),
            "fallback_count": int(row.get("fallback_count", 0)),
            "visual_loss_count": int(row.get("visual_loss_count", 0)),
            "replan_count": int(row.get("replan_count", 0)),
        }

    summary = {
        "num_rows": len(rows),
        "num_failures": len(fails),
        "by_family": {key: dict(value) for key, value in sorted(by_family.items())},
        "by_family_difficulty": {key: dict(value) for key, value in sorted(by_family_difficulty.items())},
        "by_scene": dict(sorted(by_scene.items())),
    }

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if str(args.out).strip():
        Path(args.out).write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
