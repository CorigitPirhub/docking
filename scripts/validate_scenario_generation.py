#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.scenario_support import ScenarioValidator


def main() -> None:
    cfg = load_config()
    parser = argparse.ArgumentParser(description="Validate parameterized scenario generation/labeling/classification")
    parser.add_argument("--seeds-per-subtype", type=int, default=cfg.scenario.validation_seeds_per_subtype)
    args = parser.parse_args()

    validator = ScenarioValidator(cfg)
    report = validator.validate_batch(seeds_per_subtype=args.seeds_per_subtype)

    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)
    json_path = out_dir / "scenario_validation_report.json"
    md_path = out_dir / "SCENARIO_VALIDATION_REPORT.md"

    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = [
        "# Scenario Validation Report",
        "",
        "## Overall",
        f"- total: {report['overall']['total']}",
        f"- passed: {report['overall']['passed']}",
        f"- pass_rate: {report['overall']['pass_rate']:.4f}",
        f"- ok: {report['overall']['ok']}",
        "",
        "## By Subtype",
    ]
    for sub in sorted(report["by_subtype"]):
        m = report["by_subtype"][sub]
        lines.append(
            f"- {sub}: pass_rate={m['pass_rate']:.4f}, passed={m['passed']}/{m['total']}, threshold={m['min_required']:.2f}, ok={m['ok']}"
        )
    if report["failed_examples"]:
        lines += ["", "## Failed Examples (Top 30)"]
        for row in report["failed_examples"]:
            lines.append(
                f"- {row['scenario_id']}: expected={row['expected_type']} predicted={row['predicted_type']} checks={row['checks']}"
            )
    lines += ["", f"JSON: {json_path}"]
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("scenario_validation_json", json_path)
    print("scenario_validation_md", md_path)
    print("scenario_validation_ok", report["overall"]["ok"])


if __name__ == "__main__":
    main()
