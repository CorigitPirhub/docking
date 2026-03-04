#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.config import load_config
from docking.scenario_support import ScenarioGenerator
from strategy.p2_p4_integrated import IntegratedDemoConfig, IntegratedP2P4Runner


def _scenario_plan():
    return [
        {
            "name": "A",
            "role": "P2_docking",
            "subtype": "A2",
            "seed": 51042,
            "mode": "Clustered_At_A",
            "n_vehicles": 2,
            "overrides": None,
            "demo_cfg": {
                "duration_s": 60.0,
                "inject_disturbance_for_type_c": False,
            },
        },
        {
            "name": "B",
            "role": "P3_split",
            "subtype": "B1",
            "seed": 62042,
            "mode": "Uniform_Spread",
            "n_vehicles": 2,
            "overrides": None,
            "demo_cfg": {
                "duration_s": 60.0,
                "initial_chain_vehicle_ids": (1, 2),
                "pre_split_initial_chain": False,
                "free_target_speed": 0.85,
                "inject_disturbance_for_type_c": False,
            },
        },
        {
            "name": "C",
            "role": "P4_recovery",
            "subtype": "A2",
            "seed": 51042,
            "mode": "Clustered_At_A",
            "n_vehicles": 2,
            "overrides": None,
            "demo_cfg": {
                "duration_s": 60.0,
                "inject_disturbance_for_type_c": True,
                "inject_disturbance_all_types": True,
                "disturbance_after_docking_s": 1.0,
                "disturbance_vision_prob": 0.9,
                "max_retry_per_vehicle": 2,
                "replan_delay_s": 0.5,
            },
        },
    ]


def _pair_summary(integ: dict, base: dict) -> dict:
    t0 = float(base["done_time_s"])
    t1 = float(integ["done_time_s"])
    e0 = float(base["total_energy"])
    e1 = float(integ["total_energy"])
    return {
        "time_improve_ratio": 0.0 if t0 <= 1e-9 else (t0 - t1) / t0,
        "energy_improve_ratio": 0.0 if e0 <= 1e-9 else (e0 - e1) / e0,
    }


def _case_pass(name: str, integ: dict) -> bool:
    if name == "A":
        return bool(
            integ["success"]
            and integ["dock_success_count"] >= 1
            and integ["collision_count"] == 0
            and integ["done_time_s"] < 60.0
        )
    if name == "B":
        return bool(integ["success"] and integ["split_count"] >= 1 and integ["collision_count"] == 0)
    if name == "C":
        return bool(
            integ["success"]
            and
            integ["p4_interruption_count"] >= 1
            and integ["p4_replan_success_count"] >= 1
            and integ["dock_success_count"] >= 1
            and integ["collision_count"] == 0
        )
    return False


def main() -> None:
    import argparse

    p = argparse.ArgumentParser(description="Run P2-P4 integrated end-to-end demo cases.")
    p.add_argument("--enable-sensor-noise", action="store_true", help="Enable GNSS+vision noise for all cases.")
    p.add_argument(
        "--output-json",
        type=str,
        default=str(ROOT / "experiments" / "p2_p4_integrated_demo_results.json"),
        help="Output JSON path.",
    )
    p.add_argument(
        "--output-md",
        type=str,
        default=str(ROOT / "artifacts" / "P2_P4_INTEGRATED_DEMO_REPORT.md"),
        help="Output markdown report path.",
    )
    args = p.parse_args()

    cfg = load_config()
    gen = ScenarioGenerator(cfg)

    runs = []
    for i, item in enumerate(_scenario_plan()):
        demo_cfg_dict = dict(item["demo_cfg"])
        if args.enable_sensor_noise:
            demo_cfg_dict["enable_sensor_noise"] = True
        dcfg = IntegratedDemoConfig(**demo_cfg_dict)
        case = gen.generate(
            item["subtype"],
            n_vehicles=int(item["n_vehicles"]),
            seed=int(item["seed"]),
            initial_dispersion_mode=item["mode"],
            overrides=item["overrides"],
        )
        runner_base = IntegratedP2P4Runner(
            cfg,
            case,
            policy="independent",
            seed=1000 + i,
            demo_cfg=dcfg,
        )
        runner_integrated = IntegratedP2P4Runner(
            cfg,
            case,
            policy="integrated",
            seed=2000 + i,
            demo_cfg=dcfg,
        )
        base = runner_base.run().to_dict()
        integ = runner_integrated.run().to_dict()
        pair = _pair_summary(integ, base)
        pass_flag = _case_pass(item["name"], integ)
        runs.append(
            {
                "case_name": item["name"],
                "case_meta": {
                    "role": item["role"],
                    "subtype": item["subtype"],
                    "seed": int(item["seed"]),
                    "mode": item["mode"],
                    "overrides": item["overrides"],
                    "n_vehicles": int(item["n_vehicles"]),
                    "scenario_id": case.scenario_id,
                    "demo_cfg": demo_cfg_dict,
                },
                "independent": base,
                "integrated": integ,
                "compare": pair,
                "pass": bool(pass_flag),
            }
        )

    avg_time_improve = sum(float(r["compare"]["time_improve_ratio"]) for r in runs) / max(1, len(runs))
    avg_energy_improve = sum(float(r["compare"]["energy_improve_ratio"]) for r in runs) / max(1, len(runs))
    summary = {
        "num_cases": len(runs),
        "avg_time_improve_ratio": float(avg_time_improve),
        "avg_energy_improve_ratio": float(avg_energy_improve),
        "all_case_pass": bool(all(bool(r["pass"]) for r in runs)),
    }
    result = {
        "config": {
            "notes": "case-wise demo_cfg and n_vehicles used",
            "enable_sensor_noise": bool(args.enable_sensor_noise),
        },
        "runs": runs,
        "summary": summary,
    }

    exp_dir = ROOT / "experiments"
    art_dir = ROOT / "artifacts"
    exp_dir.mkdir(exist_ok=True)
    art_dir.mkdir(exist_ok=True)
    out_json = Path(args.output_json)
    out_md = Path(args.output_md)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_md.parent.mkdir(parents=True, exist_ok=True)

    out_json.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = [
        "# P2-P4 Integrated Demo Report",
        "",
        f"- enable_sensor_noise: {bool(args.enable_sensor_noise)}",
        f"- num_cases: {summary['num_cases']}",
        f"- avg_time_improve_ratio(vs independent): {summary['avg_time_improve_ratio']:.4f}",
        f"- avg_energy_improve_ratio(vs independent): {summary['avg_energy_improve_ratio']:.4f}",
        f"- all_case_pass: {summary['all_case_pass']}",
        "",
    ]
    for r in runs:
        integ = r["integrated"]
        lines += [
            f"## Case {r['case_name']} ({r['case_meta']['subtype']}, role={r['case_meta']['role']})",
            f"- pass: {r['pass']}",
            f"- integrated_success(raw): {integ['success']}",
            f"- integrated_done_time_s: {integ['done_time_s']:.2f}",
            f"- integrated_total_energy: {integ['total_energy']:.2f}",
            f"- leader_final_train_size: {integ['leader_final_train_size']}",
            f"- dock_success_count: {integ['dock_success_count']}",
            f"- split_count: {integ['split_count']}",
            f"- p4_interruption_count: {integ['p4_interruption_count']}",
            f"- p4_replan_success_count: {integ['p4_replan_success_count']}",
            f"- p4_abort_to_independent_count: {integ['p4_abort_to_independent_count']}",
            "",
        ]
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("p2_p4_demo_json", out_json)
    print("p2_p4_demo_md", out_md)


if __name__ == "__main__":
    main()
