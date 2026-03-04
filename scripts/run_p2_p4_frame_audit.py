#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.collision import CollisionEngine
from docking.config import load_config
from docking.kinematics import AckermannModel
from docking.math_utils import angle_diff
from docking.scenario_support import ScenarioGenerator
from docking.train import TrainKinematics
from docking.types import ControlCommand, VehicleState
from strategy.p2_p4_integrated import IntegratedDemoConfig, IntegratedP2P4Runner


def _scenario_plan() -> list[dict[str, Any]]:
    return [
        {
            "name": "A",
            "subtype": "A2",
            "seed": 51042,
            "mode": "Clustered_At_A",
            "n_vehicles": 2,
            "demo_cfg": {
                "duration_s": 60.0,
                "inject_disturbance_for_type_c": False,
            },
        },
        {
            "name": "B",
            "subtype": "B1",
            "seed": 62042,
            "mode": "Uniform_Spread",
            "n_vehicles": 2,
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
            "subtype": "A2",
            "seed": 51042,
            "mode": "Clustered_At_A",
            "n_vehicles": 2,
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


def _to_state(vid: int, s: dict[str, Any]) -> VehicleState:
    return VehicleState(
        vehicle_id=int(vid),
        x=float(s["x"]),
        y=float(s["y"]),
        yaw=float(s["yaw"]),
        v=float(s["v"]),
        delta=float(s["delta"]),
    )


def _edges_set(row: dict[str, Any]) -> set[tuple[int, int]]:
    return {(int(a), int(b)) for a, b in row.get("edges", [])}


def _pending_map(row: dict[str, Any]) -> dict[int, int]:
    return {int(k): int(v) for k, v in row.get("pending_docks", {}).items()}


def _build_chains(edges: set[tuple[int, int]], vehicle_ids: list[int]) -> list[list[int]]:
    parent: dict[int, int | None] = {int(v): None for v in vehicle_ids}
    child: dict[int, int | None] = {int(v): None for v in vehicle_ids}
    for p, c in edges:
        if p in child:
            child[p] = c
        if c in parent:
            parent[c] = p
    heads = [v for v in vehicle_ids if parent[v] is None]
    chains: list[list[int]] = []
    for h in heads:
        seq = [h]
        cur = child[h]
        seen = {h}
        while cur is not None and cur not in seen:
            seq.append(cur)
            seen.add(cur)
            cur = child[cur]
        chains.append(seq)
    return chains


def _audit_run(cfg, case, out) -> dict[str, Any]:
    mon = out.monitor_trace
    if len(mon) <= 1:
        return {"frame_count": len(mon), "violation_count": 0, "violations": [], "by_type": {}}

    vehicle_ids = sorted(int(v.vehicle_id) for v in case.vehicles_init)
    ack = AckermannModel(cfg.vehicle)
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    train_kin = TrainKinematics(cfg.vehicle)
    dmax = min(math.radians(cfg.vehicle.max_steer_deg), ack.min_turning_steer_bound())
    sr_lim = math.radians(cfg.vehicle.max_steer_rate_deg_s)

    violations: list[dict[str, Any]] = []
    by_type: Counter[str] = Counter()
    per_vehicle: dict[int, Counter[str]] = defaultdict(Counter)

    def add_violation(t: float, frame: int, typ: str, vehicle_id: int | None, detail: dict[str, Any]) -> None:
        by_type[typ] += 1
        if vehicle_id is not None:
            per_vehicle[int(vehicle_id)][typ] += 1
        violations.append(
            {
                "t": float(t),
                "frame": int(frame),
                "type": str(typ),
                "vehicle_id": None if vehicle_id is None else int(vehicle_id),
                "detail": detail,
            }
        )

    for k in range(1, len(mon)):
        p = mon[k - 1]
        c = mon[k]
        t = float(c["t"])
        dt = max(1e-6, float(c["t"]) - float(p["t"]))
        edges_p = _edges_set(p)
        edges_c = _edges_set(c)
        pending_c = _pending_map(c)
        connected_pairs = set(edges_c) | {(b, a) for a, b in edges_c}
        chains_c = _build_chains(edges_c, vehicle_ids)
        chains_p = _build_chains(edges_p, vehicle_ids)

        for vid in vehicle_ids:
            sp = p["states"][str(vid)]
            sc = c["states"][str(vid)]
            st_p = _to_state(vid, sp)
            st_c = _to_state(vid, sc)

            accel = float((st_c.v - st_p.v) / dt)
            sr = float(angle_diff(st_c.delta, st_p.delta) / dt)
            yr = float(angle_diff(st_c.yaw, st_p.yaw) / dt)
            step_pos = float(np.linalg.norm(st_c.xy() - st_p.xy()))
            dx = st_c.x - st_p.x
            dy = st_c.y - st_p.y
            c0 = math.cos(-st_p.yaw)
            s0 = math.sin(-st_p.yaw)
            v_lat = float((s0 * dx + c0 * dy) / dt)

            if st_c.v > cfg.vehicle.max_speed + 1e-6 or st_c.v < -cfg.vehicle.max_reverse_speed - 1e-6:
                add_violation(t, k, "speed_bound", vid, {"v": st_c.v})
            if abs(st_c.delta) > dmax + math.radians(0.5):
                add_violation(t, k, "steer_bound", vid, {"delta_deg": math.degrees(st_c.delta), "limit_deg": math.degrees(dmax)})
            if accel > cfg.vehicle.max_accel + 0.25 or accel < -cfg.vehicle.max_decel - 0.25:
                add_violation(t, k, "accel_bound", vid, {"accel": accel})
            if abs(sr) > sr_lim + math.radians(5.0):
                add_violation(t, k, "steer_rate_bound", vid, {"steer_rate_deg_s": math.degrees(sr), "limit_deg_s": math.degrees(sr_lim)})
            if abs(yr) > cfg.vehicle.max_yaw_rate + 0.2:
                add_violation(t, k, "yaw_rate_bound", vid, {"yaw_rate": yr})
            if step_pos > cfg.vehicle.max_speed * dt + 0.12:
                add_violation(t, k, "position_jump", vid, {"step_pos": step_pos})
            if abs(v_lat) > 0.55:
                add_violation(t, k, "lateral_slip_like", vid, {"v_lat": v_lat})

            mode_p = str(sp.get("mode", "FREE"))
            mode_c = str(sc.get("mode", "FREE"))
            topo_changed = edges_p != edges_c
            if (not topo_changed) and mode_p in {"FREE", "DOCKING", "WAIT", "SPLIT"} and mode_c in {"FREE", "DOCKING", "WAIT", "SPLIT"}:
                cmd = ControlCommand(accel=float(sc.get("cmd_accel", 0.0)), steer_rate=float(sc.get("cmd_steer_rate", 0.0)))
                pred = ack.step(st_p, cmd, dt)
                pos_res = float(np.linalg.norm(pred.xy() - st_c.xy()))
                yaw_res = float(abs(math.degrees(angle_diff(pred.yaw, st_c.yaw))))
                v_res = float(abs(pred.v - st_c.v))
                delta_res = float(abs(math.degrees(angle_diff(pred.delta, st_c.delta))))
                # Allow small mismatch in soft-capture / split windows.
                if pos_res > 0.08 or yaw_res > 6.0 or v_res > 0.25 or delta_res > 8.0:
                    add_violation(
                        t,
                        k,
                        "ack_residual",
                        vid,
                        {
                            "pos_res": pos_res,
                            "yaw_res_deg": yaw_res,
                            "v_res": v_res,
                            "delta_res_deg": delta_res,
                        },
                    )

        # Jackknife / articulation constraints.
        for pa, ch in edges_c:
            s_pa = _to_state(pa, c["states"][str(pa)])
            s_ch = _to_state(ch, c["states"][str(ch)])
            phi_deg = abs(math.degrees(angle_diff(s_pa.yaw, s_ch.yaw)))
            if phi_deg > cfg.safety.jackknife_max_deg + 1.0:
                add_violation(t, k, "jackknife", None, {"parent": pa, "child": ch, "phi_deg": phi_deg})

        # Chain curvature bound and train update consistency.
        set_prev = {tuple(x) for x in chains_p}
        for chain in chains_c:
            if len(chain) < 2:
                continue
            head = int(chain[0])
            sp_h = _to_state(head, p["states"][str(head)])
            sc_h = _to_state(head, c["states"][str(head)])
            yaw_rate_h = float(angle_diff(sc_h.yaw, sp_h.yaw) / dt)
            if abs(sc_h.v) > 0.15 and abs(yaw_rate_h) > 1e-5:
                r_now = abs(sc_h.v / yaw_rate_h)
                r_req = train_kin.min_turn_radius_train(len(chain))
                if r_now + 0.15 < r_req:
                    add_violation(
                        t,
                        k,
                        "train_turn_radius",
                        head,
                        {"radius_now": r_now, "radius_req": r_req, "chain": chain},
                    )

            if tuple(chain) in set_prev:
                prev_states = [_to_state(v, p["states"][str(v)]) for v in chain]
                cmd_h = ControlCommand(
                    accel=float(c["states"][str(head)].get("cmd_accel", 0.0)),
                    steer_rate=float(c["states"][str(head)].get("cmd_steer_rate", 0.0)),
                )
                pred = train_kin.update(prev_states, cmd_h, dt)
                max_pos = 0.0
                max_yaw = 0.0
                for idx, vid in enumerate(chain):
                    s_pred = pred.states[idx]
                    s_now = _to_state(vid, c["states"][str(vid)])
                    max_pos = max(max_pos, float(np.linalg.norm(s_pred.xy() - s_now.xy())))
                    max_yaw = max(max_yaw, float(abs(math.degrees(angle_diff(s_pred.yaw, s_now.yaw)))))
                if max_pos > 0.12 or max_yaw > 10.0:
                    add_violation(
                        t,
                        k,
                        "train_residual",
                        head,
                        {"max_pos": max_pos, "max_yaw_deg": max_yaw, "chain": chain},
                    )

        # Geometry collision audit per frame.
        row_states = {vid: _to_state(vid, c["states"][str(vid)]) for vid in vehicle_ids}
        for vid, st in row_states.items():
            for obs in case.obstacles:
                if collision.collide_vehicle_obstacle(st, obs, include_clearance=False):
                    add_violation(t, k, "collision_obstacle", vid, {})
                    break
        for i, a in enumerate(vehicle_ids):
            for b in vehicle_ids[i + 1 :]:
                if (a, b) in connected_pairs:
                    continue
                legal_pending = False
                if pending_c.get(a, None) == b:
                    d = float(np.linalg.norm(
                        train_kin.geom.front_hitch(row_states[a]) - train_kin.geom.rear_hitch(row_states[b])
                    ))
                    legal_pending = d <= (cfg.docking.soft_capture_distance + 0.1)
                elif pending_c.get(b, None) == a:
                    d = float(np.linalg.norm(
                        train_kin.geom.front_hitch(row_states[b]) - train_kin.geom.rear_hitch(row_states[a])
                    ))
                    legal_pending = d <= (cfg.docking.soft_capture_distance + 0.1)
                if legal_pending:
                    continue
                if collision.collide_vehicle_vehicle(row_states[a], row_states[b], include_clearance=False):
                    add_violation(t, k, "collision_vehicle", None, {"a": a, "b": b})

    return {
        "frame_count": int(len(mon)),
        "violation_count": int(len(violations)),
        "by_type": {k: int(v) for k, v in by_type.items()},
        "by_vehicle": {str(k): {kk: int(vv) for kk, vv in c.items()} for k, c in per_vehicle.items()},
        "violations": violations,
    }


def main() -> None:
    cfg = load_config()
    gen = ScenarioGenerator(cfg)
    exp_dir = ROOT / "experiments"
    art_dir = ROOT / "artifacts"
    exp_dir.mkdir(exist_ok=True)
    art_dir.mkdir(exist_ok=True)

    all_rows: list[dict[str, Any]] = []
    for item in _scenario_plan():
        case = gen.generate(
            item["subtype"],
            n_vehicles=int(item["n_vehicles"]),
            seed=int(item["seed"]),
            initial_dispersion_mode=item["mode"],
        )
        runner = IntegratedP2P4Runner(
            cfg,
            case,
            policy="integrated",
            seed=2000 + int(item["seed"]) % 11,
            demo_cfg=IntegratedDemoConfig(**item["demo_cfg"]),
        )
        out = runner.run()
        audit = _audit_run(cfg, case, out)
        row = {
            "case_name": item["name"],
            "scenario_id": case.scenario_id,
            "result": out.to_dict(),
            "audit": audit,
        }
        all_rows.append(row)

        p_json = exp_dir / f"p2_p4_frame_audit_{item['name']}.json"
        p_json.write_text(json.dumps(row, ensure_ascii=False, indent=2), encoding="utf-8")
        print("frame_audit_case", item["name"], p_json)

    summary = {
        "num_cases": len(all_rows),
        "total_violations": int(sum(r["audit"]["violation_count"] for r in all_rows)),
        "cases": [
            {
                "case_name": r["case_name"],
                "scenario_id": r["scenario_id"],
                "violation_count": int(r["audit"]["violation_count"]),
                "by_type": r["audit"]["by_type"],
                "dock_success_count": int(r["result"]["dock_success_count"]),
                "split_count": int(r["result"]["split_count"]),
                "collision_count": int(r["result"]["collision_count"]),
                "done_time_s": float(r["result"]["done_time_s"]),
            }
            for r in all_rows
        ],
    }
    p_sum_json = exp_dir / "p2_p4_frame_audit_summary.json"
    p_sum_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = [
        "# P2-P4 Framewise Kinematic Audit",
        "",
        f"- num_cases: {summary['num_cases']}",
        f"- total_violations: {summary['total_violations']}",
        "",
    ]
    for c in summary["cases"]:
        lines += [
            f"## Case {c['case_name']} ({c['scenario_id']})",
            f"- violation_count: {c['violation_count']}",
            f"- by_type: {json.dumps(c['by_type'], ensure_ascii=False)}",
            f"- dock_success_count: {c['dock_success_count']}",
            f"- split_count: {c['split_count']}",
            f"- collision_count: {c['collision_count']}",
            f"- done_time_s: {c['done_time_s']:.2f}",
            "",
        ]
    p_sum_md = art_dir / "P2_P4_FRAME_AUDIT.md"
    p_sum_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("frame_audit_summary_json", p_sum_json)
    print("frame_audit_summary_md", p_sum_md)


if __name__ == "__main__":
    main()
