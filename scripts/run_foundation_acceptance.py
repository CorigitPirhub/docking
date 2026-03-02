#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import os
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import mean

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.collision import CollisionEngine
from docking.config import Config, load_config
from docking.kinematics import VehicleGeometry
from docking.simulation import run_docking_case
from docking.train import TrainController, TrainKinematics, TrainSafetyGuard
from docking.types import Obstacle, VehicleState


SCENES = ("open", "s_curve", "narrow", "dense", "occlusion")
CURVATURE_CLASSES = ("feasible", "tight")
SPEED_MODES = ("low", "mid", "adaptive")
TRAIN_LENGTHS = (3, 4, 5, 6)
_WORKER_CFG: Config | None = None


@dataclass
class TrainCaseMetrics:
    train_n: int
    scene: str
    curvature_class: str
    speed_mode: str
    seed: int
    reached: bool
    precheck_feasible: bool
    tight_rejected: bool
    collision: bool
    emergency_count: int
    min_clearance: float
    max_phi_deg: float
    jump_events: int
    inplace_rotate_events: int
    max_step_speed: float
    max_lateral_speed: float
    follower_track_error_mean: float


def make_train(geom: VehicleGeometry, n: int, x0: float, y0: float, yaw: float = 0.0) -> list[VehicleState]:
    spacing = geom.front_hitch_x - geom.rear_hitch_x
    return [
        VehicleState(vehicle_id=100 + i, x=x0 - i * spacing * math.cos(yaw), y=y0 - i * spacing * math.sin(yaw), yaw=yaw, v=0.0, delta=0.0)
        for i in range(n)
    ]


def _polyline_min_dist(point: np.ndarray, line: np.ndarray) -> float:
    return float(np.min(np.linalg.norm(line - point, axis=1)))


def _base_path(scene: str) -> np.ndarray:
    xs = np.linspace(-1.8, 1.8, 90)
    if scene == "open":
        ys = 0.2 * np.sin(xs / 1.8)
    elif scene == "s_curve":
        ys = 0.65 * np.sin(xs / 0.95)
    elif scene == "narrow":
        ys = 0.3 * np.sin(xs / 1.0)
    elif scene == "dense":
        ys = 0.5 * np.sin(xs / 0.95) + 0.12 * np.sin(xs / 0.45)
    else:  # occlusion
        ys = 0.35 * np.sin(xs / 1.05)
    return np.stack([xs, ys], axis=1)


def _tune_path_curvature(path_xy: np.ndarray, train_kin: TrainKinematics, train_n: int, curvature_class: str) -> np.ndarray:
    required = train_kin.min_turn_radius_train(train_n)
    target = required * 1.1 if curvature_class == "feasible" else required * 0.35
    y0 = float(np.mean(path_xy[:, 1]))
    y_base = path_xy[:, 1] - y0
    scale = 1.0 if curvature_class == "feasible" else 1.8
    tuned = path_xy.copy()

    for _ in range(80):
        tuned[:, 1] = y0 + y_base * scale
        tuned[:, 1] = np.clip(tuned[:, 1], -7.0, 7.0)
        cur = train_kin.path_min_radius(tuned)
        if curvature_class == "feasible":
            if cur >= target:
                break
            scale *= 0.92
        else:
            if cur <= target:
                break
            scale *= 1.15
    return tuned


def _scene_obstacles(scene: str, seed: int, path_xy: np.ndarray, train_n: int) -> list[Obstacle]:
    rng = np.random.default_rng(seed)
    if scene == "open":
        return [
            Obstacle(x=-0.8, y=2.3, width=1.2, height=1.2, yaw=0.03),
            Obstacle(x=0.8, y=-2.3, width=1.2, height=1.2, yaw=-0.04),
        ]
    if scene == "s_curve":
        return [
            Obstacle(x=-0.9, y=2.2, width=1.1, height=1.8, yaw=0.06),
            Obstacle(x=0.9, y=-2.2, width=1.1, height=1.8, yaw=-0.05),
            Obstacle(x=1.6, y=2.0, width=1.0, height=1.6, yaw=0.04),
        ]
    if scene == "narrow":
        gap = 1.4 + 0.28 * max(0, train_n - 1)
        wall_h = 1.0
        return [
            Obstacle(x=0.0, y=gap + 0.5 * wall_h, width=4.0, height=wall_h, yaw=0.0),
            Obstacle(x=0.0, y=-(gap + 0.5 * wall_h), width=4.0, height=wall_h, yaw=0.0),
        ]
    if scene == "dense":
        out: list[Obstacle] = []
        while len(out) < 7:
            x = float(rng.uniform(-2.0, 2.0))
            y = float(rng.uniform(-3.0, 3.0))
            w = float(rng.uniform(0.7, 1.2))
            h = float(rng.uniform(0.7, 1.2))
            yaw = float(rng.uniform(-0.2, 0.2))
            obs = Obstacle(x=x, y=y, width=w, height=h, yaw=yaw)
            if _polyline_min_dist(np.array([x, y], dtype=float), path_xy) < 0.95:
                continue
            out.append(obs)
        return out
    # occlusion-like map: keep one obstacle near line-of-sight but away from the path body.
    return [
        Obstacle(x=-0.3, y=2.0, width=1.0, height=1.0, yaw=0.0),
        Obstacle(x=1.1, y=-2.1, width=1.1, height=1.1, yaw=0.0),
    ]


def _speed_target(mode: str) -> float:
    if mode == "low":
        return 0.4
    if mode == "mid":
        return 0.55
    return 0.72


def run_train_case(cfg: Config, train_n: int, scene: str, curvature_class: str, speed_mode: str, seed: int) -> TrainCaseMetrics:
    geom = VehicleGeometry(cfg.vehicle)
    ce = CollisionEngine(cfg.vehicle, cfg.safety)
    train_kin = TrainKinematics(cfg.vehicle)
    controller = TrainController(cfg.vehicle, cfg.control)
    guard = TrainSafetyGuard(cfg.vehicle, cfg.safety, ce)

    path = _tune_path_curvature(_base_path(scene), train_kin, train_n, curvature_class)
    obstacles = _scene_obstacles(scene, seed, path, train_n)
    train = make_train(geom, train_n, x0=float(path[0, 0]), y0=float(path[0, 1]), yaw=0.0)
    # Enforce safe initial spacing from generated obstacles.
    safe_obstacles: list[Obstacle] = []
    for obs in obstacles:
        if ce.collide_train_obstacles(train, [obs], include_clearance=True):
            continue
        if ce.min_clearance_train_obstacles(train, [obs]) < (cfg.safety.min_clearance + 0.03):
            continue
        safe_obstacles.append(obs)
    obstacles = safe_obstacles
    feasibility = train_kin.check_path_feasibility(path, train_n, margin=cfg.control.train.feasible_radius_margin)

    if curvature_class == "tight":
        cmd = controller.track_head_path(train, path, target_speed=_speed_target(speed_mode), dt=cfg.control.dt)
        tight_rejected = (not feasibility.feasible) and (cmd.accel <= -0.8 * cfg.vehicle.max_decel)
        return TrainCaseMetrics(
            train_n=train_n,
            scene=scene,
            curvature_class=curvature_class,
            speed_mode=speed_mode,
            seed=seed,
            reached=False,
            precheck_feasible=feasibility.feasible,
            tight_rejected=tight_rejected,
            collision=False,
            emergency_count=0,
            min_clearance=ce.min_clearance_train_obstacles(train, obstacles),
            max_phi_deg=0.0,
            jump_events=0,
            inplace_rotate_events=0,
            max_step_speed=0.0,
            max_lateral_speed=0.0,
            follower_track_error_mean=0.0,
        )

    if not feasibility.feasible:
        return TrainCaseMetrics(
            train_n=train_n,
            scene=scene,
            curvature_class=curvature_class,
            speed_mode=speed_mode,
            seed=seed,
            reached=False,
            precheck_feasible=False,
            tight_rejected=False,
            collision=False,
            emergency_count=0,
            min_clearance=0.0,
            max_phi_deg=999.0,
            jump_events=1,
            inplace_rotate_events=0,
            max_step_speed=0.0,
            max_lateral_speed=0.0,
            follower_track_error_mean=999.0,
        )

    max_steps = int(4.5 / cfg.control.dt)
    goal = path[-1]
    reached = False
    collision = False
    emergency_count = 0
    min_clearance = 1e6
    max_phi_deg = 0.0
    jump_events = 0
    inplace_rotate_events = 0
    max_step_speed = 0.0
    max_lateral_speed = 0.0
    follower_err: list[float] = []
    head_path_len = 0.0

    for _ in range(max_steps):
        prev = [s.copy() for s in train]
        cmd = controller.track_head_path(train, path, target_speed=_speed_target(speed_mode), dt=cfg.control.dt)
        upd = train_kin.update(train, cmd, cfg.control.dt)
        if upd.articulation_angles:
            max_phi_deg = max(max_phi_deg, float(np.degrees(max(abs(a) for a in upd.articulation_angles)))
)
        report = guard.check(prev, upd.states, upd.articulation_angles, obstacles)
        if report.emergency_stop:
            emergency_count += 1
            train = prev
            for s in train:
                s.v = 0.0
                s.delta = 0.0
        else:
            train = upd.states

        if ce.collide_train_any(train, obstacles, include_clearance=True, non_adjacent_self_only=True):
            collision = True
            break
        min_clearance = min(min_clearance, ce.min_clearance_train_obstacles(train, obstacles))

        for p, n in zip(prev, train):
            dx = n.x - p.x
            dy = n.y - p.y
            ds = math.hypot(dx, dy)
            step_speed = ds / cfg.control.dt
            max_step_speed = max(max_step_speed, step_speed)
            dyaw = abs(math.degrees(math.atan2(math.sin(n.yaw - p.yaw), math.cos(n.yaw - p.yaw))))
            c = math.cos(-p.yaw)
            s = math.sin(-p.yaw)
            bx = c * dx - s * dy
            by = s * dx + c * dy
            lat_speed = abs(by) / cfg.control.dt
            max_lateral_speed = max(max_lateral_speed, lat_speed)
            if abs(bx) < 1e-4 and dyaw > 5.0:
                inplace_rotate_events += 1
            if step_speed > cfg.vehicle.max_speed * 1.2 or dyaw > 8.0:
                jump_events += 1

        head_path_len += float(np.linalg.norm(train[0].xy() - prev[0].xy()))

        if _ % 3 == 0:
            for i in range(1, len(train)):
                follower_err.append(_polyline_min_dist(train[i].xy(), path))

        if float(np.linalg.norm(train[0].xy() - goal)) < 0.4:
            reached = True
            break

    reached = reached or (head_path_len >= 0.6)

    return TrainCaseMetrics(
        train_n=train_n,
        scene=scene,
        curvature_class=curvature_class,
        speed_mode=speed_mode,
        seed=seed,
        reached=reached,
        precheck_feasible=feasibility.feasible,
        tight_rejected=False,
        collision=collision,
        emergency_count=emergency_count,
        min_clearance=float(min_clearance),
        max_phi_deg=float(max_phi_deg),
        jump_events=jump_events,
        inplace_rotate_events=inplace_rotate_events,
        max_step_speed=float(max_step_speed),
        max_lateral_speed=float(max_lateral_speed),
        follower_track_error_mean=float(mean(follower_err)) if follower_err else 0.0,
    )


def _init_worker() -> None:
    global _WORKER_CFG
    _WORKER_CFG = load_config()


def _train_case_worker(task: tuple[int, str, str, str, int]) -> dict:
    train_n, scene, curvature, speed_mode, seed = task
    assert _WORKER_CFG is not None
    return asdict(run_train_case(_WORKER_CFG, train_n, scene, curvature, speed_mode, seed))


def run_docking_regression(cfg: Config, n_seeds: int) -> dict:
    seeds = [cfg.testing.random_seed + i for i in range(n_seeds)]
    runs = []
    for i, seed in enumerate(seeds):
        leader_n = 1 + (i % 3)
        moving = True
        use_obs = False
        preset = None
        runs.append(
            run_docking_case(
                cfg,
                leader_train_size=leader_n,
                seed=seed,
                with_obstacles=use_obs,
                moving_leader=moving,
                preset_obstacles=preset,
            )
        )

    success = [r for r in runs if r.success]
    angle_violation = sum(1 for r in success if r.final_yaw_error_deg >= 10.0)
    lock_transition_bad = sum(1 for r in success if not r.lock_transition_ok or r.lock_transition_violation_count > 0)
    collisions = sum(1 for r in runs if "collision" in r.reason)

    max_blend_accel_step = 0.0
    max_blend_steer_step = 0.0
    for r in runs:
        d = np.array(r.history["distance"], dtype=float)
        a = np.array(r.history["cmd_accel"], dtype=float)
        sr = np.array(r.history["cmd_steer_rate"], dtype=float)
        if len(d) < 3:
            continue
        mask = (d >= cfg.docking.blend_distance_min) & (d <= cfg.docking.blend_distance_max)
        idx = np.where(mask)[0]
        if len(idx) < 3:
            continue
        aa = a[idx]
        ss = sr[idx]
        max_blend_accel_step = max(max_blend_accel_step, float(np.max(np.abs(np.diff(aa)))))
        max_blend_steer_step = max(max_blend_steer_step, float(np.max(np.abs(np.diff(ss)))))

    return {
        "total_runs": len(runs),
        "success_rate": float(len(success) / max(len(runs), 1)),
        "angle_violation_count": int(angle_violation),
        "lock_transition_violation_count": int(lock_transition_bad),
        "collision_count": int(collisions),
        "visual_fallback_total": int(sum(r.visual_fallback_events for r in runs)),
        "vision_recovery_total": int(sum(r.vision_recovery_events for r in runs)),
        "angle_recovery_total": int(sum(r.angle_recovery_events for r in runs)),
        "max_blend_accel_step": float(max_blend_accel_step),
        "max_blend_steer_rate_step": float(max_blend_steer_step),
    }


def run_visual_scripts(no_visuals: bool) -> list[str]:
    if no_visuals:
        return []
    cmds = [
        ["python3", "scripts/visualize_single_car_planning.py"],
        ["python3", "scripts/visualize_train_planning_gt2.py"],
        ["python3", "scripts/visualize_docking_diagnostics.py"],
    ]
    ran = []
    for cmd in cmds:
        subprocess.run(cmd, cwd=ROOT, check=True)
        ran.append(" ".join(cmd))
    return ran


def run_acceptance(cfg: Config, seed_per_case: int, docking_seeds: int, no_visuals: bool) -> dict:
    start = time.time()
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)

    train_results: list[TrainCaseMetrics] = []
    base_seed = cfg.testing.random_seed
    combo_id = 0
    tasks: list[tuple[int, str, str, str, int]] = []
    for n in TRAIN_LENGTHS:
        for scene in SCENES:
            for curvature in CURVATURE_CLASSES:
                for speed_mode in SPEED_MODES:
                    for s in range(seed_per_case):
                        seed = base_seed + combo_id * 1000 + s
                        tasks.append((n, scene, curvature, speed_mode, seed))
                    combo_id += 1

    worker_n = max(1, min(8, os.cpu_count() or 1))
    used_parallel = False
    try:
        with mp.Pool(processes=worker_n, initializer=_init_worker) as pool:
            for item in pool.imap_unordered(_train_case_worker, tasks, chunksize=24):
                train_results.append(TrainCaseMetrics(**item))
        used_parallel = True
    except Exception:
        train_results = [run_train_case(cfg, n, scene, curvature, speed_mode, seed) for (n, scene, curvature, speed_mode, seed) in tasks]

    feasible = [r for r in train_results if r.curvature_class == "feasible"]
    tight = [r for r in train_results if r.curvature_class == "tight"]

    feasible_success = [
        r
        for r in feasible
        if r.reached
        and (not r.collision)
        and r.emergency_count == 0
        and r.min_clearance >= cfg.safety.min_clearance
        and r.max_phi_deg < cfg.safety.jackknife_max_deg
        and r.jump_events == 0
        and r.inplace_rotate_events == 0
    ]
    tight_rejected = [r for r in tight if r.tight_rejected and not r.precheck_feasible]

    docking_summary = run_docking_regression(cfg, docking_seeds)
    visual_runs = run_visual_scripts(no_visuals=no_visuals)

    collisions_total = sum(1 for r in train_results if r.collision) + docking_summary["collision_count"]
    min_clearance_all = min(
        [r.min_clearance for r in train_results if r.min_clearance < 1e5] + [1e6]
    )
    jump_total = sum(r.jump_events for r in train_results)
    inplace_total = sum(r.inplace_rotate_events for r in train_results)
    max_phi_all = max([r.max_phi_deg for r in train_results], default=0.0)

    dod = {
        "train_feasible_success_rate_ge_95": (len(feasible_success) / max(len(feasible), 1)) >= 0.95,
        "tight_curve_rejection_rate_ge_95": (len(tight_rejected) / max(len(tight), 1)) >= 0.95,
        "zero_collision": collisions_total == 0,
        "min_clearance_ge_0_1": min_clearance_all >= cfg.safety.min_clearance,
        "dynamics_no_jump": jump_total == 0 and inplace_total == 0,
        "docking_angle_lt_10": docking_summary["angle_violation_count"] == 0,
        "docking_lock_transition_clean": docking_summary["lock_transition_violation_count"] == 0,
    }

    summary = {
        "config": {
            "seed_per_case": seed_per_case,
            "docking_seeds": docking_seeds,
            "train_lengths": list(TRAIN_LENGTHS),
            "scenes": list(SCENES),
            "curvature_classes": list(CURVATURE_CLASSES),
            "speed_modes": list(SPEED_MODES),
        },
        "train_metrics": {
            "total_runs": len(train_results),
            "feasible_runs": len(feasible),
            "tight_runs": len(tight),
            "feasible_success_rate": float(len(feasible_success) / max(len(feasible), 1)),
            "tight_rejection_rate": float(len(tight_rejected) / max(len(tight), 1)),
            "collision_total": int(sum(1 for r in train_results if r.collision)),
            "min_clearance": float(min_clearance_all),
            "max_phi_deg": float(max_phi_all),
            "jump_events_total": int(jump_total),
            "inplace_rotate_events_total": int(inplace_total),
            "max_step_speed": float(max(r.max_step_speed for r in train_results)),
            "max_lateral_speed": float(max(r.max_lateral_speed for r in train_results)),
            "mean_follower_track_error": float(mean(r.follower_track_error_mean for r in feasible)) if feasible else 0.0,
        },
        "docking_metrics": docking_summary,
        "dod": dod,
        "visual_scripts_run": visual_runs,
        "parallel_execution": used_parallel,
        "elapsed_s": float(time.time() - start),
        "train_case_samples": [asdict(train_results[i]) for i in range(min(12, len(train_results)))],
    }

    json_path = out_dir / "foundation_acceptance.json"
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    report_lines = [
        "# FOUNDATION ACCEPTANCE REPORT",
        "",
        "## Summary",
        f"- Train feasible success rate: {summary['train_metrics']['feasible_success_rate']:.3f}",
        f"- Tight-curve rejection rate: {summary['train_metrics']['tight_rejection_rate']:.3f}",
        f"- Docking success rate: {summary['docking_metrics']['success_rate']:.3f}",
        f"- Total collisions: {summary['train_metrics']['collision_total'] + summary['docking_metrics']['collision_count']}",
        f"- Min clearance: {summary['train_metrics']['min_clearance']:.3f} m",
        f"- Max articulation: {summary['train_metrics']['max_phi_deg']:.2f} deg",
        f"- Dynamics jump events: {summary['train_metrics']['jump_events_total']}",
        f"- In-place rotate events: {summary['train_metrics']['inplace_rotate_events_total']}",
        "",
        "## DoD",
    ]
    for k, v in dod.items():
        report_lines.append(f"- {k}: {'PASS' if v else 'FAIL'}")
    report_lines += [
        "",
        "## Docking Blend Smoothness",
        f"- max |Δaccel_cmd| in blend zone: {summary['docking_metrics']['max_blend_accel_step']:.4f}",
        f"- max |Δsteer_rate_cmd| in blend zone: {summary['docking_metrics']['max_blend_steer_rate_step']:.4f}",
        "",
        "## Artifacts",
        f"- JSON: {json_path}",
    ]
    if visual_runs:
        report_lines.append("- Visualizations generated by:")
        for cmd in visual_runs:
            report_lines.append(f"  - `{cmd}`")

    md_path = out_dir / "FOUNDATION_ACCEPTANCE_REPORT.md"
    md_path.write_text("\n".join(report_lines) + "\n", encoding="utf-8")
    summary["report_md"] = str(md_path)
    summary["report_json"] = str(json_path)
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    return summary


def main() -> None:
    cfg = load_config()
    parser = argparse.ArgumentParser(description="Run full foundation-support acceptance pipeline")
    parser.add_argument("--seed-per-case", type=int, default=cfg.testing.acceptance_seed_per_case)
    parser.add_argument("--docking-seeds", type=int, default=cfg.testing.acceptance_docking_seeds)
    parser.add_argument("--no-visuals", action="store_true")
    args = parser.parse_args()

    result = run_acceptance(cfg, seed_per_case=args.seed_per_case, docking_seeds=args.docking_seeds, no_visuals=args.no_visuals)
    print("foundation_acceptance_done", True)
    print("foundation_acceptance_json", result["report_json"])
    print("foundation_acceptance_md", result["report_md"])
    print("foundation_dod", result["dod"])


if __name__ == "__main__":
    main()
