#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from docking.collision import CollisionEngine
from docking.coop_docking import CooperativeStagingPlanner
from docking.config import Config, load_config
from docking.docking import DockingLockEvaluator
from docking.environment import Environment, random_obstacles
from docking.kinematics import AckermannModel, VehicleGeometry
from docking.math_utils import angle_diff, clamp, wrap_angle
from docking.p_minus1_baselines import ABLATION_METHOD_IDS, FULL_METHOD_IDS, METHOD_SPECS, build_method_runners
from docking.sensors import line_blocked_by_obstacles
from docking.types import ControlCommand, Obstacle, VehicleMode, VehicleState
from docking.visualization import save_docking_animation


@dataclass(frozen=True)
class Stage1Scenario:
    seed: int
    style: str
    planner_seed: int
    obstacles: list[Obstacle]
    leader: VehicleState
    follower: VehicleState
    leader_relocation_m: float
    follower_detour_ratio: float
    direct_los_blocked: bool
    large_obstacle_present: bool
    subset_tag: str
    lane: dict[str, Any]

    @staticmethod
    def from_dict(data: dict[str, Any]) -> "Stage1Scenario":
        payload = data.get("scenario", data)
        return Stage1Scenario(
            seed=int(payload["seed"]),
            style=str(payload["style"]),
            planner_seed=int(payload.get("planner_seed", payload["seed"])),
            obstacles=[Obstacle(**item) for item in payload["obstacles"]],
            leader=VehicleState(**payload["leader"]),
            follower=VehicleState(**payload["follower"]),
            leader_relocation_m=float(payload.get("leader_relocation_m", 0.0)),
            follower_detour_ratio=float(payload.get("follower_detour_ratio", 1.0)),
            direct_los_blocked=bool(payload.get("direct_los_blocked", False)),
            large_obstacle_present=bool(payload.get("large_obstacle_present", False)),
            subset_tag=str(payload.get("subset_tag", "Unlabeled")),
            lane=dict(data.get("lane", payload.get("lane", {})) or {}),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "seed": int(self.seed),
            "style": str(self.style),
            "planner_seed": int(self.planner_seed),
            "obstacles": [asdict(o) for o in self.obstacles],
            "leader": asdict(self.leader),
            "follower": asdict(self.follower),
            "leader_relocation_m": float(self.leader_relocation_m),
            "follower_detour_ratio": float(self.follower_detour_ratio),
            "direct_los_blocked": bool(self.direct_los_blocked),
            "large_obstacle_present": bool(self.large_obstacle_present),
            "subset_tag": str(self.subset_tag),
            "lane": dict(self.lane),
        }


@dataclass(frozen=True)
class Stage1Metrics:
    method_id: str
    success: bool
    reason: str
    done_time_s: float
    collision: bool
    min_clearance_m: float
    final_pos_error_m: float
    final_yaw_error_deg: float
    final_speed_error_mps: float
    trajectory_cost: float
    path_length_m: float
    failure_category: str
    fallback_count: int
    visual_loss_count: int
    replan_count: int
    leader_relocation_time_s: float
    leader_relocation_distance_m: float
    follower_detour_ratio: float
    accel_smoothness_max: float
    steer_rate_smoothness_max: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="P-1 Stage-1: 2-car docking in one random obstacle scene (with baselines).")
    p.add_argument("--seed", type=int, default=1234, help="Scenario seed (scene + initial poses).")
    p.add_argument("--obstacles", type=int, default=10, help="Number of random obstacles.")
    p.add_argument(
        "--scenario-style",
        type=str,
        choices=["typical", "random", "common_feasible", "switching_critical", "funnel_critical", "extension_critical"],
        default="typical",
        help="Scene family: `common_feasible` / `switching_critical` / `funnel_critical` / `extension_critical` (alias `typical`) / `random`.",
    )
    p.add_argument("--max-time-s", type=float, default=60.0, help="Simulation max time.")
    p.add_argument("--scenario-json", type=str, default="", help="Optional scenario JSON file. If provided, skip random generation and load this exact scene.")
    p.add_argument("--out-dir", type=str, default=str(ROOT / "artifacts"), help="Output directory for GIF/JSON/MD.")
    p.add_argument("--save-scenario-json", action="store_true", help="Save scenario JSON under experiments/.")
    p.add_argument("--skip-gifs", action="store_true", help="Skip generating GIF replays (faster iteration).")
    p.add_argument(
        "--methods",
        type=str,
        default="",
        help="Comma-separated method ids to run (default: run all). Example: co_bcfd,T_hard_switch",
    )
    p.add_argument("--gif-fps", type=int, default=25, help="GIF FPS.")
    p.add_argument("--gif-max-frames", type=int, default=420, help="Max frames per GIF (downsample if needed).")
    return p.parse_args()


def _in_bounds(cfg: Config, state: VehicleState, margin: float = 0.6) -> bool:
    half_w = 0.5 * float(cfg.environment.width) - float(margin)
    half_h = 0.5 * float(cfg.environment.height) - float(margin)
    return (-half_w <= float(state.x) <= half_w) and (-half_h <= float(state.y) <= half_h)


def _path_length(path_xy: np.ndarray | None) -> float:
    if path_xy is None or len(path_xy) < 2:
        return math.inf
    return float(np.sum(np.linalg.norm(np.diff(path_xy, axis=0), axis=1)))


def _spawn_random_scene(cfg: Config, seed: int, obstacle_count: int, style: str = "typical") -> Stage1Scenario:
    rng = np.random.default_rng(int(seed))
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    geom = VehicleGeometry(cfg.vehicle)
    style_norm = str(style).strip().lower()
    aliases = {
        "typical": "extension_critical",
        "extension": "extension_critical",
        "common": "common_feasible",
        "switching": "switching_critical",
    }
    style_norm = aliases.get(style_norm, style_norm)
    if style_norm not in {"extension_critical", "random", "common_feasible", "switching_critical", "funnel_critical"}:
        raise ValueError(f"Unsupported scenario style: {style}")
    is_extension = style_norm == "extension_critical"
    is_common = style_norm == "common_feasible"
    is_switching = style_norm == "switching_critical"
    is_funnel = style_norm == "funnel_critical"

    # We generate obstacles first, then sample leader/follower with rejection.
    for attempt in range(700 if is_extension else 320):
        if is_extension:
            min_size, max_size, margin = 1.35, 4.2, 3.1
        elif is_common:
            min_size, max_size, margin = 0.55, 1.35, 2.9
        elif is_switching:
            min_size, max_size, margin = 0.75, 2.20, 2.9
        elif is_funnel:
            min_size, max_size, margin = 0.65, 1.45, 2.8
        else:
            min_size, max_size, margin = 0.6, 1.9, 2.8
        obstacles = random_obstacles(
            cfg.environment,
            rng,
            int(obstacle_count),
            min_size=min_size,
            max_size=max_size,
            margin=margin,
        )

        # Leader: random pose but keep yaw moderate to avoid degenerate "side docking" in Stage-1.
        leader = VehicleState(
            vehicle_id=2,
            x=float(rng.uniform(-12.0, 12.0)),
            y=float(rng.uniform(-5.0, 5.0)),
            yaw=float(rng.uniform(-math.radians(35.0), math.radians(35.0))),
            v=0.0,
            delta=0.0,
            mode=VehicleMode.FREE,
        )

        # Follower: sample in a sector behind the leader, keep a minimum separation.
        r = float(rng.uniform(6.0, 12.0))
        th = float(rng.uniform(math.radians(155.0), math.radians(205.0)))  # behind leader
        # Rotate relative displacement by leader yaw.
        dx = r * math.cos(th + float(leader.yaw))
        dy = r * math.sin(th + float(leader.yaw))
        follower = VehicleState(
            vehicle_id=1,
            x=float(leader.x + dx),
            y=float(leader.y + dy),
            yaw=float(leader.yaw + rng.uniform(-math.radians(25.0), math.radians(25.0))),
            v=0.0,
            delta=0.0,
            mode=VehicleMode.DOCKING,
        )

        if not (_in_bounds(cfg, leader) and _in_bounds(cfg, follower)):
            continue

        # Ensure spawn clearance against obstacles.
        if collision.in_collision(leader, obstacles, []):
            continue
        if collision.in_collision(follower, obstacles, []):
            continue

        if float(np.linalg.norm(leader.xy() - follower.xy())) < 4.5:
            continue

        # Also keep initial hitch points reasonably away from obstacles.
        if collision.min_clearance_vehicle_obstacles(leader, obstacles) < 0.35:
            continue
        if collision.min_clearance_vehicle_obstacles(follower, obstacles) < 0.35:
            continue

        rear = geom.rear_hitch(leader)
        cam = geom.front_hitch(follower)
        blocked = bool(line_blocked_by_obstacles(cam, rear, obstacles))
        large_obs = bool(any(max(float(o.width), float(o.height)) >= 2.8 for o in obstacles))

        planner_seed = int(seed + 31 + 97 * attempt)
        staging_plan = CooperativeStagingPlanner(cfg, seed=planner_seed).plan(
            obstacles=obstacles,
            leader=leader,
            follower=follower,
            prefer_static_leader=False,
            semantic_hints={},
        )
        leader_reloc = float(np.linalg.norm(staging_plan.leader_goal.xy() - leader.xy()))
        follower_path_len = _path_length(staging_plan.follower_path_xy)
        follower_straight = float(np.linalg.norm(staging_plan.follower_goal.xy() - follower.xy()))
        detour_ratio = float(follower_path_len / max(follower_straight, 1e-6))

        if is_extension:
            if not large_obs:
                continue
            if not blocked:
                continue
            if leader_reloc < 0.85:
                continue
            if detour_ratio < 1.15:
                continue
            subset_tag = "Extension-Critical"
        elif is_common:
            if blocked:
                continue
            if leader_reloc > 0.30:
                continue
            if detour_ratio > 1.08:
                continue
            if large_obs and attempt < 80:
                continue
            subset_tag = "Common-Feasible"
        elif is_switching:
            if not blocked:
                continue
            if leader_reloc > 0.42:
                continue
            if detour_ratio > 1.12:
                continue
            subset_tag = "Occlusion-Switching-Critical"
        elif is_funnel:
            if blocked:
                continue
            if leader_reloc > 0.55:
                continue
            if detour_ratio > 1.24:
                continue
            subset_tag = "Funnel-Critical"
        else:
            if blocked and attempt < 140:
                continue
            subset_tag = "Random-Mixed"

        return Stage1Scenario(
            seed=int(seed),
            style=style_norm,
            planner_seed=planner_seed,
            obstacles=obstacles,
            leader=leader,
            follower=follower,
            leader_relocation_m=leader_reloc,
            follower_detour_ratio=detour_ratio,
            direct_los_blocked=blocked,
            large_obstacle_present=large_obs,
            subset_tag=subset_tag,
        )

    raise RuntimeError("Failed to generate a valid Stage-1 scene under current constraints (try another seed/style).")


def _trajectory_cost_step(
    *,
    cfg: Config,
    cmd: ControlCommand,
    cmd_prev: ControlCommand | None,
    clearance_m: float,
    dt: float,
) -> float:
    # A method-agnostic proxy: control effort + command variation + clearance penalty.
    w_u = 0.04
    w_du = 0.10
    w_clr = 3.0
    d_safe = float(cfg.safety.min_clearance)
    a2 = float(cmd.accel * cmd.accel + cmd.steer_rate * cmd.steer_rate)
    du2 = 0.0 if cmd_prev is None else float((cmd.accel - cmd_prev.accel) ** 2 + (cmd.steer_rate - cmd_prev.steer_rate) ** 2)
    clr_def = max(0.0, float(d_safe - clearance_m))
    return float((w_u * a2 + w_du * du2 + w_clr * (clr_def * clr_def)) * float(dt))


def _project_follower_locked(
    geom: VehicleGeometry,
    follower: VehicleState,
    leader: VehicleState,
    *,
    collision: CollisionEngine | None = None,
    obstacles: list[Obstacle] | None = None,
    clearance_floor: float | None = None,
) -> VehicleState:
    anchor = geom.rear_hitch(leader)
    yaw = float(leader.yaw)
    center = anchor - np.array([math.cos(yaw), math.sin(yaw)], dtype=float) * float(geom.front_hitch_x)
    projected = VehicleState(
        vehicle_id=int(follower.vehicle_id),
        x=float(center[0]),
        y=float(center[1]),
        yaw=float(yaw),
        v=float(leader.v),
        delta=float(leader.delta),
        mode=follower.mode,
    )
    if collision is None or obstacles is None or clearance_floor is None:
        return projected
    current_clearance_m = float(collision.min_clearance_vehicle_obstacles(follower, obstacles))
    projected_clearance_m = float(collision.min_clearance_vehicle_obstacles(projected, obstacles))
    if projected_clearance_m + 1e-6 < float(clearance_floor) <= current_clearance_m + 1e-6:
        follower_locked = follower.copy()
        follower_locked.v = float(leader.v)
        follower_locked.delta = float(leader.delta)
        return follower_locked
    return projected


def _project_terminal_hold_pair(
    geom: VehicleGeometry,
    follower: VehicleState,
    leader: VehicleState,
    *,
    collision: CollisionEngine | None = None,
    obstacles: list[Obstacle] | None = None,
    clearance_floor: float | None = None,
) -> tuple[VehicleState, VehicleState]:
    follower_hold = _project_follower_locked(
        geom,
        follower,
        leader,
        collision=collision,
        obstacles=obstacles,
        clearance_floor=clearance_floor,
    )
    leader_hold = leader.copy()
    follower_hold.v = 0.0
    leader_hold.v = 0.0
    follower_hold.delta = 0.0
    leader_hold.delta = 0.0
    return follower_hold, leader_hold


def _count_rising_edges(series: list[float], threshold: float = 0.5) -> int:
    count = 0
    prev = False
    for value in series:
        cur = bool(float(value) > threshold)
        if cur and (not prev):
            count += 1
        prev = cur
    return int(count)


def _extract_sigma_m(stage_name: str) -> float | None:
    match = re.search(r"sigma_m=([0-9.]+)", str(stage_name))
    if match is None:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


def _failure_category(reason: str, history: dict[str, list[float | str]]) -> str:
    rs = str(reason)
    if rs.startswith("collision") or rs == "out_of_bounds":
        return "collision"
    if rs == "locked":
        return "success"
    if rs != "timeout":
        return rs

    visual_loss = max((float(v) for v in history.get("visual_lost_time", [])), default=0.0)
    fallback_count = _count_rising_edges([float(v) for v in history.get("fallback_flag", [])])
    min_dist = min((float(v) for v in history.get("distance", [])), default=float("inf"))
    stage_names = [str(v) for v in history.get("stage_name", [])]
    had_funnel = any(("FUNNEL" in s) or ("LOCK_ASSIST" in s) or ("DIRECT_TRACK" in s) for s in stage_names)
    had_predock = any(("PREDOCK" in s) or ("APPROACH_PATH" in s) for s in stage_names)
    if visual_loss >= 0.75 and fallback_count >= 1:
        return "fov_loss_unrecovered"
    if had_funnel and min_dist <= 0.45:
        return "lock_condition_not_met"
    if had_predock:
        return "geometric_deadlock"
    return "timeout"


def _boundary_guard_command(state: VehicleState, cmd: ControlCommand, cfg: Config, margin: float = 1.4) -> ControlCommand:
    half_w = 0.5 * float(cfg.environment.width) - float(margin)
    half_h = 0.5 * float(cfg.environment.height) - float(margin)
    hx = math.cos(float(state.yaw))
    hy = math.sin(float(state.yaw))
    heading_out = (
        (float(state.x) > half_w and hx > 0.0)
        or (float(state.x) < -half_w and hx < 0.0)
        or (float(state.y) > half_h and hy > 0.0)
        or (float(state.y) < -half_h and hy < 0.0)
    )
    if not heading_out:
        return cmd
    return ControlCommand(accel=-float(cfg.vehicle.max_decel), steer_rate=0.0)


def _run_one(
    *,
    cfg: Config,
    scenario: Stage1Scenario,
    method_id: str,
    controller,
    max_time_s: float,
) -> tuple[Stage1Metrics, dict[str, Any]]:
    dt = float(cfg.control.dt)
    geom = VehicleGeometry(cfg.vehicle)
    model = AckermannModel(cfg.vehicle)
    collision = CollisionEngine(cfg.vehicle, cfg.safety)
    lock_eval = DockingLockEvaluator(cfg.docking, geom)

    leader = scenario.leader.copy()
    follower = scenario.follower.copy()

    history = {
        "t": [],
        "distance": [],
        "w_vis": [],
        "visual_valid": [],
        "fusion_gamma": [],
        "nis": [],
        "pos_error": [],
        "yaw_error_deg": [],
        "speed_error": [],
        "stage_id": [],
        "stage_name": [],
        "fallback_flag": [],
        "replan_count": [],
        "visual_lost_time": [],
        "cmd_accel": [],
        "cmd_steer_rate": [],
        "follower_x": [],
        "follower_y": [],
        "follower_yaw": [],
        "target_x": [],
        "target_y": [],
        "target_yaw": [],
        "leader_head_x": [],
        "leader_head_y": [],
        "leader_head_yaw": [],
        "step_cost": [],
        "min_clearance": [],
    }

    t = 0.0
    min_clear = 1e6
    path_len = 0.0
    traj_cost = 0.0
    cmd_prev_f: ControlCommand | None = None
    cmd_prev_l: ControlCommand | None = None

    done = False
    reason = "timeout"
    collision_flag = False

    # Allow geometric contact with leader *only* in near-field for docking.
    allow_contact_dist = max(0.45, float(cfg.docking.soft_capture_distance) + 0.05)
    lc_pre_ingress_deadlock_s = 12.0
    lc_ingress_deadlock_s = 18.0
    lc_lane_heading = None
    if bool(getattr(scenario, "lane", {}).get("category", "") == "LC"):
        skill = getattr(controller, "skill", None)
        staging_plan = getattr(skill, "_plan", None)
        if staging_plan is not None and str(getattr(staging_plan, "control_mode", "")) == "corridor_reciprocal":
            metadata = dict(getattr(staging_plan, "metadata", {}) or {})
            shape_budget_s = float(metadata.get("shape_budget_s", 6.0))
            settle_budget_s = float(metadata.get("settle_budget_s", 5.0))
            lc_pre_ingress_deadlock_s = float(max(16.0, shape_budget_s + settle_budget_s + 6.0))
            lc_ingress_deadlock_s = float(max(24.0, lc_pre_ingress_deadlock_s + 8.0))
            if "lane_heading" in metadata:
                lc_lane_heading = float(metadata["lane_heading"])
        if lc_lane_heading is None:
            centerline = getattr(scenario, "lane", {}).get("centerline", [])
            if isinstance(centerline, list) and len(centerline) >= 2:
                p0 = np.asarray(centerline[0], dtype=float)
                p1 = np.asarray(centerline[-1], dtype=float)
                if float(np.linalg.norm(p1 - p0)) > 1e-9:
                    lc_lane_heading = float(math.atan2(float(p1[1] - p0[1]), float(p1[0] - p0[0])))

    while t <= float(max_time_s) + 1e-9:
        leader_cmd, follower_cmd, debug = controller.compute_commands(
            leader=leader,
            follower=follower,
            timestamp=t,
            obstacles=scenario.obstacles,
        )
        leader_cmd = _boundary_guard_command(leader, leader_cmd, cfg)
        follower_cmd = _boundary_guard_command(follower, follower_cmd, cfg)

        follower_substage = str(getattr(debug, 'follower_substage', ''))
        lc_hold_requested = bool(follower_substage == 'LC_LOCK_HOLD' or follower_substage.startswith('FOLLOWER_READY'))
        if lc_hold_requested:
            p_err_hold = float(np.linalg.norm(geom.front_hitch(follower) - geom.rear_hitch(leader)))
            yaw_gap_hold = abs(float(angle_diff(float(leader.yaw), float(follower.yaw))))
            clr_hold = float(
                min(
                    collision.min_clearance_vehicle_obstacles(follower, scenario.obstacles),
                    collision.min_clearance_vehicle_obstacles(leader, scenario.obstacles),
                )
            )
            if (
                p_err_hold <= float(cfg.docking.lock_position_tol) + 0.06
                and yaw_gap_hold <= math.radians(5.0)
                and clr_hold >= float(cfg.safety.min_clearance) + 0.005
            ):
                follower_new, leader_new = _project_terminal_hold_pair(
                    geom,
                    follower,
                    leader,
                    collision=collision,
                    obstacles=scenario.obstacles,
                    clearance_floor=float(cfg.safety.min_clearance) + 0.005,
                )
            else:
                leader_new = model.step(leader, leader_cmd, dt)
                follower_new = model.step(follower, follower_cmd, dt)
        else:
            leader_new = model.step(leader, leader_cmd, dt)
            follower_new = model.step(follower, follower_cmd, dt)
        d_tail = float(np.linalg.norm(geom.front_hitch(follower_new) - geom.rear_hitch(leader_new)))
        yaw_gap = abs(float(angle_diff(float(leader_new.yaw), float(follower_new.yaw))))
        speed_gap = abs(float(leader_new.v) - float(follower_new.v))
        legal_contact = (
            d_tail <= (allow_contact_dist + 0.12)
            and yaw_gap <= math.radians(8.0)
            and speed_gap <= 0.45
        )

        # path length (system-level)
        path_len += float(np.linalg.norm(follower_new.xy() - follower.xy()))
        path_len += float(np.linalg.norm(leader_new.xy() - leader.xy()))

        # collision / clearance checks
        if not (_in_bounds(cfg, follower_new, margin=0.5) and _in_bounds(cfg, leader_new, margin=0.5)):
            reason = "out_of_bounds"
            collision_flag = False
            done = True
        else:
            for obs in scenario.obstacles:
                if collision.collide_vehicle_obstacle(follower_new, obs, include_clearance=True) or collision.collide_vehicle_obstacle(
                    leader_new, obs, include_clearance=True
                ):
                    reason = "collision_obstacle"
                    collision_flag = True
                    done = True
                    break
            if not done and collision.collide_vehicle_vehicle(follower_new, leader_new, include_clearance=False):
                if not legal_contact:
                    reason = "collision_vehicle"
                    collision_flag = True
                    done = True

        leader = leader_new
        follower = follower_new

        lc_lock_hold = bool(follower_substage == 'LC_LOCK_HOLD')
        if lc_lock_hold:
            p_err_hold = float(np.linalg.norm(geom.front_hitch(follower) - geom.rear_hitch(leader)))
            yaw_gap_hold = abs(float(angle_diff(float(leader.yaw), float(follower.yaw))))
            clr_hold = float(
                min(
                    collision.min_clearance_vehicle_obstacles(follower, scenario.obstacles),
                    collision.min_clearance_vehicle_obstacles(leader, scenario.obstacles),
                )
            )
            if (
                p_err_hold <= float(cfg.docking.lock_position_tol) + 0.06
                and yaw_gap_hold <= math.radians(5.0)
                and clr_hold >= float(cfg.safety.min_clearance) + 0.005
            ):
                follower, leader = _project_terminal_hold_pair(
                    geom,
                    follower,
                    leader,
                    collision=collision,
                    obstacles=scenario.obstacles,
                    clearance_floor=float(cfg.safety.min_clearance) + 0.005,
                )

        # Soft capture near docking interface: bounded correction avoids end-stage touch-and-pass.
        p_err_pre = float(np.linalg.norm(geom.front_hitch(follower) - geom.rear_hitch(leader)))
        yaw_gap_pre = abs(float(angle_diff(float(leader.yaw), float(follower.yaw))))
        speed_gap_pre = abs(float(leader.v) - float(follower.v))
        certified_terminal_projection = bool(
            bool(getattr(debug, 'terminal_capture_boost', False))
            and bool(getattr(debug, 'visual_valid', False))
            and str(getattr(debug, 'follower_substage', '')) == 'LOCK_ASSIST'
        )
        lc_lock_hold = bool(str(getattr(debug, 'follower_substage', '')) == 'LC_LOCK_HOLD')
        soft_capture_distance = float(0.85 if certified_terminal_projection else cfg.docking.soft_capture_distance)
        soft_capture_yaw = math.radians(float(8.0 if certified_terminal_projection else cfg.docking.soft_capture_max_yaw_deg))
        soft_capture_speed = float(0.50 if certified_terminal_projection else cfg.docking.soft_capture_max_speed_error)
        if (
            (not lc_lock_hold)
            and
            p_err_pre < soft_capture_distance
            and yaw_gap_pre < soft_capture_yaw
            and speed_gap_pre < soft_capture_speed
        ):
            projected = _project_follower_locked(
                geom,
                follower,
                leader,
                collision=collision,
                obstacles=scenario.obstacles,
                clearance_floor=float(cfg.safety.min_clearance),
            )
            corr = np.array([float(projected.x) - float(follower.x), float(projected.y) - float(follower.y)], dtype=float)
            corr_norm = float(np.linalg.norm(corr))
            max_corr = float(0.12 if certified_terminal_projection else cfg.docking.soft_capture_max_position_correction)
            if corr_norm > max_corr:
                corr = corr * (max_corr / max(corr_norm, 1e-9))
            follower.x += float(corr[0])
            follower.y += float(corr[1])
            max_yaw_corr = math.radians(float(8.0 if certified_terminal_projection else cfg.docking.soft_capture_max_yaw_correction_deg))
            yaw_corr = float(angle_diff(float(projected.yaw), float(follower.yaw)))
            yaw_corr = float(clamp(yaw_corr, -max_yaw_corr, max_yaw_corr))
            follower.yaw = float(follower.yaw + yaw_corr)
            blend = float(clamp(float(0.85 if certified_terminal_projection else cfg.docking.soft_capture_velocity_blend), 0.0, 1.0))
            follower.v = (1.0 - blend) * float(follower.v) + blend * float(projected.v)
            follower.delta = (1.0 - blend) * float(follower.delta) + blend * float(projected.delta)

        # clearance for metrics and generic cost
        clr_f = float(collision.min_clearance_vehicle_obstacles(follower, scenario.obstacles))
        clr_l = float(collision.min_clearance_vehicle_obstacles(leader, scenario.obstacles))
        clr = float(min(clr_f, clr_l))
        min_clear = float(min(min_clear, clr))
        traj_cost += _trajectory_cost_step(cfg=cfg, cmd=follower_cmd, cmd_prev=cmd_prev_f, clearance_m=clr_f, dt=dt)
        traj_cost += _trajectory_cost_step(cfg=cfg, cmd=leader_cmd, cmd_prev=cmd_prev_l, clearance_m=clr_l, dt=dt)
        cmd_prev_f = follower_cmd
        cmd_prev_l = leader_cmd

        cond = lock_eval.update(follower, leader, dt)
        if cond.locked:
            done = True
            reason = "locked"

        # history
        history["t"].append(float(t))
        history["distance"].append(float(d_tail))
        history["w_vis"].append(float(getattr(debug, "w_vis", 0.0)))
        history["visual_valid"].append(1.0 if bool(getattr(debug, "visual_valid", False)) else 0.0)
        history["fusion_gamma"].append(float(getattr(debug, "fusion_gamma", 0.0)))
        history["nis"].append(float(getattr(debug, "nis", 0.0)))
        history["pos_error"].append(float(cond.pos_error))
        history["yaw_error_deg"].append(float(math.degrees(cond.yaw_error)))
        history["speed_error"].append(float(cond.speed_error))
        st_name = str(getattr(debug, "stage", "UNKNOWN"))
        sub = str(getattr(debug, "follower_substage", ""))
        if sub:
            st_name = f"{st_name}:{sub}"
        history["stage_name"].append(st_name)
        if "STAGING" in st_name:
            stage_id = 0.0
        elif "APPROACH" in st_name or "GLOBAL" in st_name:
            stage_id = 1.0
        else:
            stage_id = 2.0
        history["stage_id"].append(float(stage_id))
        history["fallback_flag"].append(1.0 if bool(getattr(debug, "fallback_global", False)) else 0.0)
        history["replan_count"].append(float(getattr(debug, "replan_count", 0)))
        history["visual_lost_time"].append(float(getattr(debug, "visual_lost_time", 0.0)))
        history["cmd_accel"].append(float(follower_cmd.accel))
        history["cmd_steer_rate"].append(float(follower_cmd.steer_rate))
        history["follower_x"].append(float(follower.x))
        history["follower_y"].append(float(follower.y))
        history["follower_yaw"].append(float(follower.yaw))
        history["target_x"].append(float(leader.x))
        history["target_y"].append(float(leader.y))
        history["target_yaw"].append(float(leader.yaw))
        history["leader_head_x"].append(float(leader.x))
        history["leader_head_y"].append(float(leader.y))
        history["leader_head_yaw"].append(float(leader.yaw))
        history["step_cost"].append(float(getattr(debug, "step_cost", 0.0)))
        history["min_clearance"].append(float(clr))


        if not done and bool(getattr(scenario, "lane", {}).get("category", "") == "LC") and len(history["distance"]) >= int(4.0 / dt):
            win = int(4.0 / dt)
            prev_dist = float(history["distance"][-win])
            progress = float(prev_dist - d_tail)
            lx = np.asarray(history["leader_head_x"][-win:] + [float(leader.x)], dtype=float)
            ly = np.asarray(history["leader_head_y"][-win:] + [float(leader.y)], dtype=float)
            fx = np.asarray(history["follower_x"][-win:] + [float(follower.x)], dtype=float)
            fy = np.asarray(history["follower_y"][-win:] + [float(follower.y)], dtype=float)
            leader_motion_recent = float(np.sum(np.hypot(np.diff(lx), np.diff(ly))))
            follower_motion_recent = float(np.sum(np.hypot(np.diff(fx), np.diff(fy))))
            in_ingress = bool('FOLLOWER_INGRESS' in st_name or 'DOCKING' in st_name)
            sigma_series = [_extract_sigma_m(name) for name in history["stage_name"][-win:]]
            sigma_series = [float(v) for v in sigma_series if v is not None]
            sigma_now = _extract_sigma_m(st_name)
            terminal_progress_cert = bool(
                'LC_TERMINAL_MANIFOLD' in st_name
                and sigma_now is not None
                and float(sigma_now) >= 0.65
                and (
                    len(sigma_series) == 0
                    or float(max(sigma_series)) - float(min(sigma_series)) >= 0.05
                    or progress >= 0.10
                )
            )
            if (not in_ingress) and float(t) >= float(lc_pre_ingress_deadlock_s) and progress < 0.20 and leader_motion_recent < 0.45:
                reason = "lc_geometric_deadlock"
                collision_flag = False
                done = True
            elif in_ingress and float(t) >= float(lc_ingress_deadlock_s) and progress < 0.18 and follower_motion_recent < 0.45 and (not terminal_progress_cert):
                reason = "lc_geometric_deadlock"
                collision_flag = False
                done = True

        if done:
            break

        t += dt

    # final errors
    pos_err = float(np.linalg.norm(geom.front_hitch(follower) - geom.rear_hitch(leader)))
    yaw_err = abs(float(angle_diff(float(leader.yaw), float(follower.yaw))))
    sp_err = abs(float(leader.v) - float(follower.v))
    fallback_count = _count_rising_edges([float(v) for v in history.get("fallback_flag", [])])
    visual_loss_count = _count_rising_edges([1.0 - float(v) for v in history.get("visual_valid", [])])
    replan_count = int(max([0.0, *[float(v) for v in history.get("replan_count", [])]]))
    stage_ids = [float(v) for v in history.get("stage_id", [])]
    leader_x = [float(v) for v in history.get("leader_head_x", [])]
    leader_y = [float(v) for v in history.get("leader_head_y", [])]
    leader_reloc_time_s = 0.0
    if stage_ids:
        leader_reloc_time_s = float(sum(dt for sid in stage_ids if sid < 0.5))
    leader_reloc_distance_m = 0.0
    if leader_x and leader_y:
        dx = np.diff(np.asarray(leader_x, dtype=float))
        dy = np.diff(np.asarray(leader_y, dtype=float))
        leader_reloc_distance_m = float(np.sum(np.hypot(dx, dy)))
    accel_series = np.asarray(history.get("cmd_accel", []), dtype=float)
    steer_series = np.asarray(history.get("cmd_steer_rate", []), dtype=float)
    accel_smooth = 0.0 if accel_series.size < 2 else float(np.max(np.abs(np.diff(accel_series)) / max(dt, 1e-9)))
    steer_smooth = 0.0 if steer_series.size < 2 else float(np.max(np.abs(np.diff(steer_series)) / max(dt, 1e-9)))
    failure_category = _failure_category(reason, history)

    m = Stage1Metrics(
        method_id=str(method_id),
        success=bool(reason == "locked"),
        reason=str(reason),
        done_time_s=float(t),
        collision=bool(collision_flag),
        min_clearance_m=float(min_clear),
        final_pos_error_m=float(pos_err),
        final_yaw_error_deg=float(math.degrees(yaw_err)),
        final_speed_error_mps=float(sp_err),
        trajectory_cost=float(traj_cost),
        path_length_m=float(path_len),
        failure_category=str(failure_category),
        fallback_count=int(fallback_count),
        visual_loss_count=int(visual_loss_count),
        replan_count=int(replan_count),
        leader_relocation_time_s=float(leader_reloc_time_s),
        leader_relocation_distance_m=float(max(leader_reloc_distance_m, scenario.leader_relocation_m)),
        follower_detour_ratio=float(scenario.follower_detour_ratio),
        accel_smoothness_max=float(accel_smooth),
        steer_rate_smoothness_max=float(steer_smooth),
    )
    payload = {
        "method_id": str(method_id),
        "metrics": m.to_dict(),
        "history": history,
        "obstacles": [asdict(o) for o in scenario.obstacles],
        "scenario_seed": int(scenario.seed),
    }
    return m, payload


def _write_md(out_path: Path, *, scenario: Stage1Scenario, metrics: list[Stage1Metrics]) -> None:
    rows = []
    rows.append("# P-1 Stage-1 Docking Report\n")
    rows.append("## Scenario\n")
    rows.append(f"- seed: `{scenario.seed}`\n")
    rows.append(f"- style: `{scenario.style}`\n")
    rows.append(f"- obstacles: `{len(scenario.obstacles)}`\n")
    rows.append(f"- leader: `({scenario.leader.x:.2f}, {scenario.leader.y:.2f}, yaw={math.degrees(scenario.leader.yaw):.1f}deg)`\n")
    rows.append(f"- follower: `({scenario.follower.x:.2f}, {scenario.follower.y:.2f}, yaw={math.degrees(scenario.follower.yaw):.1f}deg)`\n")
    rows.append(f"- large_obstacle_present: `{scenario.large_obstacle_present}`\n")
    rows.append(f"- direct_los_blocked: `{scenario.direct_los_blocked}`\n")
    rows.append(f"- subset_tag: `{scenario.subset_tag}`\n")
    rows.append(f"- leader_relocation_m: `{scenario.leader_relocation_m:.2f}`\n")
    rows.append(f"- follower_detour_ratio: `{scenario.follower_detour_ratio:.2f}`\n")
    rows.append("\n## Results (single-scene)\n\n")
    rows.append("| method | family | success | T_done[s] | fail_cat | collision | min_clear[m] | traj_cost | replans |\n")
    rows.append("|---|---|---:|---:|---|---:|---:|---:|---:|\n")
    for m in metrics:
        family = METHOD_SPECS.get(m.method_id).family if m.method_id in METHOD_SPECS else "unknown"
        rows.append(
            f"| `{m.method_id}` | `{family}` | `{m.success}` | {m.done_time_s:.2f} | `{m.failure_category}` | `{m.collision}` | {m.min_clearance_m:.3f} | {m.trajectory_cost:.3f} | {m.replan_count} |\n"
        )
    out_path.write_text("".join(rows), encoding="utf-8")


def main() -> None:
    args = parse_args()
    cfg = load_config()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if str(args.scenario_json).strip():
        scenario_path = Path(str(args.scenario_json)).expanduser()
        scenario = Stage1Scenario.from_dict(json.loads(scenario_path.read_text(encoding="utf-8")))
    else:
        scenario = _spawn_random_scene(cfg, int(args.seed), int(args.obstacles), style=str(args.scenario_style))

    if args.save_scenario_json:
        exp_dir = ROOT / "experiments"
        exp_dir.mkdir(exist_ok=True)
        p = exp_dir / f"p_minus1_stage1_scene_seed{scenario.seed}.json"
        p.write_text(json.dumps(scenario.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")

    staging_plan = CooperativeStagingPlanner(cfg, seed=int(scenario.planner_seed)).plan(
        obstacles=scenario.obstacles,
        leader=scenario.leader,
        follower=scenario.follower,
        prefer_static_leader=False,
        semantic_hints=scenario.lane,
    )
    method_runners = build_method_runners(
        cfg,
        seed=int(args.seed),
        obstacles=scenario.obstacles,
        leader_init=scenario.leader,
        follower_init=scenario.follower,
        staging_plan=staging_plan,
        lane_hints=scenario.lane,
    )
    ordered_method_ids = [mid for mid in [*FULL_METHOD_IDS, *ABLATION_METHOD_IDS] if mid in method_runners]
    extra_method_ids = [mid for mid in method_runners.keys() if mid not in ordered_method_ids and mid in METHOD_SPECS]
    methods: list[tuple[str, Any]] = [(mid, method_runners[mid]) for mid in [*ordered_method_ids, *extra_method_ids]]

    if str(args.methods).strip():
        allow = {m.strip() for m in str(args.methods).split(",") if m.strip()}
        methods = [(mid, ctrl) for mid, ctrl in methods if mid in allow]
        if not methods:
            raise SystemExit(f"--methods filtered to empty set: {sorted(allow)}")

    metrics: list[Stage1Metrics] = []
    staging_plan_dict = {
        "leader_goal": asdict(staging_plan.leader_goal),
        "follower_goal": asdict(staging_plan.follower_goal),
        "leader_path_xy": staging_plan.leader_path_xy.astype(float).tolist(),
        "follower_path_xy": staging_plan.follower_path_xy.astype(float).tolist(),
        "score": float(staging_plan.score),
        "reason": str(staging_plan.reason),
        "control_mode": str(getattr(staging_plan, "control_mode", "generic")),
        "leader_primitive_steps": int(getattr(staging_plan, "leader_primitive_steps", 0)),
        "metadata": dict(getattr(staging_plan, "metadata", {}) or {}),
        "leader_relocation_m": float(scenario.leader_relocation_m),
        "follower_detour_ratio": float(scenario.follower_detour_ratio),
        "direct_los_blocked": bool(scenario.direct_los_blocked),
        "large_obstacle_present": bool(scenario.large_obstacle_present),
    }
    result_bundle: dict[str, Any] = {
        "scenario": scenario.to_dict(),
        "staging_plan": staging_plan_dict,
        "method_specs": {mid: asdict(spec) for mid, spec in METHOD_SPECS.items() if mid in dict(methods)},
        "runs": [],
    }

    for mid, ctrl in methods:
        if hasattr(ctrl, "reset"):
            ctrl.reset()
        m, payload = _run_one(cfg=cfg, scenario=scenario, method_id=mid, controller=ctrl, max_time_s=float(args.max_time_s))
        metrics.append(m)
        result_bundle["runs"].append(payload)

        if not bool(args.skip_gifs):
            gif_path = out_dir / f"p_minus1_stage1_{mid}_seed{scenario.seed}.gif"
            save_docking_animation(
                cfg,
                payload["history"],
                scenario.obstacles,
                str(gif_path),
                fps=int(args.gif_fps),
                max_frames=int(args.gif_max_frames),
            )
            print("stage1_gif", gif_path)

    # Save JSON + MD report.
    json_path = out_dir / f"p_minus1_stage1_results_seed{scenario.seed}.json"
    json_path.write_text(json.dumps(result_bundle, ensure_ascii=False, indent=2), encoding="utf-8")
    print("stage1_json", json_path)

    md_path = out_dir / f"P_MINUS1_STAGE1_REPORT_seed{scenario.seed}.md"
    _write_md(md_path, scenario=scenario, metrics=metrics)
    print("stage1_md", md_path)


if __name__ == "__main__":
    main()
