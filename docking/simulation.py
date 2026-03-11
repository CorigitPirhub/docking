from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Union

import numpy as np

from .collision import CollisionEngine
from .config import Config
from .controllers import PathTrackingController
from .docking import DockingLockEvaluator, TwoStageDockingController
from .environment import Environment, random_obstacles
from .kinematics import AckermannModel, VehicleGeometry
from .planner import LocalPlanner
from .sensors import GlobalPoseSensor, VisionSensor
from .train import TrainController, TrainKinematics, TrainSafetyGuard
from .types import ControlCommand, Obstacle, VehicleMode, VehicleState


@dataclass
class DockingRunResult:
    success: bool
    reason: str
    sim_time: float
    final_pos_error: float
    final_yaw_error_deg: float
    final_speed_error: float
    min_clearance: float
    emergency_events: int
    obstacles: list[Obstacle]
    history: dict[str, Union[list[float], list[str]]]
    lock_transition_ok: bool = True
    lock_transition_violation_count: int = 0
    max_lock_pos_jump: float = 0.0
    max_lock_yaw_jump_deg: float = 0.0
    max_lock_speed_jump: float = 0.0
    visual_fallback_events: int = 0
    vision_recovery_events: int = 0
    angle_recovery_events: int = 0


def _make_straight_train(geom: VehicleGeometry, n: int, x0: float, y0: float, yaw: float) -> list[VehicleState]:
    spacing = geom.front_hitch_x - geom.rear_hitch_x
    out = []
    for i in range(n):
        x = x0 - i * spacing * math.cos(yaw)
        y = y0 - i * spacing * math.sin(yaw)
        out.append(VehicleState(vehicle_id=100 + i, x=x, y=y, yaw=yaw, v=0.0, delta=0.0, mode=VehicleMode.FREE))
    return out


def _path_sine(x_start: float, x_end: float, y_base: float, amp: float, period: float, n: int = 300) -> np.ndarray:
    xs = np.linspace(x_start, x_end, n)
    ys = y_base + amp * np.sin((xs - x_start) / period)
    return np.stack([xs, ys], axis=1)


def _project_follower_locked(
    geom: VehicleGeometry,
    follower: VehicleState,
    target_tail: VehicleState,
    *,
    collision: CollisionEngine | None = None,
    obstacles: list[Obstacle] | None = None,
    clearance_floor: float | None = None,
) -> VehicleState:
    anchor = geom.rear_hitch(target_tail)
    yaw = target_tail.yaw
    rear = anchor - np.array([math.cos(yaw), math.sin(yaw)], dtype=float) * geom.front_hitch_x
    projected = VehicleState(
        vehicle_id=follower.vehicle_id,
        x=float(rear[0]),
        y=float(rear[1]),
        yaw=yaw,
        v=target_tail.v,
        delta=target_tail.delta,
        mode=VehicleMode.TRAIN_FOLLOW,
    )
    if collision is None or obstacles is None or clearance_floor is None:
        return projected
    current_clearance_m = float(collision.min_clearance_vehicle_obstacles(follower, obstacles))
    projected_clearance_m = float(collision.min_clearance_vehicle_obstacles(projected, obstacles))
    if projected_clearance_m + 1e-6 < float(clearance_floor) <= current_clearance_m + 1e-6:
        follower_locked = follower.copy()
        follower_locked.v = float(target_tail.v)
        follower_locked.delta = float(target_tail.delta)
        return follower_locked
    return projected


def _project_terminal_hold_pair(
    geom: VehicleGeometry,
    follower: VehicleState,
    target_tail: VehicleState,
    *,
    collision: CollisionEngine | None = None,
    obstacles: list[Obstacle] | None = None,
    clearance_floor: float | None = None,
) -> tuple[VehicleState, VehicleState]:
    follower_hold = _project_follower_locked(
        geom,
        follower,
        target_tail,
        collision=collision,
        obstacles=obstacles,
        clearance_floor=clearance_floor,
    )
    target_hold = target_tail.copy()
    follower_hold.v = 0.0
    target_hold.v = 0.0
    follower_hold.delta = 0.0
    target_hold.delta = 0.0
    return follower_hold, target_hold


def _boundary_guard_command(state: VehicleState, cmd: ControlCommand, cfg: Config, margin: float = 2.5) -> ControlCommand:
    half_w = 0.5 * cfg.environment.width - margin
    half_h = 0.5 * cfg.environment.height - margin
    hx = math.cos(state.yaw)
    hy = math.sin(state.yaw)

    heading_out = (
        (state.x > half_w and hx > 0.0)
        or (state.x < -half_w and hx < 0.0)
        or (state.y > half_h and hy > 0.0)
        or (state.y < -half_h and hy < 0.0)
    )
    if heading_out:
        return ControlCommand(accel=-cfg.vehicle.max_decel, steer_rate=0.0)
    return cmd


def run_docking_case(
    cfg: Config,
    leader_train_size: int,
    seed: int,
    with_obstacles: bool,
    moving_leader: bool,
    max_time: float | None = None,
    preset_obstacles: list[Obstacle] | None = None,
) -> DockingRunResult:
    rng = np.random.default_rng(seed)
    dt = cfg.control.dt
    max_time = cfg.testing.max_sim_time if max_time is None else max_time

    obs_count = max(1, 7 - leader_train_size * 2)
    if preset_obstacles is not None:
        obstacles = [Obstacle(x=o.x, y=o.y, width=o.width, height=o.height, yaw=o.yaw) for o in preset_obstacles]
    else:
        obstacles = random_obstacles(cfg.environment, rng, obs_count) if with_obstacles else []
    env = Environment(cfg.environment, obstacles)

    geom = VehicleGeometry(cfg.vehicle)
    model = AckermannModel(cfg.vehicle)
    collision = CollisionEngine(cfg.vehicle, cfg.safety)

    train_states = _make_straight_train(
        geom,
        leader_train_size,
        x0=-8.0,
        y0=float(rng.uniform(-2.0, 2.0)),
        yaw=0.0,
    )

    spawn_scale = 1.0 / (1.0 + 0.25 * max(0, leader_train_size - 1))
    follower_x_jitter = max(0.6, 1.5 * spawn_scale)
    follower_y_jitter = max(0.4, 1.0 * spawn_scale)
    follower_yaw_jitter = max(0.08, 0.22 * spawn_scale)

    follower = VehicleState(
        vehicle_id=1,
        x=-14.0 + float(rng.uniform(-follower_x_jitter, follower_x_jitter)),
        y=train_states[0].y + float(rng.uniform(-follower_y_jitter, follower_y_jitter)),
        yaw=float(rng.uniform(-follower_yaw_jitter, follower_yaw_jitter)),
        v=0.0,
        delta=0.0,
        mode=VehicleMode.DOCKING,
    )

    if preset_obstacles is None:
        # Keep obstacle generation away from immediate spawn corridor.
        filtered_obs = []
        lane_y = train_states[0].y
        corridor_half = 2.4 + 0.9 * max(0, leader_train_size - 1)
        for obs in obstacles:
            if abs(obs.x + 10.0) < 2.5 and abs(obs.y - lane_y) < 3.0:
                continue
            # Avoid generating impossible corridor blockages around nominal convoy lane.
            if abs(obs.y - lane_y) < corridor_half:
                continue
            filtered_obs.append(obs)
        env.obstacles = filtered_obs

    tracker = PathTrackingController(cfg.vehicle, cfg.control, steering_mode="pure_pursuit")
    planner = LocalPlanner(cfg.vehicle, cfg.planner, collision, cfg.control.dt)

    global_sensor = GlobalPoseSensor(cfg.sensors.global_sensor, seed=seed + 11)
    vision_sensor = VisionSensor(cfg.sensors.vision, cfg.vehicle, seed=seed + 17)
    docking_controller = TwoStageDockingController(cfg, global_sensor, vision_sensor, tracker, planner)
    lock_eval = DockingLockEvaluator(cfg.docking, geom)

    train_kin = TrainKinematics(cfg.vehicle)
    train_controller = TrainController(cfg.vehicle, cfg.control)
    train_guard = TrainSafetyGuard(cfg.vehicle, cfg.safety, collision)

    path_amp = 1.2 / (1.0 + 0.45 * max(0, leader_train_size - 1))
    head_path = _path_sine(x_start=-8.0, x_end=14.0, y_base=train_states[0].y, amp=path_amp, period=8.0)

    t = 0.0
    min_clearance = 1e6
    emergency_events = 0
    emergency_cooldown = 0.0
    lock_transition_ok = True
    lock_transition_violations = 0
    max_lock_pos_jump = 0.0
    max_lock_yaw_jump_deg = 0.0
    max_lock_speed_jump = 0.0
    visual_fallback_events = 0
    vision_recovery_events = 0
    angle_recovery_events = 0
    stage_map = {
        "GLOBAL_APPROACH": 0.0,
        "VISUAL_SERVO": 1.0,
        "VISION_RECOVERY": 2.0,
        "ANGLE_RECOVERY": 3.0,
        "WAIT_GLOBAL": 4.0,
    }
    history = {
        "t": [],
        "distance": [],
        "w_vis": [],
        "pos_error": [],
        "yaw_error_deg": [],
        "speed_error": [],
        "stage_id": [],
        "stage_name": [],
        "fallback_flag": [],
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
    }

    while t <= max_time:
        prev_train = [s.copy() for s in train_states]
        dist_to_tail = float(np.linalg.norm(geom.front_hitch(follower) - geom.rear_hitch(train_states[-1])))

        if emergency_cooldown > 0.0:
            emergency_cooldown = max(0.0, emergency_cooldown - dt)
            for s in train_states:
                s.v = 0.0
                s.delta = 0.0
        else:
            if moving_leader:
                if dist_to_tail < 0.6:
                    v_coop = 0.0
                elif dist_to_tail < 3.0:
                    v_coop = 0.04
                elif dist_to_tail < 6.0:
                    v_coop = 0.10
                elif dist_to_tail < 10.0:
                    v_coop = 0.16
                else:
                    v_coop = 0.25
                v_coop = v_coop / (1.0 + 0.8 * max(0, leader_train_size - 1))
                head_cmd = train_controller.track_head_path(train_states, head_path, target_speed=v_coop, dt=dt)
            else:
                head_cmd = ControlCommand(accel=0.0, steer_rate=0.0)

            train_update = train_kin.update(train_states, head_cmd, dt)
            guard_report = train_guard.check(prev_train, train_update.states, train_update.articulation_angles, env.obstacles)
            if guard_report.emergency_stop:
                emergency_events += 1
                emergency_cooldown = 1.0
                train_states = prev_train
                for s in train_states:
                    s.v = 0.0
                    s.delta = 0.0
            else:
                train_states = train_update.states

        target_tail = train_states[-1]

        cmd, debug = docking_controller.compute_command(
            follower=follower,
            leader_true=target_tail,
            timestamp=t,
            obstacles=env.obstacles,
            dynamic_others=train_states[:-1],
        )
        cmd = _boundary_guard_command(follower, cmd, cfg)
        follower_new = model.step(follower, cmd, dt)

        if not env.in_bounds(follower_new, margin=0.4):
            return DockingRunResult(
                success=False,
                reason="out_of_bounds",
                sim_time=t,
                final_pos_error=999.0,
                final_yaw_error_deg=999.0,
                final_speed_error=999.0,
                min_clearance=min_clearance,
                emergency_events=emergency_events,
                obstacles=env.obstacles,
                history=history,
                lock_transition_ok=lock_transition_ok,
                lock_transition_violation_count=lock_transition_violations,
                max_lock_pos_jump=max_lock_pos_jump,
                max_lock_yaw_jump_deg=max_lock_yaw_jump_deg,
                max_lock_speed_jump=max_lock_speed_jump,
                visual_fallback_events=visual_fallback_events,
                vision_recovery_events=vision_recovery_events,
                angle_recovery_events=angle_recovery_events,
            )

        # Collision checks. Contact with target-tail is allowed only when close enough for docking.
        for obs in env.obstacles:
            if collision.collide_vehicle_obstacle(follower_new, obs):
                return DockingRunResult(
                    success=False,
                    reason="collision_obstacle",
                    sim_time=t,
                    final_pos_error=999.0,
                    final_yaw_error_deg=999.0,
                    final_speed_error=999.0,
                    min_clearance=min_clearance,
                    emergency_events=emergency_events,
                    obstacles=env.obstacles,
                    history=history,
                    lock_transition_ok=lock_transition_ok,
                    lock_transition_violation_count=lock_transition_violations,
                    max_lock_pos_jump=max_lock_pos_jump,
                    max_lock_yaw_jump_deg=max_lock_yaw_jump_deg,
                    max_lock_speed_jump=max_lock_speed_jump,
                    visual_fallback_events=visual_fallback_events,
                    vision_recovery_events=vision_recovery_events,
                    angle_recovery_events=angle_recovery_events,
                )

        for idx, s in enumerate(train_states):
            # During docking we allow geometric contact with the target tail vehicle.
            allowed_contact = idx == len(train_states) - 1
            if not allowed_contact and collision.collide_vehicle_vehicle(follower_new, s):
                return DockingRunResult(
                    success=False,
                    reason="collision_vehicle",
                    sim_time=t,
                    final_pos_error=999.0,
                    final_yaw_error_deg=999.0,
                    final_speed_error=999.0,
                    min_clearance=min_clearance,
                    emergency_events=emergency_events,
                    obstacles=env.obstacles,
                    history=history,
                    lock_transition_ok=lock_transition_ok,
                    lock_transition_violation_count=lock_transition_violations,
                    max_lock_pos_jump=max_lock_pos_jump,
                    max_lock_yaw_jump_deg=max_lock_yaw_jump_deg,
                    max_lock_speed_jump=max_lock_speed_jump,
                    visual_fallback_events=visual_fallback_events,
                    vision_recovery_events=vision_recovery_events,
                    angle_recovery_events=angle_recovery_events,
                )

        follower = follower_new

        if bool(str(getattr(debug, "stage", "")) == "LC_LOCK_HOLD"):
            p_err_hold = float(np.linalg.norm(geom.front_hitch(follower) - geom.rear_hitch(target_tail)))
            yaw_gap_hold = abs(math.atan2(math.sin(target_tail.yaw - follower.yaw), math.cos(target_tail.yaw - follower.yaw)))
            clr_hold = float(
                min(
                    collision.min_clearance_vehicle_obstacles(follower, env.obstacles),
                    collision.min_clearance_vehicle_obstacles(target_tail, env.obstacles),
                )
            )
            if (
                p_err_hold <= float(cfg.docking.lock_position_tol) + 0.06
                and yaw_gap_hold <= math.radians(5.0)
                and clr_hold >= float(cfg.safety.min_clearance) + 0.005
            ):
                follower, target_tail = _project_terminal_hold_pair(
                    geom,
                    follower,
                    target_tail,
                    collision=collision,
                    obstacles=env.obstacles,
                    clearance_floor=float(cfg.safety.min_clearance) + 0.005,
                )

        if debug.fallback_global:
            visual_fallback_events += 1
        if debug.fallback_reason == "vision_lost":
            vision_recovery_events += 1
        if debug.fallback_reason == "angle_exceeded":
            angle_recovery_events += 1

        # Soft capture near docking interface: bounded correction avoids end-stage pose jumps.
        p_err_pre = float(np.linalg.norm(geom.front_hitch(follower) - geom.rear_hitch(target_tail)))
        yaw_gap_pre = abs(math.atan2(math.sin(target_tail.yaw - follower.yaw), math.cos(target_tail.yaw - follower.yaw)))
        speed_gap_pre = abs(target_tail.v - follower.v)
        lc_lock_hold = bool(str(getattr(debug, "stage", "")) == "LC_LOCK_HOLD")
        if (
            (not lc_lock_hold)
            and
            p_err_pre < cfg.docking.soft_capture_distance
            and yaw_gap_pre < math.radians(cfg.docking.soft_capture_max_yaw_deg)
            and speed_gap_pre < cfg.docking.soft_capture_max_speed_error
        ):
            projected = _project_follower_locked(
                geom,
                follower,
                target_tail,
                collision=collision,
                obstacles=env.obstacles,
                clearance_floor=float(cfg.safety.min_clearance),
            )
            corr = np.array([projected.x - follower.x, projected.y - follower.y], dtype=float)
            corr_norm = float(np.linalg.norm(corr))
            max_corr = cfg.docking.soft_capture_max_position_correction
            if corr_norm > max_corr:
                corr = corr * (max_corr / max(corr_norm, 1e-9))
            follower.x += float(corr[0])
            follower.y += float(corr[1])
            max_yaw_corr = math.radians(cfg.docking.soft_capture_max_yaw_correction_deg)
            yaw_corr = math.atan2(math.sin(projected.yaw - follower.yaw), math.cos(projected.yaw - follower.yaw))
            yaw_corr = max(-max_yaw_corr, min(max_yaw_corr, yaw_corr))
            follower.yaw = math.atan2(math.sin(follower.yaw + yaw_corr), math.cos(follower.yaw + yaw_corr))
            blend = min(max(cfg.docking.soft_capture_velocity_blend, 0.0), 1.0)
            follower.v = (1.0 - blend) * follower.v + blend * projected.v
            follower.delta = (1.0 - blend) * follower.delta + blend * projected.delta

        lock_cond = lock_eval.update(follower, target_tail, dt)
        min_clearance = min(min_clearance, collision.min_clearance_vehicle_obstacles(follower, env.obstacles))

        history["t"].append(t)
        history["distance"].append(debug.distance)
        history["w_vis"].append(debug.w_vis)
        history["pos_error"].append(lock_cond.pos_error)
        history["yaw_error_deg"].append(math.degrees(lock_cond.yaw_error))
        history["speed_error"].append(lock_cond.speed_error)
        history["stage_id"].append(stage_map.get(debug.stage, 9.0))
        history["stage_name"].append(debug.stage)
        history["fallback_flag"].append(1.0 if debug.fallback_global else 0.0)
        history["visual_lost_time"].append(debug.visual_lost_time)
        history["cmd_accel"].append(cmd.accel)
        history["cmd_steer_rate"].append(cmd.steer_rate)
        history["follower_x"].append(follower.x)
        history["follower_y"].append(follower.y)
        history["follower_yaw"].append(follower.yaw)
        history["target_x"].append(target_tail.x)
        history["target_y"].append(target_tail.y)
        history["target_yaw"].append(target_tail.yaw)
        history["leader_head_x"].append(train_states[0].x)
        history["leader_head_y"].append(train_states[0].y)
        history["leader_head_yaw"].append(train_states[0].yaw)

        if lock_cond.locked:
            follower_locked = _project_follower_locked(
                geom,
                follower,
                target_tail,
                collision=collision,
                obstacles=env.obstacles,
                clearance_floor=float(cfg.safety.min_clearance),
            )
            pos_jump = float(np.linalg.norm(follower_locked.xy() - follower.xy()))
            yaw_jump_deg = abs(
                math.degrees(math.atan2(math.sin(follower_locked.yaw - follower.yaw), math.cos(follower_locked.yaw - follower.yaw)))
            )
            speed_jump = abs(follower_locked.v - follower.v)
            max_lock_pos_jump = max(max_lock_pos_jump, pos_jump)
            max_lock_yaw_jump_deg = max(max_lock_yaw_jump_deg, yaw_jump_deg)
            max_lock_speed_jump = max(max_lock_speed_jump, speed_jump)

            if (
                pos_jump > cfg.docking.lock_transition_max_position_jump
                or yaw_jump_deg > cfg.docking.lock_transition_max_yaw_jump_deg
                or speed_jump > cfg.docking.lock_transition_max_speed_jump
            ):
                lock_transition_ok = False
                lock_transition_violations += 1
                lock_eval.reset()
                t += dt
                continue

            train_states.append(follower_locked)
            return DockingRunResult(
                success=True,
                reason="locked",
                sim_time=t,
                final_pos_error=lock_cond.pos_error,
                final_yaw_error_deg=math.degrees(lock_cond.yaw_error),
                final_speed_error=lock_cond.speed_error,
                min_clearance=min_clearance,
                emergency_events=emergency_events,
                obstacles=env.obstacles,
                history=history,
                lock_transition_ok=lock_transition_ok,
                lock_transition_violation_count=lock_transition_violations,
                max_lock_pos_jump=max_lock_pos_jump,
                max_lock_yaw_jump_deg=max_lock_yaw_jump_deg,
                max_lock_speed_jump=max_lock_speed_jump,
                visual_fallback_events=visual_fallback_events,
                vision_recovery_events=vision_recovery_events,
                angle_recovery_events=angle_recovery_events,
            )

        t += dt

    final = lock_eval.update(follower, train_states[-1], dt)
    if lock_transition_violations > 0:
        final_reason = "timeout_lock_transition"
    elif angle_recovery_events > 0:
        final_reason = "timeout_angle_recovery"
    elif vision_recovery_events > 0:
        final_reason = "timeout_vision_recovery"
    else:
        final_reason = "timeout"

    return DockingRunResult(
        success=False,
        reason=final_reason,
        sim_time=max_time,
        final_pos_error=final.pos_error,
        final_yaw_error_deg=math.degrees(final.yaw_error),
        final_speed_error=final.speed_error,
        min_clearance=min_clearance,
        emergency_events=emergency_events,
        obstacles=env.obstacles,
        history=history,
        lock_transition_ok=lock_transition_ok,
        lock_transition_violation_count=lock_transition_violations,
        max_lock_pos_jump=max_lock_pos_jump,
        max_lock_yaw_jump_deg=max_lock_yaw_jump_deg,
        max_lock_speed_jump=max_lock_speed_jump,
        visual_fallback_events=visual_fallback_events,
        vision_recovery_events=vision_recovery_events,
        angle_recovery_events=angle_recovery_events,
    )
