from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .collision import CollisionEngine
from .config import ControlConfig, SafetyConfig, VehicleConfig
from .controllers import PathTrackingController
from .kinematics import AckermannModel, VehicleGeometry
from .math_utils import angle_diff, clamp, wrap_angle
from .types import ControlCommand, Obstacle, VehicleMode, VehicleState


@dataclass
class TrainUpdateResult:
    states: list[VehicleState]
    articulation_angles: list[float]


@dataclass
class SafetyReport:
    safe: bool
    emergency_stop: bool
    max_articulation_deg: float
    reason: str = ""


@dataclass(frozen=True)
class PathFeasibility:
    feasible: bool
    min_radius: float
    required_radius: float


def _three_point_radius(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> float:
    a = float(np.linalg.norm(p1 - p0))
    b = float(np.linalg.norm(p2 - p1))
    c = float(np.linalg.norm(p2 - p0))
    if min(a, b, c) < 1e-9:
        return math.inf
    area2 = abs(float((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0])))
    if area2 < 1e-9:
        return math.inf
    # Circumradius: R = abc / (4A), where area2 = 2A from cross product magnitude.
    radius = a * b * c / max(2.0 * area2, 1e-9)
    return float(radius)


@dataclass
class TrainKinematics:
    vehicle_cfg: VehicleConfig

    def __post_init__(self) -> None:
        self.geom = VehicleGeometry(self.vehicle_cfg)
        self.model = AckermannModel(self.vehicle_cfg)

    def min_turn_radius_train(self, n: int) -> float:
        return self.vehicle_cfg.min_turn_radius_single + 0.5 * max(0, n - 1)

    def path_min_radius(self, path_xy: np.ndarray) -> float:
        if len(path_xy) < 3:
            return math.inf
        rmin = math.inf
        for i in range(1, len(path_xy) - 1):
            r = _three_point_radius(path_xy[i - 1], path_xy[i], path_xy[i + 1])
            rmin = min(rmin, r)
        return rmin

    def check_path_feasibility(self, path_xy: np.ndarray, train_size: int, margin: float = 0.0) -> PathFeasibility:
        required = self.min_turn_radius_train(train_size)
        min_radius = self.path_min_radius(path_xy)
        effective_required = required * (1.0 + max(0.0, margin))
        feasible = min_radius >= effective_required or math.isinf(min_radius)
        return PathFeasibility(feasible=feasible, min_radius=min_radius, required_radius=effective_required)

    def clamp_head_command(self, cmd: ControlCommand, train_size: int, head_state: VehicleState, dt: float) -> ControlCommand:
        _ = dt
        rmin = self.min_turn_radius_train(train_size)
        max_delta = math.atan(self.vehicle_cfg.wheelbase / max(rmin, 1e-6))
        max_delta_single = math.radians(self.vehicle_cfg.max_steer_deg)
        max_delta = min(max_delta, max_delta_single)

        # Project desired steering-rate so next-step steering does not violate train curvature.
        desired_delta = head_state.delta + cmd.steer_rate * dt
        desired_delta = clamp(desired_delta, -max_delta, max_delta)
        steer_rate = (desired_delta - head_state.delta) / dt
        return ControlCommand(accel=cmd.accel, steer_rate=steer_rate)

    def update(self, states: list[VehicleState], head_cmd: ControlCommand, dt: float) -> TrainUpdateResult:
        if not states:
            return TrainUpdateResult(states=[], articulation_angles=[])

        old = [s.copy() for s in states]
        cmd = self.clamp_head_command(head_cmd, len(states), states[0], dt)

        new_states: list[VehicleState] = [self.model.step(states[0], cmd, dt)]
        new_states[0].mode = VehicleMode.FREE

        for i in range(1, len(states)):
            prev_new = new_states[i - 1]
            follower_old = old[i]
            prev_old = old[i - 1]

            anchor = self.geom.rear_hitch(prev_new)

            # Trailer-like articulation dynamics:
            # follower yaw rate is driven by hitch velocity direction, which damps whip-like oscillations.
            anchor_old = self.geom.rear_hitch(prev_old)
            hitch_vel = (anchor - anchor_old) / max(dt, 1e-9)
            hitch_speed = float(np.linalg.norm(hitch_vel))
            if hitch_speed > 1e-6:
                hitch_dir = math.atan2(hitch_vel[1], hitch_vel[0])
            else:
                hitch_dir = prev_new.yaw

            beta = angle_diff(hitch_dir, follower_old.yaw)
            yaw_rate = hitch_speed * math.sin(beta) / max(self.geom.front_hitch_x, 1e-6)
            yaw_rate = clamp(yaw_rate, -self.vehicle_cfg.max_yaw_rate, self.vehicle_cfg.max_yaw_rate)
            yaw = wrap_angle(follower_old.yaw + yaw_rate * dt)

            # Hard articulation cap to prevent jackknife-like follower overshoot.
            max_art = math.radians(45.0)
            art = angle_diff(prev_new.yaw, yaw)
            art = clamp(art, -max_art, max_art)
            yaw = wrap_angle(prev_new.yaw - art)

            rear = anchor - np.array([math.cos(yaw), math.sin(yaw)], dtype=float) * self.geom.front_hitch_x
            vel_vec = rear - follower_old.xy()
            v = float(np.dot(vel_vec / max(dt, 1e-6), np.array([math.cos(yaw), math.sin(yaw)], dtype=float)))
            v = clamp(v, 0.0, self.vehicle_cfg.max_speed)

            yaw_rate = angle_diff(yaw, follower_old.yaw) / max(dt, 1e-6)
            if abs(v) < 0.05:
                delta = 0.0
            else:
                delta = math.atan2(self.vehicle_cfg.wheelbase * yaw_rate, v)
            delta = self.model.clamp_delta(delta)

            new_states.append(
                VehicleState(
                    vehicle_id=follower_old.vehicle_id,
                    x=float(rear[0]),
                    y=float(rear[1]),
                    yaw=yaw,
                    v=v,
                    delta=delta,
                    mode=VehicleMode.TRAIN_FOLLOW,
                )
            )

        articulation = []
        for i in range(1, len(new_states)):
            articulation.append(angle_diff(new_states[i - 1].yaw, new_states[i].yaw))

        return TrainUpdateResult(states=new_states, articulation_angles=articulation)


@dataclass
class TrainController:
    vehicle_cfg: VehicleConfig
    control_cfg: ControlConfig

    def __post_init__(self) -> None:
        self.head_tracker = PathTrackingController(self.vehicle_cfg, self.control_cfg, steering_mode="pure_pursuit")
        self.train_kin = TrainKinematics(self.vehicle_cfg)
        self._last_feasibility = PathFeasibility(feasible=True, min_radius=math.inf, required_radius=0.0)
        self._cached_feasibility_key: tuple[int, int, int] | None = None

    def _local_path_radius(self, path_xy: np.ndarray, near_idx: int, lookahead_points: int) -> float:
        if len(path_xy) < 3:
            return math.inf
        lo = max(1, near_idx - 1)
        hi = min(len(path_xy) - 2, near_idx + max(1, lookahead_points))
        rmin = math.inf
        for i in range(lo, hi + 1):
            rmin = min(rmin, _three_point_radius(path_xy[i - 1], path_xy[i], path_xy[i + 1]))
        return rmin

    def _curvature_limited_speed(self, base_speed: float, local_radius: float, train_size: int) -> float:
        tcfg = self.control_cfg.train
        rmin = self.train_kin.min_turn_radius_train(train_size)
        if math.isinf(local_radius):
            return clamp(base_speed, tcfg.min_speed, tcfg.max_speed)
        ratio = rmin / max(local_radius, 1e-6)
        ratio = clamp(ratio, 0.0, 2.0)
        speed_scale = 1.0 / (1.0 + tcfg.curvature_speed_gain * ratio)
        v = base_speed * speed_scale
        return clamp(v, tcfg.min_speed, tcfg.max_speed)

    def track_head_path(self, train_states: list[VehicleState], path_xy: np.ndarray, target_speed: float, dt: float) -> ControlCommand:
        head = train_states[0]
        tcfg = self.control_cfg.train
        path_key = (int(path_xy.__array_interface__["data"][0]), len(path_xy), len(train_states))
        if self._cached_feasibility_key != path_key:
            self._last_feasibility = self.train_kin.check_path_feasibility(
                path_xy,
                train_size=len(train_states),
                margin=tcfg.feasible_radius_margin,
            )
            self._cached_feasibility_key = path_key
        if not self._last_feasibility.feasible:
            brake = ControlCommand(accel=-self.vehicle_cfg.max_decel, steer_rate=0.0)
            return self.train_kin.clamp_head_command(brake, len(train_states), head, dt)

        nearest = self.head_tracker._nearest_index(head, path_xy)
        local_radius = self._local_path_radius(path_xy, nearest, tcfg.curvature_lookahead_points)
        target_speed_limited = self._curvature_limited_speed(target_speed, local_radius, len(train_states))
        cmd = self.head_tracker.track_path(head, path_xy, target_speed_limited)
        return self.train_kin.clamp_head_command(cmd, len(train_states), head, dt)

    @property
    def last_path_feasibility(self) -> PathFeasibility:
        return self._last_feasibility


@dataclass
class TrainSafetyGuard:
    vehicle_cfg: VehicleConfig
    safety_cfg: SafetyConfig
    collision_engine: CollisionEngine

    def check(
        self,
        prev_states: list[VehicleState],
        new_states: list[VehicleState],
        articulation_angles: list[float],
        obstacles: list[Obstacle],
    ) -> SafetyReport:
        max_phi = max([abs(a) for a in articulation_angles], default=0.0)
        phi_limit = math.radians(self.safety_cfg.jackknife_max_deg)
        if max_phi > phi_limit:
            return SafetyReport(
                safe=False,
                emergency_stop=True,
                max_articulation_deg=math.degrees(max_phi),
                reason="jackknife_risk",
            )

        # Swept guardian: interpolate with speed-adaptive density for consistent collision checking.
        diag = math.hypot(self.vehicle_cfg.car_length, self.vehicle_cfg.car_width)
        max_disp = 0.0
        for p, n in zip(prev_states, new_states):
            transl = float(np.linalg.norm(n.xy() - p.xy()))
            rot_arc = 0.5 * diag * abs(angle_diff(n.yaw, p.yaw))
            max_disp = max(max_disp, transl + rot_arc)
        interp_steps = int(math.ceil(max_disp / max(self.safety_cfg.swept_sample_distance, 1e-4)))
        interp_steps = max(self.safety_cfg.swept_min_steps, min(self.safety_cfg.swept_max_steps, interp_steps))

        for alpha_idx in range(interp_steps + 1):
            alpha = alpha_idx / interp_steps
            sample_states: list[VehicleState] = []
            for p, n in zip(prev_states, new_states):
                yaw_interp = wrap_angle(p.yaw + alpha * angle_diff(n.yaw, p.yaw))
                sample_states.append(
                    VehicleState(
                        vehicle_id=n.vehicle_id,
                        x=(1.0 - alpha) * p.x + alpha * n.x,
                        y=(1.0 - alpha) * p.y + alpha * n.y,
                        yaw=yaw_interp,
                        v=(1.0 - alpha) * p.v + alpha * n.v,
                        delta=(1.0 - alpha) * p.delta + alpha * n.delta,
                        mode=n.mode,
                    )
                )

            if self.collision_engine.collide_train_obstacles(sample_states, obstacles, include_clearance=True):
                return SafetyReport(
                    safe=False,
                    emergency_stop=True,
                    max_articulation_deg=math.degrees(max_phi),
                    reason="swept_collision_obstacle",
                )

            if self.collision_engine.collide_train_self(sample_states, include_clearance=False, non_adjacent_only=True):
                return SafetyReport(
                    safe=False,
                    emergency_stop=True,
                    max_articulation_deg=math.degrees(max_phi),
                    reason="swept_collision_self",
                )

        return SafetyReport(
            safe=True,
            emergency_stop=False,
            max_articulation_deg=math.degrees(max_phi),
            reason="ok",
        )
