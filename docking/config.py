from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class VehicleConfig:
    car_length: float
    car_width: float
    wheelbase: float
    hitch_length: float
    front_overhang: float
    rear_overhang: float
    min_turn_radius_single: float
    max_steer_deg: float
    max_steer_rate_deg_s: float
    max_speed: float
    max_reverse_speed: float
    max_accel: float
    max_decel: float
    max_yaw_rate: float


@dataclass(frozen=True)
class SensorGlobalConfig:
    sigma_pos: float
    sigma_yaw_deg: float
    rate_hz: float


@dataclass(frozen=True)
class SensorVisionConfig:
    fov_deg: float
    max_distance: float
    sigma_pos: float
    sigma_yaw_deg: float
    rate_hz: float


@dataclass(frozen=True)
class SensorsConfig:
    global_sensor: SensorGlobalConfig
    vision: SensorVisionConfig


@dataclass(frozen=True)
class DockingConfig:
    stage_switch_distance: float
    blend_distance_min: float
    blend_distance_max: float
    lock_position_tol: float
    lock_heading_tol_deg: float
    lock_speed_tol: float
    lock_hold_s: float
    max_alignment_angle_deg: float
    recovery_visual_loss_timeout_s: float
    recovery_backoff_distance: float
    soft_capture_distance: float
    soft_capture_max_yaw_deg: float
    soft_capture_max_speed_error: float
    soft_capture_max_position_correction: float
    soft_capture_max_yaw_correction_deg: float
    soft_capture_velocity_blend: float
    lock_transition_max_position_jump: float
    lock_transition_max_yaw_jump_deg: float
    lock_transition_max_speed_jump: float
    cmd_smooth_max_accel_step: float
    cmd_smooth_max_steer_rate_step: float


@dataclass(frozen=True)
class SafetyConfig:
    min_clearance: float
    jackknife_max_deg: float
    emergency_brake: float
    swept_sample_distance: float
    swept_min_steps: int
    swept_max_steps: int


@dataclass(frozen=True)
class PurePursuitConfig:
    lookahead_min: float
    lookahead_gain: float


@dataclass(frozen=True)
class StanleyConfig:
    k: float
    softening: float


@dataclass(frozen=True)
class SpeedControlConfig:
    kp: float
    max_accel_cmd: float


@dataclass(frozen=True)
class TrainControlConfig:
    curvature_speed_gain: float
    min_speed: float
    max_speed: float
    curvature_lookahead_points: int
    feasible_radius_margin: float


@dataclass(frozen=True)
class ControlConfig:
    dt: float
    main_rate_hz: float
    mid_rate_hz: float
    high_rate_hz: float
    pure_pursuit: PurePursuitConfig
    stanley: StanleyConfig
    speed: SpeedControlConfig
    train: TrainControlConfig


@dataclass(frozen=True)
class PlannerConfig:
    horizon_steps: int
    sample_accel: tuple[float, ...]
    sample_steer_rate_deg: tuple[float, ...]
    goal_weight: float
    heading_weight: float
    clearance_weight: float
    smooth_weight: float


@dataclass(frozen=True)
class EnvironmentConfig:
    width: float
    height: float
    resolution: float


@dataclass(frozen=True)
class TestingConfig:
    random_seed: int
    num_random_scenarios: int
    max_sim_time: float
    acceptance_seed_per_case: int
    acceptance_docking_seeds: int


@dataclass(frozen=True)
class ScenarioConfig:
    path_samples: int
    corridor_half_width: float
    width_req_base_margin: float
    width_req_per_trailer: float
    sweep_gain_curvature: float
    kappa_lookup_max: float
    kappa_lookup_bins: int
    dock_clearance_min: float
    dock_obs_density_max: float
    dock_los_min: float
    dock_kappa_max: float
    los_check_distance: float
    density_window_length: float
    density_window_width: float
    spawn_clearance_min: float
    cluster_radius: float
    uniform_spacing: float
    initial_pose_jitter: float
    ray_step: float
    ray_max_distance: float
    representative_vehicle_count: int
    validation_seeds_per_subtype: int
    validation_min_pass_rate: float
    gate_wall_thickness: float
    gate_turn_y_offset: float
    b_type_kappa_max: float
    turning_gate_kappa_min: float
    turning_gate_y_span_min: float
    validation_grid_resolution: float
    validation_gate_block_margin: float
    c_reconfig_window_ratio: float


@dataclass(frozen=True)
class CoordinatorConfig:
    command_ttl_s: float
    feasibility_dock_time_est_s: float
    feasibility_margin_s: float
    feasibility_recover_hysteresis_s: float
    max_command_queue: int


@dataclass(frozen=True)
class Config:
    vehicle: VehicleConfig
    sensors: SensorsConfig
    docking: DockingConfig
    safety: SafetyConfig
    control: ControlConfig
    planner: PlannerConfig
    environment: EnvironmentConfig
    testing: TestingConfig
    scenario: ScenarioConfig
    coordinator: CoordinatorConfig


def _tuple_of_floats(values: list[Any]) -> tuple[float, ...]:
    return tuple(float(v) for v in values)


def load_config(path: str | Path = "config/default_config.json") -> Config:
    data = json.loads(Path(path).read_text(encoding="utf-8"))

    vehicle = VehicleConfig(**data["vehicle"])
    sensors = SensorsConfig(
        global_sensor=SensorGlobalConfig(**data["sensors"]["global"]),
        vision=SensorVisionConfig(**data["sensors"]["vision"]),
    )
    docking = DockingConfig(**data["docking"])
    safety = SafetyConfig(**data["safety"])
    control = ControlConfig(
        dt=float(data["control"]["dt"]),
        main_rate_hz=float(data["control"]["main_rate_hz"]),
        mid_rate_hz=float(data["control"]["mid_rate_hz"]),
        high_rate_hz=float(data["control"]["high_rate_hz"]),
        pure_pursuit=PurePursuitConfig(**data["control"]["pure_pursuit"]),
        stanley=StanleyConfig(**data["control"]["stanley"]),
        speed=SpeedControlConfig(**data["control"]["speed"]),
        train=TrainControlConfig(**data["control"]["train"]),
    )
    planner = PlannerConfig(
        horizon_steps=int(data["planner"]["horizon_steps"]),
        sample_accel=_tuple_of_floats(data["planner"]["sample_accel"]),
        sample_steer_rate_deg=_tuple_of_floats(data["planner"]["sample_steer_rate_deg"]),
        goal_weight=float(data["planner"]["goal_weight"]),
        heading_weight=float(data["planner"]["heading_weight"]),
        clearance_weight=float(data["planner"]["clearance_weight"]),
        smooth_weight=float(data["planner"]["smooth_weight"]),
    )
    environment = EnvironmentConfig(**data["environment"])
    testing = TestingConfig(
        random_seed=int(data["testing"]["random_seed"]),
        num_random_scenarios=int(data["testing"]["num_random_scenarios"]),
        max_sim_time=float(data["testing"]["max_sim_time"]),
        acceptance_seed_per_case=int(data["testing"].get("acceptance_seed_per_case", 50)),
        acceptance_docking_seeds=int(data["testing"].get("acceptance_docking_seeds", 200)),
    )
    s = data.get("scenario", {})
    scenario = ScenarioConfig(
        path_samples=int(s.get("path_samples", 220)),
        corridor_half_width=float(s.get("corridor_half_width", 2.0)),
        width_req_base_margin=float(s.get("width_req_base_margin", 0.08)),
        width_req_per_trailer=float(s.get("width_req_per_trailer", 0.12)),
        sweep_gain_curvature=float(s.get("sweep_gain_curvature", 0.2)),
        kappa_lookup_max=float(s.get("kappa_lookup_max", 1.0)),
        kappa_lookup_bins=int(s.get("kappa_lookup_bins", 120)),
        dock_clearance_min=float(s.get("dock_clearance_min", 0.6)),
        dock_obs_density_max=float(s.get("dock_obs_density_max", 0.2)),
        dock_los_min=float(s.get("dock_los_min", 0.9)),
        dock_kappa_max=float(s.get("dock_kappa_max", 0.12)),
        los_check_distance=float(s.get("los_check_distance", 1.0)),
        density_window_length=float(s.get("density_window_length", 2.0)),
        density_window_width=float(s.get("density_window_width", 1.4)),
        spawn_clearance_min=float(s.get("spawn_clearance_min", 0.25)),
        cluster_radius=float(s.get("cluster_radius", 1.2)),
        uniform_spacing=float(s.get("uniform_spacing", 3.0)),
        initial_pose_jitter=float(s.get("initial_pose_jitter", 0.08)),
        ray_step=float(s.get("ray_step", 0.05)),
        ray_max_distance=float(s.get("ray_max_distance", 10.0)),
        representative_vehicle_count=int(s.get("representative_vehicle_count", 6)),
        validation_seeds_per_subtype=int(s.get("validation_seeds_per_subtype", 60)),
        validation_min_pass_rate=float(s.get("validation_min_pass_rate", 0.95)),
        gate_wall_thickness=float(s.get("gate_wall_thickness", 0.9)),
        gate_turn_y_offset=float(s.get("gate_turn_y_offset", 0.9)),
        b_type_kappa_max=float(s.get("b_type_kappa_max", 0.25)),
        turning_gate_kappa_min=float(s.get("turning_gate_kappa_min", 0.08)),
        turning_gate_y_span_min=float(s.get("turning_gate_y_span_min", 1.0)),
        validation_grid_resolution=float(s.get("validation_grid_resolution", 0.20)),
        validation_gate_block_margin=float(s.get("validation_gate_block_margin", 0.30)),
        c_reconfig_window_ratio=float(s.get("c_reconfig_window_ratio", 0.20)),
    )
    c = data.get("coordinator", {})
    coordinator = CoordinatorConfig(
        command_ttl_s=float(c.get("command_ttl_s", 5.0)),
        feasibility_dock_time_est_s=float(c.get("feasibility_dock_time_est_s", 10.0)),
        feasibility_margin_s=float(c.get("feasibility_margin_s", 5.0)),
        feasibility_recover_hysteresis_s=float(c.get("feasibility_recover_hysteresis_s", 1.5)),
        max_command_queue=int(c.get("max_command_queue", 128)),
    )

    return Config(
        vehicle=vehicle,
        sensors=sensors,
        docking=docking,
        safety=safety,
        control=control,
        planner=planner,
        environment=environment,
        testing=testing,
        scenario=scenario,
        coordinator=coordinator,
    )
