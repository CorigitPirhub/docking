from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .collision import CollisionEngine
from .config import PlannerConfig, VehicleConfig
from .kinematics import AckermannModel
from .math_utils import angle_diff, clamp
from .types import ControlCommand, Obstacle, VehicleState


@dataclass
class PlanningResult:
    command: ControlCommand
    trajectory: list[VehicleState]
    feasible: bool
    cost: float


@dataclass
class LocalPlanner:
    vehicle_cfg: VehicleConfig
    planner_cfg: PlannerConfig
    collision_engine: CollisionEngine
    dt: float

    def __post_init__(self) -> None:
        self.model = AckermannModel(self.vehicle_cfg)

    def _predict_other(self, state: VehicleState, steps: int) -> list[VehicleState]:
        out = []
        s = state.copy()
        for _ in range(steps):
            # Constant velocity and steering for short-horizon prediction.
            s = self.model.step(s, ControlCommand(accel=0.0, steer_rate=0.0), self.dt)
            out.append(s)
        return out

    def plan_step(
        self,
        ego: VehicleState,
        goal_xy: np.ndarray,
        goal_yaw: float,
        obstacles: list[Obstacle],
        dynamic_others: list[VehicleState],
    ) -> PlanningResult:
        others_pred = {o.vehicle_id: self._predict_other(o, self.planner_cfg.horizon_steps) for o in dynamic_others}

        best_cost = math.inf
        best_cmd = ControlCommand(accel=0.0, steer_rate=0.0)
        best_traj: list[VehicleState] = [ego.copy()]
        found = False

        # Nominal command steers toward goal and keeps moving.
        dx, dy = goal_xy[0] - ego.x, goal_xy[1] - ego.y
        desired_yaw = math.atan2(dy, dx)
        yaw_err = angle_diff(desired_yaw, ego.yaw)
        nominal_a = 0.8 if ego.v < 0.8 else 0.0
        nominal_sr = clamp(yaw_err / max(self.dt, 1e-3), -math.radians(20.0), math.radians(20.0))

        candidates: list[tuple[float, float]] = [(nominal_a, nominal_sr)]
        for a in self.planner_cfg.sample_accel:
            for sr_deg in self.planner_cfg.sample_steer_rate_deg:
                candidates.append((float(a), math.radians(sr_deg)))

        dist_start = float(np.linalg.norm(ego.xy() - goal_xy))

        for a, sr in candidates:
            s = ego.copy()
            traj = [s]
            feasible = True
            for k in range(self.planner_cfg.horizon_steps):
                s = self.model.step(s, ControlCommand(accel=a, steer_rate=sr), self.dt)
                traj.append(s)

                others_now = [pred[min(k, len(pred) - 1)] for pred in others_pred.values()]
                collide = False
                for obs in obstacles:
                    if self.collision_engine.collide_vehicle_obstacle(s, obs, include_clearance=False):
                        collide = True
                        break
                if not collide:
                    for o in others_now:
                        if self.collision_engine.collide_vehicle_vehicle(s, o, include_clearance=False):
                            collide = True
                            break
                if collide:
                    feasible = False
                    break

            if not feasible:
                continue

            final = traj[-1]
            dist_final = float(np.linalg.norm(final.xy() - goal_xy))
            dist_cost = dist_final
            desired_heading = math.atan2(goal_xy[1] - final.y, goal_xy[0] - final.x)
            if dist_final < 1.0:
                heading_ref = goal_yaw
            else:
                heading_ref = desired_heading
            heading_cost = abs(angle_diff(heading_ref, final.yaw))
            clearance = self.collision_engine.min_clearance_vehicle_obstacles(final, obstacles)
            clearance_cost = 0.0 if clearance > 5.0 else 1.0 / max(clearance, 1e-3)
            smooth_cost = abs(a) + abs(sr)
            progress = max(0.0, dist_start - dist_final)
            stall_penalty = max(0.0, 0.45 - final.v)

            cost = (
                self.planner_cfg.goal_weight * dist_cost
                + self.planner_cfg.heading_weight * heading_cost
                + self.planner_cfg.clearance_weight * clearance_cost
                + self.planner_cfg.smooth_weight * smooth_cost
                + 1.2 * stall_penalty
                - 1.5 * progress
            )

            if cost < best_cost:
                best_cost = cost
                best_cmd = ControlCommand(accel=float(a), steer_rate=float(sr))
                best_traj = traj
                found = True

        return PlanningResult(command=best_cmd, trajectory=best_traj, feasible=found, cost=float(best_cost))
