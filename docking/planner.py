from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .collision import CollisionEngine, bbox_distance, obstacle_polygon, polygon_bbox, polygons_intersect
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
        *,
        force_goal_yaw: bool = False,
    ) -> PlanningResult:
        # Prepare static obstacles once per plan_step call to avoid re-allocating polygons/bboxes
        # for every candidate and horizon step.
        obs_prepared: list[tuple[np.ndarray, tuple[float, float, float, float]]] = []
        for obs in obstacles:
            poly = obstacle_polygon(obs)
            obs_prepared.append((poly, polygon_bbox(poly)))

        # Predict other vehicles and precompute their polygons/bboxes for fast SAT checks.
        others_prepared: dict[int, list[tuple[list[np.ndarray], list[tuple[float, float, float, float]]]]] = {}
        for other in dynamic_others:
            pred = self._predict_other(other, self.planner_cfg.horizon_steps)
            prep_steps: list[tuple[list[np.ndarray], list[tuple[float, float, float, float]]]] = []
            for st in pred:
                polys = self.collision_engine.vehicle_polygons(st)
                boxes = [polygon_bbox(p) for p in polys]
                prep_steps.append((polys, boxes))
            others_prepared[int(other.vehicle_id)] = prep_steps

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

                collide = False
                polys_s = self.collision_engine.vehicle_polygons(s)
                boxes_s = [polygon_bbox(p) for p in polys_s]

                # Static obstacle collision (exact polygon SAT, clearance excluded).
                for obs_poly, obs_box in obs_prepared:
                    for poly_s, box_s in zip(polys_s, boxes_s):
                        if bbox_distance(box_s, obs_box) > 0.0:
                            continue
                        if polygons_intersect(poly_s, obs_poly):
                            collide = True
                            break
                    if collide:
                        break

                # Dynamic other vehicles collision (exact polygon SAT, clearance excluded).
                if not collide and others_prepared:
                    for prep_steps in others_prepared.values():
                        idx = min(k, len(prep_steps) - 1)
                        polys_o, boxes_o = prep_steps[idx]
                        for poly_s, box_s in zip(polys_s, boxes_s):
                            for poly_o, box_o in zip(polys_o, boxes_o):
                                if bbox_distance(box_s, box_o) > 0.0:
                                    continue
                                if polygons_intersect(poly_s, poly_o):
                                    collide = True
                                    break
                            if collide:
                                break
                        if collide:
                            break
                if collide:
                    feasible = False
                    break

            if not feasible:
                continue

            final = traj[-1]
            dist_final = float(math.hypot(float(goal_xy[0] - final.x), float(goal_xy[1] - final.y)))
            dist_cost = dist_final
            desired_heading = math.atan2(goal_xy[1] - final.y, goal_xy[0] - final.x)
            if force_goal_yaw or dist_final < 1.0:
                heading_ref = goal_yaw
            else:
                heading_ref = desired_heading
            heading_cost = abs(angle_diff(heading_ref, final.yaw))
            # Clearance proxy for cost (safety is enforced by explicit collision checks above).
            # Use bbox distance to avoid expensive polygon-distance computations.
            polys_f = self.collision_engine.vehicle_polygons(final)
            boxes_f = [polygon_bbox(p) for p in polys_f]
            clearance = math.inf
            for _obs_poly, obs_box in obs_prepared:
                for box_f in boxes_f:
                    clearance = min(clearance, bbox_distance(box_f, obs_box))
            if math.isinf(clearance):
                clearance = 1e6
            clearance_cost = 0.0 if clearance > 5.0 else 1.0 / max(float(clearance), 1e-3)
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
