from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

from core.utils import cross2d, heading, perp, wrap_angle
from core.vehicle import AckermannVehicle


@dataclass
class JointState:
    predecessor_id: str
    follower_id: str
    angle: float
    position_error: float


class ArticulatedTrain:
    """
    Chain order: [R1, Rk, ...], each follower front-port locks to predecessor rear-port.
    """

    def __init__(self, vehicles: List[AckermannVehicle]):
        if len(vehicles) == 0:
            raise ValueError("train requires at least one vehicle")
        self.vehicles: List[AckermannVehicle] = vehicles
        self._prev_hitch: Dict[Tuple[str, str], np.ndarray] = {}
        self._last_joint_states: List[JointState] = []

    @property
    def head(self) -> AckermannVehicle:
        return self.vehicles[0]

    @property
    def tail(self) -> AckermannVehicle:
        return self.vehicles[-1]

    def vehicle_ids(self) -> List[str]:
        return [v.spec.vehicle_id for v in self.vehicles]

    def attach_tail(self, veh: AckermannVehicle) -> None:
        veh.attached = True
        pred = self.tail
        hitch = pred.rear_port().copy()
        h = heading(pred.state.theta)
        center = hitch - veh.spec.front_port_offset * h
        veh.set_pose(float(center[0]), float(center[1]), pred.state.theta)

        self._prev_hitch[(pred.spec.vehicle_id, veh.spec.vehicle_id)] = hitch
        self.vehicles.append(veh)

    def step_head(self, v: float, steer: float, dt: float) -> None:
        self.head.step(v=v, steer=steer, dt=dt)
        self._propagate_followers(dt)

    def _propagate_followers(self, dt: float) -> None:
        self._last_joint_states = []

        for i in range(1, len(self.vehicles)):
            pred = self.vehicles[i - 1]
            fol = self.vehicles[i]
            joint_key = (pred.spec.vehicle_id, fol.spec.vehicle_id)

            hitch_target = pred.rear_port().copy()
            prev_hitch = self._prev_hitch.get(joint_key, hitch_target.copy())
            hitch_vel = (hitch_target - prev_hitch) / max(dt, 1e-6)

            h_old = heading(fol.state.theta)
            h_perp = perp(h_old)
            lf = max(fol.spec.front_port_offset, 1e-3)

            yaw_rate = float(np.dot(hitch_vel, h_perp) / lf)
            theta_new = wrap_angle(fol.state.theta + yaw_rate * dt)
            h_new = heading(theta_new)

            center_new = hitch_target - lf * h_new
            fol.set_pose(float(center_new[0]), float(center_new[1]), theta_new)

            hitch_current = fol.front_port()
            pos_err = float(np.linalg.norm(hitch_current - hitch_target))
            articulation = wrap_angle(pred.state.theta - fol.state.theta)
            self._last_joint_states.append(
                JointState(
                    predecessor_id=pred.spec.vehicle_id,
                    follower_id=fol.spec.vehicle_id,
                    angle=articulation,
                    position_error=pos_err,
                )
            )

            self._prev_hitch[joint_key] = hitch_target

    def get_joint_states(self) -> List[JointState]:
        return list(self._last_joint_states)

    def place_tail_socket(self, x: float, y: float, theta: float) -> None:
        """Place the whole straight train by fixing tail rear socket at (x, y)."""
        h = heading(theta)
        cursor = np.array([x, y], dtype=float)

        for veh in reversed(self.vehicles):
            center = cursor + veh.spec.rear_port_offset * h
            veh.set_pose(float(center[0]), float(center[1]), theta)
            cursor = veh.front_port()
