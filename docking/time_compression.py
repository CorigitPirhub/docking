from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .collision import CollisionEngine
from .config import Config
from .kinematics import VehicleGeometry
from .math_utils import angle_diff
from .sensors import line_blocked_by_obstacles
from .types import Obstacle, VehicleState


def _path_length(path_xy: np.ndarray | None) -> float:
    if path_xy is None or len(path_xy) < 2:
        return math.inf
    return float(np.sum(np.linalg.norm(np.diff(path_xy, axis=0), axis=1)))


@dataclass(frozen=True)
class PlanCompressionCertificate:
    active: bool
    family_hint: str
    leader_shift_m: float
    follower_path_len_m: float
    follower_path_clearance_m: float
    follower_turn_deg: float
    los_blocked: bool
    stage_leader_scale: float
    stage_follower_scale: float
    approach_speed_mps: float
    reason: str


class PlanCompressionCertifier:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.collision = CollisionEngine(cfg.vehicle, cfg.safety)
        self.geom = VehicleGeometry(cfg.vehicle)

    def _sample_path_states(
        self,
        path_xy: np.ndarray,
        *,
        proto: VehicleState,
        goal_yaw: float,
        sample_step: float,
    ) -> list[VehicleState]:
        if path_xy is None or len(path_xy) == 0:
            return [proto.copy()]
        if len(path_xy) == 1:
            only = proto.copy()
            only.x = float(path_xy[0, 0])
            only.y = float(path_xy[0, 1])
            only.yaw = float(goal_yaw)
            return [only]
        out: list[VehicleState] = []
        prev_yaw = float(proto.yaw)
        first = True
        for idx in range(len(path_xy) - 1):
            p0 = np.asarray(path_xy[idx], dtype=float)
            p1 = np.asarray(path_xy[idx + 1], dtype=float)
            seg = p1 - p0
            seg_len = float(np.linalg.norm(seg))
            if seg_len <= 1e-9:
                continue
            yaw = float(math.atan2(float(seg[1]), float(seg[0])))
            n = max(1, int(math.ceil(seg_len / max(sample_step, 1e-3))))
            start_j = 0 if first else 1
            for j in range(start_j, n + 1):
                alpha = float(j / n)
                xy = (1.0 - alpha) * p0 + alpha * p1
                state = proto.copy()
                state.x = float(xy[0])
                state.y = float(xy[1])
                state.yaw = yaw if idx < len(path_xy) - 2 or alpha < 0.999 else float(goal_yaw)
                if abs(angle_diff(float(state.yaw), prev_yaw)) > math.pi:
                    state.yaw = prev_yaw
                prev_yaw = float(state.yaw)
                out.append(state)
            first = False
        if not out:
            out.append(proto.copy())
        return out

    def _path_min_clearance(
        self,
        path_xy: np.ndarray,
        *,
        proto: VehicleState,
        goal_yaw: float,
        obstacles: list[Obstacle],
    ) -> float:
        min_clear = float('inf')
        for state in self._sample_path_states(
            np.asarray(path_xy, dtype=float),
            proto=proto,
            goal_yaw=float(goal_yaw),
            sample_step=float(max(self.cfg.safety.swept_sample_distance, 0.16)),
        ):
            min_clear = min(min_clear, float(self.collision.min_clearance_vehicle_obstacles(state, obstacles)))
        if math.isinf(min_clear):
            return 1e6
        return float(min_clear)

    def _max_heading_change_deg(self, path_xy: np.ndarray) -> float:
        path = np.asarray(path_xy, dtype=float)
        if len(path) < 3:
            return 0.0
        headings: list[float] = []
        for idx in range(len(path) - 1):
            seg = path[idx + 1] - path[idx]
            if float(np.linalg.norm(seg)) <= 1e-9:
                continue
            headings.append(float(math.atan2(float(seg[1]), float(seg[0]))))
        max_turn = 0.0
        for a, b in zip(headings[:-1], headings[1:]):
            max_turn = max(max_turn, abs(float(angle_diff(float(b), float(a)))))
        return float(math.degrees(max_turn))

    def certify(
        self,
        *,
        leader_ref: VehicleState,
        follower_ref: VehicleState,
        leader_goal: VehicleState,
        follower_goal: VehicleState,
        leader_path_xy: np.ndarray,
        follower_path_xy: np.ndarray,
        obstacles: list[Obstacle],
    ) -> PlanCompressionCertificate:
        leader_shift = float(np.linalg.norm(leader_goal.xy() - leader_ref.xy()))
        follower_len = float(_path_length(follower_path_xy))
        follower_clear = float(self._path_min_clearance(follower_path_xy, proto=follower_ref, goal_yaw=float(follower_goal.yaw), obstacles=obstacles))
        follower_turn = float(self._max_heading_change_deg(follower_path_xy))
        los_blocked = bool(line_blocked_by_obstacles(self.geom.front_hitch(follower_ref), self.geom.rear_hitch(leader_ref), obstacles))
        d0 = float(np.linalg.norm(self.geom.front_hitch(follower_ref) - self.geom.rear_hitch(leader_ref)))

        family_hint = 'none'
        active = False
        stage_leader_scale = 1.0
        stage_follower_scale = 1.0
        approach_speed = 1.05
        reason = 'baseline'

        sc_open_mid = bool(
            (not los_blocked)
            and leader_shift <= 0.70
            and follower_len <= 7.0
            and follower_clear >= 0.35
            and follower_turn <= 35.0
        )
        sc_blocked_short = bool(
            los_blocked
            and leader_shift <= 0.70
            and follower_len <= 6.2
            and follower_clear >= 0.90
            and follower_turn <= 47.5
        )
        sc_blocked_mid = bool(
            los_blocked
            and leader_shift <= 0.85
            and 6.5 <= follower_len <= 8.8
            and follower_clear >= 0.55
            and follower_turn <= 47.5
        )
        ec_short = bool(
            0.9 <= leader_shift <= 2.05
            and d0 <= 8.8
            and follower_len <= 10.9
            and follower_clear >= 0.45
            and follower_turn <= 58.0
        )

        if sc_open_mid:
            family_hint = 'SC_open_mid'
            active = True
            stage_leader_scale = 1.24
            stage_follower_scale = 1.34
            approach_speed = 1.18
            reason = 'open_switching_corridor'
        elif sc_blocked_short:
            family_hint = 'SC_blocked_short'
            active = True
            stage_leader_scale = 1.22
            stage_follower_scale = 1.32
            approach_speed = 1.16
            reason = 'blocked_short_switching_corridor'
        elif sc_blocked_mid:
            family_hint = 'SC_blocked_mid'
            active = True
            stage_leader_scale = 1.18
            stage_follower_scale = 1.28
            approach_speed = 1.14
            reason = 'blocked_switching_corridor'
        elif ec_short:
            family_hint = 'EC_short'
            active = True
            stage_leader_scale = 1.18
            stage_follower_scale = 1.26
            approach_speed = 1.14
            reason = 'short_extension_visible_corridor'

        return PlanCompressionCertificate(
            active=bool(active),
            family_hint=str(family_hint),
            leader_shift_m=float(leader_shift),
            follower_path_len_m=float(follower_len),
            follower_path_clearance_m=float(follower_clear),
            follower_turn_deg=float(follower_turn),
            los_blocked=bool(los_blocked),
            stage_leader_scale=float(stage_leader_scale),
            stage_follower_scale=float(stage_follower_scale),
            approach_speed_mps=float(approach_speed),
            reason=str(reason),
        )
