from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from core.utils import wrap_angle
from core.vehicle import AckermannVehicle, VehicleSpec, VehicleState
from env.world import WorldMap


@dataclass
class HybridNode:
    x: float
    y: float
    theta: float
    g: float
    f: float
    parent: Optional[Tuple[int, int, int]]
    steer: float


class HybridAStarPlanner:
    def __init__(
        self,
        world: WorldMap,
        spec: VehicleSpec,
        xy_resolution: float = 1.0,
        theta_bins: int = 72,
        step_len: float = 1.4,
        primitive_substeps: int = 5,
        max_expansions: int = 50000,
    ):
        self.world = world
        self.spec = spec
        self.xy_resolution = float(xy_resolution)
        self.theta_bins = int(theta_bins)
        self.step_len = float(step_len)
        self.primitive_substeps = int(primitive_substeps)
        self.max_expansions = int(max_expansions)

        self._steer_set = [-self.spec.max_steer, 0.0, self.spec.max_steer]
        self._nbrs2d = [
            (1, 0, self.world.resolution),
            (-1, 0, self.world.resolution),
            (0, 1, self.world.resolution),
            (0, -1, self.world.resolution),
            (1, 1, math.sqrt(2.0) * self.world.resolution),
            (1, -1, math.sqrt(2.0) * self.world.resolution),
            (-1, 1, math.sqrt(2.0) * self.world.resolution),
            (-1, -1, math.sqrt(2.0) * self.world.resolution),
        ]

    def _discretize(self, x: float, y: float, theta: float) -> Tuple[int, int, int]:
        ix = int(round(x / self.xy_resolution))
        iy = int(round(y / self.xy_resolution))
        th = wrap_angle(theta)
        it = int(round((th + math.pi) / (2.0 * math.pi) * self.theta_bins)) % self.theta_bins
        return ix, iy, it

    def _theta_center(self, it: int) -> float:
        return (it / self.theta_bins) * (2.0 * math.pi) - math.pi

    def _heuristic(
        self,
        x: float,
        y: float,
        theta: float,
        goal: VehicleState,
        holo_heuristic: Optional[np.ndarray],
    ) -> float:
        dp = math.hypot(goal.x - x, goal.y - y)
        dth = abs(wrap_angle(goal.theta - theta))
        h2d = dp
        if holo_heuristic is not None:
            gx, gy = self.world.world_to_grid(x, y)
            val = float(holo_heuristic[gy, gx])
            if np.isfinite(val):
                h2d = max(h2d, val)
            else:
                h2d = max(h2d, dp * 2.4)
        return h2d + 0.55 * dth

    def _augmented_occupancy(self, static_vehicles: Optional[Sequence[AckermannVehicle]]) -> np.ndarray:
        occ = self.world.inflated_occ.copy()
        if not static_vehicles:
            return occ

        res = self.world.resolution
        for veh in static_vehicles:
            circles, rad = veh.footprint_circles()
            rad_cells = int(np.ceil((rad + 0.85) / max(res, 1e-6)))
            for c in circles:
                gx, gy = self.world.world_to_grid(float(c[0]), float(c[1]))
                x0 = max(0, gx - rad_cells)
                x1 = min(self.world.width - 1, gx + rad_cells)
                y0 = max(0, gy - rad_cells)
                y1 = min(self.world.height - 1, gy + rad_cells)
                for ny in range(y0, y1 + 1):
                    dy = ny - gy
                    for nx in range(x0, x1 + 1):
                        dx = nx - gx
                        if dx * dx + dy * dy <= rad_cells * rad_cells:
                            occ[ny, nx] = 1
        return occ

    def _find_nearest_free(self, gx: int, gy: int, occ: np.ndarray, max_r: int = 8) -> Tuple[int, int]:
        h, w = occ.shape
        if 0 <= gx < w and 0 <= gy < h and occ[gy, gx] == 0:
            return gx, gy
        for r in range(1, max_r + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    nx, ny = gx + dx, gy + dy
                    if nx < 0 or nx >= w or ny < 0 or ny >= h:
                        continue
                    if occ[ny, nx] == 0:
                        return nx, ny
        return gx, gy

    def _compute_holonomic_heuristic(
        self,
        goal: VehicleState,
        static_vehicles: Optional[Sequence[AckermannVehicle]],
    ) -> Optional[np.ndarray]:
        occ = self._augmented_occupancy(static_vehicles)
        h, w = occ.shape
        inf = 1e15
        dist = np.full((h, w), inf, dtype=float)

        gx, gy = self.world.world_to_grid(goal.x, goal.y)
        gx, gy = self._find_nearest_free(gx, gy, occ, max_r=14)
        if gx < 0 or gx >= w or gy < 0 or gy >= h or occ[gy, gx] > 0:
            return None

        pq: List[Tuple[float, int, int]] = []
        dist[gy, gx] = 0.0
        heapq.heappush(pq, (0.0, gx, gy))

        while pq:
            cd, x, y = heapq.heappop(pq)
            if cd > dist[y, x] + 1e-9:
                continue
            for dx, dy, wgt in self._nbrs2d:
                nx, ny = x + dx, y + dy
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                if occ[ny, nx] > 0:
                    continue
                nd = cd + wgt
                if nd + 1e-9 < dist[ny, nx]:
                    dist[ny, nx] = nd
                    heapq.heappush(pq, (nd, nx, ny))

        return dist

    def _simulate_primitive(self, state: VehicleState, steer: float, forward_speed: float) -> VehicleState:
        x, y, th = state.x, state.y, state.theta
        ds = self.step_len / max(self.primitive_substeps, 1)
        dt = ds / max(abs(forward_speed), 1e-5)

        dummy = AckermannVehicle(self.spec, VehicleState(x=x, y=y, theta=th))
        for _ in range(self.primitive_substeps):
            dummy.step(v=forward_speed, steer=steer, dt=dt)
        return VehicleState(x=dummy.state.x, y=dummy.state.y, theta=dummy.state.theta)

    def _segment_collision(
        self,
        s0: VehicleState,
        s1: VehicleState,
        static_vehicles: Optional[Iterable[AckermannVehicle]],
    ) -> bool:
        samples = 4
        for k in range(samples + 1):
            t = k / samples
            x = (1.0 - t) * s0.x + t * s1.x
            y = (1.0 - t) * s0.y + t * s1.y
            theta = wrap_angle((1.0 - t) * s0.theta + t * s1.theta)
            if self.world.collision_for_pose(
                x=x,
                y=y,
                theta=theta,
                spec=self.spec,
                static_vehicles=static_vehicles,
            ):
                return True
        return False

    def plan(
        self,
        start: VehicleState,
        goal: VehicleState,
        static_vehicles: Optional[Sequence[AckermannVehicle]] = None,
        goal_pos_tol: float = 1.0,
        goal_theta_tol: float = 0.35,
    ) -> Optional[List[VehicleState]]:
        start_key = self._discretize(start.x, start.y, start.theta)

        open_heap: List[Tuple[float, Tuple[int, int, int]]] = []
        nodes: Dict[Tuple[int, int, int], HybridNode] = {}

        holo = self._compute_holonomic_heuristic(goal=goal, static_vehicles=static_vehicles)
        start_f = self._heuristic(start.x, start.y, start.theta, goal, holo)
        nodes[start_key] = HybridNode(
            x=start.x,
            y=start.y,
            theta=start.theta,
            g=0.0,
            f=start_f,
            parent=None,
            steer=0.0,
        )
        heapq.heappush(open_heap, (start_f, start_key))

        closed = set()
        expansions = 0

        while open_heap and expansions < self.max_expansions:
            _, key = heapq.heappop(open_heap)
            if key in closed:
                continue
            closed.add(key)
            expansions += 1

            cur = nodes[key]
            dx = cur.x - goal.x
            dy = cur.y - goal.y
            dth = abs(wrap_angle(cur.theta - goal.theta))
            if math.hypot(dx, dy) <= goal_pos_tol and dth <= goal_theta_tol:
                return self._reconstruct_path(nodes, key)

            for steer in self._steer_set:
                next_state = self._simulate_primitive(
                    VehicleState(x=cur.x, y=cur.y, theta=cur.theta),
                    steer=steer,
                    forward_speed=1.0,
                )

                if not self.world.in_bounds(next_state.x, next_state.y):
                    continue

                if self._segment_collision(
                    VehicleState(x=cur.x, y=cur.y, theta=cur.theta),
                    next_state,
                    static_vehicles,
                ):
                    continue

                nkey = self._discretize(next_state.x, next_state.y, next_state.theta)
                step_cost = self.step_len * (1.0 + 0.15 * abs(steer / max(self.spec.max_steer, 1e-6)))
                if abs(steer - cur.steer) > 0.4 * self.spec.max_steer:
                    step_cost += 0.2
                ng = cur.g + step_cost

                old = nodes.get(nkey)
                if old is not None and ng >= old.g - 1e-9:
                    continue

                nth = self._theta_center(nkey[2])
                h = self._heuristic(next_state.x, next_state.y, nth, goal, holo)
                node = HybridNode(
                    x=next_state.x,
                    y=next_state.y,
                    theta=next_state.theta,
                    g=ng,
                    f=ng + h,
                    parent=key,
                    steer=steer,
                )
                nodes[nkey] = node
                heapq.heappush(open_heap, (node.f, nkey))

        return None

    def _reconstruct_path(
        self,
        nodes: Dict[Tuple[int, int, int], HybridNode],
        key: Tuple[int, int, int],
    ) -> List[VehicleState]:
        rev: List[VehicleState] = []
        cur_key = key
        while True:
            n = nodes[cur_key]
            rev.append(VehicleState(x=n.x, y=n.y, theta=n.theta))
            if n.parent is None:
                break
            cur_key = n.parent
        rev.reverse()
        return rev
