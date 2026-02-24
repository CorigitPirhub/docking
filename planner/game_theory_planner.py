from __future__ import annotations

import math
import itertools
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from core.vehicle import AckermannVehicle, VehicleState
from planner.cost_map import CostMap, DockingHub
from planner.hybrid_astar import HybridAStarPlanner
from planner.topology_mcts import DockingCostModel, DockingDecision


@dataclass
class GamePlannerDebug:
    rounds: int
    converged: bool
    potential: float
    optimal_potential: float
    optimality_gap: float


class PotentialGameDockingPlanner:
    """
    Distributed best-response planner in a finite exact-potential game.

    Each trailer is treated as an agent choosing a slot index in the docking queue.
    Global potential combines individual docking costs and pairwise topological
    interference penalties (segment crossing + corridor conflict).
    """

    def __init__(
        self,
        cost_map: CostMap,
        max_rounds: int = 18,
        seed: int = 7,
        min_hub_openness: float = 0.35,
        lambda_cross: float = 5.0,
        lambda_conflict: float = 2.5,
        lambda_order_reg: float = 0.55,
        head_reachability_penalty: float = 35.0,
        head_probe_max_expansions: int = 3000,
        max_head_probes: int = 2,
    ):
        self.cost_map = cost_map
        self.max_rounds = int(max_rounds)
        self.rng = np.random.default_rng(seed)
        self.min_hub_openness = float(min_hub_openness)

        self.lambda_cross = float(lambda_cross)
        self.lambda_conflict = float(lambda_conflict)
        self.lambda_order_reg = float(lambda_order_reg)
        self.head_reachability_penalty = float(head_reachability_penalty)
        self.head_probe_max_expansions = int(head_probe_max_expansions)
        self.max_head_probes = int(max_head_probes)

        self.cost_model = DockingCostModel(cost_map)
        self.last_debug: Optional[GamePlannerDebug] = None

    def plan(
        self,
        head: AckermannVehicle,
        trailers: Sequence[AckermannVehicle],
        n_hub_samples: int = 140,
        top_k_hubs: int = 7,
    ) -> DockingDecision:
        hubs = self.cost_map.sample_docking_hubs(
            n_point_samples=n_hub_samples,
            max_hubs=top_k_hubs,
            required_corridor_len=22.0,
            rng=self.rng,
            min_openness=self.min_hub_openness,
        )
        if not hubs:
            hubs = [
                DockingHub(
                    x=50.0,
                    y=50.0,
                    heading=0.0,
                    openness=0.5,
                    corridor_clearance=2.0,
                    score=0.1,
                )
            ]

        vehicles = {v.spec.vehicle_id: v for v in trailers}
        static_for_head = list(trailers)
        head_probe_planner = HybridAStarPlanner(
            world=self.cost_map.world,
            spec=head.spec,
            xy_resolution=1.4,
            theta_bins=56,
            step_len=1.9,
            primitive_substeps=4,
            max_expansions=self.head_probe_max_expansions,
        )

        candidates: List[Tuple[float, DockingHub, List[str], GamePlannerDebug]] = []

        for hub in hubs:
            init_order = self._initial_order(hub, vehicles)
            order, rounds, converged = self._best_response(hub, vehicles, init_order)

            pot = self._potential(hub, order, vehicles)
            opt_order, opt_pot = self._global_optimum(hub, list(vehicles.keys()), vehicles)

            hvec = np.array([math.cos(hub.heading), math.sin(hub.heading)], dtype=float)
            travel = float(np.linalg.norm(head.rear_port() - np.array([hub.x, hub.y], dtype=float)))
            for k, vid in enumerate(order):
                target = np.array([hub.x, hub.y], dtype=float) - k * self.cost_model.trailer_pitch * hvec
                travel += float(np.linalg.norm(np.array([vehicles[vid].state.x, vehicles[vid].state.y]) - target))

            objective = pot + 0.20 * travel - 9.0 * hub.openness - 1.6 * min(hub.corridor_clearance, 6.0)
            gap = (pot - opt_pot) / max(abs(opt_pot), 1e-6)

            dbg = GamePlannerDebug(
                rounds=rounds,
                converged=converged,
                potential=float(pot),
                optimal_potential=float(opt_pot),
                optimality_gap=float(gap),
            )

            candidates.append((float(objective), hub, order, dbg))

        candidates.sort(key=lambda x: x[0])
        best: Optional[DockingDecision] = None
        best_debug: Optional[GamePlannerDebug] = None

        probe_budget = max(1, min(self.max_head_probes, len(candidates)))
        for i in range(probe_budget):
            base_obj, hub, order, dbg = candidates[i]
            head_pen = self._head_reachability_penalty_for_hub(
                head=head,
                hub=hub,
                static_vehicles=static_for_head,
                planner=head_probe_planner,
            )
            objective = base_obj + head_pen
            if best is None or objective < best.predicted_cost:
                best = DockingDecision(hub=hub, sequence=order, predicted_cost=float(objective))
                best_debug = dbg
            if head_pen <= 1e-9:
                break

        if best is None:
            base_obj, hub, order, dbg = candidates[0]
            best = DockingDecision(hub=hub, sequence=order, predicted_cost=float(base_obj))
            best_debug = dbg

        self.last_debug = best_debug
        return best

    def _head_reachability_penalty_for_hub(
        self,
        head: AckermannVehicle,
        hub: DockingHub,
        static_vehicles: Sequence[AckermannVehicle],
        planner: HybridAStarPlanner,
    ) -> float:
        hub_dir = np.array([math.cos(hub.heading), math.sin(hub.heading)], dtype=float)
        head_goal_center = np.array([hub.x, hub.y], dtype=float) + head.spec.rear_port_offset * hub_dir
        goal = VehicleState(x=float(head_goal_center[0]), y=float(head_goal_center[1]), theta=hub.heading)
        path = planner.plan(start=head.state, goal=goal, static_vehicles=static_vehicles)
        if path is None or len(path) < 2:
            return self.head_reachability_penalty
        return 0.0

    def _initial_order(self, hub: DockingHub, vehicles: Dict[str, AckermannVehicle]) -> List[str]:
        remaining = set(vehicles.keys())
        order: List[str] = []
        step = 0
        while remaining:
            best_vid = min(
                remaining,
                key=lambda vid: self.cost_model.action_cost(
                    vehicles[vid], hub, step=step, chain_len=1 + step
                ),
            )
            order.append(best_vid)
            remaining.remove(best_vid)
            step += 1
        return order

    def _best_response(
        self,
        hub: DockingHub,
        vehicles: Dict[str, AckermannVehicle],
        init_order: Sequence[str],
    ) -> Tuple[List[str], int, bool]:
        order = list(init_order)
        n = len(order)
        if n <= 1:
            return order, 0, True

        converged = False
        for r in range(1, self.max_rounds + 1):
            improved = False
            agent_ids = list(order)
            self.rng.shuffle(agent_ids)

            for vid in agent_ids:
                cur_idx = order.index(vid)
                best_idx = cur_idx
                best_pot = self._potential(hub, order, vehicles)

                for k in range(n):
                    if k == cur_idx:
                        continue
                    cand = list(order)
                    cand[cur_idx], cand[k] = cand[k], cand[cur_idx]
                    pot = self._potential(hub, cand, vehicles)
                    if pot + 1e-9 < best_pot:
                        best_pot = pot
                        best_idx = k

                if best_idx != cur_idx:
                    order[cur_idx], order[best_idx] = order[best_idx], order[cur_idx]
                    improved = True

            if not improved:
                converged = True
                return order, r, converged

        return order, self.max_rounds, converged

    def _potential(self, hub: DockingHub, order: Sequence[str], vehicles: Dict[str, AckermannVehicle]) -> float:
        base = 0.0
        for step, vid in enumerate(order):
            base += self.cost_model.action_cost(vehicles[vid], hub, step=step, chain_len=1 + step)

        # pairwise interaction term: segment crossing + corridor conflict
        pair_term = 0.0
        segs = []
        for step, vid in enumerate(order):
            src = np.array([vehicles[vid].state.x, vehicles[vid].state.y], dtype=float)
            target = self.cost_model.target_socket(hub, step)
            segs.append((src, target, vid, step))

        for i in range(len(segs)):
            p0, p1, vi, si = segs[i]
            for j in range(i + 1, len(segs)):
                q0, q1, vj, sj = segs[j]
                if self._segments_intersect(p0, p1, q0, q1):
                    pair_term += self.lambda_cross
                min_dist = self._segment_to_segment_min_dist(p0, p1, q0, q1)
                if min_dist < 4.0:
                    pair_term += self.lambda_conflict * (4.0 - min_dist) / 4.0

                # Soft regularization toward cost-based prior order to avoid oscillations.
                if abs(si - sj) == 1:
                    ci = self.cost_model.action_cost(vehicles[vi], hub, step=si, chain_len=1 + si)
                    cj = self.cost_model.action_cost(vehicles[vj], hub, step=sj, chain_len=1 + sj)
                    if ci > cj:
                        pair_term += self.lambda_order_reg * 0.2

        return float(base + pair_term)

    def _global_optimum(
        self,
        hub: DockingHub,
        trailer_ids: Sequence[str],
        vehicles: Dict[str, AckermannVehicle],
    ) -> Tuple[List[str], float]:
        best_perm = None
        best_pot = 1e18
        for perm in itertools.permutations(trailer_ids):
            pot = self._potential(hub, perm, vehicles)
            if pot < best_pot:
                best_pot = pot
                best_perm = list(perm)
        assert best_perm is not None
        return best_perm, float(best_pot)

    @staticmethod
    def _segments_intersect(p1: np.ndarray, p2: np.ndarray, q1: np.ndarray, q2: np.ndarray) -> bool:
        def orient(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
            return float((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))

        o1 = orient(p1, p2, q1)
        o2 = orient(p1, p2, q2)
        o3 = orient(q1, q2, p1)
        o4 = orient(q1, q2, p2)

        return (o1 * o2 < 0.0) and (o3 * o4 < 0.0)

    @staticmethod
    def _segment_to_segment_min_dist(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray) -> float:
        # Sampled approximation is sufficient for queue-coupling term.
        m = 16
        best = 1e18
        for i in range(m + 1):
            t = i / m
            pa = (1.0 - t) * a0 + t * a1
            for j in range(m + 1):
                s = j / m
                pb = (1.0 - s) * b0 + s * b1
                d = float(np.linalg.norm(pa - pb))
                if d < best:
                    best = d
        return best
