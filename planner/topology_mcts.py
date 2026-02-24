from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from core.utils import softmax, wrap_angle
from core.vehicle import AckermannVehicle
from planner.cost_map import CostMap, DockingHub


@dataclass
class DockingDecision:
    hub: DockingHub
    sequence: List[str]
    predicted_cost: float


class DockingCostModel:
    def __init__(self, cost_map: CostMap, trailer_pitch: float = 5.2):
        self.cost_map = cost_map
        self.trailer_pitch = trailer_pitch

    def target_socket(self, hub: DockingHub, step: int) -> np.ndarray:
        h = np.array([math.cos(hub.heading), math.sin(hub.heading)], dtype=float)
        return np.array([hub.x, hub.y], dtype=float) - step * self.trailer_pitch * h

    def action_cost(self, veh: AckermannVehicle, hub: DockingHub, step: int, chain_len: int) -> float:
        target = self.target_socket(hub, step)
        pos = np.array([veh.state.x, veh.state.y], dtype=float)
        dist = float(np.linalg.norm(pos - target))
        risk = self.cost_map.line_risk(pos, target)
        heading_pen = abs(wrap_angle(veh.state.theta - hub.heading))
        open_pen = 1.0 / max(hub.openness + 0.08, 0.08)
        chain_pen = 0.3 * chain_len / max(hub.corridor_clearance, 1.0)
        return 1.0 * dist + 12.0 * risk + 2.5 * heading_pen + 2.0 * open_pen + chain_pen


class TeacherSignal:
    """Heuristic prior injected into MCTS expansion as imitation teacher signal."""

    def __init__(self, cost_model: DockingCostModel):
        self.cost_model = cost_model

    def action_prior(
        self,
        remaining: Sequence[str],
        step: int,
        hub: DockingHub,
        chain_len: int,
        vehicles: Dict[str, AckermannVehicle],
    ) -> Dict[str, float]:
        scores = []
        for vid in remaining:
            c = self.cost_model.action_cost(vehicles[vid], hub, step, chain_len)
            scores.append(-c)
        probs = softmax(scores, temperature=2.5)
        return {vid: p for vid, p in zip(remaining, probs)}


class _Node:
    def __init__(self, remaining: Tuple[str, ...], step: int):
        self.remaining = remaining
        self.step = step
        self.visits = 0
        self.value_sum = 0.0
        self.children: Dict[str, _Node] = {}
        self.priors: Dict[str, float] = {}

    def q_value(self) -> float:
        if self.visits == 0:
            return 0.0
        return self.value_sum / self.visits


class TopologyMCTSPlanner:
    def __init__(
        self,
        cost_map: CostMap,
        mcts_iterations_per_hub: int = 260,
        c_puct: float = 1.2,
        seed: int = 7,
        min_hub_openness: float = 0.35,
    ):
        self.cost_map = cost_map
        self.mcts_iterations_per_hub = int(mcts_iterations_per_hub)
        self.c_puct = float(c_puct)
        self.rng = np.random.default_rng(seed)
        self.min_hub_openness = float(min_hub_openness)

        self.cost_model = DockingCostModel(cost_map)
        self.teacher = TeacherSignal(self.cost_model)

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
            fallback = DockingHub(
                x=50.0,
                y=50.0,
                heading=0.0,
                openness=0.5,
                corridor_clearance=2.0,
                score=0.1,
            )
            hubs = [fallback]

        vehicles = {v.spec.vehicle_id: v for v in trailers}
        best: Optional[DockingDecision] = None

        for hub in hubs:
            seq, cost = self._search_sequence_for_hub(hub, head, trailers, vehicles)
            # Additional explicit travel estimate keeps hubs near the feasible workspace.
            h = np.array([math.cos(hub.heading), math.sin(hub.heading)], dtype=float)
            travel = float(np.linalg.norm(head.rear_port() - np.array([hub.x, hub.y], dtype=float)))
            for k, vid in enumerate(seq):
                target = np.array([hub.x, hub.y], dtype=float) - k * self.cost_model.trailer_pitch * h
                travel += float(np.linalg.norm(np.array([vehicles[vid].state.x, vehicles[vid].state.y]) - target))

            objective = (
                cost
                + 0.25 * travel
                - 8.5 * hub.openness
                - 1.6 * min(hub.corridor_clearance, 6.0)
            )
            if best is None or objective < best.predicted_cost:
                best = DockingDecision(hub=hub, sequence=seq, predicted_cost=objective)

        assert best is not None
        return best

    def _search_sequence_for_hub(
        self,
        hub: DockingHub,
        head: AckermannVehicle,
        trailers: Sequence[AckermannVehicle],
        vehicles: Dict[str, AckermannVehicle],
    ) -> Tuple[List[str], float]:
        trailer_ids = tuple(v.spec.vehicle_id for v in trailers)
        root = _Node(remaining=tuple(sorted(trailer_ids)), step=0)

        for _ in range(self.mcts_iterations_per_hub):
            self._run_iteration(root, hub, vehicles, chain_len=1)

        seq = []
        node = root
        while node.remaining:
            if not node.children:
                self._expand(node, hub, vehicles, chain_len=1 + node.step)
            if not node.children:
                break
            best_action = max(node.children.keys(), key=lambda a: node.children[a].visits)
            seq.append(best_action)
            node = node.children[best_action]

        if len(seq) < len(trailer_ids):
            left = [vid for vid in trailer_ids if vid not in seq]
            left.sort(key=lambda vid: self.cost_model.action_cost(vehicles[vid], hub, step=len(seq), chain_len=1 + len(seq)))
            seq.extend(left)

        # Estimate sequence cost directly.
        total = 0.0
        for i, vid in enumerate(seq):
            total += self.cost_model.action_cost(vehicles[vid], hub, step=i, chain_len=1 + i)

        # Add head repositioning cost (rear-port to anchor).
        head_rear = head.rear_port()
        total += 0.8 * float(np.linalg.norm(head_rear - np.array([hub.x, hub.y], dtype=float)))
        return seq, total

    def _run_iteration(
        self,
        root: _Node,
        hub: DockingHub,
        vehicles: Dict[str, AckermannVehicle],
        chain_len: int,
    ) -> None:
        path_nodes = [root]
        path_actions: List[str] = []
        node = root
        rollout_cost = 0.0

        while node.remaining:
            if not node.children:
                self._expand(node, hub, vehicles, chain_len + node.step)
                break
            action, child = self._select(node)
            rollout_cost += self.cost_model.action_cost(
                vehicles[action], hub, step=node.step, chain_len=chain_len + node.step
            )
            path_actions.append(action)
            path_nodes.append(child)
            node = child

        if node.remaining:
            # One-step expansion action for this iteration.
            if node.children:
                action, child = self._select(node)
                rollout_cost += self.cost_model.action_cost(
                    vehicles[action], hub, step=node.step, chain_len=chain_len + node.step
                )
                path_actions.append(action)
                path_nodes.append(child)
                node = child

        # Rollout with greedy policy.
        remaining = list(node.remaining)
        step = node.step
        while remaining:
            costs = [
                self.cost_model.action_cost(
                    vehicles[vid], hub, step=step, chain_len=chain_len + step
                )
                for vid in remaining
            ]
            best_idx = int(np.argmin(costs))
            rollout_cost += costs[best_idx]
            remaining.pop(best_idx)
            step += 1

        reward = -rollout_cost
        for n in path_nodes:
            n.visits += 1
            n.value_sum += reward

    def _expand(self, node: _Node, hub: DockingHub, vehicles: Dict[str, AckermannVehicle], chain_len: int) -> None:
        remaining = list(node.remaining)
        if not remaining:
            return
        priors = self.teacher.action_prior(
            remaining=remaining,
            step=node.step,
            hub=hub,
            chain_len=chain_len,
            vehicles=vehicles,
        )
        node.priors = priors
        for vid in remaining:
            next_remaining = tuple(sorted([x for x in node.remaining if x != vid]))
            node.children[vid] = _Node(remaining=next_remaining, step=node.step + 1)

    def _select(self, node: _Node) -> Tuple[str, _Node]:
        best_action = None
        best_score = -1e18
        sqrt_parent = math.sqrt(max(node.visits, 1))

        for action, child in node.children.items():
            q = child.q_value()
            p = node.priors.get(action, 1.0 / max(len(node.children), 1))
            u = self.c_puct * p * sqrt_parent / (1 + child.visits)
            score = q + u
            if score > best_score:
                best_score = score
                best_action = action

        assert best_action is not None
        return best_action, node.children[best_action]
