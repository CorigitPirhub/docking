from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def estimate_travel_time(distance_m: float, speed_now: float, v_max: float, a_max: float) -> float:
    """Acceleration-limited travel-time estimate."""
    d = max(0.0, float(distance_m))
    if d <= 1e-9:
        return 0.0
    v0 = max(0.0, float(speed_now))
    vmax = max(1e-6, float(v_max))
    a = max(1e-6, float(a_max))
    if v0 >= vmax:
        return d / vmax

    t_acc = (vmax - v0) / a
    d_acc = (v0 + vmax) * 0.5 * t_acc
    if d_acc >= d:
        return (-v0 + math.sqrt(max(v0 * v0 + 2.0 * a * d, 0.0))) / a
    return t_acc + (d - d_acc) / vmax


@dataclass(frozen=True)
class EnergyModelConfig:
    # Nominal model for energy estimation (can be replaced by eta(n, kappa, v) in P5).
    eta_nominal: float = 0.95
    eta_min: float = 0.90
    eta_max: float = 0.995
    curvature_penalty_gain: float = 0.0
    speed_penalty_gain: float = 0.0
    speed_ref_mps: float = 1.4
    platoon_bonus_per_unit: float = 0.0032
    single_drag_gain: float = 0.06
    single_energy_per_m: float = 1.0

    def eta(self, train_size: int, kappa_abs: float, speed_mps: float) -> float:
        n = max(1, int(train_size))
        base = clamp(float(self.eta_nominal), self.eta_min, self.eta_max)
        platoon_bonus = self.platoon_bonus_per_unit * max(0, n - 2)
        curvature_penalty = self.curvature_penalty_gain * clamp(float(kappa_abs), 0.0, 1.0)
        speed_ratio = abs(float(speed_mps) - self.speed_ref_mps) / max(self.speed_ref_mps, 1e-6)
        speed_penalty = self.speed_penalty_gain * clamp(speed_ratio, 0.0, 1.0)
        return clamp(base - platoon_bonus + curvature_penalty + speed_penalty, self.eta_min, self.eta_max)

    def single_energy(self, distance_m: float, speed_mps: float) -> float:
        d = max(0.0, float(distance_m))
        v = max(0.0, float(speed_mps))
        drag = 1.0 + self.single_drag_gain * (v / max(self.speed_ref_mps, 1e-6)) ** 2
        return self.single_energy_per_m * d * drag

    def train_energy(self, distance_m: float, train_size: int, kappa_abs: float, speed_mps: float) -> float:
        return self.single_energy(distance_m, speed_mps) * self.eta(train_size=train_size, kappa_abs=kappa_abs, speed_mps=speed_mps)


@dataclass(frozen=True)
class SequenceCostWeights:
    w_catch: float = 1.0
    w_dock: float = 0.9
    w_energy: float = 2.4
    w_risk: float = 1.1
    w_switch: float = 0.35


@dataclass(frozen=True)
class SequenceCostBreakdown:
    t_catch: float
    t_dock: float
    delta_energy: float
    risk: float
    n_switch: float
    total_cost: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def compute_j_seq(
    *,
    t_catch_s: float,
    t_dock_s: float,
    delta_energy: float,
    risk: float,
    n_switch: float,
    weights: SequenceCostWeights,
) -> SequenceCostBreakdown:
    tc = max(0.0, float(t_catch_s))
    td = max(0.0, float(t_dock_s))
    de = float(delta_energy)
    rr = max(0.0, float(risk))
    ns = max(0.0, float(n_switch))
    total = (
        weights.w_catch * tc
        + weights.w_dock * td
        + weights.w_energy * de
        + weights.w_risk * rr
        + weights.w_switch * ns
    )
    return SequenceCostBreakdown(
        t_catch=tc,
        t_dock=td,
        delta_energy=de,
        risk=rr,
        n_switch=ns,
        total_cost=float(total),
    )


def risk_score(
    *,
    occlusion_level: str,
    sync_error_s: float,
    sync_tolerance_s: float,
    nmax_margin: int,
) -> float:
    occ_map = {"low": 0.08, "medium": 0.22, "high": 0.45}
    occ = occ_map.get(str(occlusion_level), 0.20)
    sync = clamp(abs(float(sync_error_s)) / max(float(sync_tolerance_s), 1e-6), 0.0, 2.0)
    margin_penalty = 0.0 if nmax_margin >= 1 else 0.6 * abs(float(nmax_margin))
    return float(occ + 0.7 * sync + margin_penalty)


def recovery_risk_score(
    *,
    occlusion_level: str,
    nmax_margin: int,
    sync_error_s: float,
    sync_tolerance_s: float,
) -> float:
    """P4 recovery risk proxy (kept separate from P2 sequence risk weights)."""
    occ_map = {"low": 0.10, "medium": 0.24, "high": 0.44}
    occ = occ_map.get(str(occlusion_level), 0.2)
    sync = abs(float(sync_error_s)) / max(float(sync_tolerance_s), 1e-6)
    margin_penalty = 0.0 if int(nmax_margin) >= 1 else 0.5 * abs(float(nmax_margin))
    return float(occ + 0.6 * clamp(sync, 0.0, 2.0) + margin_penalty)


@dataclass(frozen=True)
class ActionCostWeights:
    w_time: float = 1.0
    w_energy: float = 1.0
    w_risk: float = 1.0


@dataclass(frozen=True)
class ActionCost:
    feasible: bool
    time_s: float
    energy_delta: float
    risk: float
    total_cost: float
    breakdown: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["breakdown"] = dict(self.breakdown)
        return d


def docking_action_cost(
    *,
    feasible: bool,
    t_catch_s: float,
    t_dock_s: float,
    delta_energy: float,
    risk: float,
    n_switch: float,
    weights: SequenceCostWeights,
) -> ActionCost:
    br = compute_j_seq(
        t_catch_s=t_catch_s,
        t_dock_s=t_dock_s,
        delta_energy=delta_energy,
        risk=risk,
        n_switch=n_switch,
        weights=weights,
    )
    return ActionCost(
        feasible=bool(feasible),
        time_s=float(br.t_catch + br.t_dock),
        energy_delta=float(br.delta_energy),
        risk=float(br.risk),
        total_cost=float(br.total_cost),
        breakdown={
            "t_catch": float(br.t_catch),
            "t_dock": float(br.t_dock),
            "n_switch": float(br.n_switch),
        },
    )


def split_action_cost(
    *,
    feasible: bool,
    time_s: float = 0.25,
    energy_delta: float = 0.0,
    risk: float = 0.0,
    weights: ActionCostWeights | None = None,
) -> ActionCost:
    w = weights or ActionCostWeights()
    total = w.w_time * float(time_s) + w.w_energy * float(energy_delta) + w.w_risk * float(risk)
    return ActionCost(
        feasible=bool(feasible),
        time_s=float(max(0.0, time_s)),
        energy_delta=float(energy_delta),
        risk=float(max(0.0, risk)),
        total_cost=float(total),
        breakdown={"kind": "split"},
    )


def wait_action_cost(
    *,
    feasible: bool,
    duration_s: float,
    energy_delta: float = 0.0,
    risk: float = 0.0,
    weights: ActionCostWeights | None = None,
) -> ActionCost:
    w = weights or ActionCostWeights()
    t = max(0.0, float(duration_s))
    total = w.w_time * t + w.w_energy * float(energy_delta) + w.w_risk * float(risk)
    return ActionCost(
        feasible=bool(feasible),
        time_s=float(t),
        energy_delta=float(energy_delta),
        risk=float(max(0.0, risk)),
        total_cost=float(total),
        breakdown={"kind": "wait", "duration_s": float(t)},
    )


def local_planning_action_cost(
    *,
    feasible: bool,
    horizon_time_s: float,
    planner_cost: float,
    min_clearance_m: float,
    weights: ActionCostWeights | None = None,
) -> ActionCost:
    w = weights or ActionCostWeights()
    t = max(0.0, float(horizon_time_s))
    clearance = max(0.0, float(min_clearance_m))
    # Interpret planner-cost as a composite "risk" term and add explicit clearance penalty.
    risk = max(0.0, float(planner_cost)) + 1.0 / max(clearance, 1e-3)
    total = w.w_time * t + w.w_risk * risk
    return ActionCost(
        feasible=bool(feasible),
        time_s=float(t),
        energy_delta=0.0,
        risk=float(risk),
        total_cost=float(total),
        breakdown={
            "kind": "local_plan",
            "planner_cost": float(planner_cost),
            "min_clearance_m": float(min_clearance_m),
        },
    )


def train_follow_action_cost(
    *,
    feasible: bool,
    distance_m: float,
    train_size: int,
    kappa_abs: float,
    speed_mps: float,
    energy_cfg: EnergyModelConfig,
    risk: float = 0.0,
    weights: ActionCostWeights | None = None,
) -> ActionCost:
    w = weights or ActionCostWeights()
    d = max(0.0, float(distance_m))
    n = max(1, int(train_size))
    k = max(0.0, float(kappa_abs))
    v = max(0.0, float(speed_mps))
    energy = float(energy_cfg.train_energy(d, n, k, v))
    t = 0.0 if v <= 1e-9 else float(d / v)
    rr = max(0.0, float(risk))
    total = w.w_time * t + w.w_energy * energy + w.w_risk * rr
    return ActionCost(
        feasible=bool(feasible),
        time_s=float(t),
        energy_delta=float(energy),
        risk=float(rr),
        total_cost=float(total),
        breakdown={
            "kind": "train_follow",
            "distance_m": float(d),
            "train_size": int(n),
            "kappa_abs": float(k),
            "speed_mps": float(v),
        },
    )
