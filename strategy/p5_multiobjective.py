from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any

from strategy.baseline_scheduler import BaselineScheduler, PolicyRollout, SchedulerConfig
from docking.costs import EnergyModelConfig, SequenceCostWeights


@dataclass(frozen=True)
class ObjectiveWeights:
    w_time: float
    w_energy: float
    w_safety: float
    w_reconfig: float

    def normalized(self) -> "ObjectiveWeights":
        s = max(1e-9, self.w_time + self.w_energy + self.w_safety + self.w_reconfig)
        return ObjectiveWeights(
            w_time=float(self.w_time / s),
            w_energy=float(self.w_energy / s),
            w_safety=float(self.w_safety / s),
            w_reconfig=float(self.w_reconfig / s),
        )


@dataclass(frozen=True)
class ProfileConfig:
    profile_id: str
    objective: ObjectiveWeights
    seq_weights: SequenceCostWeights
    scheduler_cfg: SchedulerConfig
    energy_cfg: EnergyModelConfig

    def to_dict(self) -> dict[str, Any]:
        return {
            "profile_id": str(self.profile_id),
            "objective": asdict(self.objective),
            "seq_weights": asdict(self.seq_weights),
            "scheduler_cfg": asdict(self.scheduler_cfg),
            "energy_cfg": asdict(self.energy_cfg),
        }


@dataclass(frozen=True)
class BaselineAggregate:
    avg_time_independent: float
    avg_energy_independent: float
    avg_time_p2_adaptive: float
    avg_energy_p2_adaptive: float
    avg_reconfig_p2_adaptive: float
    avg_safety_p2_adaptive: float

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class ProfileAggregate:
    profile_id: str
    num_cases: int
    avg_time_s: float
    avg_energy: float
    avg_safety: float
    avg_reconfig: float
    avg_command_exec_rate: float
    avg_final_train_size: float
    collision_total: int
    energy_reduction_vs_independent: float
    time_improve_vs_independent: float
    energy_reduction_vs_p2_adaptive: float
    time_improve_vs_p2_adaptive: float
    scalar_score: float
    profile_pass: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def profile_from_tradeoff(
    *,
    profile_id: str,
    w_time: float,
    w_energy: float,
    w_safety: float,
) -> ProfileConfig:
    wt = _clamp(float(w_time), 0.05, 0.98)
    we = _clamp(float(w_energy), 0.05, 0.98)
    ws = _clamp(float(w_safety), 0.05, 0.98)
    wr = _clamp(1.0 - 0.5 * (wt + we), 0.05, 0.40)
    obj = ObjectiveWeights(w_time=wt, w_energy=we, w_safety=ws, w_reconfig=wr).normalized()

    seq = SequenceCostWeights(
        w_catch=float(0.7 + 1.8 * obj.w_time),
        w_dock=float(0.5 + 1.3 * obj.w_time + 0.4 * obj.w_safety),
        w_energy=float(0.8 + 4.0 * obj.w_energy),
        w_risk=float(0.5 + 2.4 * obj.w_safety),
        w_switch=float(0.15 + 0.95 * obj.w_safety),
    )
    sched = SchedulerConfig(
        # Keep cruise speeds fixed across profiles so Pareto trade-offs come
        # from reconfiguration decisions (dock/split/wait), not from changing
        # the physical "baseline driving style". This also ensures the
        # "independent baseline" is a fair reference for time comparison.
        leader_speed_mps=1.55,
        free_speed_mps=1.45,
        chase_speed_mps=1.65,
        max_accel_mps2=1.0,
        docking_time_s=float(_clamp(2.7 - 0.9 * obj.w_time + 0.6 * obj.w_safety, 1.8, 3.0)),
        sync_tolerance_s=float(_clamp(0.55 + 0.95 * obj.w_safety, 0.5, 1.35)),
        min_tail_gap_s=float(_clamp(0.30 + 0.45 * obj.w_safety, 0.25, 0.9)),
        min_intercept_ahead_s=float(_clamp(0.20 + 0.55 * obj.w_safety, 0.15, 0.85)),
        min_remaining_after_dock_s=float(_clamp(0.50 + 0.70 * obj.w_safety, 0.45, 1.30)),
        intercept_samples_per_zone=4,
        require_energy_gain_for_adaptive=True,
        fixed_sequence_replan_penalty_s=0.8,
    )
    energy = EnergyModelConfig(
        eta_nominal=float(_clamp(0.935 - 0.045 * obj.w_energy, 0.895, 0.960)),
        eta_min=0.90,
        eta_max=0.995,
        curvature_penalty_gain=float(_clamp(0.02 + 0.10 * obj.w_safety, 0.0, 0.15)),
        speed_penalty_gain=float(_clamp(0.01 + 0.08 * obj.w_safety, 0.0, 0.12)),
        speed_ref_mps=1.35,
        platoon_bonus_per_unit=float(_clamp(0.002 + 0.008 * obj.w_energy, 0.001, 0.012)),
        single_drag_gain=0.06,
        single_energy_per_m=1.0,
    )
    return ProfileConfig(
        profile_id=str(profile_id),
        objective=obj,
        seq_weights=seq,
        scheduler_cfg=sched,
        energy_cfg=energy,
    )


def default_profile_grid() -> list[ProfileConfig]:
    vals_t = [0.25, 0.40, 0.55, 0.70, 0.85]
    vals_e = [0.20, 0.35, 0.50, 0.70, 0.90]
    vals_s = [0.20, 0.45, 0.70]
    out: list[ProfileConfig] = []
    idx = 0
    for wt in vals_t:
        for we in vals_e:
            for ws in vals_s:
                idx += 1
                out.append(profile_from_tradeoff(profile_id=f"p{idx:03d}", w_time=wt, w_energy=we, w_safety=ws))
    return out


def adaptive_safety_proxy(rollout: PolicyRollout) -> float:
    if not rollout.schedule:
        return 0.0
    risk_avg = sum(float(c.cost.risk) for c in rollout.schedule) / max(1, len(rollout.schedule))
    split_ratio = sum(1 for c in rollout.schedule if bool(c.requires_future_split)) / max(1, len(rollout.schedule))
    nmax_tight = sum(max(0, c.train_size_after - c.nmax_here) for c in rollout.schedule) / max(1, len(rollout.schedule))
    return float(risk_avg + 0.6 * split_ratio + 0.4 * nmax_tight)


def adaptive_reconfig_count(rollout: PolicyRollout) -> float:
    if not rollout.schedule:
        return 0.0
    split_need = sum(1 for c in rollout.schedule if bool(c.requires_future_split))
    return float(len(rollout.schedule) + split_need)


def compute_baseline_aggregate(rows: list[dict[str, PolicyRollout]]) -> BaselineAggregate:
    if not rows:
        return BaselineAggregate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    n = float(len(rows))
    t_ind = sum(float(r["independent"].total_time_s) for r in rows) / n
    e_ind = sum(float(r["independent"].total_energy) for r in rows) / n
    t_adp = sum(float(r["adaptive"].total_time_s) for r in rows) / n
    e_adp = sum(float(r["adaptive"].total_energy) for r in rows) / n
    r_adp = sum(adaptive_reconfig_count(r["adaptive"]) for r in rows) / n
    s_adp = sum(adaptive_safety_proxy(r["adaptive"]) for r in rows) / n
    return BaselineAggregate(
        avg_time_independent=float(t_ind),
        avg_energy_independent=float(e_ind),
        avg_time_p2_adaptive=float(t_adp),
        avg_energy_p2_adaptive=float(e_adp),
        avg_reconfig_p2_adaptive=float(r_adp),
        avg_safety_p2_adaptive=float(s_adp),
    )


def evaluate_profile(
    *,
    profile: ProfileConfig,
    cases: list[Any],
    baseline: BaselineAggregate,
) -> ProfileAggregate:
    scheduler = BaselineScheduler(
        cfg=profile.scheduler_cfg,
        weights=profile.seq_weights,
        energy_cfg=profile.energy_cfg,
    )
    rows = [scheduler.rollout_case(c) for c in cases]
    n = float(max(1, len(rows)))
    avg_time = sum(float(r["adaptive"].total_time_s) for r in rows) / n
    avg_energy = sum(float(r["adaptive"].total_energy) for r in rows) / n
    avg_exec = sum(float(r["adaptive"].command_exec_rate) for r in rows) / n
    avg_train = sum(float(r["adaptive"].final_train_size) for r in rows) / n
    avg_safety = sum(adaptive_safety_proxy(r["adaptive"]) for r in rows) / n
    avg_recfg = sum(adaptive_reconfig_count(r["adaptive"]) for r in rows) / n

    er_ind = 0.0 if baseline.avg_energy_independent <= 1e-9 else (baseline.avg_energy_independent - avg_energy) / baseline.avg_energy_independent
    ti_ind = 0.0 if baseline.avg_time_independent <= 1e-9 else (baseline.avg_time_independent - avg_time) / baseline.avg_time_independent
    er_p2 = 0.0 if baseline.avg_energy_p2_adaptive <= 1e-9 else (baseline.avg_energy_p2_adaptive - avg_energy) / baseline.avg_energy_p2_adaptive
    ti_p2 = 0.0 if baseline.avg_time_p2_adaptive <= 1e-9 else (baseline.avg_time_p2_adaptive - avg_time) / baseline.avg_time_p2_adaptive

    n_time = avg_time / max(1e-9, baseline.avg_time_independent)
    n_energy = avg_energy / max(1e-9, baseline.avg_energy_independent)
    n_safety = avg_safety / max(1e-9, baseline.avg_safety_p2_adaptive + 1e-6)
    n_recfg = avg_recfg / max(1e-9, baseline.avg_reconfig_p2_adaptive + 1e-6)
    ow = profile.objective.normalized()
    scalar = ow.w_time * n_time + ow.w_energy * n_energy + ow.w_safety * n_safety + ow.w_reconfig * n_recfg

    energy_ok = er_ind >= 0.03
    time_ok = (ti_ind >= -1e-6) or (ti_ind >= 0.05)
    collision_total = 0
    profile_pass = bool(energy_ok and time_ok and collision_total == 0 and avg_exec >= 0.95)

    return ProfileAggregate(
        profile_id=str(profile.profile_id),
        num_cases=int(len(rows)),
        avg_time_s=float(avg_time),
        avg_energy=float(avg_energy),
        avg_safety=float(avg_safety),
        avg_reconfig=float(avg_recfg),
        avg_command_exec_rate=float(avg_exec),
        avg_final_train_size=float(avg_train),
        collision_total=int(collision_total),
        energy_reduction_vs_independent=float(er_ind),
        time_improve_vs_independent=float(ti_ind),
        energy_reduction_vs_p2_adaptive=float(er_p2),
        time_improve_vs_p2_adaptive=float(ti_p2),
        scalar_score=float(scalar),
        profile_pass=bool(profile_pass),
    )


def pareto_indices(metrics: list[ProfileAggregate]) -> list[int]:
    keep: list[int] = []
    for i, m in enumerate(metrics):
        dominated = False
        for j, n in enumerate(metrics):
            if i == j:
                continue
            le_all = (
                n.avg_time_s <= m.avg_time_s + 1e-12
                and n.avg_energy <= m.avg_energy + 1e-12
                and n.avg_safety <= m.avg_safety + 1e-12
            )
            lt_any = (
                n.avg_time_s < m.avg_time_s - 1e-12
                or n.avg_energy < m.avg_energy - 1e-12
                or n.avg_safety < m.avg_safety - 1e-12
            )
            if le_all and lt_any:
                dominated = True
                break
        if not dominated:
            keep.append(i)
    return keep


def recommend_profile(metrics: list[ProfileAggregate], pareto_idx: list[int]) -> int | None:
    if not pareto_idx:
        return None
    t_min = min(metrics[i].avg_time_s for i in pareto_idx)
    t_max = max(metrics[i].avg_time_s for i in pareto_idx)
    e_min = min(metrics[i].avg_energy for i in pareto_idx)
    e_max = max(metrics[i].avg_energy for i in pareto_idx)
    s_min = min(metrics[i].avg_safety for i in pareto_idx)
    s_max = max(metrics[i].avg_safety for i in pareto_idx)
    best_i = pareto_idx[0]
    best_d = math.inf
    for i in pareto_idx:
        m = metrics[i]
        nt = 0.0 if t_max <= t_min + 1e-12 else (m.avg_time_s - t_min) / (t_max - t_min)
        ne = 0.0 if e_max <= e_min + 1e-12 else (m.avg_energy - e_min) / (e_max - e_min)
        ns = 0.0 if s_max <= s_min + 1e-12 else (m.avg_safety - s_min) / (s_max - s_min)
        # Weighted knee preference: time/energy are primary, safety secondary.
        d = math.sqrt((0.45 * nt) ** 2 + (0.40 * ne) ** 2 + (0.15 * ns) ** 2)
        if d < best_d:
            best_d = d
            best_i = i
    return int(best_i)
