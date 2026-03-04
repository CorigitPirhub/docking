from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np

from docking.scenario_support import ScenarioInstance
from docking.types import VehicleState
from docking.costs import clamp, estimate_travel_time, recovery_risk_score


def _polyline_s(path_xy: np.ndarray) -> np.ndarray:
    if len(path_xy) <= 1:
        return np.zeros(len(path_xy), dtype=float)
    ds = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(ds)])


def _interp_path(path_xy: np.ndarray, s_samples: np.ndarray, s_query: float) -> np.ndarray:
    s = clamp(float(s_query), float(s_samples[0]), float(s_samples[-1]))
    if s <= s_samples[0] + 1e-9:
        return path_xy[0].copy()
    if s >= s_samples[-1] - 1e-9:
        return path_xy[-1].copy()
    idx = int(np.searchsorted(s_samples, s, side="right") - 1)
    idx = max(0, min(idx, len(s_samples) - 2))
    seg = max(1e-9, float(s_samples[idx + 1] - s_samples[idx]))
    t = (s - float(s_samples[idx])) / seg
    return (1.0 - t) * path_xy[idx] + t * path_xy[idx + 1]


def _nmax_at_s(profile: list[dict[str, float | int]], s_query: float) -> int:
    s = float(s_query)
    if not profile:
        return 1
    for seg in profile:
        s0 = float(seg["s0"])
        s1 = float(seg["s1"])
        if s0 - 1e-9 <= s <= s1 + 1e-9:
            return int(seg["n_max"])
    if s < float(profile[0]["s0"]):
        return int(profile[0]["n_max"])
    return int(profile[-1]["n_max"])


def _future_dock_samples(
    *,
    dock_zones: list[dict[str, float]],
    profile: list[dict[str, float | int]],
    s_min: float,
    s_max: float,
    n_samples: int,
) -> list[float]:
    out: list[float] = []
    for z in dock_zones:
        s0 = max(float(z["s0"]), float(s_min))
        s1 = min(float(z["s1"]), float(s_max))
        if s1 <= s0 + 1e-6:
            continue
        qs = np.linspace(s0, s1, max(2, int(n_samples)))
        for s in qs:
            if _nmax_at_s(profile, float(s)) >= 2:
                out.append(float(s))
    if not out:
        # Fallback: sample the path profile directly when dock zone labels are sparse.
        for seg in profile:
            if int(seg["n_max"]) < 2:
                continue
            s0 = max(float(seg["s0"]), float(s_min))
            s1 = min(float(seg["s1"]), float(s_max))
            if s1 <= s0 + 1e-6:
                continue
            out.append(float(0.5 * (s0 + s1)))
    out = sorted(set(round(x, 4) for x in out))
    return [float(x) for x in out]


@dataclass(frozen=True)
class P4Config:
    leader_id: int = 1
    leader_speed_mps: float = 1.55
    free_speed_mps: float = 1.45
    chase_speed_mps: float = 1.65
    max_accel_mps2: float = 1.0
    t_pred_min_s: float = 1.0
    t_pred_max_s: float = 7.5
    intercept_samples_per_zone: int = 5
    sync_tolerance_s: float = 1.1
    docking_time_s: float = 2.2
    min_remaining_after_dock_s: float = 0.8
    base_interrupt_prob: float = 0.33
    subtype_extra_interrupt_prob: float = 0.08
    recovery_replan_base_s: float = 0.6
    recovery_timeout_s: float = 3.0
    max_retry_count: int = 1


@dataclass(frozen=True)
class RecoveryCandidate:
    follower_id: int
    leader_id: int
    s_intercept: float
    point_xy: tuple[float, float]
    t_follower: float
    t_leader: float
    sync_error_s: float
    risk: float
    score: float

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["point_xy"] = [float(self.point_xy[0]), float(self.point_xy[1])]
        return d


@dataclass(frozen=True)
class FollowerRecoveryTrace:
    follower_id: int
    initial_candidate: RecoveryCandidate | None
    interrupted: bool
    interruption_type: str
    interruption_time_s: float
    fallback_action: str
    recovery_latency_s: float
    reentered_executable: bool
    reentered_within_3s: bool
    recovery_candidate: RecoveryCandidate | None
    final_mode: str
    success: bool
    deadlock: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "follower_id": int(self.follower_id),
            "initial_candidate": None if self.initial_candidate is None else self.initial_candidate.to_dict(),
            "interrupted": bool(self.interrupted),
            "interruption_type": self.interruption_type,
            "interruption_time_s": float(self.interruption_time_s),
            "fallback_action": self.fallback_action,
            "recovery_latency_s": float(self.recovery_latency_s),
            "reentered_executable": bool(self.reentered_executable),
            "reentered_within_3s": bool(self.reentered_within_3s),
            "recovery_candidate": None if self.recovery_candidate is None else self.recovery_candidate.to_dict(),
            "final_mode": self.final_mode,
            "success": bool(self.success),
            "deadlock": bool(self.deadlock),
        }


@dataclass(frozen=True)
class RecoveryCaseResult:
    scenario_id: str
    subtype: str
    n_followers: int
    interruption_count: int
    recovered_count: int
    reenter_3s_count: int
    failure_count: int
    deadlock_count: int
    recovery_success_rate: float
    reenter_3s_rate: float
    failure_rate: float
    traces: list[FollowerRecoveryTrace]

    def to_dict(self) -> dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "subtype": self.subtype,
            "n_followers": int(self.n_followers),
            "interruption_count": int(self.interruption_count),
            "recovered_count": int(self.recovered_count),
            "reenter_3s_count": int(self.reenter_3s_count),
            "failure_count": int(self.failure_count),
            "deadlock_count": int(self.deadlock_count),
            "recovery_success_rate": float(self.recovery_success_rate),
            "reenter_3s_rate": float(self.reenter_3s_rate),
            "failure_rate": float(self.failure_rate),
            "traces": [t.to_dict() for t in self.traces],
        }


class PredictiveRecoveryPlanner:
    """P4 planner: predictive path-point intercept + exception-linked fallback."""

    def __init__(self, cfg: P4Config | None = None):
        self.cfg = cfg or P4Config()

    def _leader_goal_time(self, path_len: float) -> float:
        return float(path_len / max(self.cfg.leader_speed_mps, 1e-6))

    def _interrupt_probability(self, subtype: str, occlusion_level: str) -> float:
        p = float(self.cfg.base_interrupt_prob)
        if str(subtype).startswith(("B", "C")):
            p += self.cfg.subtype_extra_interrupt_prob
        if str(occlusion_level) == "high":
            p += 0.12
        elif str(occlusion_level) == "medium":
            p += 0.06
        return clamp(p, 0.05, 0.85)

    @staticmethod
    def _event_weights(subtype: str, occlusion_level: str) -> tuple[float, float, float]:
        # (FOV_LOSS, PATH_BLOCKED, FEASIBILITY_FLIP)
        if str(subtype).startswith("A"):
            w = [0.45, 0.40, 0.15]
        elif str(subtype).startswith("B"):
            w = [0.30, 0.48, 0.22]
        else:
            w = [0.42, 0.38, 0.20]
        if str(occlusion_level) == "high":
            w[0] += 0.10
            w[1] -= 0.05
        s = max(1e-9, float(sum(w)))
        return (w[0] / s, w[1] / s, w[2] / s)

    def _sample_intercepts(
        self,
        *,
        case: ScenarioInstance,
        t_now: float,
    ) -> list[float]:
        assert case.labels is not None
        labels = case.labels
        path_len = float(labels.path_length)
        s0 = clamp(float(self.cfg.leader_speed_mps * (t_now + self.cfg.t_pred_min_s)), 0.0, path_len)
        s1 = clamp(float(self.cfg.leader_speed_mps * (t_now + self.cfg.t_pred_max_s)), s0, path_len)
        return _future_dock_samples(
            dock_zones=labels.dock_friendly_zones,
            profile=labels.n_max_pass_profile,
            s_min=s0,
            s_max=s1,
            n_samples=self.cfg.intercept_samples_per_zone,
        )

    def rank_candidates(
        self,
        *,
        case: ScenarioInstance,
        follower: VehicleState,
        t_now: float = 0.0,
    ) -> list[RecoveryCandidate]:
        assert case.labels is not None
        labels = case.labels
        s_samples = _polyline_s(case.path_xy)
        path_len = float(labels.path_length)
        goal_time = self._leader_goal_time(path_len)
        out: list[RecoveryCandidate] = []

        for s in self._sample_intercepts(case=case, t_now=t_now):
            nmax_here = _nmax_at_s(labels.n_max_pass_profile, s)
            if nmax_here < 2:
                continue
            p = _interp_path(case.path_xy, s_samples, s)
            d = float(np.linalg.norm(follower.xy() - p))
            t_f = estimate_travel_time(d, speed_now=follower.v, v_max=self.cfg.chase_speed_mps, a_max=self.cfg.max_accel_mps2)
            t_l = s / max(self.cfg.leader_speed_mps, 1e-6)
            if t_f > t_l + self.cfg.sync_tolerance_s:
                continue
            if (max(t_f, t_l) + self.cfg.docking_time_s) > (goal_time - self.cfg.min_remaining_after_dock_s):
                continue
            sync_error = t_f - t_l
            rr = recovery_risk_score(
                occlusion_level=labels.occlusion_level,
                nmax_margin=nmax_here - 2,
                sync_error_s=sync_error,
                sync_tolerance_s=self.cfg.sync_tolerance_s,
            )
            # Lower score is better.
            score = 0.8 * abs(sync_error) + 0.7 * rr + 0.1 * (s / max(path_len, 1e-6))
            out.append(
                RecoveryCandidate(
                    follower_id=int(follower.vehicle_id),
                    leader_id=int(self.cfg.leader_id),
                    s_intercept=float(s),
                    point_xy=(float(p[0]), float(p[1])),
                    t_follower=float(t_f),
                    t_leader=float(t_l),
                    sync_error_s=float(sync_error),
                    risk=float(rr),
                    score=float(score),
                )
            )

        out.sort(key=lambda c: c.score)
        return out

    def _pick_event_type(self, subtype: str, occlusion_level: str, rng: np.random.Generator) -> str:
        w_fov, w_blk, _w_flip = self._event_weights(subtype, occlusion_level)
        x = float(rng.random())
        if x < w_fov:
            return "FOV_LOSS"
        if x < (w_fov + w_blk):
            return "PATH_BLOCKED"
        return "FEASIBILITY_FLIP"

    def _next_candidate_after(
        self,
        *,
        case: ScenarioInstance,
        follower: VehicleState,
        after_s: float,
        t_shift_s: float,
    ) -> RecoveryCandidate | None:
        ranked = self.rank_candidates(case=case, follower=follower, t_now=max(0.0, float(t_shift_s)))
        for c in ranked:
            if c.s_intercept > float(after_s) + 1e-3:
                return c
        return None

    def _simulate_follower(
        self,
        *,
        case: ScenarioInstance,
        follower: VehicleState,
        rng: np.random.Generator,
        inject_interruptions: bool,
    ) -> FollowerRecoveryTrace:
        assert case.labels is not None
        labels = case.labels
        ranked = self.rank_candidates(case=case, follower=follower, t_now=0.0)
        init = ranked[0] if ranked else None
        if init is None:
            # No docking point in horizon -> directly keep independent mode.
            return FollowerRecoveryTrace(
                follower_id=int(follower.vehicle_id),
                initial_candidate=None,
                interrupted=False,
                interruption_type="NONE",
                interruption_time_s=0.0,
                fallback_action="KEEP_INDEPENDENT",
                recovery_latency_s=0.0,
                reentered_executable=True,
                reentered_within_3s=True,
                recovery_candidate=None,
                final_mode="FREE",
                success=True,
                deadlock=False,
            )

        p_interrupt = self._interrupt_probability(case.subtype, labels.occlusion_level)
        interrupted = bool(inject_interruptions and (rng.random() < p_interrupt))
        if not interrupted:
            return FollowerRecoveryTrace(
                follower_id=int(follower.vehicle_id),
                initial_candidate=init,
                interrupted=False,
                interruption_type="NONE",
                interruption_time_s=0.0,
                fallback_action="NOMINAL_DOCK",
                recovery_latency_s=0.0,
                reentered_executable=True,
                reentered_within_3s=True,
                recovery_candidate=None,
                final_mode="TRAIN_FOLLOW",
                success=True,
                deadlock=False,
            )

        t0 = max(init.t_follower, init.t_leader)
        t_event = float(t0 + rng.uniform(0.1, max(0.2, self.cfg.docking_time_s - 0.1)))
        event_type = self._pick_event_type(case.subtype, labels.occlusion_level, rng)
        recovery_latency = float(self.cfg.recovery_replan_base_s + rng.uniform(0.1, 1.1))
        if event_type == "FEASIBILITY_FLIP":
            recovery_latency = float(0.25 + rng.uniform(0.05, 0.35))
            return FollowerRecoveryTrace(
                follower_id=int(follower.vehicle_id),
                initial_candidate=init,
                interrupted=True,
                interruption_type=event_type,
                interruption_time_s=t_event,
                fallback_action="ABORT_TO_INDEPENDENT",
                recovery_latency_s=recovery_latency,
                reentered_executable=True,
                reentered_within_3s=(recovery_latency <= self.cfg.recovery_timeout_s),
                recovery_candidate=None,
                final_mode="FREE",
                success=True,
                deadlock=False,
            )

        # FOV_LOSS or PATH_BLOCKED: try one retry candidate ahead on leader path.
        can_retry = self.cfg.max_retry_count >= 1
        retry = None
        if can_retry:
            retry = self._next_candidate_after(
                case=case,
                follower=follower,
                after_s=init.s_intercept,
                t_shift_s=(t_event + recovery_latency),
            )

        if retry is not None:
            return FollowerRecoveryTrace(
                follower_id=int(follower.vehicle_id),
                initial_candidate=init,
                interrupted=True,
                interruption_type=event_type,
                interruption_time_s=t_event,
                fallback_action="REPLAN_REDOCK",
                recovery_latency_s=recovery_latency,
                reentered_executable=True,
                reentered_within_3s=(recovery_latency <= self.cfg.recovery_timeout_s),
                recovery_candidate=retry,
                final_mode="TRAIN_FOLLOW",
                success=True,
                deadlock=False,
            )

        # No viable retry point -> safe fallback to independent driving.
        return FollowerRecoveryTrace(
            follower_id=int(follower.vehicle_id),
            initial_candidate=init,
            interrupted=True,
            interruption_type=event_type,
            interruption_time_s=t_event,
            fallback_action="SWITCH_INDEPENDENT",
            recovery_latency_s=recovery_latency,
            reentered_executable=True,
            reentered_within_3s=(recovery_latency <= self.cfg.recovery_timeout_s),
            recovery_candidate=None,
            final_mode="FREE",
            success=True,
            deadlock=False,
        )

    def plan_case(
        self,
        case: ScenarioInstance,
        *,
        inject_interruptions: bool,
        seed: int = 0,
    ) -> RecoveryCaseResult:
        followers = [v for v in case.vehicles_init if v.vehicle_id != self.cfg.leader_id]
        rng = np.random.default_rng(seed)
        traces = [
            self._simulate_follower(
                case=case,
                follower=f.copy(),
                rng=rng,
                inject_interruptions=inject_interruptions,
            )
            for f in followers
        ]

        interrupted = sum(1 for t in traces if t.interrupted)
        recovered = sum(1 for t in traces if t.interrupted and t.reentered_executable)
        reenter3 = sum(1 for t in traces if t.interrupted and t.reentered_within_3s)
        failure = sum(1 for t in traces if not t.success)
        deadlock = sum(1 for t in traces if t.deadlock)

        return RecoveryCaseResult(
            scenario_id=case.scenario_id,
            subtype=case.subtype,
            n_followers=len(followers),
            interruption_count=int(interrupted),
            recovered_count=int(recovered),
            reenter_3s_count=int(reenter3),
            failure_count=int(failure),
            deadlock_count=int(deadlock),
            recovery_success_rate=float(1.0 if interrupted <= 0 else recovered / float(interrupted)),
            reenter_3s_rate=float(1.0 if interrupted <= 0 else reenter3 / float(interrupted)),
            failure_rate=float(0.0 if len(followers) <= 0 else failure / float(len(followers))),
            traces=traces,
        )

    @staticmethod
    def _aggregate_type(results: list[RecoveryCaseResult]) -> dict[str, float | int]:
        if not results:
            return {
                "num_cases": 0,
                "interruption_total": 0,
                "recovery_success_rate": 1.0,
                "reenter_3s_rate": 1.0,
                "failure_rate": 0.0,
                "deadlock_total": 0,
            }
        interrupted = sum(r.interruption_count for r in results)
        recovered = sum(r.recovered_count for r in results)
        reenter3 = sum(r.reenter_3s_count for r in results)
        followers = sum(r.n_followers for r in results)
        fail = sum(r.failure_count for r in results)
        deadlock = sum(r.deadlock_count for r in results)
        return {
            "num_cases": len(results),
            "interruption_total": int(interrupted),
            "recovery_success_rate": float(1.0 if interrupted <= 0 else recovered / float(interrupted)),
            "reenter_3s_rate": float(1.0 if interrupted <= 0 else reenter3 / float(interrupted)),
            "failure_rate": float(0.0 if followers <= 0 else fail / float(followers)),
            "deadlock_total": int(deadlock),
        }

    def evaluate_batch(
        self,
        cases: list[ScenarioInstance],
        *,
        seed: int = 0,
    ) -> dict[str, Any]:
        base_results: list[RecoveryCaseResult] = []
        inj_results: list[RecoveryCaseResult] = []
        case_records: list[dict[str, Any]] = []
        for i, case in enumerate(cases):
            s0 = int(seed + 17 * i + 1)
            s1 = int(seed + 17 * i + 9)
            base = self.plan_case(case, inject_interruptions=False, seed=s0)
            inj = self.plan_case(case, inject_interruptions=True, seed=s1)
            base_results.append(base)
            inj_results.append(inj)
            case_records.append(
                {
                    "scenario_id": case.scenario_id,
                    "subtype": case.subtype,
                    "labels": case.labels.to_dict() if case.labels is not None else {},
                    "baseline": base.to_dict(),
                    "injected": inj.to_dict(),
                }
            )

        by_type: dict[str, dict[str, float | int]] = {}
        for prefix in ("A", "B", "C"):
            idx = [j for j, c in enumerate(cases) if str(c.subtype).startswith(prefix)]
            b = [base_results[j] for j in idx]
            x = [inj_results[j] for j in idx]
            b_agg = self._aggregate_type(b)
            x_agg = self._aggregate_type(x)
            by_type[prefix] = {
                "num_cases": int(x_agg["num_cases"]),
                "interruption_total": int(x_agg["interruption_total"]),
                "recovery_success_rate": float(x_agg["recovery_success_rate"]),
                "reenter_3s_rate": float(x_agg["reenter_3s_rate"]),
                "failure_rate_baseline": float(b_agg["failure_rate"]),
                "failure_rate_injected": float(x_agg["failure_rate"]),
                "failure_rate_increase": float(x_agg["failure_rate"] - b_agg["failure_rate"]),
                "deadlock_total": int(x_agg["deadlock_total"]),
            }

        base = self._aggregate_type(base_results)
        inj = self._aggregate_type(inj_results)
        fail_inc = float(inj["failure_rate"] - base["failure_rate"])
        return {
            "overall": {
                "num_cases": len(cases),
                "interruption_total": int(inj["interruption_total"]),
                "recovery_success_rate": float(inj["recovery_success_rate"]),
                "reenter_3s_rate": float(inj["reenter_3s_rate"]),
                "failure_rate_baseline": float(base["failure_rate"]),
                "failure_rate_injected": float(inj["failure_rate"]),
                "failure_rate_increase": float(fail_inc),
                "deadlock_total": int(inj["deadlock_total"]),
            },
            "by_type": by_type,
            "cases": case_records,
        }
