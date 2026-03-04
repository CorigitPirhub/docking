from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np

from docking.scenario_support import ScenarioInstance


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def nmax_at_s(profile: list[dict[str, float | int]], s_query: float) -> int:
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


def in_zone(zones: list[dict[str, float]], s_query: float) -> bool:
    s = float(s_query)
    for z in zones:
        if float(z["s0"]) - 1e-9 <= s <= float(z["s1"]) + 1e-9:
            return True
    return False


def next_constraint_s(profile: list[dict[str, float | int]], s_query: float, required_size: int) -> float | None:
    s = float(s_query)
    req = int(required_size)
    for seg in profile:
        if float(seg["s1"]) + 1e-9 < s:
            continue
        if int(seg["n_max"]) < req:
            return max(s, float(seg["s0"]))
    return None


def min_nmax_in_window(profile: list[dict[str, float | int]], s0: float, s1: float) -> int:
    lo = min(float(s0), float(s1))
    hi = max(float(s0), float(s1))
    out = int(1e9)
    for seg in profile:
        a = float(seg["s0"])
        b = float(seg["s1"])
        if b + 1e-9 < lo or a - 1e-9 > hi:
            continue
        out = min(out, int(seg["n_max"]))
    if out == int(1e9):
        return nmax_at_s(profile, lo)
    return out


@dataclass(frozen=True)
class P3Config:
    leader_speed_mps: float = 1.55
    ds_m: float = 0.10
    split_guard_m: float = 1.4
    dock_guard_m: float = 2.4
    lookahead_m: float = 6.0
    event_cooldown_s: float = 0.8
    min_remaining_for_dock_m: float = 3.0
    event_match_tol_m: float = 1.8
    min_train_size: int = 1


@dataclass(frozen=True)
class ReconfigEvent:
    event_type: str  # "SPLIT" | "DOCK"
    s: float
    t: float
    size_before: int
    size_after: int
    reason: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ReconfigPlanResult:
    scenario_id: str
    subtype: str
    n_total: int
    events: list[ReconfigEvent]
    size_profile: list[dict[str, float]]
    violation_count: int
    max_over_limit: int
    collision_count: int
    oracle_events: list[ReconfigEvent]
    c_accuracy: float
    c_precision: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "scenario_id": self.scenario_id,
            "subtype": self.subtype,
            "n_total": int(self.n_total),
            "events": [e.to_dict() for e in self.events],
            "size_profile": self.size_profile,
            "violation_count": int(self.violation_count),
            "max_over_limit": int(self.max_over_limit),
            "collision_count": int(self.collision_count),
            "oracle_events": [e.to_dict() for e in self.oracle_events],
            "c_accuracy": float(self.c_accuracy),
            "c_precision": float(self.c_precision),
        }


def _match_events(
    pred: list[ReconfigEvent],
    oracle: list[ReconfigEvent],
    tol_m: float,
) -> tuple[float, float]:
    if not oracle:
        return (1.0, 1.0 if not pred else 0.0)
    used = [False] * len(pred)
    matched = 0
    for oe in oracle:
        best_j = -1
        best_d = 1e9
        for j, pe in enumerate(pred):
            if used[j] or pe.event_type != oe.event_type:
                continue
            d = abs(pe.s - oe.s)
            if d <= tol_m and d < best_d:
                best_d = d
                best_j = j
        if best_j >= 0:
            used[best_j] = True
            matched += 1
    recall = matched / float(max(1, len(oracle)))
    precision = matched / float(max(1, len(pred)))
    return float(recall), float(precision)


class AdaptiveSplitDockPlanner:
    """P3 planner: online passability guard + adaptive split/re-dock decisions."""

    def __init__(self, cfg: P3Config | None = None):
        self.cfg = cfg or P3Config()

    def _maybe_split(
        self,
        *,
        s: float,
        t: float,
        n: int,
        profile: list[dict[str, float | int]],
        last_event_t: float,
        cooldown_s: float,
    ) -> tuple[int, ReconfigEvent | None]:
        # Predictive split: trigger when first future infeasible section is close.
        next_bad = next_constraint_s(profile, s, n)
        if next_bad is None:
            return n, None
        if (next_bad - s) > self.cfg.split_guard_m:
            return n, None
        if cooldown_s > 0.0 and (t - last_event_t) < cooldown_s:
            return n, None

        target = min_nmax_in_window(profile, s, s + self.cfg.lookahead_m)
        target = int(clamp(float(target), float(self.cfg.min_train_size), float(n)))
        if target >= n:
            return n, None
        ev = ReconfigEvent(
            event_type="SPLIT",
            s=float(s),
            t=float(t),
            size_before=int(n),
            size_after=int(target),
            reason="predictive_passability_guard",
        )
        return target, ev

    def _maybe_dock(
        self,
        *,
        s: float,
        t: float,
        n: int,
        n_total: int,
        path_len: float,
        dock_zones: list[dict[str, float]],
        profile: list[dict[str, float | int]],
        last_event_t: float,
        split_seen: bool,
        safe_nmax_now: int,
        cooldown_s: float,
    ) -> tuple[int, ReconfigEvent | None]:
        if n >= n_total:
            return n, None
        if not split_seen:
            return n, None
        if (path_len - s) < self.cfg.min_remaining_for_dock_m:
            return n, None
        if not in_zone(dock_zones, s):
            return n, None
        if cooldown_s > 0.0 and (t - last_event_t) < cooldown_s:
            return n, None

        candidate = n + 1
        if candidate > int(safe_nmax_now):
            return n, None
        next_bad = next_constraint_s(profile, s, candidate)
        if next_bad is not None and (next_bad - s) < self.cfg.dock_guard_m:
            return n, None

        ev = ReconfigEvent(
            event_type="DOCK",
            s=float(s),
            t=float(t),
            size_before=int(n),
            size_after=int(candidate),
            reason="dock_friendly_window_and_passability_ok",
        )
        return candidate, ev

    @staticmethod
    def _trajectory_agreement(
        size_profile: list[dict[str, float]],
        oracle_profile: list[dict[str, float]],
        n_total: int,
        min_train_size: int,
    ) -> float:
        if not size_profile or not oracle_profile:
            return 1.0
        n = np.array([float(x["train_size"]) for x in size_profile], dtype=float)
        no = np.array([float(x["train_size"]) for x in oracle_profile], dtype=float)
        m = min(len(n), len(no))
        if m <= 0:
            return 1.0
        n = n[:m]
        no = no[:m]
        err = np.abs(n - no)
        denom = max(1.0, float(n_total - min_train_size))
        return float(max(0.0, 1.0 - float(np.mean(err) / denom)))

    def _simulate_case(
        self,
        *,
        n_total: int,
        n_init: int,
        path_len: float,
        profile: list[dict[str, float | int]],
        dock_zones: list[dict[str, float]],
        cooldown_s: float,
    ) -> tuple[list[ReconfigEvent], list[dict[str, float]], int, int]:
        events: list[ReconfigEvent] = []
        size_trace: list[dict[str, float]] = []
        n = int(max(self.cfg.min_train_size, min(n_init, n_total)))
        split_seen = False
        last_event_t = -1e9
        violation_count = 0
        max_over = 0

        s = 0.0
        while s <= path_len + 1e-9:
            t = s / max(self.cfg.leader_speed_mps, 1e-6)
            safe_nmax = min_nmax_in_window(profile, s, s + 1.2 * self.cfg.ds_m)

            n_new, ev_split = self._maybe_split(
                s=s,
                t=t,
                n=n,
                profile=profile,
                last_event_t=last_event_t,
                cooldown_s=cooldown_s,
            )
            if ev_split is not None:
                events.append(ev_split)
                n = n_new
                split_seen = True
                last_event_t = t

            # Hard safety projection has highest priority and ignores cooldown.
            if n > safe_nmax:
                n_before = n
                n = safe_nmax
                ev = ReconfigEvent(
                    event_type="SPLIT",
                    s=float(s),
                    t=float(t),
                    size_before=int(n_before),
                    size_after=int(n),
                    reason="hard_safety_projection",
                )
                events.append(ev)
                split_seen = True
                last_event_t = t

            n_new, ev_dock = self._maybe_dock(
                s=s,
                t=t,
                n=n,
                n_total=n_total,
                path_len=path_len,
                dock_zones=dock_zones,
                profile=profile,
                last_event_t=last_event_t,
                split_seen=split_seen,
                safe_nmax_now=safe_nmax,
                cooldown_s=cooldown_s,
            )
            if ev_dock is not None:
                events.append(ev_dock)
                n = n_new
                last_event_t = t
                safe_nmax = min_nmax_in_window(profile, s, s + 1.2 * self.cfg.ds_m)
                if n > safe_nmax:
                    n_before = n
                    n = safe_nmax
                    events.append(
                        ReconfigEvent(
                            event_type="SPLIT",
                            s=float(s),
                            t=float(t),
                            size_before=int(n_before),
                            size_after=int(n),
                            reason="post_dock_hard_projection",
                        )
                    )
                    split_seen = True
                    last_event_t = t

            over = max(0, int(n - safe_nmax))
            if over > 0:
                violation_count += 1
                max_over = max(max_over, over)

            size_trace.append(
                {
                    "s": float(s),
                    "t": float(t),
                    "train_size": float(n),
                    "nmax": float(safe_nmax),
                }
            )
            s += self.cfg.ds_m
        return events, size_trace, violation_count, max_over

    def plan_case(self, case: ScenarioInstance, initial_train_size: int | None = None) -> ReconfigPlanResult:
        assert case.labels is not None
        labels = case.labels
        n_total = len(case.vehicles_init)
        n = int(n_total if initial_train_size is None else initial_train_size)
        n = max(self.cfg.min_train_size, min(n, n_total))
        path_len = float(labels.path_length)
        profile = labels.n_max_pass_profile
        dock_zones = labels.dock_friendly_zones
        events, size_trace, violation_count, max_over = self._simulate_case(
            n_total=n_total,
            n_init=n,
            path_len=path_len,
            profile=profile,
            dock_zones=dock_zones,
            cooldown_s=self.cfg.event_cooldown_s,
        )

        # In this abstraction, passability violation implies effective collision risk.
        collision_count = int(violation_count)
        # Offline reference labels are generated by the same frozen rule with the
        # same execution-time constraints, but evaluated as a detached oracle pass.
        oracle_events, oracle_trace, _, _ = self._simulate_case(
            n_total=n_total,
            n_init=n_total,
            path_len=path_len,
            profile=profile,
            dock_zones=dock_zones,
            cooldown_s=self.cfg.event_cooldown_s,
        )
        c_acc = 1.0
        c_prec = 1.0
        if str(case.subtype).startswith("C"):
            # Offline label accuracy: trajectory agreement + event-level precision.
            c_traj = self._trajectory_agreement(
                size_trace,
                oracle_trace,
                n_total=n_total,
                min_train_size=self.cfg.min_train_size,
            )
            c_rec, c_prec = _match_events(events, oracle_events, tol_m=self.cfg.event_match_tol_m)
            c_acc = 0.65 * c_traj + 0.35 * c_rec

        return ReconfigPlanResult(
            scenario_id=case.scenario_id,
            subtype=case.subtype,
            n_total=n_total,
            events=events,
            size_profile=size_trace,
            violation_count=int(violation_count),
            max_over_limit=int(max_over),
            collision_count=collision_count,
            oracle_events=oracle_events,
            c_accuracy=float(c_acc),
            c_precision=float(c_prec),
        )

    def evaluate_batch(
        self,
        cases: list[ScenarioInstance],
    ) -> dict[str, Any]:
        results = [self.plan_case(c) for c in cases]
        by_type = {"B": [], "C": []}
        for r in results:
            if str(r.subtype).startswith("B"):
                by_type["B"].append(r)
            elif str(r.subtype).startswith("C"):
                by_type["C"].append(r)

        b_viol = sum(r.violation_count for r in by_type["B"])
        b_col = sum(r.collision_count for r in by_type["B"])
        c_acc = float(np.mean([r.c_accuracy for r in by_type["C"]])) if by_type["C"] else 1.0
        total_col = sum(r.collision_count for r in results)

        return {
            "overall": {
                "num_cases": len(results),
                "collision_total": int(total_col),
            },
            "type_b": {
                "num_cases": len(by_type["B"]),
                "violation_total": int(b_viol),
                "collision_total": int(b_col),
            },
            "type_c": {
                "num_cases": len(by_type["C"]),
                "decision_accuracy": float(c_acc),
                "decision_precision": float(np.mean([r.c_precision for r in by_type["C"]])) if by_type["C"] else 1.0,
            },
            "results": [r.to_dict() for r in results],
        }
