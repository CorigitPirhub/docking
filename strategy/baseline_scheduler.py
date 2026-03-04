from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any

import numpy as np

from docking.scenario_support import ScenarioInstance
from docking.types import VehicleState
from runtime.command_bus import CommandHeader, DockingCommand
from docking.costs import (
    EnergyModelConfig,
    SequenceCostBreakdown,
    SequenceCostWeights,
    clamp,
    compute_j_seq,
    estimate_travel_time,
    risk_score,
)


@dataclass(frozen=True)
class SchedulerConfig:
    leader_id: int = 1
    leader_speed_mps: float = 1.55
    free_speed_mps: float = 1.45
    chase_speed_mps: float = 1.65
    max_accel_mps2: float = 1.0
    docking_time_s: float = 2.4
    sync_tolerance_s: float = 0.9
    min_tail_gap_s: float = 0.4
    min_intercept_ahead_s: float = 0.35
    min_remaining_after_dock_s: float = 0.8
    intercept_samples_per_zone: int = 4
    require_energy_gain_for_adaptive: bool = True
    fixed_sequence_replan_penalty_s: float = 0.8


@dataclass(frozen=True)
class InterceptCandidate:
    follower_id: int
    leader_id: int
    s_intercept: float
    point_xy: tuple[float, float]
    t_follower: float
    t_leader: float
    dock_start: float
    dock_finish: float
    train_size_after: int
    nmax_here: int
    requires_future_split: bool
    cost: SequenceCostBreakdown

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["point_xy"] = [float(self.point_xy[0]), float(self.point_xy[1])]
        d["cost"] = asdict(self.cost)
        return d


@dataclass(frozen=True)
class PolicyRollout:
    policy: str
    scenario_id: str
    subtype: str
    total_time_s: float
    total_energy: float
    commands_issued: int
    commands_succeeded: int
    command_exec_rate: float
    docked_followers: int
    final_train_size: int
    schedule: list[InterceptCandidate]

    def to_dict(self) -> dict[str, Any]:
        return {
            "policy": self.policy,
            "scenario_id": self.scenario_id,
            "subtype": self.subtype,
            "total_time_s": float(self.total_time_s),
            "total_energy": float(self.total_energy),
            "commands_issued": int(self.commands_issued),
            "commands_succeeded": int(self.commands_succeeded),
            "command_exec_rate": float(self.command_exec_rate),
            "docked_followers": int(self.docked_followers),
            "final_train_size": int(self.final_train_size),
            "schedule": [c.to_dict() for c in self.schedule],
        }


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


def _point_radius(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> float:
    a = float(np.linalg.norm(p1 - p0))
    b = float(np.linalg.norm(p2 - p1))
    c = float(np.linalg.norm(p2 - p0))
    if min(a, b, c) < 1e-9:
        return math.inf
    area2 = abs(float((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0])))
    if area2 < 1e-9:
        return math.inf
    return a * b * c / max(2.0 * area2, 1e-9)


def _curvature_at(path_xy: np.ndarray, idx: int) -> float:
    if len(path_xy) < 3:
        return 0.0
    i = max(1, min(int(idx), len(path_xy) - 2))
    r = _point_radius(path_xy[i - 1], path_xy[i], path_xy[i + 1])
    if math.isinf(r):
        return 0.0
    return 1.0 / max(r, 1e-9)


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


def _future_min_nmax(profile: list[dict[str, float | int]], s_query: float) -> int:
    s = float(s_query)
    if not profile:
        return 1
    out = int(1e9)
    for seg in profile:
        if float(seg["s1"]) + 1e-9 < s:
            continue
        out = min(out, int(seg["n_max"]))
    if out == int(1e9):
        return int(profile[-1]["n_max"])
    return out


class BaselineScheduler:
    """P2 baseline scheduler with predictive intercept-point docking."""

    def __init__(
        self,
        *,
        cfg: SchedulerConfig | None = None,
        weights: SequenceCostWeights | None = None,
        energy_cfg: EnergyModelConfig | None = None,
    ):
        self.cfg = cfg or SchedulerConfig()
        self.weights = weights or SequenceCostWeights()
        self.energy_cfg = energy_cfg or EnergyModelConfig()

    def _leader_goal_time(self, path_len: float) -> float:
        return path_len / max(self.cfg.leader_speed_mps, 1e-6)

    def _follower_route_length(self, state: VehicleState, goal_xy: np.ndarray, detour_factor: float) -> float:
        direct = float(np.linalg.norm(state.xy() - goal_xy))
        return max(0.0, direct * max(1.0, detour_factor))

    def _intercept_route_length(self, state: VehicleState, intercept_xy: np.ndarray, detour_factor: float) -> float:
        direct = float(np.linalg.norm(state.xy() - intercept_xy))
        return max(0.0, direct * max(1.0, 0.9 * detour_factor))

    def _sample_intercept_s(
        self,
        *,
        dock_zones: list[dict[str, float]],
        profile: list[dict[str, float | int]],
        leader_s: float,
        path_len: float,
        conservative: bool,
    ) -> list[float]:
        out: list[float] = []
        for z in dock_zones:
            s0 = max(float(z["s0"]), leader_s + self.cfg.min_intercept_ahead_s)
            s1 = min(float(z["s1"]), path_len - 1e-6)
            if s1 <= s0 + 1e-6:
                continue
            n = max(2, self.cfg.intercept_samples_per_zone)
            qs = np.linspace(s0, s1, n)
            for s in qs:
                nmax_here = _nmax_at_s(profile, float(s))
                if nmax_here < 2:
                    continue
                if conservative and _future_min_nmax(profile, float(s)) < 2:
                    continue
                out.append(float(s))
        # Fallback for sparse/missing dock zones.
        if not out:
            for seg in profile:
                if int(seg["n_max"]) < 2:
                    continue
                s0 = max(float(seg["s0"]), leader_s + self.cfg.min_intercept_ahead_s)
                s1 = min(float(seg["s1"]), path_len - 1e-6)
                if s1 <= s0 + 1e-6:
                    continue
                out.append(float(0.5 * (s0 + s1)))
        out = sorted(set(round(s, 4) for s in out))
        return [float(s) for s in out]

    def _candidate_for_s(
        self,
        *,
        follower: VehicleState,
        leader_id: int,
        train_size_now: int,
        tail_ready_time: float,
        s_intercept: float,
        leader_s: float,
        case: ScenarioInstance,
        s_samples: np.ndarray,
        t_now: float,
        require_energy_gain: bool,
    ) -> InterceptCandidate | None:
        assert case.labels is not None
        labels = case.labels
        path_len = float(labels.path_length)
        profile = labels.n_max_pass_profile
        n_after = int(train_size_now + 1)
        nmax_here = _nmax_at_s(profile, s_intercept)
        if n_after > nmax_here:
            return None

        n_future = _future_min_nmax(profile, s_intercept)
        need_split = n_future < n_after

        p_int = _interp_path(case.path_xy, s_samples, s_intercept)
        d_int = self._intercept_route_length(follower, p_int, labels.detour_factor)
        t_f = estimate_travel_time(d_int, speed_now=follower.v, v_max=self.cfg.chase_speed_mps, a_max=self.cfg.max_accel_mps2)
        s_gap = max(0.0, float(s_intercept) - max(0.0, float(leader_s)))
        t_l_rel = s_gap / max(self.cfg.leader_speed_mps, 1e-6)
        t_l_abs = float(t_now) + float(t_l_rel)
        if t_l_rel < self.cfg.min_intercept_ahead_s:
            return None
        if t_f > t_l_rel + self.cfg.sync_tolerance_s:
            return None

        dock_start = max(t_now + t_f, t_l_abs, tail_ready_time)
        dock_extra = 0.6 if need_split else 0.0
        dock_finish = dock_start + self.cfg.docking_time_s + dock_extra

        leader_goal_t = self._leader_goal_time(path_len)
        if dock_finish > leader_goal_t - self.cfg.min_remaining_after_dock_s:
            return None

        idx = int(np.argmin(np.abs(s_samples - s_intercept)))
        kappa = _curvature_at(case.path_xy, idx)

        # Delta-energy is relative to independent driving for this follower.
        d_goal = self._follower_route_length(follower, case.goal_xy, labels.detour_factor)
        e_ind = self.energy_cfg.single_energy(d_goal, self.cfg.free_speed_mps)
        rem = max(0.0, path_len - s_intercept)
        e_dock = self.energy_cfg.single_energy(d_int, self.cfg.chase_speed_mps) + self.energy_cfg.train_energy(rem, n_after, kappa, self.cfg.leader_speed_mps)
        delta_e = e_dock - e_ind
        if require_energy_gain and delta_e >= -1e-6:
            return None

        rr = risk_score(
            occlusion_level=labels.occlusion_level,
            sync_error_s=(t_now + t_f) - t_l_abs,
            sync_tolerance_s=self.cfg.sync_tolerance_s,
            nmax_margin=nmax_here - n_after,
        )
        if need_split:
            rr += 0.3

        n_switch = 2.0 if need_split else 1.0
        c = compute_j_seq(
            t_catch_s=dock_start,
            t_dock_s=self.cfg.docking_time_s + dock_extra,
            delta_energy=delta_e,
            risk=rr,
            n_switch=n_switch,
            weights=self.weights,
        )
        return InterceptCandidate(
            follower_id=follower.vehicle_id,
            leader_id=leader_id,
            s_intercept=float(s_intercept),
            point_xy=(float(p_int[0]), float(p_int[1])),
            t_follower=float(t_now + t_f),
            t_leader=float(t_l_abs),
            dock_start=float(dock_start),
            dock_finish=float(dock_finish),
            train_size_after=n_after,
            nmax_here=nmax_here,
            requires_future_split=bool(need_split),
            cost=c,
        )

    def select_best_intercept(
        self,
        *,
        follower: VehicleState,
        train_size_now: int,
        tail_ready_time: float,
        case: ScenarioInstance,
        leader_s: float = 0.0,
        t_now: float = 0.0,
        conservative: bool,
        require_energy_gain: bool,
    ) -> InterceptCandidate | None:
        assert case.labels is not None
        labels = case.labels
        path_len = float(labels.path_length)
        s_samples = _polyline_s(case.path_xy)
        candidates_s = self._sample_intercept_s(
            dock_zones=labels.dock_friendly_zones,
            profile=labels.n_max_pass_profile,
            leader_s=float(max(0.0, leader_s)),
            path_len=path_len,
            conservative=conservative,
        )
        best: InterceptCandidate | None = None
        for s in candidates_s:
            c = self._candidate_for_s(
                follower=follower,
                leader_id=self.cfg.leader_id,
                train_size_now=train_size_now,
                tail_ready_time=tail_ready_time,
                s_intercept=s,
                leader_s=float(leader_s),
                case=case,
                s_samples=s_samples,
                t_now=float(max(0.0, t_now)),
                require_energy_gain=require_energy_gain,
            )
            if c is None:
                continue
            if best is None or c.cost.total_cost < best.cost.total_cost:
                best = c
        return best

    def _rollout_independent(self, case: ScenarioInstance) -> PolicyRollout:
        assert case.labels is not None
        labels = case.labels
        path_len = float(labels.path_length)
        goal_t = self._leader_goal_time(path_len)
        leader_energy = self.energy_cfg.single_energy(path_len, self.cfg.leader_speed_mps)

        arrivals: list[float] = [goal_t]
        energy_followers = 0.0
        for s in case.vehicles_init:
            if s.vehicle_id == self.cfg.leader_id:
                continue
            d_goal = self._follower_route_length(s, case.goal_xy, labels.detour_factor)
            arrivals.append(estimate_travel_time(d_goal, speed_now=s.v, v_max=self.cfg.free_speed_mps, a_max=self.cfg.max_accel_mps2))
            energy_followers += self.energy_cfg.single_energy(d_goal, self.cfg.free_speed_mps)

        total_t = max(arrivals) if arrivals else goal_t
        total_e = leader_energy + energy_followers
        return PolicyRollout(
            policy="independent",
            scenario_id=case.scenario_id,
            subtype=case.subtype,
            total_time_s=float(total_t),
            total_energy=float(total_e),
            commands_issued=0,
            commands_succeeded=0,
            command_exec_rate=1.0,
            docked_followers=0,
            final_train_size=1,
            schedule=[],
        )

    def _leader_energy_with_schedule(self, case: ScenarioInstance, schedule: list[InterceptCandidate]) -> float:
        assert case.labels is not None
        labels = case.labels
        path_len = float(labels.path_length)
        s_samples = _polyline_s(case.path_xy)
        ordered = sorted(schedule, key=lambda c: c.s_intercept)
        seg_s0 = 0.0
        n = 1
        total = 0.0
        for c in ordered:
            seg = max(0.0, c.s_intercept - seg_s0)
            idx = int(np.argmin(np.abs(s_samples - 0.5 * (seg_s0 + c.s_intercept))))
            kappa = _curvature_at(case.path_xy, idx)
            total += self.energy_cfg.train_energy(seg, n, kappa, self.cfg.leader_speed_mps)
            seg_s0 = c.s_intercept
            n += 1
        seg = max(0.0, path_len - seg_s0)
        idx = int(np.argmin(np.abs(s_samples - 0.5 * (seg_s0 + path_len))))
        kappa = _curvature_at(case.path_xy, idx)
        total += self.energy_cfg.train_energy(seg, n, kappa, self.cfg.leader_speed_mps)
        return float(total)

    def _build_rollout(
        self,
        case: ScenarioInstance,
        policy: str,
        schedule: list[InterceptCandidate],
        commands_issued: int,
        commands_succeeded: int,
        extra_time_penalty_s: float = 0.0,
    ) -> PolicyRollout:
        assert case.labels is not None
        labels = case.labels
        leader_goal_t = self._leader_goal_time(float(labels.path_length))
        leader_energy = self._leader_energy_with_schedule(case, schedule)

        dock_map = {c.follower_id: c for c in schedule}
        arrivals = [leader_goal_t]
        energy_followers = 0.0
        for s in case.vehicles_init:
            if s.vehicle_id == self.cfg.leader_id:
                continue
            c = dock_map.get(s.vehicle_id, None)
            if c is None:
                d_goal = self._follower_route_length(s, case.goal_xy, labels.detour_factor)
                arrivals.append(estimate_travel_time(d_goal, speed_now=s.v, v_max=self.cfg.free_speed_mps, a_max=self.cfg.max_accel_mps2))
                energy_followers += self.energy_cfg.single_energy(d_goal, self.cfg.free_speed_mps)
                continue

            p_int = np.array(c.point_xy, dtype=float)
            d_int = self._intercept_route_length(s, p_int, labels.detour_factor)
            rem = max(0.0, labels.path_length - c.s_intercept)
            idx = int(np.argmin(np.abs(_polyline_s(case.path_xy) - c.s_intercept)))
            kappa = _curvature_at(case.path_xy, idx)
            energy_followers += self.energy_cfg.single_energy(d_int, self.cfg.chase_speed_mps)
            energy_followers += self.energy_cfg.train_energy(rem, c.train_size_after, kappa, self.cfg.leader_speed_mps)
            arrivals.append(leader_goal_t)

        total_t = max(arrivals) + max(0.0, float(extra_time_penalty_s))
        total_e = leader_energy + energy_followers
        exec_rate = 1.0 if commands_issued <= 0 else float(commands_succeeded) / float(commands_issued)
        return PolicyRollout(
            policy=policy,
            scenario_id=case.scenario_id,
            subtype=case.subtype,
            total_time_s=float(total_t),
            total_energy=float(total_e),
            commands_issued=int(commands_issued),
            commands_succeeded=int(commands_succeeded),
            command_exec_rate=float(exec_rate),
            docked_followers=len(schedule),
            final_train_size=1 + len(schedule),
            schedule=schedule,
        )

    def _rollout_fixed_sequence(self, case: ScenarioInstance) -> PolicyRollout:
        assert case.labels is not None
        followers = sorted([s for s in case.vehicles_init if s.vehicle_id != self.cfg.leader_id], key=lambda s: s.vehicle_id)
        tail_ready = 0.0
        train_size = 1
        schedule: list[InterceptCandidate] = []
        issued = 0
        succeeded = 0
        s_samples = _polyline_s(case.path_xy)
        labels = case.labels
        candidate_s = self._sample_intercept_s(
            dock_zones=labels.dock_friendly_zones,
            profile=labels.n_max_pass_profile,
            leader_s=0.0,
            path_len=float(labels.path_length),
            conservative=False,
        )
        if not candidate_s:
            return self._build_rollout(case, "fixed_sequence", [], 0, 0)

        # Naive: always try earliest candidate first, no re-ranking.
        for f in followers:
            issued += 1
            c = self._candidate_for_s(
                follower=f,
                leader_id=self.cfg.leader_id,
                train_size_now=train_size,
                tail_ready_time=tail_ready,
                s_intercept=float(candidate_s[0]),
                leader_s=0.0,
                case=case,
                s_samples=s_samples,
                t_now=0.0,
                require_energy_gain=False,
            )
            if c is None:
                continue
            schedule.append(c)
            tail_ready = c.dock_finish + self.cfg.min_tail_gap_s
            train_size += 1
            succeeded += 1
        extra_penalty = self.cfg.fixed_sequence_replan_penalty_s * max(0, issued - succeeded)
        return self._build_rollout(case, "fixed_sequence", schedule, issued, succeeded, extra_time_penalty_s=extra_penalty)

    def _rollout_adaptive(self, case: ScenarioInstance) -> PolicyRollout:
        followers = [s for s in case.vehicles_init if s.vehicle_id != self.cfg.leader_id]
        remaining = {s.vehicle_id: s for s in followers}
        schedule: list[InterceptCandidate] = []
        tail_ready = 0.0
        train_size = 1
        issued = 0
        succeeded = 0

        while remaining:
            best: InterceptCandidate | None = None
            best_vid: int | None = None
            for vid, st in remaining.items():
                c = self.select_best_intercept(
                    follower=st,
                    train_size_now=train_size,
                    tail_ready_time=tail_ready,
                    case=case,
                    conservative=False,
                    require_energy_gain=self.cfg.require_energy_gain_for_adaptive,
                )
                if c is None:
                    continue
                if best is None or c.cost.total_cost < best.cost.total_cost:
                    best = c
                    best_vid = vid
            if best is None or best_vid is None:
                break
            issued += 1
            succeeded += 1
            schedule.append(best)
            tail_ready = best.dock_finish + self.cfg.min_tail_gap_s
            train_size += 1
            remaining.pop(best_vid, None)

        return self._build_rollout(case, "adaptive", schedule, issued, succeeded)

    def rollout_case(self, case: ScenarioInstance) -> dict[str, PolicyRollout]:
        return {
            "independent": self._rollout_independent(case),
            "fixed_sequence": self._rollout_fixed_sequence(case),
            "adaptive": self._rollout_adaptive(case),
        }

    def build_docking_commands(self, rollout: PolicyRollout, state_seq: int = 0, source: str = "p2_baseline") -> list[DockingCommand]:
        cmds: list[DockingCommand] = []
        for i, c in enumerate(rollout.schedule):
            cmd = DockingCommand(
                header=CommandHeader(
                    command_id=f"{source}_{rollout.scenario_id}_{i:03d}",
                    state_seq=int(state_seq),
                    issued_at=float(c.dock_start),
                    deadline_at=float(c.dock_start + 5.0),
                    priority=5,
                    source=source,
                ),
                follower_id=int(c.follower_id),
                leader_id=int(c.leader_id),
                intercept_path_s=float(c.s_intercept),
            )
            cmds.append(cmd)
        return cmds


def aggregate_rollouts(rows: list[dict[str, PolicyRollout]]) -> dict[str, Any]:
    if not rows:
        return {
            "num_cases": 0,
            "avg_energy_reduction_vs_independent": 0.0,
            "avg_time_reduction_vs_fixed_sequence": 0.0,
            "adaptive_command_exec_rate": 1.0,
        }

    e_ratio = []
    t_ratio = []
    issued = 0
    succ = 0
    by_subtype: dict[str, dict[str, float]] = {}
    for row in rows:
        ind = row["independent"]
        fix = row["fixed_sequence"]
        adp = row["adaptive"]
        if ind.total_energy > 1e-9:
            e_ratio.append((ind.total_energy - adp.total_energy) / ind.total_energy)
        if fix.total_time_s > 1e-9:
            t_ratio.append((fix.total_time_s - adp.total_time_s) / fix.total_time_s)
        issued += adp.commands_issued
        succ += adp.commands_succeeded

        st = adp.subtype
        sub = by_subtype.setdefault(
            st,
            {
                "count": 0.0,
                "energy_reduction_sum": 0.0,
                "time_reduction_sum": 0.0,
                "command_exec_sum": 0.0,
            },
        )
        sub["count"] += 1.0
        sub["energy_reduction_sum"] += 0.0 if ind.total_energy <= 1e-9 else (ind.total_energy - adp.total_energy) / ind.total_energy
        sub["time_reduction_sum"] += 0.0 if fix.total_time_s <= 1e-9 else (fix.total_time_s - adp.total_time_s) / fix.total_time_s
        sub["command_exec_sum"] += adp.command_exec_rate

    summary_by_subtype: dict[str, dict[str, float]] = {}
    for st, v in by_subtype.items():
        c = max(1.0, v["count"])
        summary_by_subtype[st] = {
            "count": int(v["count"]),
            "avg_energy_reduction_vs_independent": float(v["energy_reduction_sum"] / c),
            "avg_time_reduction_vs_fixed_sequence": float(v["time_reduction_sum"] / c),
            "avg_command_exec_rate": float(v["command_exec_sum"] / c),
        }

    return {
        "num_cases": len(rows),
        "avg_energy_reduction_vs_independent": float(np.mean(e_ratio) if e_ratio else 0.0),
        "avg_time_reduction_vs_fixed_sequence": float(np.mean(t_ratio) if t_ratio else 0.0),
        "adaptive_command_exec_rate": float(1.0 if issued <= 0 else succ / float(issued)),
        "by_subtype": summary_by_subtype,
    }
