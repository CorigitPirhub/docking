import numpy as np

from docking.config import load_config
from docking.coop_docking import CooperativeStagingPlanner, StagingPlan
from docking.dock_skill import CooperativeDockingSkill
from docking.types import VehicleState


def _path(start: VehicleState, goal: VehicleState) -> np.ndarray:
    return np.asarray([[float(start.x), float(start.y)], [float(goal.x), float(goal.y)]], dtype=float)


def test_small_leader_shift_plan_is_staticized(monkeypatch) -> None:
    cfg = load_config()
    leader = VehicleState(vehicle_id=0, x=0.0, y=0.0, yaw=0.0, v=0.0, delta=0.0)
    follower = VehicleState(vehicle_id=1, x=-3.0, y=0.4, yaw=0.0, v=0.0, delta=0.0)

    shifted_leader = leader.copy()
    shifted_leader.x = 0.2
    shifted_follower_goal = follower.copy()
    shifted_follower_goal.x = -1.2
    shifted_follower_goal.y = 0.0
    shifted_follower_goal.vehicle_id = 1
    small_plan = StagingPlan(
        leader_goal=shifted_leader,
        follower_goal=shifted_follower_goal,
        leader_path_xy=_path(leader, shifted_leader),
        follower_path_xy=_path(follower, shifted_follower_goal),
        score=1.0,
        reason="small_shift",
    )

    static_follower_goal = follower.copy()
    static_follower_goal.x = -1.0
    static_follower_goal.y = 0.0
    static_follower_goal.vehicle_id = 1
    static_plan = StagingPlan(
        leader_goal=leader.copy(),
        follower_goal=static_follower_goal,
        leader_path_xy=_path(leader, leader),
        follower_path_xy=_path(follower, static_follower_goal),
        score=0.8,
        reason="ok",
    )

    calls: list[bool] = []

    def fake_plan(self, *, obstacles, leader, follower, prefer_static_leader=False):
        _ = self, obstacles, leader, follower
        calls.append(bool(prefer_static_leader))
        return static_plan

    monkeypatch.setattr(CooperativeStagingPlanner, "plan", fake_plan)

    skill = CooperativeDockingSkill(
        cfg,
        seed=0,
        obstacles=[],
        leader_init=leader,
        follower_init=follower,
        plan=small_plan,
    )

    assert calls == [True]
    assert skill._leader_goal is not None
    assert skill._follower_goal is not None
    assert abs(float(skill._leader_goal.x) - float(leader.x)) < 1e-9
    assert abs(float(skill._follower_goal.x) - float(static_follower_goal.x)) < 1e-9
    assert skill._stage == "DOCKING"
    assert not skill._leader_relocated


def test_large_leader_shift_plan_is_preserved(monkeypatch) -> None:
    cfg = load_config()
    leader = VehicleState(vehicle_id=0, x=0.0, y=0.0, yaw=0.0, v=0.0, delta=0.0)
    follower = VehicleState(vehicle_id=1, x=-3.0, y=0.4, yaw=0.0, v=0.0, delta=0.0)

    shifted_leader = leader.copy()
    shifted_leader.x = 0.6
    shifted_follower_goal = follower.copy()
    shifted_follower_goal.x = -0.8
    shifted_follower_goal.y = 0.0
    shifted_follower_goal.vehicle_id = 1
    plan = StagingPlan(
        leader_goal=shifted_leader,
        follower_goal=shifted_follower_goal,
        leader_path_xy=_path(leader, shifted_leader),
        follower_path_xy=_path(follower, shifted_follower_goal),
        score=1.0,
        reason="large_shift",
    )

    def fail_plan(*args, **kwargs):
        raise AssertionError("static replanning should not run for large relocation plans")

    monkeypatch.setattr(CooperativeStagingPlanner, "plan", fail_plan)

    skill = CooperativeDockingSkill(
        cfg,
        seed=0,
        obstacles=[],
        leader_init=leader,
        follower_init=follower,
        plan=plan,
    )

    assert skill._leader_goal is not None
    assert abs(float(skill._leader_goal.x) - float(shifted_leader.x)) < 1e-9
    assert skill._stage == "STAGING"
    assert skill._leader_relocated
