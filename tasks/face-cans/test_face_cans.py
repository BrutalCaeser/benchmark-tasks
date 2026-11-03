"""Checks of face_cans.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_face_cans.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("face_cans.py"))
M = TASK.module
FACED_X = M.EDGE_X + M.CAN_R + 0.0015  # near side 1.5 mm behind the edge, the window's middle


def can_state(x, y, bottom_z=None, tilt_deg=0.0, moving=False):
    """An upright (or tilted about y) can whose bounding box matches its pose."""
    bottom_z = M.SHELF_TOP if bottom_z is None else bottom_z
    t = math.radians(tilt_deg)
    half_x = M.CAN_R * math.cos(t) + M.CAN_H / 2 * math.sin(t)
    half_z = M.CAN_H / 2 * math.cos(t) + M.CAN_R * math.sin(t)
    centre = np.array([x, y, bottom_z + half_z])
    half = np.array([half_x, M.CAN_R, half_z])
    return ObjectState(
        pos=centre,
        quat_xyzw=np.array([0.0, math.sin(t / 2), 0.0, math.cos(t / 2)]),
        lin_vel=np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3),
        ang_vel=np.zeros(3),
        low=centre - half,
        high=centre + half,
    )


def shelf_state():
    centre = np.array(M.SHELF.pos)
    half = np.array(M.SHELF.size) / 2
    return ObjectState(pos=centre, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=centre - half, high=centre + half)


def scene(**cans):
    """Every can at the back, except those given as name=ObjectState."""
    out = {"shelf": shelf_state()}
    for name, y in M.CAN_Y.items():
        out[name] = cans.get(name, can_state(M.BACK_X, y))
    return out


def all_faced(**overrides):
    return scene(**{name: overrides.get(name, can_state(FACED_X, y)) for name, y in M.CAN_Y.items()})


def test_the_start_scores_zero():
    assert not M.success(scene()) and M.score(scene()) == 0.0


def test_all_four_faced_is_success():
    assert M.success(all_faced()) and M.score(all_faced()) == 1.0


def test_one_faced_can_scores_a_quarter():
    assert M.score(scene(can_2=can_state(FACED_X, M.CAN_Y["can_2"]))) == 0.25


@pytest.mark.parametrize(("front_mm", "faced"), [(-3.5, False), (-2.5, True), (0.0, True), (5.5, True), (6.5, False)])
def test_the_window_runs_from_3_mm_past_to_6_mm_behind_the_edge(front_mm, faced):
    x = M.EDGE_X + M.CAN_R + front_mm / 1000
    assert M.score(scene(can_1=can_state(x, M.CAN_Y["can_1"]))) == (0.25 if faced else 0.0)


def test_a_can_on_the_table_scores_zero():
    off = can_state(M.EDGE_X - 0.05, M.CAN_Y["can_1"], bottom_z=M.TOP)
    assert M.score(all_faced(can_1=off)) == 0.75


def test_a_can_standing_on_another_can_scores_zero():
    stacked = can_state(FACED_X, M.CAN_Y["can_2"], bottom_z=M.SHELF_TOP + M.CAN_H)
    assert M.score(all_faced(can_1=stacked)) == 0.75


def test_a_tipped_can_scores_zero():
    assert M.score(all_faced(can_3=can_state(FACED_X, M.CAN_Y["can_3"], tilt_deg=15))) == 0.75


def test_a_moving_can_is_not_yet_faced():
    assert not M.success(all_faced(can_4=can_state(FACED_X, M.CAN_Y["can_4"], moving=True)))


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
