"""Checks of balance_centrifuge.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_balance_centrifuge.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("balance_centrifuge.py"))
M = TASK.module
OPPOSITE_PAIRS = [(0, 3), (1, 4), (2, 5)]


def _rot_y(t):
    return np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])


def _rot_z(s):
    return np.array([[math.cos(s), -math.sin(s), 0.0], [math.sin(s), math.cos(s), 0.0], [0.0, 0.0, 1.0]])


def tube_state(bx, by, floor_z, tilt_deg=0.0, spin_deg=0.0, moving=False):
    """A tube whose lower end is centred at (bx, by), with its lowest point at floor_z, spun on its axis, then tilted
    about world y. Its bounding box is built the way Arena 0.3.1 builds it: the turned corners of its local box."""
    t, s = math.radians(tilt_deg), math.radians(spin_deg)
    rot = _rot_y(t) @ _rot_z(s)
    axis = rot[:, 2]
    bottom_centre = np.array([bx, by, floor_z + M.TUBE_R * math.sin(t)])
    centre = bottom_centre + axis * M.TUBE_H / 2
    signs = [(i, j, k) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]
    corners = np.array([[i * M.TUBE_R, j * M.TUBE_R, k * M.TUBE_H / 2] for i, j, k in signs])
    world = corners @ rot.T + centre
    sy, cy, sz, cz = math.sin(t / 2), math.cos(t / 2), math.sin(s / 2), math.cos(s / 2)
    return ObjectState(pos=centre, quat_xyzw=np.array([sy * sz, sy * cz, cy * sz, cy * cz]),
                       lin_vel=np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3), ang_vel=np.zeros(3),
                       low=world.min(axis=0), high=world.max(axis=0))


def in_pocket(k, **kw):
    return tube_state(*M.POCKETS[k], M.FLOOR_Z, **kw)


def in_holder(side):
    return tube_state(*M.RACK_XY[side], M.TOP)


def scene(left=None, right=None):
    return {"tube_left": left or in_holder("left"), "tube_right": right or in_holder("right")}


def test_the_pockets_are_evenly_spaced_and_opposite_pairs_are_180_degrees_apart():
    cx, cy = M.ROTOR_XY
    for k, (px, py) in enumerate(M.POCKETS):
        assert math.hypot(px - cx, py - cy) == pytest.approx(M.POCKET_RING_R)
    for a, b in OPPOSITE_PAIRS:
        (ax, ay), (bx, by) = M.POCKETS[a], M.POCKETS[b]
        assert (ax + bx) / 2 == pytest.approx(cx) and (ay + by) / 2 == pytest.approx(cy)


def test_the_start_scores_zero():
    assert not M.success(scene()) and M.score(scene()) == 0.0


@pytest.mark.parametrize(("a", "b"), OPPOSITE_PAIRS + [(b, a) for a, b in OPPOSITE_PAIRS])
def test_opposite_pockets_are_success(a, b):
    objects = scene(in_pocket(a), in_pocket(b))
    assert M.success(objects) and M.score(objects) == 1.0


@pytest.mark.parametrize(("a", "b"), [(0, 1), (1, 2), (0, 2), (1, 5), (3, 4)])
def test_both_seated_but_unbalanced_scores_zero(a, b):
    objects = scene(in_pocket(a), in_pocket(b))
    assert not M.success(objects) and M.score(objects) == 0.0


def test_one_tube_seated_with_the_other_still_in_its_holder_scores_a_quarter():
    assert M.score(scene(left=in_pocket(1))) == 0.25
    assert M.score(scene(right=in_pocket(4))) == 0.25


def test_one_tube_seated_with_the_other_on_the_table_away_from_the_rotor_scores_a_quarter():
    assert M.score(scene(in_pocket(1), tube_state(0.17, -0.12, M.TOP, tilt_deg=90))) == 0.25


def test_one_tube_seated_with_the_other_lying_across_the_rotor_scores_zero():
    cx, cy = M.ROTOR_XY
    across = tube_state(cx - M.TUBE_H / 2, cy - 0.03, M.FLOOR_Z + M.POCKET_DEPTH, tilt_deg=90)
    assert M.score(scene(in_pocket(1), across)) == 0.0


def test_one_tube_seated_with_the_other_perched_on_a_pocket_rim_scores_zero():
    px, py = M.POCKETS[4]
    on_rim = tube_state(px + M.POCKET_INNER / 2, py, M.FLOOR_Z + M.POCKET_DEPTH)
    assert M._pocket_of(on_rim) is None
    assert M.score(scene(in_pocket(1), on_rim)) == 0.0


def test_a_tube_leaning_against_the_pocket_wall_still_counts():
    assert M.score(scene(in_pocket(1, tilt_deg=14), in_pocket(4))) == 1.0


def test_a_tube_tilted_past_18_degrees_is_not_seated():
    tilted = in_pocket(1, tilt_deg=25)
    assert M._pocket_of(tilted) is None
    assert M.score(scene(tilted, in_pocket(4))) == 0.0  # not seated, and on the rotor, so no 0.25


def test_a_tube_spun_on_its_axis_still_counts():
    assert M.score(scene(in_pocket(1, spin_deg=45), in_pocket(4, spin_deg=30))) == 1.0


def test_an_upright_tube_in_a_pocket_corner_is_seated():
    """Touching two walls, its centre is 5 mm off along each axis, 7.1 mm in all (found in review)."""
    for k in range(6):
        px, py = M.POCKETS[k]
        for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            assert M._pocket_of(tube_state(px + sx * 0.005, py + sy * 0.005, M.FLOOR_Z)) == k


def test_seating_is_the_square_opening():
    px, py = M.POCKETS[1]
    half = M.POCKET_INNER / 2
    assert M._pocket_of(tube_state(px + half - 0.001, py + half - 0.001, M.FLOOR_Z)) == 1
    assert M._pocket_of(tube_state(px + half + 0.001, py, M.FLOOR_Z)) is None


def test_a_tube_held_above_its_pocket_is_not_seated():
    px, py = M.POCKETS[1]
    held = tube_state(px, py, M.FLOOR_Z + 0.02)
    assert M._pocket_of(held) is None
    assert M.score(scene(held, in_pocket(4))) == 0.0  # not seated, and over the rotor, so no 0.25


def test_a_tube_lying_on_the_table_is_not_seated():
    lying = tube_state(0.30, 0.15, M.TOP, tilt_deg=90)
    assert M.score(scene(lying, in_pocket(4))) == 0.25


def test_a_tube_rocking_in_its_pocket_still_counts():
    assert M.success(scene(in_pocket(1, tilt_deg=6, moving=True), in_pocket(4)))


def test_the_demo_loads_opposite_pockets():
    plan = M.demo()
    let_go = {arm: [w.pos[:2] for w in plan[arm] if w.label == "let go"] for arm in plan}
    assert let_go["left"] == [pytest.approx(M.POCKETS[1])] and let_go["right"] == [pytest.approx(M.POCKETS[4])]


def test_the_right_arm_waits_until_the_left_arm_has_left_the_rotor():
    plan = M.demo()
    left_done = sum(w.duration for w in plan["left"])
    right_starts_moving_in = sum(w.duration for w in plan["right"][:2])  # above its tube, then the wait
    assert right_starts_moving_in == pytest.approx(left_done)


def _reachable(side, pocket):
    plan = ArmPlan([M.above_holder(side), *M.load_waypoints(side, pocket)], station.BASES[side])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    return plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1


@pytest.mark.parametrize(("left_pocket", "right_pocket"), OPPOSITE_PAIRS)
def test_every_balanced_answer_is_within_reach(left_pocket, right_pocket):
    """Each arm loads its own tube; all three opposite pairs can be loaded that way, so the task has three right
    answers, not one."""
    assert _reachable("left", left_pocket) and _reachable("right", right_pocket)


def test_putting_each_tube_in_the_pocket_nearest_its_arm_is_unbalanced():
    """The lazy answer fails: the pockets nearest the two arm bases are next to each other, 60 degrees apart."""
    nearest = {side: min(range(6), key=lambda k: math.dist(M.POCKETS[k], station.BASES[side][:2]))
               for side in ("left", "right")}
    assert (nearest["left"], nearest["right"]) == (2, 3)
    assert M.score(scene(in_pocket(nearest["left"]), in_pocket(nearest["right"]))) == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
