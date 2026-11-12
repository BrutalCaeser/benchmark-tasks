"""Checks of weigh_baby.py's scoring, layout and demo, without the simulator. Needs yam_arena (the official stack),
from this folder:

    python -m pytest -q test_weigh_baby.py
"""

import json
import math
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

# the manikin's mass is drawn at random each load; these checks fix it at 3.62 kg (and load others where they need to)
os.environ["WEIGH_MANIKIN_KG"] = "3.62"
TASK = load(Path(__file__).with_name("weigh_baby.py"))
M = TASK.module
from mechanism import MechanismState  # noqa: E402  (the task file puts its folder on the path)


YAW_Q = np.array([0.0, 0.0, math.sin(M.YAW / 2), math.cos(M.YAW / 2)])  # the weights' turn doesn't enter the score


def _weight(name, bar_x, along_mm, th=0.0, lift=0.0, moving=False):
    """A weight's state at ``along_mm`` from the pivot, sitting on its bar (or lifted by ``lift`` m)."""
    box_z = M.KG_BOX_Z if name == "kg_weight" else M.G_BOX_Z
    _, y = M._to_world(bar_x, along_mm)
    d, up = along_mm / 1000, (M.BAR_Z - M.PZ) / 1000 + box_z + lift  # in the beam's frame, from the pivot
    pos = np.array([M.PIVOT_XY[0] + d * math.cos(th) - up * math.sin(th), y,
                    M.TOP + M.PZ / 1000 + d * math.sin(th) + up * math.cos(th)])
    return ObjectState(pos=pos, quat_xyzw=YAW_Q, lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3),
                       ang_vel=np.zeros(3), low=pos - 0.01, high=pos + 0.01)


def state(kg=0, g=0.0, th=0.0, kg_off=0.0, lift=0.0, moving=False):
    """kg: the notch; g: the small weight's reading; kg_off: mm off the notch along the bar."""
    scale = MechanismState(pos=np.array([*M.PIVOT_XY, M.TOP]), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                           ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), joints={"beam": th, "tray": 0.0})
    return {
        "scale": scale,
        "kg_weight": _weight("kg_weight", M.KG_X, M.KG_Y0 + kg * M.KG_STEP + kg_off, th, lift, moving),
        "g_weight": _weight("g_weight", M.G_X, M.G_Y0 + g / 1000 * M.G_SPAN, th, 0.0, moving),
    }


def test_start_scores_0():
    s = state(M.KG_START, M.G_START)
    assert M.measures(s)["seated"]
    assert M.score(s) == 0.0


def test_the_right_reading_succeeds_and_scores_0_9_until_held():
    # 1.0 comes only from success() held for SUCCESS_HOLD_S; score() alone gives 0.9
    assert M.score(state(3, 620)) == 0.9
    assert M.success(state(3, 620))
    assert M.SUCCESS_HOLD_S == 1.0
    assert M.measures(state(3, 620))["error"] == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("g,credit", [(601, 0.9), (639, 0.9), (599, 0.75), (641, 0.75), (581, 0.75), (659, 0.75),
                                      (579, 0.5), (661, 0.5), (521, 0.5), (719, 0.5), (519, 0.25), (721, 0.25),
                                      (0, 0.25), (1000, 0.25)])
def test_credit_bands(g, credit):
    assert M.score(state(3, g)) == credit


def test_a_wrong_notch_scores_0_whatever_the_small_weight():
    for g in (0, 500, 1000):
        assert M.score(state(2, g)) == 0.0
        assert M.score(state(4, g)) == 0.0


def test_the_large_weight_must_sit_in_its_notch():
    assert M.score(state(3, 620, lift=0.002)) == 0.0  # held up off the notch bars
    assert M.score(state(3, 620, lift=0.0007)) == 0.9
    assert M.score(state(3, 620, kg_off=2.0)) == 0.0  # between notches
    assert M.score(state(3, 620, kg_off=0.9)) > 0


def test_full_credit_needs_the_beam_floating_and_the_weights_still():
    for s, credit in ((state(3, 620, th=M.STOP), 0.75), (state(3, 620, th=-M.STOP), 0.75),  # held on a stop
                      (state(3, 620, th=math.radians(1.0)), 0.9), (state(3, 620, moving=True), 0.75)):
        assert M.score(s) == credit
        assert M.success(s) == (credit == 0.9)


def test_score_never_gives_1_0():
    for kg in range(17):
        for g in range(0, 1001, 5):
            for deg in (-1.2, -0.6, 0.0, 0.6, 1.2):
                for moving in (False, True):
                    s = state(kg, g, th=math.radians(deg), moving=moving)
                    assert M.score(s) <= 0.9
                    assert M.success(s) == (M.score(s) == 0.9)


def test_seating_follows_the_tilted_bar():
    for deg in (-1.1, -0.5, 0.5, 1.1):
        assert M.measures(state(3, 620, th=math.radians(deg)))["seated"]
        assert not M.measures(state(3, 620, th=math.radians(deg), lift=0.002))["seated"]


def test_the_reading_is_the_beams_balance():
    # moving the large weight one notch, or the small one its whole bar, balances 1 kg more in the tray
    assert M.M_KG * M.KG_STEP == pytest.approx(1.0 * M.A)
    assert M.M_G * M.G_SPAN == pytest.approx(1.0 * M.A)
    # the counterweight balances the empty tray with both weights at 0
    moment = M._beam_moment(M.TARE_H) + M.M_KG * M.KG_Y0 / 1000 + M.M_G * M.G_Y0 / 1000
    assert moment == pytest.approx(M.TRAY_MASS * M.A / 1000, abs=1e-7)


def test_layout():
    assert len(M.NOTCH_Y) == 17 and M.NOTCH_Y[-1] <= M.BAR_Y1 - 6  # 0 to 16 kg, the weight's sleeve on the bar
    assert M.G_Y0 + M.G_SPAN <= M.BAR_Y1 - 5
    assert M.BAR_Z < M.PZ  # both weights hang below the pivot


def _load_with(mass):
    """The task module loaded fresh, with the manikin's mass set (None: drawn at random)."""
    old = os.environ.pop("WEIGH_MANIKIN_KG", None)
    if mass is not None:
        os.environ["WEIGH_MANIKIN_KG"] = str(mass)
    try:
        return load(Path(__file__).with_name("weigh_baby.py")).module
    finally:
        os.environ.pop("WEIGH_MANIKIN_KG", None)
        if old is not None:
            os.environ["WEIGH_MANIKIN_KG"] = old


def test_the_mass_is_drawn_for_each_trial_within_the_stated_range():
    masses = [_load_with(None).BABY for _ in range(12)]
    assert all(2.5 <= m <= 4.5 for m in masses)
    assert len(set(masses)) > 6  # it varies from trial to trial
    assert M.MASS_RANGE == (2.5, 4.5)


def test_the_score_follows_the_actual_mass_so_no_preset_works():
    m = _load_with(2.71)
    assert m.score(state(2, 710)) == 0.9 and m.success(state(2, 710))
    assert m.score(state(3, 620)) == 0.0  # the 3.62 kg reading: a wrong notch, 910 g off
    assert m.score(state(2, 500)) == 0.25  # the right notch, 210 g off
    assert m.score(state(2, 620)) == 0.5  # 90 g off
    m = _load_with(4.05)
    assert m.score(state(4, 50)) == 0.9
    assert m.score(state(3, 620)) == 0.0
    # a preset reading is within 20 g of the drawn mass for 2 % of trials at most
    assert 2 * M.DONE / (M.MASS_RANGE[1] - M.MASS_RANGE[0]) <= 0.02


def test_any_notch_counts_if_the_beam_balances():
    m = _load_with(3.004)
    assert m.score(state(2, 1000)) == 0.9  # 2 kg + 1,000 g balances 3.000 kg as well as 3 kg + 0 g does
    assert m.score(state(3, 0)) == 0.9


def _frame(down):
    """A right wrist camera frame from VIEW with the red pointer where it shows on the down or the up stop."""
    img = np.full((176, 320, 3), 200, dtype=np.uint8)
    x, y = M.POINTER_DOWN if down else M.POINTER_UP
    img[int(y) - 6:int(y) + 6, int(x) - 2:int(x) + 2] = (200, 30, 30)
    return img


def _run_search(mass, demo):
    """Play the demo's looks against a beam that tips down when the reading is above ``mass``; the final reading."""
    plan = M._plan
    end = max(w for k, _, (a, w) in plan["steps"] if k != "end") + 1.0
    for t in np.arange(0.0, end, 0.05):
        reading = plan["kg"] + plan["g_at"] / 1000
        M._look(SimpleNamespace(state_time=t, images={"right_cam": _frame(reading > mass)}))
    return plan["kg"] + plan["g_at"] / 1000, plan


def test_the_pointer_reader():
    assert M.pointer(_frame(True)) == pytest.approx(M.POINTER_DOWN, abs=1.0)
    assert M.tips_down([M.pointer(_frame(True))]) and not M.tips_down([M.pointer(_frame(False))])
    assert M.pointer(np.full((176, 320, 3), 200, dtype=np.uint8)) is None


@pytest.mark.parametrize("mass", [2.5, 2.501, 2.62, 2.998, 3.0, 3.004, 3.333, 3.62, 3.996, 4.0, 4.012, 4.37, 4.5])
def test_the_demo_finds_the_reading_from_the_pointer_alone(mass):
    demo = TASK.demo()
    final, plan = _run_search(mass, demo)
    assert abs(final - mass) <= 0.010, (mass, final, plan["log"])
    # every move it made went into waypoints still ahead of it: the last one aims at the final reading
    slide = [w for w in demo["right"] if w.label.startswith("slide it to")][-1]
    assert slide.label == f"slide it to {plan['g_at']:.0f} g"


def test_the_demo_reads_only_the_camera():
    # the plan knows nothing of the mass: with the camera showing nothing it reads "up" and climbs one notch only
    TASK.demo()
    plan = M._plan
    for t in np.arange(0.0, 40.0, 0.05):
        M._look(SimpleNamespace(state_time=t, images={"right_cam": np.full((176, 320, 3), 200, dtype=np.uint8)}))
    assert plan["kg"] == M.KG_FIRST + 1  # read as "tips up" each time: one notch up, and no further
    assert "BABY" not in M.demo.__code__.co_names and "BABY" not in M._look.__code__.co_names
    assert "BABY" not in M._decide.__code__.co_names


def test_demo_reaches_and_fits_the_run_limit():
    demo = TASK.demo()
    _run_search(3.62, demo)  # fill the moves in, as a run would
    for arm in ("left", "right"):
        plan = ArmPlan(demo[arm], station.BASES[arm])
        for t in np.arange(0.0, plan.duration, 0.05):
            plan.action7(t)
        assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, arm
    assert M.KG_AIM_PAST < M.KG_STEP / 2  # still nearest the aimed notch
    assert M.G_AIM_PAST / M.G_SPAN * 1000 < 20  # even with no drag, the reading stays inside the 20 g band
    duration = sum(w.duration for w in demo["right"])
    spec = json.loads(Path(__file__).with_name("task.json").read_text())
    limit = int(spec["run_limit"].split()[0].replace(",", ""))
    assert duration < limit / 20 * 0.7


def test_setup_text_matches_the_layout():
    setup = json.loads(Path(__file__).with_name("task.json").read_text())["setup"]

    def cm_x(along_mm):
        return f"{round((M.PIVOT_XY[0] + along_mm / 1000) * 100, 3):g}"

    def cm_y(across_mm):
        return f"{round((M.PIVOT_XY[1] - across_mm / 1000) * 100, 3):g}"

    for words in (f"pivots at x {M.PIVOT_XY[0] * 100:g}, y {M.PIVOT_XY[1] * 100:g}, {M.PZ / 10:g} cm",
                  f"run from x {cm_x(M.BAR_Y0)} to {cm_x(M.BAR_Y1)}", f"axes {M.BAR_Z / 10:g} cm",
                  f"inner bar, at y {cm_y(M.KG_X)}", f"outer bar, at y {cm_y(M.G_X)}",
                  f"0 kg at x {cm_x(M.KG_Y0)}, then one every {M.KG_STEP / 10:g} cm to 16 kg at x {cm_x(M.NOTCH_Y[16])}",
                  f"0 g at x {cm_x(M.G_Y0)} to 1,000 g at x {cm_x(M.G_Y0 + M.G_SPAN)}",
                  f"{round(M.M_KG * 1000)} g. The small", f"{round(M.M_G * 1000)} g. Both", f"at x {cm_x(M.TARE_Y)}",
                  f"rocks {math.degrees(M.STOP):g} degrees", f"{M.A:g} mm from the pivot",
                  f"from {M.MASS_RANGE[0]:g} to {M.MASS_RANGE[1]:g} kg to the gram"):
        assert words in setup, words


def test_the_weights_clear_the_scale_body():
    # the base collides with the loose weights (not with the beam): the large weight's sleeve must hang clear of it
    # even at the far notch with the beam on its lower stop
    sleeve_bottom = M.BAR_Z - 3 - M.KG_DOWN - M.KG_WALL - M.NOTCH_Y[16] * math.sin(M.STOP)
    under_bars = [s for s in M.SCALE.base.shapes if abs(s.center[0]) < 0.03 and s.size[1] > 0.5]  # runs the bars' length
    housing_top = max(s.center[2] + s.size[2] / 2 for s in under_bars) * 1000
    assert sleeve_bottom - housing_top > 3


def test_the_reading_does_not_change_with_the_beams_tilt():
    for deg in (-1.1, -0.5, 0.5, 1.1):
        assert M.measures(state(3, 620, th=math.radians(deg)))["error"] < 0.0005


def test_the_tray_hangs_on_the_knife_edge():
    knife = [s for s in M.BEAM_SHAPES if s.material == "knife"]
    foot = [s for s in M.TRAY_SHAPES if s.material == "knife"]
    assert len(knife) == 1 and len(foot) == 1
    k, f = knife[0], foot[0]
    # the knife's axis on the pivot's line at the short arm's length, the hanger's foot resting on its top
    assert k.center == pytest.approx((0.0, -M.A / 1000, M.PZ / 1000))
    assert f.center[1] == pytest.approx(-M.A / 1000)
    assert f.center[2] - f.size[2] / 2 == pytest.approx(M.PZ / 1000 + k.size[0])


def test_both_weights_start_at_0():
    assert (M.KG_START, M.G_START) == (0, 0)
    assert M.KG_W.pos[:2] == pytest.approx(M._to_world(M.KG_X, M.NOTCH_Y[0]))
    assert M.G_W.pos[:2] == pytest.approx(M._to_world(M.G_X, M.G_Y0))
