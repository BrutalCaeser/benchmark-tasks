"""Checks of mirror_fault_code.py's scoring, layout and demo, without the simulator. Needs yam_arena (the official
stack), from this folder:

    python -m pytest -q test_mirror_fault_code.py
"""

import json
import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("mirror_fault_code.py"))
M = TASK.module
TOP = M.TOP


def _q(yaw_deg=0.0, tilt_deg=0.0):
    """Turned about z by yaw, then tipped about the (turned) x axis by tilt."""
    y, t = math.radians(yaw_deg) / 2, math.radians(tilt_deg) / 2
    qz = np.array([0.0, 0.0, math.sin(y), math.cos(y)])
    qx = np.array([math.sin(t), 0.0, 0.0, math.cos(t)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def _obj(pos, half, yaw=0.0, tilt=0.0, moving=False):
    pos = np.array(pos, dtype=float)
    return ObjectState(pos=pos, quat_xyzw=_q(yaw, tilt), lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3),
                       ang_vel=np.zeros(3), low=pos - np.array(half), high=pos + np.array(half))


MIRROR_HALF = np.array([0.0548, 0.05, 0.035])  # the mirror's box: base 80 mm, glass leaning 69 mm past its centre
MIRROR_DX = 0.0145  # its box centre lies this far past the base's centre, toward the glass
CART_HALF = np.array([0.02, 0.02, 0.0365])


def mirror_at(base_xy, yaw=0.0, tilt=0.0, lift=0.0, moving=False):
    return _obj((base_xy[0] + MIRROR_DX, base_xy[1], TOP + MIRROR_HALF[2] + lift), MIRROR_HALF, yaw, tilt, moving)


def cart_at(xy, z=TOP, tilt=0.0, moving=False):
    return _obj((xy[0], xy[1], z + CART_HALF[2]), CART_HALF, tilt=tilt, moving=moving)


def scene(mirror=None, moved=None, pump_dx=0.0, moving=None):
    """The start, with any cartridge moved: ``moved`` maps a number to (xy, z, tilt)."""
    moved, moving = moved or {}, moving or set()
    objs = {"pump": _obj((*(M.START_XY["pump"] + [pump_dx, 0.0]), TOP + 0.07), (0.075, 0.07, 0.07)),
            "mirror": mirror or mirror_at(M.MIRROR_START)}
    for n, x in M.CARTS.items():
        xy, z, tilt = moved.get(n, ((x, M.CART_Y), TOP, 0.0))
        objs[f"cartridge_{n}"] = cart_at(xy, z, tilt, moving=n in moving)
    return objs


SET = mirror_at(M.MIRROR_GOAL)
ON_PAD = {M.CODE: (M.PAD_XY, TOP + M.PAD[2], 0.0)}
LIFTED = {M.CODE: ((0.30, 0.16), TOP + 0.10, 0.0)}


def test_start_scores_zero():
    assert M.score(scene()) == 0.0 and not M.success(scene())


def test_credit_levels():
    assert M.score(scene(mirror=SET)) == 0.25
    assert M.score(scene(moved=LIFTED)) == 0.25
    assert M.score(scene(mirror=SET, moved=LIFTED)) == 0.5
    assert M.score(scene(moved=ON_PAD)) == 0.5
    assert not M.success(scene(moved=ON_PAD))  # the right cartridge on the pad without the mirror set: no success
    assert M.score(scene(mirror=SET, moved=ON_PAD, moving={M.CODE})) == 0.75
    done = scene(mirror=SET, moved=ON_PAD)
    assert M.score(done) == M.COMPLETE == 0.9 and M.success(done)


def test_score_never_reaches_one():
    """1.0 comes only through success() held SUCCESS_HOLD_S: task_score takes the last step's score() otherwise."""
    assert M.SUCCESS_HOLD_S == 1.0
    assert max(M.score(scene(mirror=m, moved=c)) for m in (None, SET) for c in (None, LIFTED, ON_PAD)) < 1.0


@pytest.mark.parametrize("n", [1, 2, 4])
def test_a_wrong_cartridge_taken_scores_zero(n):
    x = M.CARTS[n]
    for move in (((x + 0.016, M.CART_Y), TOP, 0.0), ((x, M.CART_Y), TOP, 16.0), (M.PAD_XY, TOP + M.PAD[2], 0.0)):
        moved = {**ON_PAD, n: move}
        assert M.score(scene(mirror=SET, moved=moved)) == 0.0
        assert not M.success(scene(mirror=SET, moved=moved))
    nudged = {**ON_PAD, n: ((x + 0.014, M.CART_Y), TOP, 0.0)}  # bumped 14 mm: still left alone
    assert M.score(scene(mirror=SET, moved=nudged)) == 0.9


def test_a_wrong_cartridge_on_the_pad_earns_nothing():
    """The right number is CODE; the others are look-alikes: a guess that picks one scores 0."""
    assert M.CODE == 3 and set(M.CARTS) == {1, 2, 3, 4}
    for n in (1, 2, 4):
        assert M.score(scene(mirror=SET, moved={n: (M.PAD_XY, TOP + M.PAD[2], 0.0)})) == 0.0


def test_the_pump_moved_scores_zero():
    assert M.score(scene(mirror=SET, moved=ON_PAD, pump_dx=0.006)) == 0.0
    assert M.score(scene(mirror=SET, moved=ON_PAD, pump_dx=0.004)) == 0.9


@pytest.mark.parametrize("dx, dy, yaw, tilt, lift, ok", [
    (0.0, 0.0, 0.0, 0.0, 0.0, True),
    (-0.021, 0.0, 0.0, 0.0, 0.0, False),  # base into the hood's mouth
    (-0.019, 0.0, 0.0, 0.0, 0.0, True),
    (0.042, 0.0, 0.0, 0.0, 0.0, False),  # glass past the wall's face
    (0.039, 0.0, 0.0, 0.0, 0.0, True),
    (0.0, 0.041, 0.0, 0.0, 0.0, False), (0.0, 0.039, 0.0, 0.0, 0.0, True),
    (0.0, -0.041, 0.0, 0.0, 0.0, False),
    (0.0, 0.0, 21.0, 0.0, 0.0, False), (0.0, 0.0, -19.0, 0.0, 0.0, True),  # turned away from the pump
    (0.0, 0.0, 180.0, 0.0, 0.0, False),  # glass turned to the wall
    (0.0, 0.0, 0.0, 11.0, 0.0, False), (0.0, 0.0, 0.0, 9.0, 0.0, True),  # tipped
    (0.0, 0.0, 0.0, 0.0, 0.005, False),  # held above the table
])
def test_the_mirror_counts_only_set_behind_the_pump_facing_it(dx, dy, yaw, tilt, lift, ok):
    m = mirror_at((M.MIRROR_GOAL[0] + dx, M.MIRROR_GOAL[1] + dy), yaw, tilt, lift)
    assert M.score(scene(mirror=m)) == (0.25 if ok else 0.0)


@pytest.mark.parametrize("dx, dy, tilt, dz, ok", [
    (0.0, 0.0, 0.0, 0.0, True), (0.019, 0.0, 0.0, 0.0, True), (0.021, 0.0, 0.0, 0.0, False),
    (0.0, -0.021, 0.0, 0.0, False), (0.0, 0.0, 11.0, 0.0, False), (0.0, 0.0, 9.0, 0.0, True),
    (0.0, 0.0, 0.0, 0.005, False),  # held over the pad
    (0.0, 0.0, 90.0, 0.0, False),  # lying on its side
])
def test_on_the_pad(dx, dy, tilt, dz, ok):
    moved = {M.CODE: ((M.PAD_XY[0] + dx, M.PAD_XY[1] + dy), TOP + M.PAD[2] + dz, tilt)}
    expected = 0.9 if ok else 0.5  # off its slot and the mirror set: 0.25 + 0.25
    assert M.score(scene(mirror=SET, moved=moved)) == expected


def test_moving_parts_block_completion():
    for who in ("mirror", "pump", "cartridge_3", "cartridge_1"):
        objs = scene(mirror=SET, moved=ON_PAD)
        objs[who] = ObjectState(pos=objs[who].pos, quat_xyzw=objs[who].quat_xyzw, lin_vel=np.array([0.011, 0, 0]),
                                ang_vel=np.zeros(3), low=objs[who].low, high=objs[who].high)
        assert M.score(objs) == 0.75 and not M.success(objs), who
        objs[who] = ObjectState(pos=objs[who].pos, quat_xyzw=objs[who].quat_xyzw, lin_vel=np.zeros(3),
                                ang_vel=np.array([0, 0, 0.6]), low=objs[who].low, high=objs[who].high)
        assert M.score(objs) == 0.75, who


def test_layout():
    m = M.MIRROR_PART
    assert m.mass and M.PUMP.mass == 20.0 and all(c.mass for c in M.CARTRIDGES)
    assert M.GLASS[1:] == (57.0, 89.0) and math.degrees(M.TILT) == pytest.approx(50.0)
    # the mirror set at the goal fits between the hood's mouth and the wall: it goes down from above, clear of the hood
    base_x = M.MIRROR_GOAL[0]
    assert base_x - M.BASE[0] / 2000 > M.HOOD_END_X + 0.008
    glass_top_x = base_x + M.GLASS[2] / 1000 * math.sin(M.TILT) + M.GLASS[0] / 2000
    assert glass_top_x < M.WALL_X
    # the box offsets the tests use match the part
    lo = base_x - M.BASE[0] / 2000
    hi = glass_top_x
    assert (lo + hi) / 2 - base_x == pytest.approx(MIRROR_DX, abs=0.0015)
    # a horizontal sight line from the display's centre meets the glass
    reach_z = (M.DISPLAY_Z - M.BASE[2]) / math.cos(M.TILT)
    assert 0 < reach_z < M.GLASS[2]
    # the display and its digit fit inside the hood (the carried base passes beside the hood's mouth, checked above)
    assert M.HOOD_HALF_W * 2 - M.HOOD_T > M.DISPLAY[0] and M.DISPLAY_Z + M.DISPLAY[1] / 2 < M.HOOD_Z[1]
    assert M.DIGIT_H / 2 + M.STROKE / 2 < M.DISPLAY[1] / 2 and M.DIGIT_W / 2 + M.STROKE / 2 < M.DISPLAY[0] / 2
    assert M.HOOD_Z[0] < M.DISPLAY_Z - M.DISPLAY[1] / 2
    # the visor cuts every direct sight line to the display steeper than 40 degrees (a straight-down wrist camera behind
    # the pump looks from above that: traced, mirror removed)
    bottom = M.DISPLAY_Z - M.DISPLAY[1] / 2
    assert math.degrees(math.atan2(M.HOOD_Z[1] - bottom, M.HOOD_D)) < 40.0


def _fake_observation(t):
    class Obs:
        images = {"right_cam": np.zeros((176, 320, 3), np.uint8)}
        state_time = t
    return Obs()


def _play_read(digit, monkeypatch):
    """demo(), then the observations of a run whose reads all give ``digit``."""
    monkeypatch.setattr(M, "read_digit", lambda image: digit)
    demo = TASK.demo()
    t0, t1 = M._reading["window"]
    for t in np.arange(0.0, t1 + 0.5, 0.05):
        M._look(_fake_observation(float(t)))
    return demo


def test_the_demo_waits_for_the_code():
    """Before the read, and if nothing is read, the left arm stays at its standby pose: the layout alone fetches nothing."""
    demo = TASK.demo()
    assert all(w.pos == M.STANDBY for w in demo["left"])
    assert "CODE" not in "".join(TASK.demo.__code__.co_names + M._look.__code__.co_names + M._fetch.__code__.co_names)


@pytest.mark.parametrize("digit", [None, 1, 2, 3, 4])
def test_the_demo_fetches_the_cartridge_it_read(digit, monkeypatch):
    demo = _play_read(digit, monkeypatch)
    xs = {round(w.pos[0], 3) for w in demo["left"] if w.pos[1] == M.CART_Y}
    if digit is None:
        assert M._reading["read"] == 0 and all(w.pos == M.STANDBY for w in demo["left"])
    else:
        assert M._reading["read"] == digit and xs == {M.CARTS[digit]}
    # the read is made while the right arm holds the mirror behind the pump, before the left arm leaves standby
    t0, t1 = M._reading["window"]
    hold = demo["right"][M.READ_STEP]
    assert "hold" in hold.label and hold.pos == M.READ_POSE and hold.finger == M.FINGER_OPEN
    assert t1 == pytest.approx(sum(w.duration for w in demo["right"][:M.READ_STEP + 1]))
    assert t1 - t0 == pytest.approx(M.READ_HOLD - M.READ_SKIP)
    assert demo["right"][M.READ_STEP - 3].finger == M.FINGER_OPEN  # the mirror has been let go: it is set
    assert t1 < demo["left"][0].duration


@pytest.mark.parametrize("digit", [1, 2, 3, 4])
def test_demo_reaches(digit, monkeypatch):
    demo = _play_read(digit, monkeypatch)
    for arm in ("left", "right"):
        plan = ArmPlan(demo[arm], station.BASES[arm])
        for t in np.arange(0.0, plan.duration, 0.05):
            plan.action7(t)
        assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, arm
    spec = json.loads(Path(__file__).with_name("task.json").read_text())
    limit = int(spec["run_limit"].split()[0].replace(",", ""))
    longest = max(sum(w.duration for w in demo[a]) for a in demo)
    assert longest < limit / 20 * 0.6
    assert demo["right"][5].pos[:2] == pytest.approx((M.MIRROR_GOAL[0], M.MIRROR_GOAL[1] + M.POST_Y / 1000))


def _paint(rows, size=(31, 50), at=(25, 111)):
    """A right wrist frame with the display drawn from a reference grid: each cell darkened by its share in ninths."""
    img = np.full((176, 320, 3), (200, 180, 150), np.uint8)  # tan, like the table
    h, w = size
    box = np.full((h, w, 3), (207, 224, 210), np.uint8)
    gh, gw = M.READ_GRID
    for i, row in enumerate(rows):
        for j, v in enumerate(row):
            cell = box[i * h // gh:(i + 1) * h // gh, j * w // gw:(j + 1) * w // gw]
            k = cell.shape[0] * cell.shape[1]
            flat = cell.reshape(-1, 3)
            flat[np.round(np.linspace(0, k - 1, round(int(v) / 9 * k))).astype(int)] = (10, 10, 10)  # spread out
            cell[:] = flat.reshape(cell.shape)
    img[at[0]:at[0] + h, at[1]:at[1] + w] = box
    return img


@pytest.mark.parametrize("digit", [1, 2, 3, 4])
def test_read_digit_reads_each_reference(digit):
    assert M.read_digit(_paint(M.REFERENCE[digit])) == digit
    assert M.read_digit(_paint(M.REFERENCE[digit], at=(80, 40))) == digit  # wherever the display shows
    wider = _paint(M.REFERENCE[digit])
    wider[24, 110:162] = wider[56, 110:162] = wider[24:57, 110] = wider[24:57, 161] = (207, 224, 210)
    assert M.read_digit(wider) == digit  # its box 1 px bigger all round, as the blurred edges sometimes come out


def test_read_digit_refuses_what_it_cannot_read():
    assert M.read_digit(np.full((176, 320, 3), (200, 180, 150), np.uint8)) is None  # no display in view
    blank = ("0" * M.READ_GRID[1],) * M.READ_GRID[0]
    assert M.read_digit(_paint(blank)) is None  # the display, no digit
    mixed = [[str((int(a) + int(b)) // 2) for a, b in zip(r2, r3)] for r2, r3 in zip(M.REFERENCE[2], M.REFERENCE[3])]
    assert M.read_digit(_paint(mixed)) is None  # halfway between 2 and 3: neither clearly
    assert min(np.abs(np.array([[int(c) for c in r] for r in M.REFERENCE[a]])
                      - np.array([[int(c) for c in r] for r in M.REFERENCE[b]])).sum() / 9
               for a in M.REFERENCE for b in M.REFERENCE if a < b) > M.MATCH_MAX + M.MATCH_MARGIN
