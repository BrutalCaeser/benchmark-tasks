"""On the station table, to the robot's right: a white mechanical baby scale, its weighing beam running away from the
robot along the scale's right-hand edge, with an infant training manikin lying in the tray to the beam's left. The beam
has two bars side by side: the inner one carries the large sliding weight, which sits in notches, one per kilogram; the
outer one carries the small sliding weight, which slides freely along a 0 to 1,000 g bar. Both weights start at 0. At
the far end of the beam a red pointer rocks next to a fixed black counterpointer. Weigh the manikin: move the large weight
notch by notch until the beam tips down, then back one notch; then slide the small weight until the pointer floats level
with the counterpointer, and let go.
"""

import math
import os
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism  # noqa: E402
from parts import Part, Piece  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WHITE, GREY, STEEL = (0.94, 0.94, 0.95), (0.62, 0.62, 0.65), (0.75, 0.76, 0.79)
BLACK, RED, SKIN = (0.10, 0.10, 0.11), (0.85, 0.10, 0.10), (0.93, 0.78, 0.66)

# -- the scale, mm, in its own frame: origin on the table under the beam's pivot, y along the beam (away from the robot
# in the world), x across it (the tray side is -x), z up. The frame is turned so that its +y is the world's +x. ------
PIVOT_XY = (0.18, -0.17)  # world, m
YAW = -math.pi / 2
PZ = 110.0  # the pivot's height
A = 6.0  # the knife edge's distance from the pivot: the short arm that carries the tray's load
BAR_Z = PZ - 14.0  # the bars' axes: below the pivot, so the weights steady the beam
KG_X, G_X = -22.0, 22.0  # the large weight's bar (inner) and the small weight's bar (outer)
BAR_Y0, BAR_Y1 = 15.0, 285.0
KG_Y0, KG_STEP = 30.0, 15.0  # the 0 kg notch, then one notch per kilogram, 0 to 16 kg
G_Y0, G_SPAN = 30.0, 250.0  # 0 g on the small weight's bar; 1,000 g is G_SPAN further
M_KG = A / KG_STEP  # kg: moving it one notch balances 1 kg more in the tray
M_G = A / G_SPAN  # kg: moving it G_SPAN balances 1 kg more
# the manikin's mass, kg: drawn afresh each time the task file loads (each trial), uniformly from a newborn's range, to
# the gram. Nothing in the scene shows it, so the reading has to come from the beam. WEIGH_MANIKIN_KG sets it, to check a
# trial at a known mass.
MASS_RANGE = (2.5, 4.5)
BABY = float(os.environ.get("WEIGH_MANIKIN_KG") or round(random.SystemRandom().uniform(*MASS_RANGE), 3))
print(f"weigh_baby: the manikin weighs {BABY:.3f} kg in this trial (for the record; the policy never sees it)", flush=True)
TRAY_MASS = 0.6
BEAM_MASS = 0.9
STOP = math.radians(1.2)  # the beam rocks between these stops
TARE_Y, TARE_SIZE = -80.0, (40.0, 40.0)  # the counterweight on the short arm; its height is solved below


def _m(v):
    return tuple(float(x) / 1000 for x in v)


def _box(c, s, color=STEEL, material="metal"):
    return Shape("box", _m(c), _m(s), color, material=material)


def _cyl(c, r, length, axis, color=STEEL, material="metal"):
    return Shape("cylinder", _m(c), (r / 1000, length / 1000), color, axis, material)


NOTCH_Y = tuple(KG_Y0 + k * KG_STEP for k in range(17))
NOTCH_R, NOTCH_HALF = 2.0, 3.3  # each notch's two round bars: radius, and their axes from the notch's centre


def _beam_shapes(tare_h):
    return [
        _box((KG_X, (BAR_Y0 + BAR_Y1) / 2, BAR_Z), (8, BAR_Y1 - BAR_Y0, 6)),
        _box((G_X, (BAR_Y0 + BAR_Y1) / 2, BAR_Z), (8, BAR_Y1 - BAR_Y0, 6)),
        _box((0, 10, BAR_Z), (52, 10, 6)),
        _box((0, 290, BAR_Z), (52, 10, 6)),
        _box((0, 302.5, BAR_Z), (4, 15, 2), RED),  # the pointer
        _cyl((0, 0, PZ), 5, 16, "X"),
        _box((0, 5, (PZ + BAR_Z) / 2), (4, 10, 14)),
        # the knife edge: its axis on the pivot's line, so the tray's load neither steadies nor tips the beam
        _cyl((0, -A, PZ), 1.5, 60, "X", material="knife"),
        _box((0, TARE_Y, PZ - 1.3), (*TARE_SIZE, tare_h)),
        # each kilogram notch: two 4 mm round bars across the bar's top, 2.6 mm apart at their closest, between which
        # the large weight's 2.4 mm key stands flat on the bar; their round tops lead the key in
        # WHY flat on the bar, not cradled on the round bars: cradled, the weight rocked on its two contacts and the beam
        # crept for 15 s after each move (official sim, 2026-11-07)
        *[_cyl((KG_X, y + dy, BAR_Z + 3 + NOTCH_R), NOTCH_R, 8, "X", material="notch")
          for y in NOTCH_Y for dy in (-NOTCH_HALF, NOTCH_HALF)],
    ]


def _volume(shape):
    if shape.kind == "box":
        return float(np.prod(shape.size))
    r, length = shape.size
    return math.pi * r * r * length


def _centre_of_mass(shapes):
    """The physics engine spreads a link's mass over its shapes by volume."""
    v = np.array([_volume(s) for s in shapes])
    return (v[:, None] * np.array([s.center for s in shapes])).sum(axis=0) / v.sum()


def _beam_moment(tare_h):
    """kg m about the pivot from the beam's own mass (positive tips the long arm down)."""
    return BEAM_MASS * (_centre_of_mass(_beam_shapes(tare_h))[1])


def _solve_tare():
    """The counterweight's height that balances the empty tray with both weights at 0."""
    want = TRAY_MASS * A / 1000 - M_KG * KG_Y0 / 1000 - M_G * G_Y0 / 1000
    lo, hi = 1.0, 120.0
    for _ in range(80):  # the beam's moment falls as the counterweight grows
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _beam_moment(mid) > want else (lo, mid)
    return (lo + hi) / 2


TARE_H = _solve_tare()
BEAM_SHAPES = _beam_shapes(TARE_H)
TRAY_SHAPES = [
    _box((-24, -A, PZ + 1.5 + 20), (8, 4, 40), GREY, "knife"),  # the hanger, its foot on the knife edge
    _box((-40, -A, PZ + 40), (40, 4, 4), GREY),
    _box((-170, 150, PZ + 41), (220, 520, 6), WHITE, "tray"),
    _box((-170, -105, PZ + 50), (220, 10, 12), WHITE, "tray"),  # the tray's raised ends
    _box((-170, 405, PZ + 50), (220, 10, 12), WHITE, "tray"),
    Shape("capsule", _m((-170, 120, PZ + 44 + 40)), (0.040, 0.220), SKIN, "Y", "tray"),  # the manikin's body
    Shape("capsule", _m((-170, 315, PZ + 44 + 45)), (0.045, 0.001), SKIN, "Y", "tray"),  # its head
]
BASE_SHAPES = [
    _box((-161, 150, 70), (242, 552, 140), WHITE, "body"),  # under the tray: with the beam's housing, 552 by 312
    _box((0, 150, 30), (60, 552, 60), WHITE, "body"),  # under the beam: clear of the large weight's sleeve
    _box((-34, 0, (60 + PZ + 10) / 2), (8, 16, PZ + 10 - 60), GREY, "body"),  # the pivot's post
    _box((0, 320, BAR_Z), (4, 16, 2), BLACK, "body"),  # the counterpointer, at the pointer's height when level
    _box((0, 325, (60 + BAR_Z) / 2), (4, 4, BAR_Z - 60), BLACK, "body"),
]
SCALE = mechanism(
    "scale", pos=(*PIVOT_XY, TOP), yaw=YAW,
    base=Link("base", 6.0, tuple(BASE_SHAPES)),
    links=(Link("beam", BEAM_MASS, tuple(BEAM_SHAPES)), Link("tray", TRAY_MASS + BABY, tuple(TRAY_SHAPES))),
    joints=(Joint("beam", "revolute", "beam", _m((0, 0, PZ)), "X", -STOP, STOP, damping=0.1),
            Joint("tray", "prismatic", "tray", _m((-170, 150, PZ)), "Z", -0.004, 0.004, damping=1.0)),
    # WHY no friction at the knife edge: the knife turns under the hanger's foot as the beam rocks, and any friction
    # there acts as a brake on the beam (at 0.5 it held the beam against half a kilogram, official sim, 2026-11-07).
    # Real scales rest on hardened knife edges for this reason.
    # WHY slippery notch bars: the key has to slide down their round tops into the notch
    materials={"metal": (0.5, 0.4), "tray": (0.6, 0.5), "body": (0.6, 0.5), "knife": (0.0, 0.0), "notch": (0.1, 0.1)},
)


def _to_world(x, y):
    """A point of the scale's frame (mm) in the world (m)."""
    return (PIVOT_XY[0] + y / 1000, PIVOT_XY[1] - x / 1000)


KG_UP, KG_DOWN, KG_WALL, KG_KEY = 4.4, 12.0, 2.0, (6.0, 2.4, 4.4)  # inside the sleeve: above and below the bar
KG_TAB = (8.0, 8.0, 13.0)
G_UP, G_DOWN, G_WALL, G_TAB = 0.5, 0.5, 1.5, (6.0, 6.0, 12.0)


def _weight(name, x, y, mass, up, down, length, wall, tab, key=None, color=BLACK):
    """A sliding weight: a sleeve around its bar (the origin on the bar's axis), a tab on top to pinch and, for the
    large weight, a key under the sleeve's top that drops between a notch's two round bars."""
    inner_x, top_z = 9.0, 3 + up
    pieces = [
        Piece(_m((0, 0, top_z + wall / 2)), _m((inner_x + 2 * wall, length, wall))),
        Piece(_m((0, 0, -3 - down - wall / 2)), _m((inner_x + 2 * wall, length, wall))),
        *[Piece(_m((s * (inner_x / 2 + wall / 2), 0, (top_z - 3 - down) / 2)),
                _m((wall, length, top_z + 3 + down + 2 * wall))) for s in (-1, 1)],
        Piece(_m((0, 0, top_z + wall + tab[2] / 2)), _m(tab), GREY),
    ]
    if key:
        pieces.append(Piece(_m((0, 0, top_z - key[2] / 2)), _m(key)))
    size = (inner_x + 2 * wall, length, top_z + wall + tab[2] + 3 + down + wall)
    return Part(name, pos=(*_to_world(x, y), TOP + BAR_Z / 1000), size=_m(size), mass=mass, color=color,
                pieces=tuple(pieces), yaw=YAW)


KG_START, G_START = 0.0, 0.0  # kg, g: both weights start at 0
KG_W = _weight("kg_weight", KG_X, KG_Y0 + KG_START * KG_STEP, M_KG, KG_UP, KG_DOWN, 12.0, KG_WALL, KG_TAB, KG_KEY)
G_W = _weight("g_weight", G_X, G_Y0 + G_START / 1000 * G_SPAN, M_G, G_UP, G_DOWN, 10.0, G_WALL, G_TAB)
OBJECTS = [SCALE, KG_W, G_W]


def _bbox_z(up, down, wall, tab):
    """The bounding box centre's height above the bar's axis (Arena reports a part's box centre)."""
    return ((3 + up + wall + tab[2]) + (-3 - down - wall)) / 2 / 1000


KG_BOX_Z, G_BOX_Z = _bbox_z(KG_UP, KG_DOWN, KG_WALL, KG_TAB), _bbox_z(G_UP, G_DOWN, G_WALL, G_TAB)

# -- scoring --------------------------------------------------------------------------------------------------------
SUCCESS_HOLD_S = 1.0
STILL = 0.01  # m/s
IN_NOTCH = 0.001  # the large weight's centre within this of a notch, along the bar
SEATED = 0.0008  # and no higher than this above sitting in it
FREE = STOP - math.radians(0.1)  # the beam floats: clear of both stops
DONE, BANDS = 0.020, ((0.040, 0.75), (0.100, 0.5))  # kg from the true mass
# WHY score() stops at 0.9: the run scores 1.0 only through success() held for SUCCESS_HOLD_S, and otherwise the last
# step's score(); a 1.0 from score() would give full marks to a beam still settling in the run's last second
COMPLETE = 0.9


def _on_beam(state, th):
    """A weight's box centre in the beam's frame: (along the beam from the pivot, above the pivot), mm."""
    x, z = float(state.pos[0]) - PIVOT_XY[0], float(state.pos[2]) - TOP - PZ / 1000
    return (x * math.cos(th) + z * math.sin(th)) * 1000, (-x * math.sin(th) + z * math.cos(th)) * 1000


def measures(objects):
    """The reading from where the two weights are on the beam, and the beam's state."""
    th = float(objects["scale"].joints["beam"])
    kg, g = objects["kg_weight"], objects["g_weight"]
    kg_y, kg_up = _on_beam(kg, th)
    g_y, _ = _on_beam(g, th)
    notch = int(round((kg_y - KG_Y0) / KG_STEP))
    lifted = kg_up - (BAR_Z - PZ + KG_BOX_Z * 1000)  # above sitting on the bar
    seated = 0 <= notch <= 16 and abs(kg_y - NOTCH_Y[notch]) <= IN_NOTCH * 1000 and lifted <= SEATED * 1000
    reading = (kg_y - KG_Y0) / KG_STEP + (g_y - G_Y0) / G_SPAN  # kg, as the beam weighs it
    return {
        "beam_deg": math.degrees(th), "kg_y": kg_y, "g_y": g_y, "notch": notch, "seated": seated,
        "reading": reading, "error": abs(reading - BABY), "free": abs(th) < FREE,
        "still": max(float(np.linalg.norm(kg.lin_vel)), float(np.linalg.norm(g.lin_vel))) < STILL,
    }


def _complete(m) -> bool:
    """Weighed: the large weight in its notch, the reading within 20 g, the beam floating and both weights still."""
    return m["seated"] and m["error"] <= DONE and m["free"] and m["still"]


def score(objects) -> float:
    m = measures(objects)
    if not m["seated"]:
        return 0.0
    if _complete(m):
        return COMPLETE
    for band, credit in BANDS:
        if m["error"] <= band:
            return credit
    return 0.25 if m["notch"] == int(BABY) else 0.0


def success(objects) -> bool:
    return _complete(measures(objects))


# -- the scripted demo: it weighs the manikin from the pointer, as a person does --------------------------------------
# yam_scripted plays demo() open loop from the layout, and the manikin's mass is not in the layout. So the demo sets a
# weight, lets go, and holds its right wrist camera over the beam's far end (VIEW), where the red pointer shows against
# the black counterpointer; _look() reads those frames and says whether the beam tips down (the reading too high) or
# up, and writes the next moves into the waypoints still ahead (yam_scripted reads them at every control step).
# The large weight: 3 kg first (the manikin weighs at least 2.5), then one notch up or down; then back one if it
# tipped. The small weight: halving the bracket from 0 to 1,000 g six times leaves a 16 g bracket, and it ends in the
# middle of it.
OPEN_NARROW = -0.010  # 20 mm between the fingers: clear of the other bar's weight
# WHY fully closed on the large weight's tab: pinched to a 5 mm gap, the fingers slid up the 8 mm tab and the 400 g
# weight rose 0.3 mm of a 3 mm lift, its key still between the notch bars (traced, official sim, 2026-11-07)
KG_PINCH, G_PINCH = FINGER_CLOSED, -0.0018  # across the 8 mm and 6 mm tabs
# WHY a 15 mm lift: the arm sags under the 400 g weight; told to lift it 10 mm, it rose 3 mm, told 15 mm, 8.4 mm
# (traced, official sim, 2026-11-07). It has to clear the 4 mm notch bars, and the sleeve leaves it 12 mm.
LIFT = 0.015
AXIS_Z = TOP + BAR_Z / 1000
KG_TCP_Z = AXIS_Z + (3 + KG_UP + KG_WALL) / 1000 + 0.013 + 0.002  # fingertips 2 mm above the sleeve
G_TCP_Z = AXIS_Z + (3 + G_UP + G_WALL) / 1000 + 0.013 + 0.002
UP = 0.04
# WHY 3 mm toward the bar's far end, whichever way the weight moves: carried along its bar, it came to rest 2.4 mm nearer
# the pivot than the arm's target going out and 1.5 mm nearer coming back; aimed 3 mm past the notch on the way back,
# its key landed on the notch's near bar and slid off between notches (traced, official sim, 2026-11-12). The notch
# bars' round tops take up the rest.
KG_AIM_PAST = 3.0
# WHY 1 mm toward the far end: slid along its bar, the small weight stopped 1.1 to 1.3 mm nearer the pivot than the
# arm's target (traced, official sim, 2026-11-12)
G_AIM_PAST = 1.0
# WHY every move ends outward: slid back toward the pivot, the small weight trailed the fingers by 4 to 7 mm (16 to 31 g;
# aimed at 250 g it rested at 278 g, official sim, 2026-11-12), while slid outward it trails by about 1 mm. So it goes
# 40 g under the target first and comes out onto it.
G_UNDER = 40.0
KG_FIRST, KG_TRIES = 3, 3  # the first notch tried, and how many notch moves the plan has room for
G_HALVINGS = 6
VIEW, VIEW_YAW = (0.43, -0.22, TOP + 0.14), -0.3  # the right wrist camera sees the pointer beside the counterpointer
LOOK_S, READ_S, MARGIN_S = 2.2, 1.0, 0.15  # held at VIEW; frames of the last READ_S s, ending MARGIN_S before it ends
# The pointer in the right wrist camera at VIEW (pixels, x then y, centre of its red pixels) with the beam on its up
# stop and on its down stop, from runs with the weights set to tip it each way (official sim, 2026-11-12: about 69 and
# 50 red pixels, 13 px apart); level is halfway.
POINTER_UP, POINTER_DOWN = (43.2, 59.5), (55.0, 53.5)


def kg_world(notch, aim=0.0):
    return _to_world(KG_X, NOTCH_Y[notch] + aim)


def g_world(grams, aim=0.0):
    return _to_world(G_X, G_Y0 + grams / 1000 * G_SPAN + aim)


def pointer(image):
    """The centre (x, y) of the red pointer's pixels in a right wrist camera frame, or None if fewer than 4 show."""
    a = np.asarray(image, dtype=int)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    ys, xs = np.nonzero((r > 110) & (g < 70) & (b < 70) & (r > 2 * g))
    return (float(xs.mean()), float(ys.mean())) if len(xs) >= 4 else None


def tips_down(centres) -> bool:
    """Whether the pointer sat below level, from its centres in the frames read: their mean, along the line from where
    it shows on the up stop to where it shows on the down stop, past the midpoint."""
    up, down = np.array(POINTER_UP), np.array(POINTER_DOWN)
    c = np.mean(np.array(centres), axis=0)
    return float(np.dot(c - (up + down) / 2, down - up)) > 0.0


def _kg_move(frm, to):
    """Waypoints that take the large weight from notch frm to notch to and come back to VIEW (all holds if equal)."""
    if frm == to:
        return [Waypoint(d, VIEW, OPEN_NARROW, "hold: the large weight stays", VIEW_YAW) for d in _KG_TIMES]
    a, b = kg_world(frm), kg_world(to, KG_AIM_PAST)
    return [
        Waypoint(_KG_TIMES[0], (*a, KG_TCP_Z + UP), OPEN_NARROW, f"over the large weight at {frm} kg"),
        Waypoint(_KG_TIMES[1], (*a, KG_TCP_Z), OPEN_NARROW, "down to its tab"),
        Waypoint(_KG_TIMES[2], (*a, KG_TCP_Z), KG_PINCH, "pinch the tab"),
        Waypoint(_KG_TIMES[3], (*a, KG_TCP_Z + LIFT), KG_PINCH, "lift it off the notch"),
        Waypoint(_KG_TIMES[4], (*b, KG_TCP_Z + LIFT), KG_PINCH, f"carry it to {to} kg"),
        Waypoint(_KG_TIMES[5], (*b, KG_TCP_Z + 0.002), KG_PINCH, "down onto the notch"),
        Waypoint(_KG_TIMES[6], (*b, KG_TCP_Z + 0.002), OPEN_NARROW, "let go: it settles into the notch"),
        Waypoint(_KG_TIMES[7], (*b, KG_TCP_Z + UP), OPEN_NARROW, "up"),
        Waypoint(_KG_TIMES[8], VIEW, OPEN_NARROW, "to the pointer", VIEW_YAW),
    ]


def _g_move(frm, to):
    """Waypoints that slide the small weight from frm to to grams and come back to VIEW. It always ends on an outward
    slide: first past the target toward the pivot, then out onto it."""
    a, b = g_world(frm), g_world(to, G_AIM_PAST)
    under = g_world(max(0.0, to - G_UNDER), G_AIM_PAST)
    return [
        Waypoint(_G_TIMES[0], (*a, G_TCP_Z + UP), OPEN_NARROW, f"over the small weight at {frm:.0f} g"),
        Waypoint(_G_TIMES[1], (*a, G_TCP_Z), OPEN_NARROW, "down to its tab"),
        Waypoint(_G_TIMES[2], (*a, G_TCP_Z), G_PINCH, "pinch the tab"),
        Waypoint(_G_TIMES[3], (*under, G_TCP_Z), G_PINCH, f"slide it to just under {to:.0f} g"),
        Waypoint(_G_TIMES[4], (*b, G_TCP_Z), G_PINCH, f"slide it to {to:.0f} g"),
        Waypoint(_G_TIMES[5], (*b, G_TCP_Z), OPEN_NARROW, "let go"),
        Waypoint(_G_TIMES[6], (*b, G_TCP_Z + UP), OPEN_NARROW, "up"),
        Waypoint(_G_TIMES[7], VIEW, OPEN_NARROW, "to the pointer", VIEW_YAW),
    ]


_KG_TIMES = (1.5, 1.0, 0.6, 0.8, 2.0, 1.0, 0.6, 0.6, 1.5)
# WHY a quick let-go and lift: the weights are still from the end of the slide, so the 1 s hold that ends a trial can
# start while the fingers are still on the tab; at 0.6 s each, a run ended with the fingers just opening round it
# (official sim, 2026-11-12). At 0.3 and 0.4 s the hand is 4 cm clear before 1 s has passed.
_G_TIMES = (1.5, 1.0, 0.6, 2.4, 1.0, 0.3, 0.4, 1.5)
_plan = {"steps": [], "at": 0, "kg": 0, "g": (0.0, 1000.0), "g_at": 0.0, "reads": [], "log": []}


def _fill(slots, waypoints):
    for slot, w in zip(slots, waypoints):
        slot.pos, slot.finger, slot.label, slot.yaw = w.pos, w.finger, w.label, w.yaw


def _decide(down):
    """After a look: the next move, written into the next step's slots."""
    p = _plan
    i, steps = p["at"], p["steps"]
    kind, _, _ = steps[i]
    p["log"].append((kind, p["kg"], p["g_at"], down))
    if i + 1 >= len(steps):
        return
    nxt_kind, slots, _ = steps[i + 1]
    if kind == "kg":
        tried = sum(1 for k, *_ in steps[:i + 1] if k == "kg")
        if tried == 1:
            to = p["kg"] - 1 if down else p["kg"] + 1
        elif p["kg"] > KG_FIRST and down:  # tipped at 4 kg: back one
            to = p["kg"] - 1
        else:
            to = p["kg"]
        if nxt_kind == "kg":
            _fill(slots, _kg_move(p["kg"], to))
            p["kg"] = to
            return
    if kind == "g":
        lo, hi = p["g"]
        p["g"] = (lo, p["g_at"]) if down else (p["g_at"], hi)
    lo, hi = p["g"]
    to = (lo + hi) / 2
    _fill(slots, _g_move(p["g_at"], to))
    p["g_at"] = to


def _look(observation):
    """During each look window, collect the pointer's centre; after it, decide the next move."""
    p = _plan
    if p["at"] >= len(p["steps"]) or "right_cam" not in observation.images:
        return
    t = float(observation.state_time)
    start, end = p["steps"][p["at"]][2]
    if start <= t <= end:
        c = pointer(np.asarray(observation.images["right_cam"]))
        if c is not None:
            p["reads"].append(c)
    elif t > end:
        down = tips_down(p["reads"]) if p["reads"] else False
        print(f"weigh_baby demo: step {p['at']} ({p['steps'][p['at']][0]}), large weight {p['kg']} kg, small "
              f"{p['g_at']:.0f} g: beam tips {'down' if down else 'up'} ({len(p['reads'])} frames)", flush=True)
        _decide(down)
        p["at"] += 1
        p["reads"] = []


def _watch():
    """Hand each observation yam_scripted acts on to _look() (the policy is imported by the time it calls demo())."""
    scripted = sys.modules.get("yam_arena.scripted")
    if scripted is None:
        return
    policy = scripted.YamScriptedPolicy
    policy._weigh_look = staticmethod(_look)
    if not getattr(policy.act, "_weigh_watch", False):
        act = policy.act

        def act_and_look(self, observation):
            type(self)._weigh_look(observation)
            return act(self, observation)

        act_and_look._weigh_watch = True
        policy.act = act_and_look


def demo(delay=0.0):
    """``delay`` (s) slows the right arm's first move: the plan must not depend on starting at once."""
    right = _kg_move(0, KG_FIRST)
    right[0] = Waypoint(right[0].duration + delay, right[0].pos, right[0].finger, right[0].label)
    look = lambda: Waypoint(LOOK_S, VIEW, OPEN_NARROW, "watch the pointer against the counterpointer", VIEW_YAW)
    steps, t = [], 0.0
    right.append(look())
    t = sum(w.duration for w in right)
    steps.append(("kg", [], (t - MARGIN_S - READ_S, t - MARGIN_S)))
    for kind, n in (("kg", KG_TRIES - 1), ("g", G_HALVINGS)):
        for _ in range(n):
            slots = [Waypoint(w.duration, VIEW, OPEN_NARROW, "wait for the pointer", VIEW_YAW)
                     for w in (_kg_move(0, 1) if kind == "kg" else _g_move(0.0, 1.0))]
            right += slots + [look()]
            t = sum(w.duration for w in right)
            steps.append((kind, slots, (t - MARGIN_S - READ_S, t - MARGIN_S)))
    final = [Waypoint(w.duration, VIEW, OPEN_NARROW, "wait", VIEW_YAW) for w in _g_move(0.0, 1.0)]
    right += final + [Waypoint(4.0, (0.20, -0.30, TOP + 0.20), FINGER_OPEN, "away, hands off while the beam settles")]
    steps.append(("end", final, (math.inf, math.inf)))
    _plan.update(steps=steps, at=0, kg=KG_FIRST, g=(0.0, 1000.0), g_at=0.0, reads=[], log=[])
    _watch()
    left = [Waypoint(1.0, (0.20, 0.25, TOP + 0.20), FINGER_OPEN, "out of the way")]
    return {"left": left, "right": right}
