"""On the station table, to the robot's right: a white mechanical baby scale, its weighing beam running away from the
robot along the scale's right-hand edge, with an infant training manikin lying in the tray to the beam's left. The beam
has two bars side by side: the inner one carries the large sliding weight, which sits in notches, one per kilogram; the
outer one carries the small sliding weight, which slides freely along a 0 to 1,000 g bar. Both weights start at 0. At
the far end of the beam a red pointer rocks next to a fixed black counterpointer. Weigh the manikin: move the large weight
notch by notch until the beam tips down, then back one notch; then slide the small weight until the pointer floats level
with the counterpointer, and let go.
"""

import math
import pathlib
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
BABY = 3.62  # the manikin, kg
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


# -- the scripted demo: the right arm moves the large weight to 3 kg, then the small weight to 620 g ----------------
# It goes straight to the manikin's known reading; a policy has to find it from the pointer against the counterpointer.
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
KG_AT = (int(BABY), round((BABY - int(BABY)) * 1000))  # 3 kg, 620 g
KG_FROM = _to_world(KG_X, KG_Y0 + KG_START * KG_STEP)
# WHY 3 mm past the 3 kg notch: carried along its bar, the weight stopped 2.9 to 4.1 mm short of the arm's target
# (traced, official sim, 2026-11-07); the notch bars' round tops take up the rest
KG_AIM_PAST = 3.0
KG_TO = _to_world(KG_X, NOTCH_Y[KG_AT[0]] + KG_AIM_PAST)
G_FROM = _to_world(G_X, G_Y0 + G_START / 1000 * G_SPAN)
# WHY 0.5 mm past 620 g: slid along its bar, the small weight stopped 0.4 mm short of the arm's target (traced,
# official sim, 2026-11-07)
G_AIM_PAST = 0.5
G_TO = _to_world(G_X, G_Y0 + KG_AT[1] / 1000 * G_SPAN + G_AIM_PAST)


def demo():
    up = 0.04
    right = [
        Waypoint(2.0, (*KG_FROM, KG_TCP_Z + up), OPEN_NARROW, "over the large weight"),
        Waypoint(1.5, (*KG_FROM, KG_TCP_Z), OPEN_NARROW, "down to its tab"),
        Waypoint(0.8, (*KG_FROM, KG_TCP_Z), KG_PINCH, "pinch the tab"),
        Waypoint(1.0, (*KG_FROM, KG_TCP_Z + LIFT), KG_PINCH, "lift it off the notch"),
        Waypoint(3.0, (*KG_TO, KG_TCP_Z + LIFT), KG_PINCH, "slide it to 3 kg"),
        Waypoint(1.5, (*KG_TO, KG_TCP_Z + 0.002), KG_PINCH, "down onto the notch"),
        Waypoint(0.8, (*KG_TO, KG_TCP_Z + 0.002), OPEN_NARROW, "let go: it settles into the notch"),
        Waypoint(1.0, (*KG_TO, KG_TCP_Z + up), OPEN_NARROW, "up"),
        Waypoint(2.0, (*G_FROM, G_TCP_Z + up), OPEN_NARROW, "over the small weight"),
        Waypoint(1.5, (*G_FROM, G_TCP_Z), OPEN_NARROW, "down to its tab"),
        Waypoint(0.8, (*G_FROM, G_TCP_Z), G_PINCH, "pinch the tab"),
        Waypoint(5.0, (*G_TO, G_TCP_Z), G_PINCH, "slide it to 620 g"),
        Waypoint(0.8, (*G_TO, G_TCP_Z), OPEN_NARROW, "let go"),
        Waypoint(1.0, (*G_TO, G_TCP_Z + up), OPEN_NARROW, "up"),
        Waypoint(6.0, (0.20, -0.30, TOP + 0.20), FINGER_OPEN, "away, hands off while the beam settles"),
    ]
    left = [Waypoint(1.0, (0.20, 0.25, TOP + 0.20), FINGER_OPEN, "out of the way")]
    return {"left": left, "right": right}
