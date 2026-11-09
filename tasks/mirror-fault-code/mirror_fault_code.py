"""On the station table, straight ahead: a heavy grey pump with a wall behind it. Its controller shows a one-digit fault
code on a small backlit display on the pump's back face, which faces the wall, set in a short hood; no camera can see it.
To the robot's right lies an inspection mirror on a weighted base, its glass tilted 50 degrees, with a grey post to hold
it by. To the robot's left stand four blue filter cartridges, numbered 1 to 4 on their white tops, and a green pad. Set
the mirror on the table behind the pump, facing it, read the code in the mirror, and stand the cartridge with that number
on the green pad. Leave the other cartridges where they are.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from parts import GLOW, MIRROR, Part, Piece  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

GREY, DARK, STEEL = (0.55, 0.57, 0.60), (0.25, 0.26, 0.28), (0.70, 0.71, 0.74)
BLUE, WHITE, BLACK = (0.15, 0.35, 0.75), (0.95, 0.95, 0.95), (0.05, 0.05, 0.05)
GREEN, TAN = (0.20, 0.65, 0.30), (0.80, 0.70, 0.55)

CODE = 3  # the digit on the hidden display, and so the cartridge to fetch

# A seven-segment digit: segment name -> (offset to the viewer's right, offset up), in units of (width, height).
SEG_AT = {"a": (0, 0.5), "g": (0, 0.0), "d": (0, -0.5), "f": (-0.5, 0.25), "b": (0.5, 0.25),
          "e": (-0.5, -0.25), "c": (0.5, -0.25)}
DIGITS = {1: "bc", 2: "abged", 3: "abgcd", 4: "fgbc"}


def _m(v):
    return tuple(float(x) / 1000 for x in v)


# -- the pump: one heavy loose part, origin on the table under the centre of its body, mm in its own frame -------------
PUMP_XY = (0.185, -0.04)  # world, m
BODY = (120.0, 140.0, 140.0)
FACE_X = BODY[0] / 2  # its back face, toward the wall
DISPLAY_Z, DISPLAY = 50.0, (60.0, 60.0)  # the display's centre height and its width and height
HOOD_D, HOOD_T, HOOD_HALF_W = 30.0, 6.0, 40.0  # the hood round the display: depth, wall thickness, half width
DIGIT_H, DIGIT_W, STROKE = 40.0, 24.0, 6.0
PUMP_MASS = 20.0


def _pump():
    z0, z1 = DISPLAY_Z - 32.0, DISPLAY_Z + 38.0  # the hood's cheeks run from z0 up to the visor's underside at z1
    pieces = [Piece(_m((0, 0, BODY[2] / 2)), _m(BODY), GREY),
              Piece(_m((FACE_X + 1, 0, DISPLAY_Z)), _m((2, *DISPLAY)), GLOW),
              Piece(_m((FACE_X + HOOD_D / 2, 0, z1 + HOOD_T / 2)), _m((HOOD_D, 2 * HOOD_HALF_W + HOOD_T, HOOD_T)), DARK),
              *[Piece(_m((FACE_X + HOOD_D / 2, s * HOOD_HALF_W, (z0 + z1) / 2)), _m((HOOD_D, HOOD_T, z1 - z0)), DARK)
                for s in (1, -1)]]
    for seg in DIGITS[CODE]:
        right, up = SEG_AT[seg]
        size = (1.0, DIGIT_W, STROKE) if seg in "agd" else (1.0, STROKE, DIGIT_H / 2)
        # the display faces +x: its viewer's right is world -y
        pieces.append(Piece(_m((FACE_X + 2.5, -right * DIGIT_W, DISPLAY_Z + up * DIGIT_H)), _m(size), BLACK))
    return Part("pump", pos=(*PUMP_XY, TOP), size=_m((BODY[0] + HOOD_D, BODY[1], BODY[2])), mass=PUMP_MASS,
                color=GREY, pieces=tuple(pieces))


PUMP = _pump()
HOOD_END_X = PUMP_XY[0] + (FACE_X + HOOD_D) / 1000  # 0.275
WALL_X = 0.42  # the wall's face toward the pump
WALL = Box("wall", pos=(WALL_X + 0.01, PUMP_XY[1], TOP + 0.10), size=(0.02, 0.50, 0.20), color=TAN)

# -- the inspection mirror: a weighted base, a glass tilted TILT from vertical (its top away from the post side), a post
MIRROR_START = (0.25, -0.22)  # world, m: the base's centre
BASE = (80.0, 100.0, 12.0)
GLASS = (3.0, 57.0, 89.0)  # thickness, width, height: a 2-1/4 by 3-1/2 inch head
TILT = math.radians(50.0)
POST = (12.0, 12.0, 60.0)
POST_Y = -(BASE[1] / 2 - POST[1] / 2 - 2)  # at the base's -y edge, clear of the glass
MIRROR_MASS = 0.35


def _mirror():
    h = GLASS[2] / 2
    glass_c = (h * math.sin(TILT), 0.0, BASE[2] + h * math.cos(TILT))
    pieces = (Piece(_m((0, 0, BASE[2] / 2)), _m(BASE), STEEL),
              Piece(_m(glass_c), _m(GLASS), MIRROR, rot=(0.0, TILT, 0.0)),
              Piece(_m((0, POST_Y, BASE[2] + POST[2] / 2)), _m(POST), DARK))
    return Part("mirror", pos=(*MIRROR_START, TOP), size=_m((150.0, BASE[1], 90.0)), mass=MIRROR_MASS, color=STEEL,
                pieces=pieces)


MIRROR_PART = _mirror()

# -- four filter cartridges, numbered on their tops, and the green pad --------------------------------------------------
CART = (40.0, 40.0, 70.0)
CART_MASS = 0.12
CART_Y = 0.22
CARTS = {2: 0.20, 4: 0.26, 1: 0.32, 3: 0.38}  # number -> x (world, m): the numbers are not in order
PAD_XY, PAD = (0.30, 0.10), (0.08, 0.08, 0.002)


def _cartridge(n, x):
    pieces = [Piece(_m((0, 0, CART[2] / 2)), _m(CART), BLUE),
              Piece(_m((0, 0, CART[2] + 1)), _m((CART[0] - 2, CART[1] - 2, 2)), WHITE)]
    h, w, t = 24.0, 14.0, 3.0
    for seg in DIGITS[n]:
        right, up = SEG_AT[seg]
        # read from the robot's side: up is +x, the viewer's right is -y
        size = (t, w, 1.0) if seg in "agd" else (h / 2, t, 1.0)
        pieces.append(Piece(_m((up * h, -right * w, CART[2] + 2.5)), _m(size), BLACK))
    return Part(f"cartridge_{n}", pos=(x, CART_Y, TOP), size=_m((CART[0], CART[1], CART[2] + 3)), mass=CART_MASS,
                color=BLUE, pieces=tuple(pieces))


CARTRIDGES = [_cartridge(n, x) for n, x in CARTS.items()]
PAD_BOX = Box("pad", pos=(*PAD_XY, TOP + PAD[2] / 2), size=PAD, color=GREEN)
OBJECTS = [PUMP, WALL, MIRROR_PART, *CARTRIDGES, PAD_BOX]

# -- scoring ----------------------------------------------------------------------------------------------------------
SUCCESS_HOLD_S = 1.0
COMPLETE = 0.9  # score() tops out here: 1.0 comes only through success() held 1 s
ZONE_X = (HOOD_END_X, WALL_X)  # the mirror's whole footprint between the hood's mouth and the wall
ZONE_Y = 0.04  # its centre within this of the pump's centre line, sideways
FLAT_DEG, FACING_DEG = 10.0, 20.0  # the mirror's base flat, its glass turned to face the pump
ON_TABLE = 0.004  # m: a lowest point this close to its surface is resting on it
OFF_SLOT = 0.03  # m: the right cartridge counts as taken out at this distance from its start
LEFT_ALONE, LEFT_TILT_DEG = 0.015, 15.0  # a wrong cartridge further than this, or tipped, has been taken
PAD_TOL, UPRIGHT_DEG = 0.02, 10.0
PUMP_MOVED = 0.005
STILL, SPIN = 0.01, 0.5


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _tilt_deg(state):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(state.quat_xyzw)[2, 2])))))


def _yaw_deg(state):
    r = _rot(state.quat_xyzw)
    return math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0])))


def _bbox_xy(part):
    """The world-aligned bounding box centre (x, y) of a part at its start, as the simulator reports ``pos``."""
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for p in part.pieces:
        rx, ry, rz = p.rot
        c, s = math.cos(ry), math.sin(ry)
        r = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) if ry else np.eye(3)
        for corner in np.array(np.meshgrid(*[[-0.5, 0.5]] * 3)).T.reshape(-1, 3):
            pt = np.array(p.center) + r @ (corner * np.array(p.size))
            lo, hi = np.minimum(lo, pt), np.maximum(hi, pt)
    return np.array(part.pos[:2]) + (lo[:2] + hi[:2]) / 2


START_XY = {p.name: _bbox_xy(p) for p in (PUMP, *CARTRIDGES)}


def measures(objects):
    """What the score is made of, from the objects' states."""
    mirror, pump = objects["mirror"], objects["pump"]
    right = objects[f"cartridge_{CODE}"]
    wrong_taken = any(
        np.hypot(*(np.array(objects[c.name].pos[:2]) - START_XY[c.name])) > LEFT_ALONE
        or _tilt_deg(objects[c.name]) > LEFT_TILT_DEG
        for c in CARTRIDGES if c.name != right_name())
    mirror_set = (ZONE_X[0] <= float(mirror.low[0]) and float(mirror.high[0]) <= ZONE_X[1]
                  and abs(float(mirror.pos[1]) - PUMP_XY[1]) <= ZONE_Y and float(mirror.low[2]) <= TOP + ON_TABLE
                  and _tilt_deg(mirror) <= FLAT_DEG and abs(_yaw_deg(mirror)) <= FACING_DEG)
    off_slot = float(np.hypot(*(np.array(right.pos[:2]) - START_XY[right_name()]))) > OFF_SLOT
    on_pad = (float(np.hypot(right.pos[0] - PAD_XY[0], right.pos[1] - PAD_XY[1])) <= PAD_TOL
              and _tilt_deg(right) <= UPRIGHT_DEG and abs(float(right.low[2]) - (TOP + PAD[2])) <= ON_TABLE)
    pump_moved = float(np.hypot(*(np.array(pump.pos[:2]) - START_XY["pump"]))) > PUMP_MOVED
    loose = [mirror, pump, *(objects[c.name] for c in CARTRIDGES)]
    still = all(o.resting(STILL, SPIN) for o in loose)
    return {"wrong_taken": wrong_taken, "mirror_set": mirror_set, "off_slot": off_slot, "on_pad": on_pad,
            "pump_moved": pump_moved, "still": still}


def right_name():
    return f"cartridge_{CODE}"


def score(objects) -> float:
    """0 if a wrong cartridge has been taken or the pump moved; else 0.25 for the mirror set behind the pump, plus 0.25
    for the right cartridge out of its place or 0.5 for it on the pad; all of that and everything still: 0.9."""
    m = measures(objects)
    if m["wrong_taken"] or m["pump_moved"]:
        return 0.0
    credit = (0.25 if m["mirror_set"] else 0.0) + (0.5 if m["on_pad"] else 0.25 if m["off_slot"] else 0.0)
    if m["mirror_set"] and m["on_pad"] and m["still"]:
        return COMPLETE
    return credit


def success(objects) -> bool:
    m = measures(objects)
    return (m["mirror_set"] and m["on_pad"] and m["still"]) and not (m["wrong_taken"] or m["pump_moved"])


# -- the scripted demonstration: fixed waypoints from the known layout ------------------------------------------------
# It plays the layout as built; a policy has to read the code in the mirror (it shows in a wrist camera held over it).
MIRROR_GOAL = (0.32, PUMP_XY[1])  # the base's centre behind the pump: footprint 0.28 to 0.39 in x
GRAB_Z = TOP + (BASE[2] + POST[2] - 18) / 1000  # the fingers round the post's upper part
CARRY_Z = TOP + 0.12  # carried round the pump's side, its base 6.6 cm up
READ_Z = TOP + 0.09  # the code shows in the glass from here (traced)
ACROSS_X = math.pi / 2  # fingers closing along world x, across the post
POST_PINCH = -0.004
CART_GRAB_Z = TOP + 0.05
CART_PINCH = -0.016  # 40 mm box: 4 mm of squeeze
CART_CARRY_Z = TOP + 0.15


def demo(delay=0.0):
    """``delay`` (s) slows both arms' first move: the plan must not depend on starting at once."""
    mx, my = MIRROR_START
    gx, gy = MIRROR_GOAL
    post_from, post_to = (mx, my + POST_Y / 1000), (gx, gy + POST_Y / 1000)
    post_side = (gx, my + POST_Y / 1000)  # beside the pump, level with the goal: then straight in behind it
    right = [
        Waypoint(2.0 + delay, (*post_from, CARRY_Z), FINGER_OPEN, "over the mirror's post", ACROSS_X),
        Waypoint(1.5, (*post_from, GRAB_Z), FINGER_OPEN, "down to the post", ACROSS_X),
        Waypoint(0.8, (*post_from, GRAB_Z), POST_PINCH, "pinch the post", ACROSS_X),
        Waypoint(1.5, (*post_from, CARRY_Z), POST_PINCH, "lift", ACROSS_X),
        Waypoint(1.5, (*post_side, CARRY_Z), POST_PINCH, "past the pump's side", ACROSS_X),
        Waypoint(2.5, (*post_to, CARRY_Z), POST_PINCH, "in behind the pump", ACROSS_X),
        Waypoint(2.0, (*post_to, GRAB_Z + 0.002), POST_PINCH, "down onto the table", ACROSS_X),
        Waypoint(0.8, (*post_to, GRAB_Z + 0.002), FINGER_OPEN, "let go", ACROSS_X),
        Waypoint(1.0, (*post_to, READ_Z), FINGER_OPEN, "up a little: the right wrist camera looks into the glass", ACROSS_X),
        Waypoint(2.0, (*post_to, READ_Z), FINGER_OPEN, "hold: the code shows in the mirror", ACROSS_X),
        Waypoint(1.0, (*post_to, CARRY_Z), FINGER_OPEN, "up", ACROSS_X),
        Waypoint(2.0, (0.18, -0.25, TOP + 0.18), FINGER_OPEN, "out of the way"),
    ]
    wait = sum(w.duration for w in right) - 2.0  # (includes the delay)
    cx = CARTS[CODE]
    left = [
        Waypoint(wait, (0.18, 0.25, TOP + 0.18), FINGER_OPEN, "wait to the left"),
        Waypoint(2.0, (cx, CART_Y, CART_CARRY_Z), FINGER_OPEN, f"over cartridge {CODE}"),
        Waypoint(1.5, (cx, CART_Y, CART_GRAB_Z), FINGER_OPEN, "down"),
        Waypoint(0.8, (cx, CART_Y, CART_GRAB_Z), CART_PINCH, "grip it"),
        Waypoint(1.5, (cx, CART_Y, CART_CARRY_Z), CART_PINCH, "lift"),
        Waypoint(2.0, (*PAD_XY, CART_CARRY_Z), CART_PINCH, "over the pad"),
        Waypoint(2.0, (*PAD_XY, CART_GRAB_Z + PAD[2] + 0.001), CART_PINCH, "down onto the pad"),
        Waypoint(0.8, (*PAD_XY, CART_GRAB_Z + PAD[2] + 0.001), FINGER_OPEN, "let go"),
        Waypoint(1.5, (*PAD_XY, CART_CARRY_Z), FINGER_OPEN, "up"),
        Waypoint(2.0, (0.18, 0.25, TOP + 0.18), FINGER_OPEN, "out of the way, hands off"),
    ]
    return {"left": left, "right": right}
