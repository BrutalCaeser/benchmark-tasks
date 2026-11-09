"""On the station table, straight ahead: a heavy grey pump with a wall behind it. Its controller shows a one-digit fault
code on a small backlit display on the pump's back face, which faces the wall, set in a deep hood, out of the cameras'
view. To the robot's right lies an inspection mirror on a weighted base, its glass tilted 50 degrees, with a grey post
to hold it by. To the robot's left stand four blue filter cartridges, numbered 1 to 4 on their white tops, and a green
pad. Set the mirror on the table behind the pump, facing it, read the code in the mirror, and stand the cartridge with
that number on the green pad. Leave the other cartridges where they are.
"""

import itertools
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from parts import GLOW, MIRROR, Part, Piece  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

GREY, DARK, STEEL = (0.55, 0.57, 0.60), (0.25, 0.26, 0.28), (0.70, 0.71, 0.74)
BLUE, WHITE, BLACK = (0.15, 0.35, 0.75), (0.95, 0.95, 0.95), (0.05, 0.05, 0.05)
GREEN, TAN = (0.20, 0.65, 0.30), (0.80, 0.70, 0.55)

# The digit on the hidden display, and so the cartridge to fetch. The environment variable is only for checking that the
# demo reads the code (its runs with 1, 2 and 4); the official task is 3.
CODE = int(os.environ.get("MIRROR_FAULT_CODE", "3"))

# A seven-segment digit: segment name -> (offset to the viewer's right, offset up), in units of (width, height).
SEG_AT = {"a": (0, 0.5), "g": (0, 0.0), "d": (0, -0.5), "f": (-0.5, 0.25), "b": (0.5, 0.25),
          "e": (-0.5, -0.25), "c": (0.5, -0.25)}
DIGITS = {1: "bc", 2: "abged", 3: "abgcd", 4: "fgbc"}
if CODE not in DIGITS:
    raise ValueError(f"MIRROR_FAULT_CODE must be 1 to 4, got {CODE}")


def _m(v):
    return tuple(float(x) / 1000 for x in v)


# -- the pump: one heavy loose part, origin on the table under the centre of its body, mm in its own frame -------------
PUMP_XY = (0.185, -0.04)  # world, m
BODY = (120.0, 140.0, 140.0)
FACE_X = BODY[0] / 2  # its back face, toward the wall
DISPLAY_Z, DISPLAY = 50.0, (60.0, 36.0)  # the display's centre height and its width and height
HOOD_D, HOOD_T, HOOD_HALF_W = 55.0, 6.0, 40.0  # the hood round the display: depth, wall thickness, half width
DIGIT_H, DIGIT_W, STROKE = 28.0, 18.0, 5.0
PUMP_MASS = 20.0
HOOD_Z = (20.0, DISPLAY_Z + DISPLAY[1] / 2 + 4.0)  # the cheeks' foot, the visor's underside: 4 mm over the display


def _pump():
    z0, z1 = HOOD_Z  # the hood's cheeks run from z0 up to the visor's underside at z1
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
HOOD_END_X = PUMP_XY[0] + (FACE_X + HOOD_D) / 1000  # 0.300: the hood's mouth
WALL_X = 0.47  # the wall's face toward the pump: the wrist camera clears it while setting the mirror
WALL = Box("wall", pos=(WALL_X + 0.01, PUMP_XY[1], TOP + 0.125), size=(0.02, 0.40, 0.25), color=TAN)  # clear of the cartridges

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


# -- the scripted demonstration --------------------------------------------------------------------------------------
# yam_scripted plays demo() open loop from the known layout, and the code is not in the layout: the demo reads it. demo()
# wraps the policy's act() so that each observation reaches _look(), which reads the right wrist camera once the mirror
# is set and the arm holds over it; the left arm's fetch stays on standby until then and goes to the cartridge read.
MIRROR_GOAL = (0.36, PUMP_XY[1])  # the base's centre behind the pump: footprint 0.32 to 0.429 in x
GRAB_Z = TOP + (BASE[2] + POST[2] - 18) / 1000  # the fingers round the post's upper part
CARRY_Z = TOP + 0.12  # carried round the pump's side, its base 6.6 cm up (the base stays clear of the hood's mouth)
POST_YAW = math.pi / 2  # fingers closing along world x, across the post
READ_YAW = 0.0  # the wrist turned for the read: its camera looks into the glass from the wall's side
POST_PINCH = FINGER_CLOSED  # squeezed hard: the mirror hangs off the post's side and must not swing
CART_GRAB_Z = TOP + 0.05
CART_PINCH = -0.016  # 40 mm box: 4 mm of squeeze
CART_CARRY_Z = TOP + 0.15
STANDBY = (0.18, 0.25, TOP + 0.18)  # the left arm waits here, and stays if the code was not read
READ_POSE = (0.35, PUMP_XY[1] - 0.025, TOP + 0.08)  # the right wrist over the set mirror: the code shows in its camera
READ_HOLD, READ_SKIP = 2.0, 0.5  # s held there; frames from READ_SKIP s in are read
READ_STEP = 10  # the right arm's waypoint that holds there

_reading = {"window": (0.0, 0.0), "votes": [], "fetch": [], "read": None}


# The display as the right wrist camera sees it in the mirror from READ_POSE: the box round its lit (light green) pixels,
# split into a grid, each cell's share of dark pixels in ninths (0 to 9). One reference per digit, measured from frames of
# runs with each code.
READ_GRID = (6, 9)
REFERENCE = {
    1: ("222001000", "000004200", "000003300", "100003400", "200003400", "200112211"),
    2: ("221555000", "000225200", "000125200", "100754000", "200600000", "200677111"),
    3: ("221555000", "000225200", "000125300", "100456400", "200003400", "200577311"),
    4: ("222101000", "002504200", "000625300", "100456400", "200003400", "200112211"),
}
MATCH_MAX, MATCH_MARGIN = 2.0, 0.8  # summed cell differences: a read is this close to its digit and this much closer
# than to any other. Each edge of the box may move 1 px either way (its edge pixels blur; a run whose box came out 1 px
# wider read nothing without this). Frames from the runs with each code and from that run: at most 1.54 from their own
# digit, at least 2.02 nearer it.


def read_digit(image):
    """The digit (1 to 4) the display shows in the mirror, in a right wrist camera frame from READ_POSE; None if the
    display is not found or the frame matches no digit clearly."""
    a = np.asarray(image, dtype=int)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    lit = (g > 170) & (r > 150) & (g - r > 8) & (g - b > 4)  # the display's light green (the pad is a deeper green)
    rows, cols = np.flatnonzero(lit.sum(axis=1) >= 3), np.flatnonzero(lit.sum(axis=0) >= 3)
    if len(rows) < 8 or len(cols) < 8:
        return None
    refs = {d: np.array([[int(c) for c in row] for row in ref]) / 9 for d, ref in REFERENCE.items()}
    dist = dict.fromkeys(refs, math.inf)
    for top, bottom, left, right in itertools.product((-1, 0, 1), repeat=4):
        r0, r1 = max(rows[0] + top, 0), rows[-1] + 1 + bottom
        c0, c1 = max(cols[0] + left, 0), cols[-1] + 1 + right
        dark = a[r0:r1, c0:c1].max(axis=2) < 110
        (h, w), (gh, gw) = dark.shape, READ_GRID
        cells = np.array([[dark[i * h // gh:(i + 1) * h // gh, j * w // gw:(j + 1) * w // gw].mean() for j in range(gw)]
                          for i in range(gh)])
        for d, ref in refs.items():
            dist[d] = min(dist[d], float(np.abs(cells - ref).sum()))
    best, second = sorted(dist, key=dist.get)[:2]
    return best if dist[best] <= MATCH_MAX and dist[second] - dist[best] >= MATCH_MARGIN else None


def _look(observation):
    """Read the code during the hold; once the hold is over, point the left arm's fetch at the cartridge read."""
    r = _reading
    if r["read"] is not None or "right_cam" not in observation.images:
        return
    t = float(observation.state_time)
    if r["window"][0] <= t <= r["window"][1]:
        digit = read_digit(np.asarray(observation.images["right_cam"]))
        if digit is not None:
            r["votes"].append(digit)
    elif t > r["window"][1]:
        votes = r["votes"]
        r["read"] = max(sorted(set(votes)), key=votes.count) if votes else 0
        n = r["read"]
        print(f"mirror_fault_code demo: read {n or 'nothing'} in the mirror ({votes.count(n)} of {len(votes)} frames"
              f" read at t {r['window'][0]:.1f}-{r['window'][1]:.1f} s; votes {dict((d, votes.count(d)) for d in set(votes))})",
              flush=True)
        if n:
            for slot, w in zip(r["fetch"], _fetch(n)):
                slot.pos, slot.finger, slot.label = w.pos, w.finger, w.label


def _watch():
    """Hand each observation yam_scripted acts on to _look() (the policy is imported by the time it calls demo())."""
    scripted = sys.modules.get("yam_arena.scripted")
    if scripted is None:
        return
    policy = scripted.YamScriptedPolicy
    policy._mirror_look = staticmethod(_look)
    if not getattr(policy.act, "_mirror_watch", False):
        act = policy.act

        def act_and_look(self, observation):
            type(self)._mirror_look(observation)
            return act(self, observation)

        act_and_look._mirror_watch = True
        policy.act = act_and_look


def _fetch(n):
    """The left arm's fetch of cartridge n from its place in the layout to the pad."""
    cx = CARTS[n]
    return [
        Waypoint(2.0, (cx, CART_Y, CART_CARRY_Z), FINGER_OPEN, f"over cartridge {n}"),
        Waypoint(1.5, (cx, CART_Y, CART_GRAB_Z), FINGER_OPEN, "down"),
        Waypoint(0.8, (cx, CART_Y, CART_GRAB_Z), CART_PINCH, "grip it"),
        Waypoint(1.5, (cx, CART_Y, CART_CARRY_Z), CART_PINCH, "lift"),
        Waypoint(2.0, (*PAD_XY, CART_CARRY_Z), CART_PINCH, "over the pad"),
        Waypoint(2.0, (*PAD_XY, CART_GRAB_Z + PAD[2] + 0.001), CART_PINCH, "down onto the pad"),
        Waypoint(0.8, (*PAD_XY, CART_GRAB_Z + PAD[2] + 0.001), FINGER_OPEN, "let go"),
        Waypoint(1.5, (*PAD_XY, CART_CARRY_Z), FINGER_OPEN, "up"),
    ]


def demo(delay=0.0):
    """``delay`` (s) slows both arms' first move: the plan must not depend on starting at once."""
    mx, my = MIRROR_START
    gx, gy = MIRROR_GOAL
    post_from, post_to = (mx, my + POST_Y / 1000), (gx, gy + POST_Y / 1000)
    post_side = (gx, my + POST_Y / 1000)  # beside the pump, level with the goal: then straight in behind it
    right = [
        Waypoint(2.0 + delay, (*post_from, CARRY_Z), FINGER_OPEN, "over the mirror's post", POST_YAW),
        Waypoint(1.5, (*post_from, GRAB_Z), FINGER_OPEN, "down to the post", POST_YAW),
        Waypoint(0.8, (*post_from, GRAB_Z), POST_PINCH, "pinch the post", POST_YAW),
        Waypoint(1.5, (*post_from, CARRY_Z), POST_PINCH, "lift", POST_YAW),
        Waypoint(1.5, (*post_side, CARRY_Z), POST_PINCH, "past the pump's side", POST_YAW),
        Waypoint(2.5, (*post_to, CARRY_Z), POST_PINCH, "in behind the pump", POST_YAW),
        Waypoint(2.5, (*post_to, GRAB_Z + 0.002), POST_PINCH, "down onto the table", POST_YAW),
        Waypoint(0.8, (*post_to, GRAB_Z + 0.002), FINGER_OPEN, "let go", POST_YAW),
        Waypoint(1.0, (*post_to, CARRY_Z), FINGER_OPEN, "up", POST_YAW),
        Waypoint(1.5, READ_POSE, FINGER_OPEN, "over the mirror", READ_YAW),
        Waypoint(READ_HOLD, READ_POSE, FINGER_OPEN, "hold: the code shows in the right wrist camera", READ_YAW),
        Waypoint(2.0, (0.18, -0.25, TOP + 0.18), FINGER_OPEN, "out of the way"),
    ]
    hold_end = sum(w.duration for w in right[:READ_STEP + 1])
    fetch = [Waypoint(w.duration, STANDBY, FINGER_OPEN, "standby: the code is not read yet") for w in _fetch(1)]
    _reading.update(window=(hold_end - READ_HOLD + READ_SKIP, hold_end), votes=[], fetch=fetch, read=None)
    _watch()
    left = [
        Waypoint(hold_end + 0.5, STANDBY, FINGER_OPEN, "wait to the left until the code is read"),
        *fetch,
        Waypoint(2.0, STANDBY, FINGER_OPEN, "out of the way, hands off"),
    ]
    return {"left": left, "right": right}
