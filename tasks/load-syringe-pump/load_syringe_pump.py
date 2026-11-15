"""On the station table, 33 cm out and 2 cm left of the midline: a white syringe pump, its cradle (two white rods the
barrel rests between) running left-right. At the cradle's left end is the narrow slot that takes the syringe's barrel
flange, and past it, on its own track, the grey plunger driver, pulled back. To the right, nearer the arms, a filled syringe (a clear barrel 3 cm across with a
flange at one end and a white plunger in it) lies on two black stands, flange to the left. Behind the cradle stands
the barrel clamp: an arm on a sprung post, its pad resting down in the empty cradle, and beside it the pump's handle.
Load it as the Medfusion 3500's card says: lift the clamp and swivel it away onto the handle, lay the barrel in the
cradle with the flange down in its slot, bring the driver up against the end of the plunger without pushing it in,
then turn the clamp back and lower it onto the barrel.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism, on_reset  # noqa: E402
from parts import Part, Piece  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WHITE, DARK, GREY = (0.92, 0.92, 0.92), (0.15, 0.15, 0.17), (0.55, 0.55, 0.58)
CLEAR, PLUNGER_WHITE, STAND_BLACK = (0.80, 0.85, 0.90), (0.95, 0.95, 0.95), (0.12, 0.12, 0.13)

# -- the pump, mm, in its frame: origin on the table under the cradle's middle; the cradle runs along y ---------
PUMP_XY = (0.33, 0.02)
FLOOR_Z = 25.0  # the seated barrel's lowest point; the slot's depth is measured from here
BARREL_R, BARREL_L = 14.8, 100.0  # stand-in for a 60 mL syringe [UNVERIFIED]
AXIS_Z = FLOOR_Z + BARREL_R  # the barrel's axis, seated
BODY_TOP = FLOOR_Z - 1.5  # the pump's top, under the barrel: the barrel rests on the rods, not on the body
SLIT_Y, SLIT_W, SLIT_DEPTH = 31.25, 3.5, 14.0  # the flange slot: centre, width (flange 2.5 mm), depth below the floor
FLANGE_T, FLANGE_W, FLANGE_H = 2.5, 40.0, 50.0
# WHY a rounded mouth (5 mm): with square edges the flange landed on the body 3 to 5 mm short of the slot, the arm
# wandering 2 to 5 mm on the way down, and the syringe sat cocked 8 degrees (official sim, 2026-11-05, five runs).
# Real flange clips have a lead-in for the same reason.
FUNNEL_R = 5.0
DRIVER_HOME_GAP = 30.0  # the driver's face starts this far from the plunger's thumb disc
STOPPER_AT = 35.0  # the stopper's centre from the barrel's centre, toward the flange: a full syringe
# WHY a 20 mm stopper with 0.1 mm play: a 10 mm stopper with 0.3 mm play let the plunger sag 3.6 degrees under its thumb
# disc, whose rim then met the driver 0.94 mm before its centre did (traced, 2026-11-06). Real stoppers fit tightly.
STOPPER_L, STOPPER_SQ = 20.0, 18.67  # two squares, one turned 45 degrees: corners 13.2 mm out, the bore 13.3
THUMB_FROM_STOPPER = 53.5  # the thumb disc's outer face from the stopper's centre
_fy = BARREL_L / 2 + FLANGE_T / 2  # the flange's centre from the barrel's centre
BARREL_SEAT_Y = SLIT_Y - _fy  # the barrel's centre, seated, in the pump frame (-20 mm)
# the cradle: two rods along y that the barrel rests between, touching it 45 degrees either side of its lowest line, so
# a barrel set down a little off the line rolls onto it (the barrel's staves face 45 degrees there, so the contact is on
# a flat face and the seated axis sits at AXIS_Z)
ROD_R = 5.0
ROD_X = (BARREL_R + ROD_R) * math.sqrt(0.5)  # 14.0
ROD_Z = AXIS_Z - (BARREL_R + ROD_R) * math.sqrt(0.5)  # 25.8

near, far = SLIT_Y - SLIT_W / 2, SLIT_Y + SLIT_W / 2  # 29.5, 33.0
PUMP_BASE = (
    ("body", (0, (-75 + near - FUNNEL_R) / 2, BODY_TOP / 2), (50, near - FUNNEL_R + 75, BODY_TOP)),
    ("funnel_near", (0, near - FUNNEL_R / 2, (BODY_TOP - FUNNEL_R) / 2), (50, FUNNEL_R, BODY_TOP - FUNNEL_R)),
    ("funnel_far", (0, far + FUNNEL_R / 2, (BODY_TOP - FUNNEL_R) / 2), (50, FUNNEL_R, BODY_TOP - FUNNEL_R)),
    ("body_past", (0, (far + FUNNEL_R + 45) / 2, BODY_TOP / 2), (50, 45 - far - FUNNEL_R, BODY_TOP)),
    ("slit_floor", (0, SLIT_Y, (FLOOR_Z - SLIT_DEPTH) / 2), (50, SLIT_W, FLOOR_Z - SLIT_DEPTH)),
    ("track", (0, 82.5, 10), (50, 75, 20)),
)
ROD_Y = (-75.0, near - FUNNEL_R)  # the rods run the body's length, up to the slot's mouth

# the barrel clamp, behind the cradle (+x): a post that lifts against a spring, and on it an arm that swivels about the
# post, with a pad under its tip that comes down on the barrel's top and a knob on the post to lift and turn it by.
# Medfusion 3500 quick reference card: "Lift the syringe barrel clamp and swivel it away", "rest it on the handle", and
# at the end "Turn and lower the barrel clamp onto the barrel of the syringe". Stand-in sizes [UNVERIFIED]: measure them on the real pump.
POST_X, POST_Y = 40.0, -40.0  # the post's axis, behind the cradle, 2 cm toward the nose from the barrel's middle
PAD_BOTTOM = AXIS_Z + BARREL_R  # the pad's underside with the clamp at 0: on a seated barrel's top (54.6 mm)
ARM_T = 6.0  # the arm and its pad, one bar 6 mm thick
ARM_Z = PAD_BOTTOM + ARM_T / 2
PAD = ((0.0, POST_Y, ARM_Z), (16.0, 30.0, ARM_T))  # the arm's tip, over the barrel's axis
ARM = (((POST_X + 6.0 + 8.0) / 2, POST_Y, ARM_Z), (POST_X + 6.0 - 8.0, 10.0, ARM_T))  # from the pad to the post
KNOB_R, KNOB_H = 7.0, 24.0
KNOB_Z = ARM_Z + ARM_T / 2 + KNOB_H / 2  # the knob's centre, on the arm at the post
POST_ROD = ((POST_X, POST_Y, (10.0 + ARM_Z - ARM_T / 2) / 2), (4.0, ARM_Z - ARM_T / 2 - 10.0))  # centre, (radius, length)
POST_BLOCK = ((POST_X, POST_Y, BODY_TOP / 2), (14.0, 14.0, BODY_TOP))
CLAMP_DOWN, CLAMP_UP = -0.006, 0.020  # the post's travel (m): down to its stop in an empty cradle, up to its top
# WHY a light spring (0.5 N on a barrel, with the clamp's own 0.3 N): the arm gives about 6 mm per newton, so a stiff
# spring would leave the lifting hand short of clearing the barrel [UNVERIFIED: the real clamp's force isn't published]
CLAMP_K, CLAMP_SET = 25.0, -0.020  # N/m, and the spring's target below the stop
SWIVEL_MAX = math.radians(97.0)  # the arm swivels from over the cradle (0) to past the handle (90)
HANDLE_TOP = PAD_BOTTOM - 4.6  # swivelled onto the handle, the arm rests 4.6 mm lower than on a barrel
HANDLE = ((POST_X + 1.0, POST_Y - (POST_X - 0.0), HANDLE_TOP / 2), (30.0, 30.0, HANDLE_TOP))  # under the parked pad


def _m(v):
    return tuple(x / 1000 for x in v)


def _pump_shapes():
    shapes = [Shape("box", _m(c), _m(s), DARK if name == "slit_floor" else WHITE, contact_offset=0.001)
              for name, c, s in PUMP_BASE]
    for y in (near - FUNNEL_R, far + FUNNEL_R):  # the slot's rounded mouth
        shapes.append(Shape("cylinder", _m((0, y, BODY_TOP - FUNNEL_R)), (FUNNEL_R / 1000, 0.050), WHITE, axis="X",
                            contact_offset=0.001))
    for x in (ROD_X, -ROD_X):  # the cradle's rods
        shapes.append(Shape("cylinder", _m((x, sum(ROD_Y) / 2, ROD_Z)), (ROD_R / 1000, (ROD_Y[1] - ROD_Y[0]) / 1000),
                            WHITE, axis="Y", contact_offset=0.001))
    shapes.append(Shape("box", _m(POST_BLOCK[0]), _m(POST_BLOCK[1]), WHITE, contact_offset=0.001))  # the clamp's housing
    return tuple(shapes)


THUMB_FACE_Y = BARREL_SEAT_Y + STOPPER_AT + THUMB_FROM_STOPPER  # the thumb disc's outer face, seated (68.5 mm)
DRIVER_FACE_Y = THUMB_FACE_Y + DRIVER_HOME_GAP
DRIVER = ((0, DRIVER_FACE_Y + 10, 35.5), (40, 20, 30))
TAB = ((0, DRIVER_FACE_Y + 10, 55.5), (12, 6, 10))  # the grip on the driver's top, where a nurse squeezes it
DRIVER_TRAVEL = 0.040
PUMP = mechanism(
    "pump", (*PUMP_XY, TOP), base=Link("pump_body", 1.0, _pump_shapes()),
    links=(Link("driver", 0.05, (Shape("box", _m(DRIVER[0]), _m(DRIVER[1]), GREY, contact_offset=0.001),
                                  Shape("box", _m(TAB[0]), _m(TAB[1]), DARK, contact_offset=0.001))),
           Link("clamp_post", 0.01, (Shape("cylinder", _m(POST_ROD[0]), _m(POST_ROD[1]), GREY, contact_offset=0.001),)),
           Link("clamp_arm", 0.02, (Shape("box", _m(ARM[0]), _m(ARM[1]), WHITE, contact_offset=0.001),
                                    Shape("box", _m(PAD[0]), _m(PAD[1]), DARK, contact_offset=0.001),
                                    Shape("cylinder", _m((POST_X, POST_Y, KNOB_Z)), _m((KNOB_R, KNOB_H)), DARK,
                                          contact_offset=0.001)))),
    joints=(Joint("driver_slide", "prismatic", "driver", _m(DRIVER[0]), "Y", -DRIVER_TRAVEL, 0.0, damping=2.0),
            Joint("clamp_lift", "prismatic", "clamp_post", _m((POST_X, POST_Y, BODY_TOP)), "Z", CLAMP_DOWN, CLAMP_UP,
                  stiffness=CLAMP_K, damping=3.0, target=CLAMP_SET),
            Joint("clamp_swivel", "revolute", "clamp_arm", _m((POST_X, POST_Y, ARM_Z)), "Z", -math.radians(3.0),
                  SWIVEL_MAX, damping=0.01, parent="clamp_post")),
    materials={"default": (1.2, 1.0)})
PUMP_HANDLE = Box("pump_handle", pos=(PUMP_XY[0] + HANDLE[0][0] / 1000, PUMP_XY[1] + HANDLE[0][1] / 1000,
                                      TOP + HANDLE[0][2] / 1000), size=_m(HANDLE[1]), color=WHITE)

# -- the syringe: barrel (frame: barrel centre, axis +y toward the flange), plunger (frame: stopper centre) -------
_rm, _wall = BARREL_R - 0.75, 1.5
BARREL_PIECES = tuple(
    Piece(_m((_rm * math.sin(a), 0.0, _rm * math.cos(a))), _m((5.8, BARREL_L, _wall)), rot=(0.0, a, 0.0))
    for a in (2 * math.pi * k / 16 for k in range(16)))
_bore = BARREL_R - _wall  # 13.3
FLANGE_PIECES = (Piece(_m((0, _fy, _bore + (FLANGE_H / 2 - _bore) / 2)), _m((FLANGE_W, FLANGE_T, FLANGE_H / 2 - _bore))),
                 Piece(_m((0, _fy, -_bore - (FLANGE_H / 2 - _bore) / 2)), _m((FLANGE_W, FLANGE_T, FLANGE_H / 2 - _bore))),
                 Piece(_m((_bore + (FLANGE_W / 2 - _bore) / 2, _fy, 0)), _m((FLANGE_W / 2 - _bore, FLANGE_T, 2 * _bore))),
                 Piece(_m((-_bore - (FLANGE_W / 2 - _bore) / 2, _fy, 0)), _m((FLANGE_W / 2 - _bore, FLANGE_T, 2 * _bore))))
NOSE_PIECES = (Piece(_m((0, -BARREL_L / 2 - 1, 0)), _m((20, 2, 20))), Piece(_m((0, -BARREL_L / 2 - 7, 0)), _m((6, 10, 6))))
SYRINGE_PIECES = BARREL_PIECES + FLANGE_PIECES + NOSE_PIECES
PLUNGER_PIECES = (Piece(_m((0, 0, 0)), _m((STOPPER_SQ, STOPPER_L, STOPPER_SQ))),
                  Piece(_m((0, 0, 0)), _m((STOPPER_SQ, STOPPER_L, STOPPER_SQ)), rot=(0, math.pi / 4, 0)),
                  Piece(_m((0, (STOPPER_L / 2 + 51.5) / 2, 0)), _m((8, 51.5 - STOPPER_L / 2, 8))),
                  Piece(_m((0, 52.5, 0)), _m((30, 2, 30)), PLUNGER_WHITE))
THUMB_HALF = 0.015  # the thumb disc is 30 mm square


def _bbox_centre(pieces):
    """The pieces' bounding-box centre in the part frame (the turned staves counted as boxes at their extents)."""
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for p in pieces:
        half = np.asarray(p.size) / 2
        r = np.array([[math.cos(p.rot[1]), 0, math.sin(p.rot[1])], [0, 1, 0], [-math.sin(p.rot[1]), 0, math.cos(p.rot[1])]])
        corners = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * half @ r.T + p.center
        lo, hi = np.minimum(lo, corners.min(axis=0)), np.maximum(hi, corners.max(axis=0))
    return (lo + hi) / 2, lo, hi


STAND_XY = (0.25, -0.20)
STAND_H = 0.026
START_Z = TOP + STAND_H + BARREL_R / 1000  # barrel resting on the stands
SYRINGE = Part("syringe", pos=(*STAND_XY, START_Z), size=_m((FLANGE_W, BARREL_L + 14, FLANGE_H)), mass=0.075,
               color=CLEAR, pieces=SYRINGE_PIECES)
PLUNGER = Part("plunger", pos=(STAND_XY[0], STAND_XY[1] + STOPPER_AT / 1000, START_Z), size=_m((30, 53.5 + STOPPER_L / 2, 30)),
               mass=0.012, color=PLUNGER_WHITE, pieces=PLUNGER_PIECES)
STANDS = [Box(n, pos=(STAND_XY[0], STAND_XY[1] + dy, TOP + STAND_H / 2), size=(0.020, 0.010, STAND_H), color=STAND_BLACK)
          for n, dy in (("stand_a", -0.030), ("stand_b", 0.010))]
OBJECTS = [PUMP, PUMP_HANDLE, SYRINGE, PLUNGER, *STANDS]

# -- scoring --------------------------------------------------------------------------------------------------------
SUCCESS_HOLD_S = 1.0
SEAT_AXIS = 0.001  # seated: the barrel's axis within this of its seat (height and sideways)
SEAT_TILT_DEG = 1.5  # seated: level within this
SEAT_YAW_DEG = 1.5  # seated: square to the cradle within this (a flange in its slot allows about 1.4)
FLANGE_IN = 0.0008  # the flange's centre within this of the slot's (it has 0.5 mm a side)
# in the cradle: the barrel's nose end resting on the rods (its axis there within 3 mm of the seat sideways, 2.5 mm up
# or down and 5 mm along, where the rods end) and the barrel within 15 degrees of the cradle's line. A syringe cocked
# with its flange on the pump's body beside the slot is in the cradle; one held above it, slid off the rods' end or
# turned end for end is not.
IN_CRADLE = (0.003, 0.0025, 15.0, 0.005)
PUSHED = 0.001  # the plunger pushed in more than this, at any point: a dose the patient wasn't meant to get
PULLED = 0.003  # the plunger drawn back more than this, at any point (or out of the barrel)
BORE = (0.001, 5.0)  # the plunger in its barrel: the stopper within 1 mm of the axis, the rod within 5 degrees of it
NEAR, FLUSH = 0.005, 0.001  # the driver's face within these of the thumb disc, short of it
# WHY 0.2 mm the other way: touching, the solver lets the driver's face sink a little into the disc (-0.007 mm at the
# end of the official demo, 2026-11-12). Further past than this, the face is beyond the disc and holds nothing: the
# plunger was moved out of its way or out of the barrel.
PAST = 0.0002
COMPLETE = 0.9  # score() for the complete state; 1.0 comes only from success() holding 1 s
STILL, SPIN = 0.01, 0.5  # m/s, rad/s
DRIVER_STILL = 0.0005  # the driver moved at most this in the last control step (0.01 m/s at 20 Hz)
# clamped: the arm swung back over the cradle (within 5 degrees) and its pad down on the barrel's top (the post within
# 0.5 mm of where the pad meets a seated barrel), not moving (as the driver). Held up off the barrel by a hand, or left on
# the handle, the barrel is loose in the cradle.
# WHY 0.5 mm: at rest on the barrel the post sits 0.06 mm up (official sim, 2026-11-15); with 1.5 mm, a hand still
# lowering the clamp, its pad 1.4 mm above the barrel, already counted as clamped.
CLAMP_SWIVEL_DEG, CLAMP_ON = 5.0, 0.0005
CLAMP_STILL = (0.0005, math.radians(1.0))  # the post's and the arm's change in a control step
_SC, _, _ = _bbox_centre(SYRINGE_PIECES)
_PC, _, _ = _bbox_centre(PLUNGER_PIECES)
SEAT = np.array([PUMP_XY[0], PUMP_XY[1] + BARREL_SEAT_Y / 1000, TOP + AXIS_Z / 1000])  # the barrel's centre, seated
SLIT_W_Y = PUMP_XY[1] + SLIT_Y / 1000

# the plunger's furthest travel in and out since the reset: a push or a pull that is later undone still counts
_TRAVEL = {"in": 0.0, "out": 0.0}
# the pump's joints at the last control step and how far each moved in it. A joint reports no velocity, so motion is the
# change between steps. success() and score() both see each step's states, so a step counts once: when the states are
# new.
_MOVED = ("driver_slide", "clamp_lift", "clamp_swivel")
_JOINTS = {"states": None, "at": None, "step": dict.fromkeys(_MOVED, 0.0)}


@on_reset
def _clear_memory():
    _TRAVEL["in"] = _TRAVEL["out"] = 0.0
    _JOINTS.update(states=None, at=None, step=dict.fromkeys(_MOVED, 0.0))


def _joint_steps(objects) -> dict:
    if objects is not _JOINTS["states"]:
        at = {n: float(objects["pump"].joints[n]) for n in _MOVED}
        last = _JOINTS["at"]
        step = {n: 0.0 if last is None else abs(at[n] - last[n]) for n in _MOVED}
        _JOINTS.update(states=objects, at=at, step=step)
    return _JOINTS["step"]


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _frame(state, centre):
    """A part's frame origin and rotation from its state (pos is its bounding box's centre)."""
    r = _rot(state.quat_xyzw)
    return np.asarray(state.pos, float) - r @ centre, r


def measures(objects):
    """The quantities the score uses, in metres and degrees. Also records the plunger's furthest travel in and out."""
    s_org, s_r = _frame(objects["syringe"], _SC)
    p_org, p_r = _frame(objects["plunger"], _PC)
    axis = s_r @ np.array([0.0, 1.0, 0.0])
    flange = s_org + s_r @ np.array([0.0, _fy / 1000, 0.0])
    nose = s_org - axis * (BARREL_L / 2000)
    nose_seat = SEAT - np.array([0.0, BARREL_L / 2000, 0.0])
    thumb = p_org + p_r @ np.array([0.0, THUMB_FROM_STOPPER / 1000, 0.0])
    # the disc's outermost point along the cradle: a tilted disc meets the driver at its rim
    thumb_out = thumb[1] + THUMB_HALF * (abs(p_r[1, 0]) + abs(p_r[1, 2]))
    driver_face = PUMP_XY[1] + DRIVER_FACE_Y / 1000 + objects["pump"].joints["driver_slide"]
    along = float(np.dot(p_org - s_org, axis))
    pushed = STOPPER_AT / 1000 - along
    _TRAVEL["in"], _TRAVEL["out"] = max(_TRAVEL["in"], pushed), max(_TRAVEL["out"], -pushed)
    radial = p_org - s_org - along * axis
    rod = p_r @ np.array([0.0, 1.0, 0.0])
    joints = objects["pump"].joints
    steps = _joint_steps(objects)
    return {
        "side": float(s_org[0] - SEAT[0]), "up": float(s_org[2] - SEAT[2]),
        "nose_side": float(nose[0] - nose_seat[0]), "nose_up": float(nose[2] - nose_seat[2]),
        "nose_along": float(nose[1] - nose_seat[1]),
        "tilt": math.degrees(math.asin(min(1.0, abs(float(axis[2]))))),
        "yaw": math.degrees(math.atan2(abs(float(axis[0])), abs(float(axis[1])))),
        "off_line": math.degrees(math.acos(min(1.0, abs(float(axis[1]))))),
        "flange": float(flange[1] - SLIT_W_Y),
        "pushed": pushed, "most_in": _TRAVEL["in"], "most_out": _TRAVEL["out"],
        "radial": float(np.linalg.norm(radial)),
        "rod_angle": math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(rod, axis)))))),
        "gap": float(driver_face - thumb_out),
        "still": all(objects[n].resting(STILL, SPIN) for n in ("syringe", "plunger"))
        and steps["driver_slide"] <= DRIVER_STILL,
        "clamp_swivel": math.degrees(float(joints["clamp_swivel"])), "clamp_lift": float(joints["clamp_lift"]),
        "clamp_still": steps["clamp_lift"] <= CLAMP_STILL[0] and steps["clamp_swivel"] <= CLAMP_STILL[1],
    }


def clamped(m) -> bool:
    return abs(m["clamp_swivel"]) <= CLAMP_SWIVEL_DEG and abs(m["clamp_lift"]) <= CLAMP_ON and m["clamp_still"]


def _grade(objects) -> float:
    """0 if the plunger was ever pushed in more than 1 mm (a dose) or drawn back more than 3 mm, if it isn't in its
    barrel, or if the syringe isn't in the cradle; 0.2 in the cradle but not seated (the flange out of its slot, the
    syringe cocked). Seated, by the driver (its face within 5 mm short of the thumb disc: near; from 1 mm short to 0.2 mm
    into it with everything still: flush; more than 0.2 mm past the disc, holding nothing, counts as not near) and the
    clamp: unclamped 0.4, near 0.5, flush 0.6; clamped 0.5, near 0.75, flush 1.0 (score() pays that 0.9 at a step; 1.0
    is only for success() held 1 s)."""
    m = measures(objects)
    if m["most_in"] > PUSHED or m["most_out"] > PULLED or m["radial"] > BORE[0] or m["rod_angle"] > BORE[1]:
        return 0.0
    if abs(m["nose_side"]) > IN_CRADLE[0] or abs(m["nose_up"]) > IN_CRADLE[1] or m["off_line"] > IN_CRADLE[2] \
            or abs(m["nose_along"]) > IN_CRADLE[3]:
        return 0.0
    seated = abs(m["side"]) <= SEAT_AXIS and abs(m["up"]) <= SEAT_AXIS and m["tilt"] <= SEAT_TILT_DEG \
        and m["yaw"] <= SEAT_YAW_DEG and abs(m["flange"]) <= FLANGE_IN
    if not seated:
        return 0.2
    driver = 0 if not -PAST <= m["gap"] <= NEAR else 1 if m["gap"] > FLUSH or not m["still"] else 2
    return (0.5, 0.75, 1.0)[driver] if clamped(m) else (0.4, 0.5, 0.6)[driver]


def score(objects) -> float:
    """The credit at this step. The complete state pays COMPLETE, not 1.0: a trial that ends without success() having
    held 1 s scores its last step, so a syringe that only settled in the final second doesn't get full marks."""
    g = _grade(objects)
    return COMPLETE if g == 1.0 else g


def success(objects) -> bool:
    return _grade(objects) == 1.0


# -- the scripted demo: the right arm loads the syringe, the left arm brings the driver up --------------------------
GRIP_Z = 0.009  # TCP above the barrel's axis: fingertips 4 mm below it
SQUEEZE, RELEASE = -0.012, -0.020  # across the 29.6 mm barrel
TARGET = (PUMP_XY[0], PUMP_XY[1] + BARREL_SEAT_Y / 1000)  # barrel centre over the cradle, flange over its slot
SEAT_TCP_Z = TOP + AXIS_Z / 1000 + GRIP_Z
TAB_TCP = (PUMP_XY[0], PUMP_XY[1] + TAB[0][1] / 1000, TOP + (TAB[0][2] - TAB[1][2] / 2) / 1000 + 0.013 + 0.001)
PINCH_TAB = -0.004  # across the tab's 12 mm
# WHY 1.2 mm past touching: the pinched driver lags its target by about 1.3 mm, and the flange settles against its
# slot's near wall, 0.3 mm short of centre (traced, 2026-11-07). Aimed 0.8 mm past, the driver ended 0.87 mm short of
# the thumb disc's rim, near the 1 mm window's edge. With no lag at all it would move the plunger about 0.9 mm, inside
# the 1 mm allowance.
FLANGE_SETTLE = 0.3  # mm short of the slot's centre, where the flange comes to rest
SLIDE = -(DRIVER_HOME_GAP + 1.2) / 1000
ACROSS_X = math.pi / 2
KNOB_XY = (PUMP_XY[0] + POST_X / 1000, PUMP_XY[1] + POST_Y / 1000)  # the knob stays over the post however it swivels
PINCH_KNOB = -0.005  # 10 mm between the fingers across the 14 mm knob
# the TCP 4 mm above the knob's centre: the closed fingertips reach 13 mm below it, 3 mm clear of the arm
KNOB_TCP_Z = TOP + KNOB_Z / 1000 + 0.004
# WHY aim 11 mm up: the clamp must rise about 5 mm to clear a barrel's top, and the hand rides low under the spring and
# the clamp's weight (about 1 N there)
LIFT_TO = 0.011
REST_ON = (HANDLE_TOP - PAD_BOTTOM) / 1000  # the post's height with the arm resting on the handle
ON_BARREL_PUSH = 0.003  # lowering onto the barrel or the handle, the hand aims this far below the pad's resting place


def _knob(q, up=0.0):
    """The TCP at the knob with the clamp's post at ``q`` (m), ``up`` above that."""
    return (*KNOB_XY, KNOB_TCP_Z + q + up)


def _turn(q, yaw0, yaw1, total, label, n=8):
    """The pinched knob turned from ``yaw0`` to ``yaw1`` in ``n`` short steps, the hand staying over the post."""
    return [Waypoint(total / n, _knob(q), PINCH_KNOB, f"{label} ({k}/{n})", yaw0 + (yaw1 - yaw0) * k / n)
            for k in range(1, n + 1)]


def _lift(q0, q1, yaw, total, label, n=5):
    return [Waypoint(total / n, _knob(q0 + (q1 - q0) * k / n), PINCH_KNOB, f"{label} ({k}/{n})", yaw)
            for k in range(1, n + 1)]


def demo(delay=0.0):
    """``delay`` (s) holds both arms at their start first: the plan must not depend on starting at once. The order is
    the Medfusion card's: clamp away onto the handle and syringe in (right arm), driver up to the plunger (left arm),
    clamp turned back and lowered onto the barrel (right arm)."""
    # WHY the right arm works the clamp: the knob, behind the cradle, is beyond the left arm's reach at these heights
    # (IK scan, 2026-11-15)
    over_cradle, parked = 0.0, math.pi / 2  # the hand's turn on the knob with the arm over the cradle / on the handle
    right = [
        Waypoint(2.0 + delay, _knob(CLAMP_DOWN, 0.015), FINGER_OPEN, "over the clamp's knob", over_cradle),
        Waypoint(1.2, _knob(CLAMP_DOWN), FINGER_OPEN, "down round the knob", over_cradle),
        Waypoint(0.4, _knob(CLAMP_DOWN), FINGER_OPEN, "settle", over_cradle),
        Waypoint(0.6, _knob(CLAMP_DOWN), PINCH_KNOB, "pinch the knob", over_cradle),
        *_lift(CLAMP_DOWN, LIFT_TO, over_cradle, 1.2, "lift the clamp"),
        *_turn(LIFT_TO, over_cradle, parked, 2.4, "swivel it away"),
        Waypoint(0.4, _knob(LIFT_TO), PINCH_KNOB, "settle over the handle", parked),
        *_lift(LIFT_TO, REST_ON - ON_BARREL_PUSH, parked, 1.2, "lower it onto the handle"),
        Waypoint(0.6, _knob(REST_ON - ON_BARREL_PUSH), FINGER_OPEN, "let go", parked),
        Waypoint(1.0, _knob(REST_ON, 0.015), FINGER_OPEN, "up", parked),
        Waypoint(2.0, (*STAND_XY, START_Z + 0.06), FINGER_OPEN, "over the syringe", ACROSS_X),
        Waypoint(1.5, (*STAND_XY, START_Z + GRIP_Z), FINGER_OPEN, "down to the barrel", ACROSS_X),
        Waypoint(0.8, (*STAND_XY, START_Z + GRIP_Z), SQUEEZE, "grip the barrel", ACROSS_X),
        Waypoint(1.5, (*STAND_XY, TOP + 0.12), SQUEEZE, "lift", ACROSS_X),
        Waypoint(2.0, (*TARGET, TOP + 0.12), SQUEEZE, "carry", ACROSS_X),
        Waypoint(1.5, (*TARGET, SEAT_TCP_Z + 0.030), SQUEEZE, "over the cradle", ACROSS_X),
        Waypoint(3.5, (*TARGET, SEAT_TCP_Z - 0.001), SQUEEZE, "slowly down: the flange into its slot", ACROSS_X),
        Waypoint(0.8, (*TARGET, SEAT_TCP_Z - 0.001), RELEASE, "let go", ACROSS_X),
        Waypoint(1.5, (*TARGET, TOP + 0.12), RELEASE, "clear", ACROSS_X),
        Waypoint(2.0, (0.20, -0.25, TOP + 0.15), FINGER_OPEN, "out of the way", ACROSS_X),
    ]
    t_loaded = sum(w.duration for w in right) - 2.0  # the right hand is clear of the pump
    # WHY the left arm waits high and away and pinches the driver along x: with its fingers open along y over the
    # pump, one finger knocked the plunger's thumb disc (official sim, 2026-11-05)
    away = (TAB_TCP[0], TAB_TCP[1] + 0.07, TAB_TCP[2] + 0.10)
    left_across = -math.pi / 2  # the left arm turns this way for fingers along x (at +90 its wrist falls short here)
    left = [
        Waypoint(t_loaded, away, FINGER_OPEN, "wait high and away", left_across),
        Waypoint(2.0, (TAB_TCP[0], TAB_TCP[1], TAB_TCP[2] + 0.03), FINGER_OPEN, "over the driver's grip", left_across),
        Waypoint(1.5, TAB_TCP, FINGER_OPEN, "down to the grip", left_across),
        Waypoint(0.8, TAB_TCP, PINCH_TAB, "pinch the grip", left_across),
        Waypoint(3.0, (TAB_TCP[0], TAB_TCP[1] + SLIDE * 0.8, TAB_TCP[2]), PINCH_TAB, "slide", left_across),
        Waypoint(2.0, (TAB_TCP[0], TAB_TCP[1] + SLIDE, TAB_TCP[2]), PINCH_TAB, "creep up to the plunger", left_across),
        Waypoint(0.8, (TAB_TCP[0], TAB_TCP[1] + SLIDE, TAB_TCP[2]), FINGER_OPEN, "let go", left_across),
        Waypoint(1.5, (TAB_TCP[0], TAB_TCP[1] + SLIDE, TAB_TCP[2] + 0.05), FINGER_OPEN, "straight up", left_across),
        Waypoint(1.5, away, FINGER_OPEN, "out of the way", left_across),
    ]
    t_driven = sum(w.duration for w in left) - 1.5  # the left hand is up off the driver
    right += [
        Waypoint(t_driven - sum(w.duration for w in right), (0.20, -0.25, TOP + 0.15), FINGER_OPEN,
                 "wait for the driver", ACROSS_X),
        Waypoint(2.0, _knob(REST_ON, 0.015), FINGER_OPEN, "over the clamp's knob", parked),
        Waypoint(1.2, _knob(REST_ON), FINGER_OPEN, "down round the knob", parked),
        Waypoint(0.4, _knob(REST_ON), FINGER_OPEN, "settle", parked),
        Waypoint(0.6, _knob(REST_ON), PINCH_KNOB, "pinch the knob", parked),
        *_lift(REST_ON, LIFT_TO, parked, 1.0, "lift it off the handle"),
        *_turn(LIFT_TO, parked, over_cradle, 2.4, "turn it back over the barrel"),
        Waypoint(0.4, _knob(LIFT_TO), PINCH_KNOB, "settle over the barrel", over_cradle),
        *_lift(LIFT_TO, -ON_BARREL_PUSH, over_cradle, 2.0, "lower it onto the barrel"),
        Waypoint(0.6, _knob(-ON_BARREL_PUSH), FINGER_OPEN, "let go", over_cradle),
        Waypoint(1.0, _knob(0.0, 0.015), FINGER_OPEN, "up", over_cradle),
        Waypoint(1.5, (0.20, -0.25, TOP + 0.15), FINGER_OPEN, "out of the way", ACROSS_X),
    ]
    left.append(Waypoint(sum(w.duration for w in right) - sum(w.duration for w in left), away, FINGER_OPEN, "wait",
                         left_across))
    return {"left": left, "right": right}
