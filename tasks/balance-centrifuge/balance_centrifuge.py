"""On the station table, between the arms and a little out: a centrifuge rotor, fixed to the table. It is a dark grey
disc (20 cm across, 1 cm thick) with a hub at its centre and six square pockets evenly spaced around the hub, each
4 cm deep and 4 cm across inside. Two blue sample tubes (3 cm across, 11.5 cm tall, the same weight) stand upright in
small white holders nearer the arms, one on the left and one on the right. The rotor and holders are fixed; the tubes
are loose.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x forward from the arms, y to the left, z up (m); the arm bases stand at x 0, y +-0.305.
ROTOR_XY = (0.24, 0.0)
BASE_R, BASE_H = 0.100, 0.010
FLOOR_Z = TOP + BASE_H  # the pockets' floor: the top of the rotor disc
HUB_R, HUB_H = 0.015, 0.045
POCKET_RING_R = 0.065  # pocket centres sit this far from the rotor centre
POCKET_ANGLES_DEG = (30, 90, 150, 210, 270, 330)  # 0 = straight ahead (+x), counterclockwise seen from above
POCKET_INNER, POCKET_WALL, POCKET_DEPTH = 0.040, 0.004, 0.040
RACK_INNER, RACK_WALL, RACK_HEIGHT = 0.034, 0.004, 0.030
RACK_XY = {"left": (0.17, 0.20), "right": (0.17, -0.20)}

# A 50 mL conical tube holding 40 mL of water. (A 15 mL tube, 17 g and 17 mm across, spun up to 22 rad/s in the gripper
# and never settled in its pocket: too little rotational inertia. Trace and replay, 2026-11-02.)
TUBE_R, TUBE_H, TUBE_KG = 0.015, 0.115, 0.053
TUBES = {"tube_left": "left", "tube_right": "right"}  # each tube starts in front of that arm

POCKETS = [
    (ROTOR_XY[0] + POCKET_RING_R * math.cos(math.radians(a)), ROTOR_XY[1] + POCKET_RING_R * math.sin(math.radians(a)))
    for a in POCKET_ANGLES_DEG
]  # pocket k sits at POCKET_ANGLES_DEG[k]
ROTOR_GREY, POCKET_GREY = (0.30, 0.30, 0.32), (0.74, 0.75, 0.77)
HOLDER_WHITE, TUBE_BLUE = (0.92, 0.92, 0.92), (0.10, 0.35, 0.90)  # saturated, so the tubes show in the 320 px top view


def _cell(prefix, centre_xy, inner, wall, height, base_z, color):
    """Four fixed walls around a square opening (inner x inner m), standing on base_z, sides along the world axes."""
    cx, cy = centre_xy
    off = inner / 2 + wall / 2
    z = base_z + height / 2
    outer = inner + 2 * wall
    return [
        Box(f"{prefix}_n", pos=(cx, cy + off, z), size=(outer, wall, height), color=color),
        Box(f"{prefix}_s", pos=(cx, cy - off, z), size=(outer, wall, height), color=color),
        Box(f"{prefix}_e", pos=(cx + off, cy, z), size=(wall, inner, height), color=color),
        Box(f"{prefix}_w", pos=(cx - off, cy, z), size=(wall, inner, height), color=color),
    ]


OBJECTS = [
    Cylinder("rotor_base", pos=(*ROTOR_XY, TOP + BASE_H / 2), radius=BASE_R, height=BASE_H, color=ROTOR_GREY),
    Cylinder("rotor_hub", pos=(*ROTOR_XY, FLOOR_Z + HUB_H / 2), radius=HUB_R, height=HUB_H, color=ROTOR_GREY),
    *[wall for k, xy in enumerate(POCKETS)
      for wall in _cell(f"pocket{k}", xy, POCKET_INNER, POCKET_WALL, POCKET_DEPTH, FLOOR_Z, POCKET_GREY)],
    *[wall for side, xy in RACK_XY.items()
      for wall in _cell(f"holder_{side}", xy, RACK_INNER, RACK_WALL, RACK_HEIGHT, TOP, HOLDER_WHITE)],
    *[Cylinder(name, pos=(*RACK_XY[side], TOP + TUBE_H / 2 + 0.001), radius=TUBE_R, height=TUBE_H, mass=TUBE_KG,
               color=TUBE_BLUE)
      for name, side in TUBES.items()],
]

# A tube is seated when its bottom is down on a pocket's floor: lowest point within 5 mm of the floor, the centre of its
# lower end inside the pocket's square opening, leaning at most 18 degrees (it can lean about 14 degrees against the
# walls). A radius would be wrong: in a corner an upright tube's centre is sqrt(5^2 + 5^2) = 7.1 mm off (found
# in review). Opposite pockets are three apart (180 degrees).
# It need not be at rest: PhysX has no rolling resistance, so a released tube can keep rocking around its pocket for
# many seconds (5-10 cm/s, traces of 2026-11-02) where a real one settles at once, and which pocket it is in is what
# balance depends on. The 1 s success hold still rules out a tube bouncing through.
MAX_FLOOR_GAP = 0.005
MAX_TILT_DEG = 18.0

# Where a tube is comes from its centre and axis, never from low/high: Arena builds those from the corners of the
# tube's local box, which swing out when the tube spins on its axis (docs/robocurve/rules.md §3).


def _axis(quat_xyzw):
    """The tube's axis (its local z) in the world frame."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    return 2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)


def _tilt_deg(tube) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, abs(_axis(tube.quat_xyzw)[2])))))


def _lowest_z(tube) -> float:
    """World z of the tube's lowest point (exact for a cylinder)."""
    c = min(1.0, abs(_axis(tube.quat_xyzw)[2]))
    return float(tube.pos[2]) - (TUBE_H / 2 * c + TUBE_R * math.sqrt(1.0 - c * c))


def _bottom_centre_xy(tube):
    """x, y of the centre of the tube's lower end."""
    a = _axis(tube.quat_xyzw)
    sign = -1.0 if a[2] >= 0 else 1.0  # the end that is lower
    return float(tube.pos[0]) + sign * a[0] * TUBE_H / 2, float(tube.pos[1]) + sign * a[1] * TUBE_H / 2


def _pocket_of(tube):
    """The index of the pocket the tube is seated in, or None."""
    if not (_tilt_deg(tube) <= MAX_TILT_DEG and abs(_lowest_z(tube) - FLOOR_Z) <= MAX_FLOOR_GAP):
        return None
    bx, by = _bottom_centre_xy(tube)
    half = POCKET_INNER / 2
    for k, (px, py) in enumerate(POCKETS):
        if abs(bx - px) <= half and abs(by - py) <= half:
            return k
    return None


# The second tube counts as not yet loaded only if it is clear of the rotor: no point of its axis within 10 cm (the
# disc's radius) of the rotor centre, seen from above. A tube lying across the rotor or perched on a rim is neither.
CLEAR_OF_ROTOR = BASE_R


def _clear_of_rotor(tube) -> bool:
    """The horizontal distance from the rotor centre to the tube's axis (a segment) is at least CLEAR_OF_ROTOR."""
    a = _axis(tube.quat_xyzw)
    cx, cy = float(tube.pos[0]), float(tube.pos[1])
    ax, ay = a[0] * TUBE_H / 2, a[1] * TUBE_H / 2  # half the axis, seen from above
    rx, ry = ROTOR_XY[0] - cx, ROTOR_XY[1] - cy
    length2 = ax * ax + ay * ay
    t = 0.0 if length2 == 0.0 else max(-1.0, min(1.0, (rx * ax + ry * ay) / length2))
    return math.hypot(rx - t * ax, ry - t * ay) >= CLEAR_OF_ROTOR


def _balanced(pockets) -> bool:
    return None not in pockets and (pockets[0] - pockets[1]) % 6 == 3


def success(objects):
    """Both tubes seated, in opposite pockets; must hold for 1 s."""
    return _balanced([_pocket_of(objects[name]) for name in TUBES])


def distance(objects):
    """The farther tube's bottom centre from its nearest pocket centre (m)."""
    return max(
        min(math.hypot(_bottom_centre_xy(objects[name])[0] - px, _bottom_centre_xy(objects[name])[1] - py)
            for px, py in POCKETS)
        for name in TUBES
    )


def score(objects):
    """Partial credit (success scores 1.0): 0.25 when exactly one tube is seated and the other is clear of the rotor
    (progress; nothing unsafe is finished); 0.0 when both are seated but not opposite (an unbalanced rotor, the outcome
    the rule exists to prevent), when the other tube is on or across the rotor, or when none is seated."""
    pockets = [_pocket_of(objects[name]) for name in TUBES]
    seated = sum(p is not None for p in pockets)
    if seated == 2:
        return 1.0 if _balanced(pockets) else 0.0
    if seated == 1:
        other = next(name for name, p in zip(TUBES, pockets) if p is None)
        return 0.25 if _clear_of_rotor(objects[other]) else 0.0
    return 0.0


GRIP_BELOW_TOP = 0.045  # the gripper holds a tube 4.5 cm below its top
_TCP_TO_BOTTOM = TUBE_H - GRIP_BELOW_TOP  # from the gripping point down to the tube's bottom
_GRIP_Z = TOP + TUBE_H - GRIP_BELOW_TOP
_CARRY_Z = FLOOR_Z + POCKET_DEPTH + 0.015 + _TCP_TO_BOTTOM  # the tube's bottom 1.5 cm over the pocket rims
_SET_Z = FLOOR_Z + _TCP_TO_BOTTOM  # the tube's bottom on the floor: set down, not dropped
_TOP_Z = max(TOP + TUBE_H + PAD_TIP_PAST_TCP + 0.015, _CARRY_Z)  # pads 1.5 cm over the tube tops in the holders
_CLEAR_Z = max(_TOP_Z, FLOOR_Z + TUBE_H + PAD_TIP_PAST_TCP + 0.015)  # pads clear a loaded tube's top


def load_waypoints(side, pocket):
    """One arm's moves to take the tube from its holder and set it into ``pocket``, starting above the holder: grip,
    lift straight up, carry, pause, lower slowly onto the floor, hold, open slowly, wait, go up, go back over the
    holder."""
    (rx, ry), (px, py) = RACK_XY[side], POCKETS[pocket]
    return [
        Waypoint(1.5, (rx, ry, _GRIP_Z), FINGER_OPEN, "down"),
        Waypoint(0.8, (rx, ry, _GRIP_Z), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (rx, ry, _CARRY_Z), FINGER_CLOSED, "lift out of the holder"),
        Waypoint(2.0, (px, py, _CARRY_Z), FINGER_CLOSED, "carry over the pocket"),
        Waypoint(0.5, (px, py, _CARRY_Z), FINGER_CLOSED, "settle"),
        Waypoint(2.0, (px, py, _SET_Z), FINGER_CLOSED, "lower into the pocket"),
        Waypoint(0.5, (px, py, _SET_Z), FINGER_CLOSED, "hold on the floor"),
        Waypoint(1.0, (px, py, _SET_Z), FINGER_OPEN, "let go"),
        Waypoint(0.8, (px, py, _SET_Z), FINGER_OPEN, "wait, fingers open"),
        Waypoint(1.0, (px, py, _CLEAR_Z), FINGER_OPEN, "up"),
        Waypoint(1.5, (rx, ry, _CLEAR_Z), FINGER_OPEN, "back over the holder"),
    ]


def above_holder(side, duration=2.0, label="above the tube"):
    return Waypoint(duration, (*RACK_XY[side], _TOP_Z), FINGER_OPEN, label)


def demo():
    """The left arm loads its tube into the pocket at 90 degrees (left of the hub) and moves back over its holder; only
    then does the right arm, waiting over its own holder, load its tube into the pocket at 270 degrees (right of the
    hub), opposite the first. The two grippers can't be over the rotor together: their housings touch (trace and video,
    2026-11-02). Each grips its tube 4.5 cm below the top, lifts it straight up until its bottom clears the pocket rims
    by 1.5 cm, carries it over its pocket, pauses, lowers it slowly onto the pocket floor, holds it there, opens slowly,
    waits with the fingers open, and goes straight back up."""
    left = [above_holder("left"), *load_waypoints("left", 1)]
    left_time = sum(w.duration for w in left)
    right = [
        above_holder("right"),
        above_holder("right", left_time - 2.0, "wait for the left arm"),
        *load_waypoints("right", 4),
    ]
    return {"left": left, "right": right}
