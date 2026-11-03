"""On the station table: a white shelf board (20 cm deep, 56 cm wide, 3 cm thick) lies across the front of both arms.
Its front edge is the one nearest the arms, 13 cm in front of the arm bases. Four red cans (6.6 cm across, 10 cm tall)
stand at the back of the board, 19 and 7 cm left of centre and 7 and 19 cm right of centre: two in front of each arm.
The board is fixed; the cans are loose. A can is faced when it stands upright and still on the board with its frontmost
point between 6 mm behind and 3 mm past the board's front edge.
"""

import math

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

SHELF = Box("shelf", pos=(0.23, 0.0, TOP + 0.015), size=(0.20, 0.56, 0.03), color=(0.90, 0.90, 0.88))  # fixed
EDGE_X = SHELF.pos[0] - SHELF.size[0] / 2  # the near edge, 0.13 m in front of the arm bases
SHELF_TOP = SHELF.pos[2] + SHELF.size[2] / 2

CAN_R, CAN_H, CAN_KG = 0.033, 0.10, 0.35  # a soup can, full
BACK_X = EDGE_X + SHELF.size[0] - CAN_R - 0.007  # each can starts 7 mm from the board's back edge
CAN_Y = {"can_1": 0.19, "can_2": 0.07, "can_3": -0.07, "can_4": -0.19}  # left to right, seen from behind the arms
CANS = [
    Cylinder(name, pos=(BACK_X, y, SHELF_TOP + CAN_H / 2 + 0.001), radius=CAN_R, height=CAN_H, mass=CAN_KG,
             color=(0.72, 0.10, 0.10))
    for name, y in CAN_Y.items()
]
OBJECTS = [SHELF, *CANS]

# A can's frontmost point, relative to the edge: at most 3 mm past it, at most 6 mm behind it (a 9 mm window).
FRONT_BAND = (-0.003, 0.006)
# The scripted demo sets each can down 1-4 mm forward of where it aims (three identical runs, 2026-11-02: the cans'
# fronts ended 1.3 mm past to 1.5 mm behind the edge), so it aims 2.5 mm behind the edge.
DEMO_AIM = 0.0025
MAX_TILT_DEG = 10.0


# Where a can's surface is comes from its centre and axis, not from ``low``/``high``: Arena builds those from the
# corners of the can's local box (a square post), so a can spinning on its own axis reads up to r(sqrt(2) - 1) =
# 13.7 mm further forward than it is.


def _axis(quat_xyzw) -> tuple[float, float, float]:
    """The can's axis (its local z) in the world frame."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    return 2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)


def _reach(axis_component: float) -> float:
    """How far the can's surface reaches from its centre along a world direction, given the axis's component along
    it: exact for a cylinder (half the height times |a.d| plus the radius times sqrt(1 - (a.d)^2)), and spin-free."""
    c = min(1.0, abs(axis_component))
    return CAN_H / 2 * c + CAN_R * math.sqrt(1.0 - c * c)


def _front(can) -> float:
    """World x of the can's frontmost point (the point nearest the arms)."""
    return float(can.pos[0]) - _reach(_axis(can.quat_xyzw)[0])


def _bottom(can) -> float:
    """World z of the can's lowest point."""
    return float(can.pos[2]) - _reach(_axis(can.quat_xyzw)[2])


def _tilt_deg(quat_xyzw) -> float:
    """Angle between the can's axis and the vertical."""
    return math.degrees(math.acos(max(-1.0, min(1.0, _axis(quat_xyzw)[2]))))


def _faced(can) -> bool:
    """Upright, standing on the board itself (not on the table or on another can), frontmost point within the band, and
    still (``resting()``: under 1 cm/s and 0.5 rad/s)."""
    front = _front(can) - EDGE_X
    return (
        FRONT_BAND[0] <= front <= FRONT_BAND[1]
        and _tilt_deg(can.quat_xyzw) <= MAX_TILT_DEG
        and abs(_bottom(can) - SHELF_TOP) <= 0.005
        and can.resting()
    )


def success(objects):
    """All four cans faced; must hold for 1 s."""
    return all(_faced(objects[name]) for name in CAN_Y)


def distance(objects):
    """The farthest can's frontmost point from the middle of the band (m)."""
    target = EDGE_X + sum(FRONT_BAND) / 2
    return max(abs(_front(objects[name]) - target) for name in CAN_Y)


def score(objects):
    """Partial credit (success scores 1.0): 0.25 for each faced can. A can off the board, on another can or lying down
    scores 0."""
    return 0.25 * sum(_faced(objects[name]) for name in CAN_Y)


def demo():
    """Each arm faces its two cans, one at a time, each in its own lane so nothing passes over another can: hover with
    the finger pads 1.5 cm above the can tops, come straight down, grip 3.5 cm below the top, lift 1.5 cm, slide
    forward until its front is 2.5 mm behind the edge (see DEMO_AIM), set down, open, and go straight back up before
    moving on. The left arm takes its outer can first and the right arm its inner can first, so the grippers never meet
    in the middle."""
    hover_z = SHELF_TOP + CAN_H + PAD_TIP_PAST_TCP + 0.015  # pads clear the can tops
    grip_z = SHELF_TOP + CAN_H - 0.035
    carry_z = grip_z + 0.015
    set_z = grip_z + 0.002  # the can's bottom 2 mm above the board when let go
    front_x = EDGE_X + CAN_R + DEMO_AIM

    def face(name, first):
        y = CAN_Y[name]
        return [
            Waypoint(2.0 if first else 1.5, (BACK_X, y, hover_z), FINGER_OPEN, f"{name}: above"),
            Waypoint(1.5, (BACK_X, y, grip_z), FINGER_OPEN, f"{name}: down"),
            Waypoint(0.8, (BACK_X, y, grip_z), FINGER_CLOSED, f"{name}: grip"),
            Waypoint(0.6, (BACK_X, y, carry_z), FINGER_CLOSED, f"{name}: lift"),
            Waypoint(2.0, (front_x, y, carry_z), FINGER_CLOSED, f"{name}: slide forward"),
            Waypoint(0.8, (front_x, y, set_z), FINGER_CLOSED, f"{name}: set down"),
            Waypoint(0.6, (front_x, y, set_z), FINGER_OPEN, f"{name}: let go"),
            Waypoint(1.0, (front_x, y, hover_z), FINGER_OPEN, f"{name}: up"),
        ]

    return {"left": face("can_1", True) + face("can_2", False), "right": face("can_3", True) + face("can_4", False)}
