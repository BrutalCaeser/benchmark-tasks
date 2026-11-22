"""A patient cart about to be wheeled into an MRI scanner room: take the ferromagnetic items off it.

On the bench:
- an MR-safe cart top (84 x 38 cm shelf, 2 cm thick, plastic over a foam core), a red band across its front edge, and
  behind it a yellow warning sign reading DANGER / MR ZONE IV / MAGNET ON in raised block letters
- on the cart: a steel medical oxygen cylinder (M6 size: 6.4 cm across, 29 cm long with its valve, green shoulder, brass
  valve with handwheel and a black regulator block with a gauge) lying in a two-chock cradle; a pair of trauma shears
  (steel blades, orange handles and finger loops); a 15 cm steel adjustable wrench; and, MR-safe, a plastic water bottle, a
  plastic clipboard with a patient chart and a plastic emesis basin
- on the bench in front of the cart: a red plastic tray (72 x 20 cm, 4 cm walls)

The instruction gives only the goal (get the cart ready to go into the MRI room with the patient). Steel objects taken
into an MRI scanner room are pulled in by the magnet and become projectiles (ACR Manual on MR Safety); the robot has
to recognise the steel items and put them in the tray, the designated safe side, but leave the plastic items the
patient needs.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import mech  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20.0
DEBUG = False

# -- the cart top: an MR-safe plastic shelf on the table (legs and casters not modelled) -----------------------
CART_C, CART_SIZE = (0.61, 0.0), (0.38, 0.84, 0.02)
CART_X0, CART_X1 = CART_C[0] - CART_SIZE[0] / 2, CART_C[0] + CART_SIZE[0] / 2  # 0.42 .. 0.80
CART_Y = CART_SIZE[1] / 2  # +-0.42
SH = TOP + CART_SIZE[2]  # the shelf's top surface
STEEL, GREY, RED, YELLOW, BLACK, WHITE = ((0.78, 0.79, 0.80), (0.45, 0.46, 0.48), (0.85, 0.08, 0.06),
                                          (0.98, 0.80, 0.05), (0.08, 0.08, 0.09), (0.94, 0.94, 0.94))
DARK_STEEL = (0.50, 0.52, 0.56)

# -- the "out" tray on the bench in front of the cart --------------------------------------------------------
TRAY_C, TRAY_SIZE, TRAY_WALL, TRAY_H = (0.27, 0.0), (0.20, 0.72), 0.004, 0.04
TF = TOP + 0.004  # the tray floor's top surface

# -- steel items -----------------------------------------------------------------------------------------------
CYL_R, CYL_BODY = 0.032, 0.176
CYL_START = (0.50, 0.14, SH + CYL_R + 0.001)  # body centre; axis along y, valve end toward +y
SHEARS_START = (0.50, -0.10, SH + 0.007)  # the pivot hub; blades toward +y, loops toward -y
WRENCH_HEAD = (0.04, 0.05, 0.013)  # the jaw end; the handle runs from it along +x
WRENCH_START = (0.50, -0.30, SH + WRENCH_HEAD[2] / 2)
# -- MR-safe items --------------------------------------------------------------------------------------------
BOTTLE_R, BOTTLE_H = 0.033, 0.20
BOTTLE_START = (0.73, 0.27, SH + BOTTLE_H / 2)
CLIP_SIZE = (0.23, 0.31, 0.012)
CLIP_START = (0.68, 0.04, SH + CLIP_SIZE[2] / 2)
BASIN_SIZE = (0.11, 0.20, 0.004)
BASIN_START = (0.72, -0.225, SH + BASIN_SIZE[2] / 2)

FERRO = ("o2_cylinder", "trauma_shears", "wrench")
SAFE = ("water_bottle", "clipboard", "emesis_basin")
START = {"o2_cylinder": CYL_START, "trauma_shears": SHEARS_START, "wrench": WRENCH_START,
         "water_bottle": BOTTLE_START, "clipboard": CLIP_START, "emesis_basin": BASIN_START}

# -- scoring constants -----------------------------------------------------------------------------------------
SAFE_TOL = 0.005  # the designated safe side is the tray: the item's whole bounding box within its outline (+5 mm)
LET_GO_M = 0.07  # no gripper centre within this distance (on the table plane) of the item
STILL_S, STILL_M = 1.0, 0.003
MOVED_M = 0.05  # an MR-safe item moved this far from where it started is disturbed
TILT_COS = math.cos(math.radians(30.0))  # MR-safe item's up axis within 30 degrees of vertical
FLOOR_Z = TOP - 0.03  # an item whose centre is below this has fallen off the bench


def _p(c):
    return tuple(float(v) for v in c)


# -- scene -----------------------------------------------------------------------------------------------------
OBJECTS = [
    Box("cart_top", size=CART_SIZE, pos=_p((*CART_C, TOP + CART_SIZE[2] / 2)), color=(0.93, 0.92, 0.88)),
    Box("front_band", size=(0.03, CART_SIZE[1], 0.002), pos=_p((CART_X0 + 0.02, 0.0, SH + 0.001)), color=RED),
    Box("zone_sign", size=(0.012, 0.36, 0.20), pos=_p((0.83, 0.0, TOP + 0.10)), color=YELLOW),
    Box("sign_band", size=(0.013, 0.34, 0.05), pos=_p((0.8235, 0.0, TOP + 0.17)), color=RED),
    Box("tray_floor", size=(TRAY_SIZE[0], TRAY_SIZE[1], 0.004), pos=_p((*TRAY_C, TOP + 0.002)), color=RED),
    Box("tray_front", size=(TRAY_WALL, TRAY_SIZE[1], TRAY_H), pos=_p((TRAY_C[0] - TRAY_SIZE[0] / 2 + 0.002, 0.0, TOP + TRAY_H / 2)), color=RED),
    Box("tray_back", size=(TRAY_WALL, TRAY_SIZE[1], TRAY_H), pos=_p((TRAY_C[0] + TRAY_SIZE[0] / 2 - 0.002, 0.0, TOP + TRAY_H / 2)), color=RED),
    Box("tray_left", size=(TRAY_SIZE[0], TRAY_WALL, TRAY_H), pos=_p((TRAY_C[0], TRAY_SIZE[1] / 2 - 0.002, TOP + TRAY_H / 2)), color=RED),
    Box("tray_right", size=(TRAY_SIZE[0], TRAY_WALL, TRAY_H), pos=_p((TRAY_C[0], -TRAY_SIZE[1] / 2 + 0.002, TOP + TRAY_H / 2)), color=RED),
] + [
    Box(f"chock_{i}", size=(0.014, 0.02, 0.02), pos=_p((CYL_START[0] + sx * 0.043, CYL_START[1] + sy * 0.065, SH + 0.01)), color=GREY)
    for i, (sx, sy) in enumerate([(1, 1), (-1, 1), (1, -1), (-1, -1)])
] + [
    Cylinder("o2_cylinder", radius=CYL_R, height=CYL_BODY, pos=_p(CYL_START), color=(0.62, 0.64, 0.67), mass=1.3),
    Box("trauma_shears", size=(0.032, 0.032, 0.014), pos=_p(SHEARS_START), color=STEEL, mass=0.12),
    Box("wrench", size=WRENCH_HEAD, pos=_p(WRENCH_START), color=DARK_STEEL, mass=0.2),
    Cylinder("water_bottle", radius=BOTTLE_R, height=BOTTLE_H, pos=_p(BOTTLE_START), color=(0.72, 0.86, 0.96), mass=0.5),
    Box("clipboard", size=CLIP_SIZE, pos=_p(CLIP_START), color=(0.20, 0.42, 0.75), mass=0.25),
    Box("emesis_basin", size=BASIN_SIZE, pos=_p(BASIN_START), color=(0.60, 0.80, 0.90), mass=0.06),
]
_cp = mech.cylinder_part
_Q = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))  # the cylinder lies on its side, axis along y
mech.orient("o2_cylinder", _Q)
GREEN, BRASS = (0.10, 0.55, 0.25), (0.80, 0.62, 0.20)
mech.compound("o2_cylinder", [
    _cp(CYL_R, 0.045, (0.0, 0.1105, 0.0), GREEN, (90.0, 0.0, 0.0)),  # green shoulder
    _cp(0.020, 0.012, (0.0, 0.139, 0.0), (0.55, 0.57, 0.60), (90.0, 0.0, 0.0)),  # neck
    ((0.034, 0.045, 0.034), (0.0, 0.1675, 0.0), BRASS),  # brass valve body
    _cp(0.018, 0.012, (0.0, 0.196, 0.0), BLACK, (90.0, 0.0, 0.0)),  # handwheel
    ((0.030, 0.030, 0.040), (0.0, 0.1675, 0.037), BLACK),  # regulator block
    _cp(0.016, 0.006, (0.018, 0.1675, 0.037), WHITE, (0.0, 90.0, 0.0)),  # its gauge face
])
ORANGE = (0.95, 0.45, 0.05)
_h = 0.007  # shears' hub half height; parts are given relative to the hub's centre
_loop = []
for sx in (1, -1):
    _loop += [((0.006, 0.045, 0.010), (sx * 0.025 + sx * 0.012, -0.080, -0.002), ORANGE),
              ((0.006, 0.045, 0.010), (sx * 0.025 - sx * 0.012, -0.080, -0.002), ORANGE),
              ((0.030, 0.006, 0.010), (sx * 0.025, -0.080 + 0.0195, -0.002), ORANGE),
              ((0.030, 0.006, 0.010), (sx * 0.025, -0.080 - 0.0195, -0.002), ORANGE),
              ((0.014, 0.045, 0.008), (sx * 0.010, -0.032, -0.003), ORANGE)]
mech.compound("trauma_shears", [((0.020, 0.070, 0.004), (0.0, 0.050, -0.005), STEEL)] + _loop)
mech.compound("wrench", [((0.11, 0.022, 0.009), (0.075, 0.0, -0.002), DARK_STEEL),  # handle
                         ((0.012, 0.016, 0.0135), (0.0, 0.0, 0.0), GREY)])  # adjusting worm

mech.compound("water_bottle", [_cp(0.014, 0.02, (0.0, 0.0, 0.11), WHITE), _cp(0.018, 0.008, (0.0, 0.0, 0.058), (0.2, 0.5, 0.8))])
mech.compound("clipboard", [((0.21, 0.29, 0.003), (0.0, 0.0, 0.0075), WHITE), ((0.04, 0.07, 0.010), (-0.095, 0.0, 0.011), GREY)])
# -- the warning sign's lettering: raised block letters (2 mm proud of the board), as straight strokes -----------
_GLYPH = {
    "D": [(0, 0, 0, 1), (0, 1, .6, 1), (.6, 1, 1, .7), (1, .7, 1, .3), (1, .3, .6, 0), (.6, 0, 0, 0)],
    "A": [(0, 0, .5, 1), (.5, 1, 1, 0), (.22, .42, .78, .42)],
    "N": [(0, 0, 0, 1), (0, 1, 1, 0), (1, 0, 1, 1)],
    "G": [(1, 1, 0, 1), (0, 1, 0, 0), (0, 0, 1, 0), (1, 0, 1, .45), (1, .45, .5, .45)],
    "E": [(0, 0, 0, 1), (0, 1, 1, 1), (0, .5, .8, .5), (0, 0, 1, 0)],
    "R": [(0, 0, 0, 1), (0, 1, 1, 1), (1, 1, 1, .5), (1, .5, 0, .5), (.4, .5, 1, 0)],
    "M": [(0, 0, 0, 1), (0, 1, .5, .45), (.5, .45, 1, 1), (1, 1, 1, 0)],
    "Z": [(0, 1, 1, 1), (1, 1, 0, 0), (0, 0, 1, 0)],
    "O": [(0, 0, 0, 1), (0, 1, 1, 1), (1, 1, 1, 0), (1, 0, 0, 0)],
    "I": [(.5, 0, .5, 1)],
    "V": [(0, 1, .5, 0), (.5, 0, 1, 1)],
    "T": [(0, 1, 1, 1), (.5, 1, .5, 0)],
}
SIGN_C = (0.83, 0.0, TOP + 0.10)


def _lettering(text, x_face, z_bottom, h, w, gap, stroke, rgb):
    """Strokes of ``text`` on the sign's face (it faces the arms, so reading left to right runs toward -y), as
    compound parts of the sign board: (size, offset from the board's centre, colour, rotation)."""
    widths = [stroke if c == "I" else (w * 0.6 if c == " " else w) for c in text]
    u = -(sum(widths) + gap * (len(text) - 1)) / 2
    parts = []
    for c, cw in zip(text, widths):
        for u0, v0, u1, v1 in _GLYPH.get(c, []):
            a = (u + stroke / 2 + u0 * (cw - stroke), z_bottom + stroke / 2 + v0 * (h - stroke))
            b = (u + stroke / 2 + u1 * (cw - stroke), z_bottom + stroke / 2 + v1 * (h - stroke))
            du, dv = b[0] - a[0], b[1] - a[1]
            length = math.hypot(du, dv)
            mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            parts.append(((0.002, length + stroke, stroke), (x_face - 0.001 - SIGN_C[0], -mid[0], mid[1] - SIGN_C[2]),
                          rgb, (math.degrees(math.atan2(dv, -du)), 0.0, 0.0)))
        u += cw + gap
    return parts


mech.compound("zone_sign",
              _lettering("DANGER", 0.817, TOP + 0.154, 0.032, 0.022, 0.007, 0.0045, WHITE)
              + _lettering("MR ZONE IV", 0.824, TOP + 0.075, 0.050, 0.028, 0.008, 0.006, BLACK)
              + _lettering("MAGNET ON", 0.824, TOP + 0.026, 0.030, 0.020, 0.006, 0.0045, BLACK))
_bw = 0.032
mech.compound("emesis_basin", [((0.004, 0.20, _bw), (0.053, 0.0, 0.018), (0.60, 0.80, 0.90)),
                               ((0.004, 0.20, _bw), (-0.053, 0.0, 0.018), (0.60, 0.80, 0.90)),
                               ((0.11, 0.004, _bw), (0.0, 0.098, 0.018), (0.60, 0.80, 0.90)),
                               ((0.11, 0.004, _bw), (0.0, -0.098, 0.018), (0.60, 0.80, 0.90))])


# -- measurements ---------------------------------------------------------------------------------------------


def _rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def on_cart(xy, margin=0.0) -> bool:
    return (CART_X0 - margin <= xy[0] <= CART_X1 + margin) and abs(xy[1]) <= CART_Y + margin


def all_clear(objects, name) -> bool:
    """Every part of the item is clear of the cart's outline and inside the tray's outline (the designated safe side):
    tested on the item's whole world-aligned bounding box, which the simulator gives for the full body (the cylinder
    with its valve and regulator, the shears with blades and loops, the wrench with its handle)."""
    lo, hi = np.asarray(objects[name].low, float), np.asarray(objects[name].high, float)
    off_cart = hi[0] < CART_X0 or lo[0] > CART_X1 or hi[1] < -CART_Y or lo[1] > CART_Y
    tx, ty = TRAY_SIZE[0] / 2 + SAFE_TOL, TRAY_SIZE[1] / 2 + SAFE_TOL
    in_tray = (TRAY_C[0] - tx <= lo[0] and hi[0] <= TRAY_C[0] + tx and TRAY_C[1] - ty <= lo[1] and hi[1] <= TRAY_C[1] + ty)
    return bool(off_cart and in_tray)


def upright(objects, name) -> bool:
    return float((_rot(objects[name].quat_xyzw) @ np.array([0.0, 0.0, 1.0]))[2]) >= TILT_COS


# -- scoring ---------------------------------------------------------------------------------------------------


def _fresh():
    return dict(start=None, hist={}, n=0, steel_fell=False, safe_removed=False, disturbed=False)


_state = _fresh()
_WINDOW = int(STILL_S * CONTROL_HZ)


def _still(st, name) -> bool:
    h = st["hist"][name]
    if len(h) < _WINDOW:
        return False
    a = np.asarray(h[-_WINDOW:])
    return bool(np.all(np.linalg.norm(a - a[-1], axis=1) < STILL_M))


@mech.on_tick
def _reset(sim):
    if sim.t <= 1.5 / CONTROL_HZ:
        _state.clear()
        _state.update(_fresh())


def _cleared(objects, st, name) -> bool:
    p = np.asarray(objects[name].pos, float)
    if p[2] < FLOOR_Z or p[2] > TOP + 0.20 or not all_clear(objects, name):
        return False
    if not _still(st, name):
        return False
    return all(np.linalg.norm(np.asarray(objects[a].pos, float)[:2] - p[:2]) >= LET_GO_M for a in ("left_tcp", "right_tcp"))


def score(objects):
    """Cleared: a steel item (oxygen cylinder, trauma shears, wrench) resting, still for 1 s, with every part of it (its
    whole bounding box) clear of the cart's outline and inside the tray's outline (the designated safe side), no
    gripper centre within 7 cm of it.
    Each cleared steel item is worth 0.25. The last 0.25 (1.0 in all) needs all three cleared and every MR-safe item
    (water bottle, clipboard, emesis basin) never moved 5 cm or more from where it started, never tilted more than 30
    degrees, and still on the cart. A steel item that falls off the bench: 0 for good. An MR-safe item that leaves the
    cart (its centre beyond the cart's outline): capped at 0.5. Doing nothing: 0."""
    st = _state
    st["n"] += 1
    if st["start"] is None:
        st["start"] = {k: np.asarray(objects[k].pos, float).copy() for k in START}
    for k in START:
        st["hist"][k] = (st["hist"].get(k, []) + [np.asarray(objects[k].pos, float)])[-(_WINDOW + 5):]
    for k in FERRO:
        if float(objects[k].pos[2]) < FLOOR_Z:
            st["steel_fell"] = True
    for k in SAFE:
        p = np.asarray(objects[k].pos, float)
        if p[2] < FLOOR_Z or not on_cart(p[:2]):
            st["safe_removed"] = True
        if float(np.linalg.norm(p - st["start"][k])) >= MOVED_M or not upright(objects, k):
            st["disturbed"] = True
    cleared = [_cleared(objects, st, k) for k in FERRO]
    if DEBUG and st["n"] % 20 == 0:
        print(f"DBG t={st['n'] / CONTROL_HZ:.0f} " + " ".join(
            f"{k}=({objects[k].pos[0]:.3f},{objects[k].pos[1]:.3f},{objects[k].pos[2] - TOP:.3f},c={int(c)},"
            f"a={int(all_clear(objects, k))},x={objects[k].low[0]:.3f}..{objects[k].high[0]:.3f},"
            f"y={objects[k].low[1]:.3f}..{objects[k].high[1]:.3f})"
            for k, c in zip(FERRO, cleared)) + f" removed={st['safe_removed']} disturbed={st['disturbed']} fell={st['steel_fell']}",
              flush=True)
    if st["steel_fell"]:
        return 0.0
    s = 0.25 * sum(cleared)
    if all(cleared) and not st["disturbed"] and not st["safe_removed"]:
        s = 1.0
    if st["safe_removed"]:
        s = min(s, 0.5)
    return s


# -- scripted demo ----------------------------------------------------------------------------------------------
# Straight-down grips. Cylinder (left arm): grip across its body, fingers along x; carried to the tray. The right arm
# takes the shears (grip at the pivot hub, fingers along x) and the wrench (at its jaw end, fingers along y across it).

PARK_L, PARK_R = (0.18, 0.27, TOP + 0.20), (0.20, -0.15, TOP + 0.20)  # right parks clear of the wrench
YAW_X = math.pi / 2  # fingers close along x


def _move(xy, z, finger, label, dur=1.5, yaw=YAW_X):
    return Waypoint(dur, (xy[0], xy[1], z), finger, label, yaw)


def _carry(src, z_src, dst, z_dst, name, lift=0.055, yaw=YAW_X):
    zc = max(z_src, z_dst) + lift
    return [
        _move(src, z_src + 0.04, FINGER_OPEN, f"over the {name}", 2.0, yaw),
        _move(src, z_src, FINGER_OPEN, f"down around the {name}", 1.5, yaw),
        _move(src, z_src, FINGER_CLOSED, f"grip the {name}", 0.7, yaw),
        _move(src, zc, FINGER_CLOSED, f"lift the {name}", 1.5, yaw),
        _move(dst, zc, FINGER_CLOSED, f"carry the {name} to the tray", 2.5, yaw),
        _move(dst, z_dst + 0.002, FINGER_CLOSED, f"lower the {name} into the tray", 1.5, yaw),
        _move(dst, z_dst + 0.002, FINGER_OPEN, "let go", 0.7, yaw),
        _move(dst, z_dst + 0.09, FINGER_OPEN, "up clear", 1.2, yaw),
    ]


def demo():
    cyl = (CYL_START[0], CYL_START[1])
    left = _carry(cyl, SH + 0.03, (TRAY_C[0], CYL_START[1]), TF + 0.03, "oxygen cylinder")
    left.append(Waypoint(2.0, PARK_L, FINGER_OPEN, "park"))
    right = _carry((SHEARS_START[0], SHEARS_START[1]), SH + 0.010, (TRAY_C[0], -0.06), TF + 0.010, "trauma shears")
    right += _carry((WRENCH_START[0], WRENCH_START[1]), SH + 0.008, (0.21, -0.29), TF + 0.008, "wrench", yaw=0.0)
    right.append(Waypoint(2.0, PARK_R, FINGER_OPEN, "park"))
    right.append(Waypoint(2.0, PARK_R, FINGER_OPEN, "still"))
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    left.append(Waypoint(tr - tl, PARK_L, FINGER_OPEN, "stay parked"))
    return {"left": left, "right": right}
