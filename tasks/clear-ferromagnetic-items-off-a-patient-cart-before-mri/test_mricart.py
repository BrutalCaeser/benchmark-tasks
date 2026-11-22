"""Scorer tests for mricart.py, run without the simulator:  python test_mricart.py"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import mricart as M  # noqa: E402

AWAY = {"left_tcp": (0.18, 0.27, M.TOP + 0.20), "right_tcp": (0.20, -0.15, M.TOP + 0.20)}
# Positions are bounding-box centres, as the simulator reports them; HALF is each steel item's bounding-box half size
# lying as it starts (cylinder along y with its valve, shears with blades and loops along y, wrench handle along x).
HALF = {"o2_cylinder": (0.032, 0.145, 0.045), "trauma_shears": (0.040, 0.094, 0.007), "wrench": (0.075, 0.025, 0.007)}
TRAY = {"o2_cylinder": (0.27, 0.197, M.TOP + 0.05), "trauma_shears": (0.27, -0.069, M.TOP + 0.01),
        "wrench": (0.265, -0.29, M.TOP + 0.01)}


def _obj(pos, q=(0.0, 0.0, 0.0, 1.0), half=(0.01, 0.01, 0.01)):
    pos, half = np.asarray(pos, float), np.asarray(half, float)
    return ObjectState(pos, np.asarray(q, float), np.zeros(3), np.zeros(3), pos - half, pos + half)


def frame(over=None, tcp=None):
    d = {k: _obj(v, half=HALF.get(k, (0.01, 0.01, 0.01))) for k, v in M.START.items()}
    for k, v in (over or {}).items():
        d[k] = v if isinstance(v, ObjectState) else _obj(v, half=HALF.get(k, (0.01, 0.01, 0.01)))
    for k, v in (tcp or AWAY).items():
        d[k] = _obj(v)
    return d


def run(over=None, n=60, tcp=None, pre=None):
    M._state.clear()
    M._state.update(M._fresh())
    s = 0.0
    for f in [frame()] + (pre or []):
        s = M.score(f)
    for _ in range(n):
        s = M.score(frame(over, tcp))
    return s


def test():
    assert run() == 0.0                                           # doing nothing
    assert run(TRAY) == 1.0                                       # all three cleared
    assert run({"o2_cylinder": TRAY["o2_cylinder"]}) == 0.25
    assert run({k: TRAY[k] for k in ("o2_cylinder", "wrench")}) == 0.5
    assert run({"o2_cylinder": TRAY["o2_cylinder"]}, n=10) == 0.0               # not still for 1 s yet
    assert run({**TRAY, "water_bottle": (0.30, 0.27, M.TOP + 0.10)}) == 0.5     # safe item taken off the cart
    assert run({**TRAY, "clipboard": (0.68, 0.14, M.SH + 0.006)}) == 0.75       # safe item shoved 10 cm, still on cart
    tilted = _obj(M.START["water_bottle"], (0.0, 0.7071, 0.0, 0.7071))           # bottle knocked over
    assert run({**TRAY, "water_bottle": tilted}) == 0.75
    assert run({**TRAY, "trauma_shears": (0.27, -0.06, M.TOP - 0.2)}) == 0.0    # steel on the floor
    assert run({"o2_cylinder": TRAY["o2_cylinder"], "wrench": (0.21, -0.29, M.TOP - 0.3)}) == 0.0
    assert run({**TRAY, "emesis_basin": (0.72, 0.9, M.TOP + 0.01)}) == 0.5      # safe item off to the far side
    held = {"left_tcp": (0.27, 0.14, M.TOP + 0.1), "right_tcp": AWAY["right_tcp"]}
    assert run(TRAY, tcp=held) == 0.5                                           # cylinder still held: 2 cleared
    assert run({**TRAY, "wrench": (0.555, -0.30, M.SH + 0.007)}) == 0.5         # wrench left on the cart
    # the review's case: a 29 cm cylinder turned along x, its centre 11 cm in front of the cart, its valve end still
    # over the cart: not cleared
    along_x = _obj((0.31, 0.14, M.TOP + 0.05), (0.0, 0.0, -0.7071, 0.7071), (0.145, 0.032, 0.045))
    assert run({**TRAY, "o2_cylinder": along_x}) == 0.5
    assert run({"o2_cylinder": (0.50, 0.62, M.TOP + 0.05)}) == 0.0              # off the cart but not in the tray
    assert run({**TRAY, "o2_cylinder": (0.27, 0.25, M.TOP + 0.05)}) == 0.5      # valve end over the tray's end wall
    assert run({**TRAY, "trauma_shears": (0.27, -0.30, M.TOP + 0.01)}) == 0.5   # shears' loops over the tray's end
    assert run({**TRAY, "wrench": (0.33, -0.29, M.TOP + 0.01)}) == 0.5          # wrench handle past the tray's back
    # falls then recovered: still 0
    drop = [frame({"wrench": (0.21, -0.29, M.TOP - 0.3)})]
    assert run(TRAY, pre=drop) == 0.0
    print("all mricart scorer tests passed")


if __name__ == "__main__":
    test()
