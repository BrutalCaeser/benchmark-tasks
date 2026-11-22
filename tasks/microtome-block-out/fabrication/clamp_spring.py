"""Force check for the fixture's cassette clamp spring (no simulator): python3 clamp_spring.py

The spring stands upright in the spring bridge's 8 mm pocket, 40 mm in front of the rocker's pivot, and presses down on
the top of the lever's forward arm. Pushing the lever's foot forward turns the rocker, lifts that arm and squeezes the
spring. This prints the spring's length and force, the torque on the rocker and the push at the lever's foot, at rest
and opened, beside the simulation's rocker spring, and checks the 15 degree point against the 3.5 to 5.5 N target,
with the catalogue's free length and rate tolerances.

Spring numbers are Lee Spring's catalogue values for LC 026D 10 S (stainless), read from
https://www.leespring.com/product/compression-spring-lc026d10s-stainless-steel on 2026-11-22.
Geometry is the fixture's (assembly coordinates, mm, carriage at its start): pivot z 201.45; the arm's top 2.0 mm above
the pivot; the pocket's axis 40 mm in front of the pivot; the pocket's ceiling (the cap's underside) at z 226.5; the
lever's foot 41 mm in front of and 101.45 mm below the pivot, pushed square to the bar's original line (along +y).
"""

import math

RATE, RATE_TOL = 0.736, 0.07          # N/mm, catalogue rate and its tolerance
FREE, FREE_TOL = 25.40, 0.84          # mm, catalogue free length and its tolerance
SOLID, F_SOLID = 5.82, 15.93          # mm, catalogue solid height; N, catalogue load at solid
ARM, ARM_UP = 40.0, 2.0               # mm: pocket axis in front of the pivot; the arm's top above the pivot
CEILING = 226.5 - 201.45              # mm: pocket ceiling above the pivot
FOOT_Y, FOOT_Z = 41.0, -101.45        # mm: the lever's foot from the pivot
MODEL_K, MODEL_T0 = 1.2, 0.06         # simulation: 1.2 N m/rad, target -0.06 rad (0.072 N m at rest)
LO, HI = 3.5, 5.5                     # N at the foot, 15 degrees open


def rise(th):
    """How far the arm's top, on the pocket's axis, rises when the rocker opens th radians."""
    return (ARM + ARM_UP * math.sin(th)) * math.tan(th) + ARM_UP * math.cos(th) - ARM_UP


def foot(th, free=FREE, rate=RATE):
    length = CEILING - ARM_UP - rise(th)          # spring length between the arm and the ceiling
    f = rate * (free - length)                    # spring force, N
    torque = f * ARM / 1000.0                     # N m (the spring acts on the pocket's axis, 40 mm out)
    arm_foot = -(FOOT_Y * math.sin(th) + FOOT_Z * math.cos(th)) / 1000.0
    return length, f, torque, torque / arm_foot, arm_foot


def main():
    print("deg  length_mm  spring_N  torque_Nm  foot_N  model_Nm  model_foot_N")
    for deg in (0, 5, 10, 15, 18):
        th = math.radians(deg)
        length, f, t, fn, af = foot(th)
        mt = MODEL_K * (th + MODEL_T0)
        print(f"{deg:3d}  {length:9.2f}  {f:8.2f}  {t:9.3f}  {fn:6.2f}  {mt:8.3f}  {mt / af:12.2f}")
    th15 = math.radians(15)
    print(f"preload at rest {FREE - (CEILING - ARM_UP):.2f} mm; 15 deg: arm rises {rise(th15):.2f} mm")
    solid_deg = next(d / 10 for d in range(0, 400) if CEILING - ARM_UP - rise(math.radians(d / 10)) <= SOLID)
    print(f"solid ({SOLID} mm, {F_SOLID} N) at about {solid_deg:.1f} deg; deflection used at 15 deg "
          f"{FREE - foot(th15)[0]:.2f} of {FREE - SOLID:.2f} mm")
    corners = [foot(th15, FREE + a * FREE_TOL, RATE + b * RATE_TOL)[3] for a in (-1, 1) for b in (-1, 1)]
    lo, hi = min(corners), max(corners)
    print(f"15 deg foot force, catalogue tolerance corners: {lo:.2f} to {hi:.2f} N (target {LO} to {HI} N)")
    assert LO <= foot(th15)[3] <= HI and LO <= lo and hi <= HI, "out of the 3.5 to 5.5 N band"
    assert solid_deg > 18.0, "the spring must not go solid before the demo's 18 degree opening"
    print("ok")


if __name__ == "__main__":
    main()
