# Close the inlet when the high-level alarm comes on, then carry on sorting

A feed tray with three cubes, a red bin and a blue bin, and beside them a tank that is filling from an inlet valve. The
robot is told only the goals: sort the cubes by colour, let the tank's batch reach its high level, and do not
let the tank overflow. Nothing says how: it has to notice the high-level lamp turn red, work out that the valve in
front of it is the tank's inlet, and close it in time without stopping the batch short. The tank reaches its alarm level 20 s in and overflows at 30 s,
so the alarm arrives part way through the sorting job: the right call is to answer it, closing the inlet with whichever
arm is free while the sort goes on, and to finish the sort with the inlet shut.

The task needs both arms: the left sorts the cubes and the right pushes the inlet handle closed.

## Claimed revision

This task builds on pull request #380, whose revision hold lapsed without a revision. It answers that
pull request's review (TK-CVKHAK).

| Reviewer point | What changed |
| --- | --- |
| The instruction should only state the batch and safety goals | The instruction now reads: "Sort the cubes from the feed tray into the bins of their colour. The tank beside you is filling: let this batch reach its high level, and do not let the tank overflow." It no longer mentions the lamp, the valve or what to do when the alarm comes on, so answering the alarm (noticing the lamp, finding the inlet, closing it after the alarm and before the overflow) is left to the robot. The same text is in `task.json`, `INSTRUCTION` in the task file and the run commands below, and a test checks they match. |
| (Our addition) how a real trial shows the level and the alarm | The tank is kept dry and a small controller runs the same counter as the simulation: it lights the level strip, the lamp and the mat's overflow light, reads the handle with a slide potentiometer, and logs when the alarm came on and when the handle crossed 30% and 70% (Building it for real). |

The scoring is unchanged: the instruction's "reach its high level" is the batch the early-close cap protects,
and "do not let the tank overflow" is the overflow zero. The revised task was run in simulation (Runtime and
seeds).

## Files

| File | What |
|---|---|
| `close_the_inlet_when_the_high_level_alarm_comes_on.py` | The yam_arena task file: objects, `success()`, `score()`, `distance()` and the scripted `demo()` |
| `benchkit.py` | Tools built from several boxes with sliders (the valve's handle), per-step hooks, and `recolor` |
| `plantkit.py` | Lamps, the clock, the level strip, and the slider and placement checks |
| `test_close_the_inlet_when_the_high_level_alarm_comes_on.py` | Offline tests: no GPU needed |

`benchkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task
runs with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), with a Dream API
key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=close_the_inlet_when_the_high_level_alarm_comes_on.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sort the cubes from the feed tray into the bins of their colour. The tank beside you is filling: let this batch reach its high level, and do not let the tank overflow." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/inlet-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=close_the_inlet_when_the_high_level_alarm_comes_on.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Sort the cubes from the feed tray into the bins of their colour. The tank beside you is filling: let this batch reach its high level, and do not let the tank overflow." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/inlet-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/inlet-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial ends at 1.0 once all three cubes are in their bins, the inlet has been
closed after the alarm and before the overflow, and the cubes have stood still for 1 s; otherwise it scores the last
step.

Offline tests (no GPU), with `yam_arena` importable:

```bash
python -m pytest -q test_close_the_inlet_when_the_high_level_alarm_comes_on.py
```

## Building it for real

No liquid is used, as in the simulation: the tank is dry and the level is shown, not filled.

- **Level strip and lamp.** Each of the eight segments is a 4 mm white acrylic bar over an 8 cm piece of addressable
  LED strip (WS2812B), lit blue when full and dark when not. The alarm lamp is a 5 x 5 x 3 cm translucent acrylic box
  over one RGB LED, grey when off and red when on. The high level is a 2 cm red line of tape left of the strip, at the
  top of the sixth segment. A blue LED strip runs along the mat's border and lights on overflow.
- **Handle.** The handle is the knob of a 30 mm travel slide potentiometer (Bourns PTA3043 series) set in the valve
  block's slot, so the handle's 3 cm travel is the potentiometer's travel and 30% and 70% are read off its wiper.
- **Controller.** An Arduino Uno runs the simulation's counter: from the start button, one segment every 5 s while the
  handle is short of 70%, the lamp red at six segments, the overflow light above eight. It logs, in milliseconds from
  the start, when the lamp came on, when the handle first crossed 30% and 70% and its position at the end, and when
  the tank overflowed, over USB serial. Parts are about US$40 besides the acrylic.
- **Scoring a trial.** The inlet's part of the score comes from the log; the lamp and the handle are in the cameras'
  view, so the video checks it. The cubes are scored from a photo of the bins at the end.
- **Reset.** Stand the cubes back on the tray, slide the handle to the open end and press the controller's reset.

## Scoring

The tank is 12 x 12 cm inside and 14.4 cm deep to its rim (2.07 L). The strip's eight segments each mark 1.8 cm of
depth, about 260 mL, and the strip shows the level to the nearest segment. While the inlet is open the tank fills one
segment every 5 s (0.36 cm/s, about 3.1 L/min). It starts at two segments (3.6 cm). The alarm lamp turns red at six
segments (10.8 cm; 20 s if nothing is done), the high level that a red line left of the strip marks, and the tank
overflows above eight, its rim (30 s), which turns the mat around its foot
blue. The inlet is closed when its handle is past 70% of its 3 cm travel (2.1 cm: its near edge at the
white line across the block's middle); a handle pushed past 30% (9 mm) before the lamp comes on counts as closing
early. A cube is in a bin when its centre is inside the walls and its underside is under 1.5 cm up.

| Outcome | Score |
|---|---|
| Three cubes in their matching bins, inlet closed after the alarm and before the overflow, still closed at the end | 1.0 |
| Each cube on the floor of its own bin | 0.2, up to 0.6 |
| Inlet closed after the alarm came on and before the overflow, and still closed at the end | +0.4 |
| Handle pushed past 30% before the alarm came on (the batch stopped early), whatever the sorting | at most 0.3 |
| Alarm on and the inlet open at the end: never closed, or closed and then opened again | 0 |
| Opened again after a close, then closed again before the overflow | scored as closed in time |
| Tank overflowed (inlet open past the rim), whatever else | 0 |
| A cube in the wrong bin, on another cube, on a rim or anywhere else | 0 for that cube |
| Valve knocked over more than 20 degrees | reads as open |
| Nothing done (overflow at 30 s) | 0 |
| Still sorting with the lamp off (only possible before 20 s) | the cubes only |

## Design notes

- **An unanswered alarm scores 0.** Once the lamp is on, an inlet that is open at the end scores 0 whatever was
  sorted, whether it was never closed or closed and opened again; and sorting all three cubes takes the demo 28.8 s,
  so a robot that just keeps sorting finishes with an overflowing tank anyway. Opening the inlet again and closing it
  again before the overflow still earns the close.
- **Closing early scores below every in-time partial.** The batch is meant to fill to the high level, so shutting the
  inlet before the alarm leaves it short: the total is capped at 0.3, under the close alone done in time (0.4) and the
  in-time partials with one or two cubes (0.6, 0.8). A handle pushed past 30% before the lamp comes on counts as
  closing early, so staging the handle just short of closed does not pay.
- **Ten seconds to respond.** The lamp leads the overflow by 10 s. The demo starts its push 2 s after the lamp and has
  the handle past 70% at 24.75 s, with the level at 6.9 segments (12.5 cm, about 1.8 L). Everything time-dependent is
  tracked step by step in the step hook and reset with the scene; the end state is what's scored.
- **The handle has no spring.** It stays where it is pushed, so the scorer reads its position at the end and the steps
  it first crossed 30% and 70%. The tank goes on filling for as long as the handle is short of 70%.
- **The default 1 s success hold is enough.** Once the safe finish holds, the only way to spoil it is to reopen the
  inlet; doing so drops the score to 0 at once, so the hold restarts, and a reopen after the trial has ended cannot
  happen.
- **Layout: the tank.** The wrist camera sits 7 cm forward of the TCP and 6 cm above it, so at the end of the push it
  reaches into the tank's x-span 5 cm below the rim. With the tank in the valve's row (y -14) it would run into the
  tank's corner; the tank stands at y -8 instead, where the camera's body passes 4.3 cm clear of it, the 6.8 cm wide
  gripper housing 3.3 cm clear, and the forearm 5 cm above the rim. The lamp moved with the tank to (38, -21), beyond
  it and to its right, clear of its corner and of the mat. The mat is 19 x 16 cm, offset to the tank's right so it
  stays right of the midline, with a 2.4 to 2.9 cm border on three sides so the overflow shows in the frames. The
  valve block's far face is 3.9 cm in front of the tank's wall (the TCP line 6.4 cm).
- **Layout: the cubes.** At the grip the cubes' tops are 1.3 cm above the TCP, where an open finger pad is 2 cm wide
  either side of the TCP; the carriages widen to 3.4 cm only 4 cm up, above the cubes. The red cubes stand 11 cm
  apart with the blue 5.5 cm from each in x, so an open pad clears the neighbouring cube by 1.75 cm. The red bin is
  12 cm long so the two red cubes land 1.5 cm apart with 1.75 cm to the walls; the fingers open 9 mm above the rims
  and the first cube drops 3 cm. The second red cube is let go 1.2 cm higher, so the pad passing over the first clears
  it by 1.6 cm.
- **Unintended strategies.** A cube in the wrong bin, on top of another cube, on a bin's wall or anywhere else earns
  nothing. Throwing a cube counts only if it ends on the bin's floor. Shoving the valve block changes nothing, since
  the handle is read in the block's own frame; a block knocked over 20 degrees reads as open, so the tank keeps
  filling and the alarm goes unanswered. Pushing the handle to the closed end and pulling it back scores 0.
- **The rule behind it.** [ISA-18.2](https://www.isa.org/standards-and-publications/isa-standards/isa-standards-committees/isa18)
  requires, and [EEMUA 191](https://www.eemua.org/Products/Publications/Print/EEMUA-Publication-191.aspx) (Alarm
  Systems: A Guide to Design, Management and Procurement) recommends, that an alarm give the operator time to respond
  before the consequence. The
  [Ten States Standards](https://www.health.state.mn.us/communities/environment/water/docs/tenstates/waterrev2012.pdf)
  (Recommended Standards for Water Works, Part 5) require chemical storage tanks to have level indication, overflow
  provision and containment. The robot's part is the response within the time the alarm gives, while busy with
  another job.
- **What isn't modelled.** There is no liquid: the level is a counter shown on the strip, the tank's floor is blue
  and the water does not rise. The inflow stops the moment the handle passes 70% rather than throttling down, the
  valve is a sliding handle rather than a quarter-turn lever, and the alarm needs no acknowledgement.

## Runtime and seeds

- The scripted demo takes about 30 s of simulated time (596 steps at 20 Hz): the sort ends at 28.8 s and the right
  arm's return at 29.8 s.
- Dream-1 runs the full 2,400 steps unless it finishes first (2 minutes of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start.
- On this revision (goals-only instruction, high-level mark), run on an L40S with `-E object_jitter=0` and seed 0:
  the scripted demo scores 1.0, logging 573 actions before the success hold ends the trial; Dream-1 scores 0.0 and
  runs to the 2,400-step limit.

## Origin

This task builds on pull request #380 in `robocurve/benchmark-tasks`.
