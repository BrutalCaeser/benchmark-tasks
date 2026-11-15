# Load a filled syringe into a syringe pump: flange in its slot, driver up to the plunger without pushing it

A syringe pump stands on the table with its plunger driver pulled back and its barrel clamp down in the empty cradle,
and a filled syringe lies beside it on two stands. The robot loads it the way a nurse does: it lifts the barrel clamp
and swivels it away onto the pump's handle, lays the barrel in the cradle with the flange at the barrel's end down in the
flange slot, slides the plunger driver along its track until it is up against the end of the plunger without pushing
the plunger in, then turns the clamp back and lowers it onto the barrel. The steps are from the Medfusion 3500 quick
reference card: "Lift the syringe barrel clamp and swivel it away", "rest it on the handle"; "Make sure the flange on
the syringe barrel is secure in the syringe barrel flange clip"; "gently advance the plunger driver toward the syringe
plunger" until "it is flush with the syringe plunger"; then "Turn and lower the barrel clamp onto the barrel of the
syringe". The operator's manual: "The clamp holds the syringe barrel securely in place."

Both halves are physical. The 2.5 mm flange has to go straight down into a 3.5 mm slot while the barrel settles into
its cradle, so the syringe has to arrive over the slot within a few millimetres and level. Then the driver has to be
brought up to a plunger that is free to move: overshoot by a millimetre and the plunger goes in (a dose the patient
wasn't meant to get), stop short and the pump would siphon or under-deliver. And the clamp has to be lifted clear of the
cradle, swung onto its handle, then swung back and set down on the barrel without knocking the seated syringe. One arm
works the clamp and loads the syringe; the other works the driver.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder. The standard `yam_arena` embodiment runs it: importing
the task file installs the pump (`mechanism.py`) and the syringe and plunger (`parts.py`) into the scene, so nothing needs
installing.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=load_syringe_pump.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the filled syringe into the syringe pump. Lift the barrel clamp and swivel it away onto the handle, lay the syringe in the pump's cradle with the flange at the end of its barrel down in the flange slot, slide the grey plunger driver along its track until it is up against the end of the plunger without pushing the plunger in, then turn the barrel clamp back and lower it onto the barrel." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/load-syringe-pump-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=load_syringe_pump.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the filled syringe into the syringe pump. Lift the barrel clamp and swivel it away onto the handle, lay the syringe in the pump's cradle with the flange at the end of its barrel down in the flange slot, slide the grey plunger driver along its track until it is up against the end of the plunger without pushing the plunger in, then turn the barrel clamp back and lower it onto the barrel." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/load-syringe-pump-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/load-syringe-pump-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the syringe is seated, the clamp is down on the barrel,
the driver is up against the plunger and everything has been still for 1 s; otherwise it scores the last step.

## Scoring
The syringe is **in the cradle** when its barrel's nose end rests on the cradle's rods (the axis there within 3 mm of
the cradle's centre line sideways, 2.5 mm of its seated height and 5 mm of its seated place along the cradle) and the barrel is within 15 degrees of the cradle's
line. It is **seated** when the axis is within 1 mm of its seat (sideways and up), level and square to the cradle within
1.5 degrees, and the flange's centre is within 0.8 mm of the slot's. The plunger is **in the barrel** when its stopper is
within 1 mm of the axis and its rod within 5 degrees of it. The barrel is **clamped** when the clamp is swung back over
the cradle (within 5 degrees) with its pad down on the barrel's top (the post within 0.5 mm of that height) and not
moving (under 0.5 mm and 1 degree a step): left on the handle, or held up off the barrel, it isn't. The driver is
**near** when its face is within 5 mm short of the thumb disc, and **flush** from 1 mm short of the disc to 0.2 mm into
it (contact overlap) with everything still (under 1 cm/s and 0.5 rad/s; the driver moving under 0.5 mm a step). A face
more than 0.2 mm past the disc holds nothing and counts as not near.

| State | Score |
|---|---|
| Plunger pushed in more than 1 mm at any point (a dose given), even if pulled back later | 0 |
| Plunger drawn back more than 3 mm at any point, or out of the barrel | 0 |
| Syringe not in the cradle: on its stands, held above, off the side or off the rods' end, turned end for end | 0 |
| In the cradle, not seated: flange on the body beside its slot, syringe cocked | 0.2 |
| Seated, not clamped: driver back / near / flush | 0.4 / 0.5 / 0.6 |
| Seated and clamped, driver back | 0.5 |
| Seated and clamped, driver near | 0.75 |
| Seated and clamped, driver flush, everything still | 0.9 |
| The same, held 1 s: the trial ends there | 1.0 |

Possible scores: 0, 0.2, 0.4, 0.5, 0.6, 0.75, 0.9, 1.0. A trial that ends without the full state having held 1 s scores its last
step, so a syringe that settles only in the final second gets 0.9. Ramming the driver home pushes the plunger in and
scores 0, and so does pulling the plunger out of the barrel. On a real pump: a side video for the flange in its slot,
the clamp down on the barrel, the driver at the thumb disc and the plunger never moving off its graduation.

## Design notes
- **Where the pieces come from.** `mechanism.py` is copied from robocurve/benchmark-tasks pull request #223 (MIT
  License; the notice is at the top of the file) with one change, marked CHANGED: a joint may ride on another moving
  link instead of the base, for the clamp's arm, which swivels on its lifting post. The pump is a fixed body with the
  driver on a sliding joint and the clamp on its post, and `score()` reads their positions. `parts.py` (ours) builds the barrel and the plunger as loose parts from boxes.
- **How precise it has to be.** The flange has 0.5 mm of play a side in its slot. The barrel's axis has to end within
  1 mm of its seat, and the driver from 1 mm short of the thumb disc to 0.2 mm into it (the official demo ends at
  -0.007 mm, touching) while the plunger moves less than 1 mm in. A run that
  seated the syringe and then drew the plunger 27 mm back, nearly out of the barrel, scored 0 (official sim, 2026-11-07).
- **The cradle** is two rods the barrel rests between, touching it 45 degrees either side of its lowest line, so a
  barrel set down 2 mm off the line rolls onto it (official sim). What decides the score is the flange in its 3.5 mm slot
  and the driver at the plunger, not where the arm let go across the cradle.
- **The flange slot's mouth** is rounded (5 mm), as a flange clip's lead-in is. With square edges the flange landed on
  the pump's body beside the slot and the syringe sat cocked 8 degrees: the misloaded flange PMDA warns about.
- **The plunger.** The stopper slides with little friction (a real one needs a firm push to start), so any contact
  moves it a little. The scorer allows 1 mm of plunger travel in and 3 mm out: the driver can come up against the thumb
  disc, but driving the plunger in, or drawing it back or out, scores 0. The plunger's travel is recorded from the
  reset, so a push that is later pulled back still counts.
- **The barrel clamp** is a post that lifts on a light spring and an arm that swivels about it, with a pad at its tip
  over the barrel's axis. At the start it hangs in the empty cradle, 6 mm below where a barrel's top would be, so the
  syringe can't be seated without first moving it. Swivelled a quarter turn it rests on the handle 4.6 mm lower than on a
  barrel, so it stays out of the way without a hand; lifted about 5 mm it clears a seated barrel to swing back over it.
  Its spring is light (with its own weight, 0.8 N on a barrel) because the arm gives about 6 mm per newton. The right
  arm works it: the knob, behind the cradle, is out of the left arm's reach at these heights.
- **Sizes** are a stand-in for a 60 mL syringe (barrel 29.6 mm across); the clamp's are a stand-in too.

## Runtime and seeds
- With the barrel clamp (official sim, inspect-robots 1.0.0, 2026-11-15): the scripted demo reaches 1.0 at step 953
  (48 s of simulated time): the clamp swivelled 89.9 degrees onto the handle and rested 4.6 mm down, the syringe seated,
  the driver flush with the thumb disc, the clamp turned back and resting on the barrel 0.06 mm above its seated height,
  and the plunger moved 0.01 mm. Dream-1 used all 1,800 steps and scored 0.0: its arms moved over the table, but the
  syringe stayed on its stands. The delayed starts and offsets below were run before the clamp was added.
- Before the clamp, per trial on one L40S: the scripted demo reached 1.0 at step 575 (29 s of simulated time, about 2 min of wall clock
  with stored frames); Dream-1 uses all 1,200 steps (about 4 min) and scored 0.0: its arms moved over the table, but
  the syringe stayed on its stands. On the revised task (2026-11-12) the stock Dream adapter rejected Dream-1's first
  reply (a left gripper of -0.062, past its 0.05 slack around 0..1) on both tries; with that slack widened to 0.1 for
  one run, Dream-1 again used all 1,200 steps and scored 0.0, the syringe on its stands.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start. The demo also reaches 1.0 started 0.05
  and 0.10 s late (`demo(delay=...)`), and with the syringe set down 2 mm either way across the cradle or 1 mm either
  way along it (official sim, no stored frames, 2026-11-07). After the revision (official sim, inspect-robots 1.0.0,
  2026-11-12): the demo reaches 1.0 at step 575, also started 0.05 and 0.10 s late, and ends with the driver 0.007 mm
  into the thumb disc; the same demo with the driver aimed 5 mm past touching pushes the plunger in and scores 0.
- `test_load_syringe_pump.py` checks the scoring without a GPU (`python -m pytest -q test_load_syringe_pump.py`): the
  start, every credit level clamped and not, each limit just inside and just outside (seat, tilt, yaw, flange, in the
  cradle, the clamp's swivel and height, a clamp still moving, driver,
  stillness and spin, a driver still moving, plunger in, out and in its barrel), a push or pull later undone, the
  plunger pulled out and left anywhere or turned end for end, the driver run past a plunger drawn out of its way, a
  push then a retract (0 for the rest of the trial), negative gaps, the syringe off the rods' end or turned end for end, the
  1 s hold (the complete state alone scores 0.9), the layout, the setup text's numbers, and the demo's reach and
  delay. A mutation check (35 deliberate breaks of the scorer, layout and demo, each run against the tests) caught 35
  of 35 before the revision (2026-11-07).
