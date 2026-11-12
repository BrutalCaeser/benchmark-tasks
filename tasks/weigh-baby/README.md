# Weigh an infant manikin on a beam scale: set the kilogram notch, then slide the gram weight until the pointer is level

A mechanical baby scale stands on the table with an infant training manikin in its tray and both sliding weights at 0.
The robot weighs the manikin the way a medical assistant or nurse weighs a baby on a seca 725: it moves the large
sliding weight from notch to notch until the beam tips, backs it one notch, then slides the small weight along its gram
bar until the pointer floats level with the counterpointer, and lets go. Weighing patients is a core medical-assistant
duty: O*NET's task list for Medical Assistants (31-9092.00) includes "Interview patients to obtain medical information
and measure their vital signs, weight, and height." The steps are from the seca 725 instructions for use
([PDF](https://www.seca.com/fileadmin/documents/manual/seca_man_725_745_int_DE_EN_FR_IT_ES_PT_PL.pdf), "Weighing"):
"Move the large sliding weight until pointer and counterpointer are at approximately the same level", then "Move the
small sliding weight until pointer and counterpointer are at exactly the same level". The same page warns: "If you touch
the scale, weight will not be measured correctly."

**The manikin's mass changes every trial.** It is drawn at random when the task file loads, uniformly from 2.5 to
4.5 kg to the gram, and nothing in the scene shows it, so no preset of the weights can stand in for weighing: a fixed
reading lands within the 20 g band in at most 2 % of trials. The score is computed against that trial's mass. To run a
trial at a known mass (for checking), set `WEIGH_MANIKIN_KG`, for example `WEIGH_MANIKIN_KG=3.62`.

The large weight's notch decides the kilograms (one notch off is a kilogram wrong), and the small weight has to stop
within 5 mm of the balance point on its 25 cm bar: 20 g. The only feedback is the pointer, which moves about 2 mm per
10 g against a counterpointer that does not move. The beam floats only within about 30 g of the balance; 24 g past it,
the beam already tips 0.86 degrees toward its stop (official sim, inspect-robots 0.60.0, 2026-11-07). So the robot has
to move a weight, let go, watch the pointer, and move it again.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder. The standard `yam_arena` embodiment runs it: importing
the task file installs the scale (`mechanism.py`) and the sliding weights (`parts.py`) into the scene, so nothing needs
installing.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=weigh_baby.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Weigh the infant manikin on the beam scale. Move the large sliding weight one notch at a time until the beam tips down, then back one notch. Then slide the small weight along its bar until the red pointer floats level with the black counterpointer, and let go of both weights." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/weigh-baby-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=weigh_baby.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Weigh the infant manikin on the beam scale. Move the large sliding weight one notch at a time until the beam tips down, then back one notch. Then slide the small weight along its bar until the red pointer floats level with the black counterpointer, and let go of both weights." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/weigh-baby-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/weigh-baby-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the reading is within 20 g, the beam floats and both
weights are still, held for 1 s; otherwise it scores the last step, at most 0.9.

## Scoring
The **reading** is the large weight's notch in kilograms plus the small weight's place on its bar (0 to 1,000 g over
25 cm), measured at the weights' centres. It is exactly what the beam balances: one notch of the large weight, or the
small weight's whole bar, balances 1 kg in the tray. The large weight **sits in a notch** when its centre is within
1 mm of the notch along the bar and it is lifted no more than 0.8 mm.

| State | Score |
|---|---|
| Large weight not sitting in a notch: held up or between notches | 0 |
| In a notch, reading more than 100 g from the manikin's mass: in the notch of its whole kilograms | 0.25 |
| The same, in any other notch | 0 |
| Reading within 100 g | 0.5 |
| Reading within 40 g | 0.75 |
| Reading within 20 g, beam floating clear of both stops, both weights still | 0.9 |
| The same, held for 1 s (the trial ends) | 1.0 |

Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1.0. The start (both weights at 0) scores 0. Any notch counts if the reading
is right: 2 kg with the small weight at 1,000 g balances the same as 3 kg at 0 g. The score reads the weights, not the
beam, so holding the beam level or pressing on the tray earns nothing. It does not see the hands; a weight still held
has to sit at the right reading with the beam floating clear of its stops, which needs the mass, so holding one is no
way around weighing. On a real scale: read the two weights and compare with the manikin's mass, and photograph the pointer against
the counterpointer.

## Design notes
- **Where the mechanism comes from.** `mechanism.py` is copied unchanged from robocurve/benchmark-tasks pull request
  #223 (MIT License; the notice is at the top of the file): the scale is a fixed base with the beam on a hinge and the
  tray on a vertical slide, and `score()` reads the beam's angle. `parts.py` (ours) builds the two sliding weights as
  loose parts from boxes.
- **How the beam weighs.** The tray rests on a knife edge 6 mm from the pivot on the beam's short arm, so the manikin's
  weight tips the beam; the weights on the long arm tip it back. A counterweight balances the empty tray with both
  weights at 0. The bars run below the pivot, so the beam settles at an angle that grows with the imbalance instead of
  falling to a stop: 0.36 to 0.40 degrees per 10 g (official sim, inspect-robots 0.60.0, 2026-11-07). On
  inspect-robots 1.0.0 the beam sits at 0.00 to +0.03 degrees with readings of 3.6145 to 3.6149 kg, about 5 g under the
  manikin's mass; the seca 725's stated accuracy below 4 kg is plus or minus 10 g.
- **The knife edge has no friction**, like a real scale's hardened knife edge.
- **The notches.** Each kilogram notch is two round bars across the bar's top; the large weight's key stands between
  them. The weight lifts out to move and sets back in (a real one is pulled forward to release it).
- **The demo weighs from the camera, not from the answer.** `demo()` never reads the mass. After each move the right
  arm lets go and holds its wrist camera at one pose over the beam's far end, where the red pointer shows against the
  black counterpointer; the demo finds the red pixels in the last second of frames and calls the beam "down" when they
  sit past the midpoint between where they show on the up stop and on the down stop (13 px apart, measured in the
  official sim). It tries the 3 kg notch first (the manikin weighs at least 2.5 kg), then one notch up or down, and
  backs off one notch if the beam tipped; then it halves the small weight's bracket six times, from 0 to 1,000 g, and
  leaves the weight in the middle of the last 16 g bracket. The moves are written into the waypoints still ahead
  (yam_scripted reads them every control step) through a wrapper on the scripted policy's `act()`.
- **Sizes.** The scale's footprint is the seca 725's 552 by 312 mm and its capacity is 16 kg, as in the manual's
  technical data. The beam's proportions, the weights' masses and the 25 cm gram bar follow from the lever ratio, not
  from seca drawings.

## Runtime and seeds
- Per trial on one L40S, inspect-robots 1.0.0, official settings (2026-11-12). The scripted demo reached 1.0 with the
  mass drawn at random (3.451 kg; 1,728 steps, 86 s of simulated time) and at set masses across the range: 2.505,
  3.013 (13 g above a whole kilogram), 4.358 and 4.49 kg. It also reached 1.0 with every move starting 0.05 s late
  (3.375 kg) and 0.10 s late (4.424 kg). Each trial ends once the reading is within 20 g with the beam floating and
  the weights still for 1 s, so some end before the last halving.
- The old fixed demo (3 kg and 620 g, the answer for a 3.62 kg manikin) scores 0 against a 2.95 kg manikin: a preset
  doesn't weigh.
- Dream-1 used all 3,600 steps and scored 0.0 (manikin 3.861 kg): its right arm moved over the beam's far end and its
  left arm stayed near its base.
- Two demo fixes came from these runs, both traced in the official sim: the large weight comes to rest about 2 mm
  nearer the pivot than the arm's target whichever way it moves, so it is aimed 3 mm toward the far end; and the small
  weight trails the fingers 4 to 7 mm when slid toward the pivot but about 1 mm when slid outward, so every move ends
  on an outward slide.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start; only the manikin's mass changes.
- `test_weigh_baby.py` checks the scoring without a GPU (`python -m pytest -q test_weigh_baby.py`): the start, every
  credit band either side, 0.9 until held and never 1.0 from `score()`, a weight held up or between notches, the beam
  on a stop, moving weights, seating and the reading on a tilted beam, the mass drawn within its range and varying,
  the score following the drawn mass (a preset fails), any notch counting when the beam balances, the counterweight's
  balance, the knife edge and hanger, the weights' clearance from the scale's body, the layout, the setup text, the
  demo's pointer reader, its search played against a simulated beam at 13 masses from 2.5 to 4.5 kg (each within
  10 g), that it never reads the mass, and its reach and length.
