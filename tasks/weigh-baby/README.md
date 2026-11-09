# Weigh an infant manikin on a beam scale: set the kilogram notch, then slide the gram weight until the pointer is level

A mechanical baby scale stands on the table with an infant training manikin in its tray and both sliding weights at 0.
The robot weighs the manikin the way a nurse weighs a baby on a seca 725: it moves the large sliding weight from notch to notch
until the beam tips, backs it one notch, then slides the small weight along its gram bar until the pointer floats level
with the counterpointer, and lets go. The steps are from the seca 725 instructions for use
([PDF](https://www.seca.com/fileadmin/documents/manual/seca_man_725_745_int_DE_EN_FR_IT_ES_PT_PL.pdf), "Weighing"):
"Move the large sliding weight until pointer and counterpointer are at approximately the same level", then "Move the
small sliding weight until pointer and counterpointer are at exactly the same level". The same page warns: "If you touch
the scale, weight will not be measured correctly."

The answer is nowhere in the scene: the robot has to find it from the beam. The large weight's notch decides the
kilograms (one notch off is a kilogram wrong), and the small weight has to stop within 5 mm of the balance point on its
25 cm bar: 20 g. The only feedback is the pointer, which moves about 2 mm per 10 g against a counterpointer that does
not move. The beam floats only within about 30 g of the balance; 24 g past it, the beam already tips 0.86 degrees
toward its stop (official sim, inspect-robots 0.60.0, 2026-11-07). So the robot has to move a weight, let go, watch the pointer, and move it
again.

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
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/weigh-baby-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=weigh_baby.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Weigh the infant manikin on the beam scale. Move the large sliding weight one notch at a time until the beam tips down, then back one notch. Then slide the small weight along its bar until the red pointer floats level with the black counterpointer, and let go of both weights." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/weigh-baby-demo

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
| Large weight not sitting in a notch: held up, between notches, or in a wrong notch (at least 380 g off) | 0 |
| Large weight in the 3 kg notch, reading more than 100 g from 3.62 kg | 0.25 |
| Reading within 100 g | 0.5 |
| Reading within 40 g | 0.75 |
| Reading within 20 g, beam floating clear of both stops, both weights still | 0.9 |
| The same, held for 1 s (the trial ends) | 1.0 |

Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1.0. The start (both weights at 0) scores 0. The score reads the weights, not the
beam, so holding the beam level or pressing on the tray earns nothing. It cannot see whether a hand is still on a
weight. On a real scale: read the two weights and compare with the manikin's mass, and photograph the pointer against
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
- **The demo** moves the large weight to the 3 kg notch and the small weight to 620 g, the manikin's known reading. A
  policy has to find that reading from the pointer, which the wrist cameras see beside the counterpointer.
- **Sizes.** The scale's footprint is the seca 725's 552 by 312 mm and its capacity is 16 kg, as in the manual's
  technical data. The beam's proportions, the weights' masses and the 25 cm gram bar follow from the lever ratio, not
  from seca drawings.

## Runtime and seeds
- Per trial on one L40S, inspect-robots 1.0.0: the scripted demo reaches 1.0 at step 439 (22 s of simulated time,
  about 2 min of wall clock with stored frames). Dream-1 uses all 2,400 steps and scored 0.0: its right arm reached
  along the beam toward the far end and hovered there, and its left arm stayed near its base (official runs,
  2026-11-07 and 2026-11-09).
- Timing: with every move of the demo starting 0.05 s or 0.10 s later, it still reaches 1.0 (readings 3.6149 and
  3.6145 kg; inspect-robots 1.0.0, 2026-11-09).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- `test_weigh_baby.py` checks the scoring without a GPU (`python -m pytest -q test_weigh_baby.py`): the start, every
  credit band either side, 0.9 until held and never 1.0 from `score()`, a wrong notch, a weight held up or between
  notches, the beam on a stop, moving weights, seating and the reading on a tilted beam, the counterweight's balance,
  the knife edge and hanger, the weights' clearance from the scale's body, the layout, the setup text's positions,
  and the demo's reach and aims. A mutation check (22 deliberate breaks of the scorer, layout and demo, each run
  against the tests) catches 22 of 22.
