# Load two tubes into a centrifuge so it can spin

Two sample tubes stand in holders beside a centrifuge rotor with six pockets. The robot is asked to load both so the
rotor is ready to spin. The lab rule it has to apply without being told: tubes of equal weight go in opposite pockets,
or the spinning rotor is unbalanced. Loading both tubes side by side is the failure this task exists to catch, so it
scores 0, below loading just one.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=balance_centrifuge.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load both sample tubes into the centrifuge rotor so it is ready to spin." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/centrifuge-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=balance_centrifuge.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load both sample tubes into the centrifuge rotor so it is ready to spin." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/centrifuge-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/centrifuge-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once both tubes have sat in opposite pockets for 1 s;
otherwise it scores the last step.

## Scoring
A tube is seated when, at the end, it is down in a pocket:
- its lowest point is within 5 mm of the rotor disc's top surface (it is down in the pocket, not resting on a rim);
- the centre of its lower end is inside the pocket's 4 cm square opening (in a corner, an upright tube's centre is
  7.1 mm from the pocket's centre, so a radius would wrongly reject it);
- it leans at most 18 degrees (a seated tube can lean about 14 degrees against the pocket walls).

It need not have stopped moving: in the sim a released tube can keep rocking in its pocket for seconds (PhysX has no
rolling resistance), and which pocket it is in is what balance depends on. The 1 s success hold rules out a tube that
only bounces through.

| Outcome | Score |
|---|---|
| Both tubes seated in opposite pockets | 1.0 |
| Exactly one tube seated, the other clear of the rotor (no part of its axis within 10 cm of the rotor centre) | 0.25 |
| Both seated, not opposite (an unbalanced rotor) | 0 |
| One seated, the other lying on or across the rotor or perched on a rim | 0 |
| No tube seated | 0 |

On a real table, open both grippers and take a top-down photo: it shows which pockets hold tubes. A tube that is
fully down stands 7.5 cm above the pocket rim.

## Design notes
- **The instruction doesn't state the balance rule, on purpose.** People who load centrifuges are told to load and
  spin, not how to balance; applying the rule unprompted is what this task measures. The task is still unambiguous:
  with two equal tubes in a six-pocket rotor, every correct answer is an opposite pair, and all three pairs are
  reachable, each arm loading its own tube (the left arm reaches the pockets at 30, 90, 150 and 210 degrees, the right
  arm those at 150, 210, 270 and 330; checked with the official IK; 0 degrees is straight ahead, counterclockwise from
  above).
- **Proximity doesn't pass.** The pocket nearest each arm is next to the other arm's (150 and 210 degrees), so putting
  each tube in the nearest pocket scores 0. Picking reachable pockets at random is balanced 3 times in 14 (about 21%).
- **One tube scores more than two badly placed ones.** A single tube is also unbalanced if spun, but the job is
  visibly unfinished (the second tube is still clear of the rotor; the score checks that). Two tubes in the wrong pockets look finished, which is
  the failure the rule exists to prevent. So a robot that doesn't know the rule does better to stop than to guess.
- **Seating ignores motion and grip.** In the sim a released tube can rock in its pocket (PhysX has no rolling
  resistance); requiring it to be still would fail correct placements at random. And the real-world check opens the
  grippers before the photo, so a tube held down in a pocket ends up seated either way.

## Runtime and seeds
- The scripted demo loads the left tube, then the right one (the grippers can't both be over the rotor), in about 24 s
  of simulated time.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- `test_balance_centrifuge.py` checks the scoring on every opposite and unbalanced pair, a tube leaning, rocking, spun,
  held up, lying down or off centre, the demo's order, the reach of all three balanced answers and of the demo, and
  that the nearest pockets are unbalanced (`python -m pytest -q test_balance_centrifuge.py`, no
  GPU needed).
