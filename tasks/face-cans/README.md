# Face cans to the shelf edge

Four cans stand at the back of a shelf board. The robot pulls each one forward until its front is flush with the
board's front edge (the one nearest the arms), without tipping it or pushing it off. It is the "facing" step stockers repeat across a store: a
short reach, a 12.5 cm carry, and a 9 mm window to stop in. Two cans stand in front of each arm.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=face_cans.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pull each can forward to the front edge of the shelf, keeping it upright and on the shelf." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/face-cans-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=face_cans.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pull each can forward to the front edge of the shelf, keeping it upright and on the shelf." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/face-cans-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/face-cans-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all four cans have been faced for 1 s; otherwise it
scores 0.25 per faced can at the last step.

## Scoring
A can is faced when, at the end:
- it stands upright, tilted at most 10 degrees, and still (moving under 1 cm/s);
- it stands directly on the board, not on the table or on another can;
- its frontmost point is between 6 mm behind and 3 mm past the board's front edge.

0.25 per faced can, 1.0 for all four. On a real table, check each can with a ruler held against the board's edge and a
photo from the side.

## Runtime and seeds
- The scripted demo faces all four cans in 18 s of simulated time (two cans per arm, both arms at once); the trial then ends
  with success. On an L40S the whole demo run, including starting Isaac Sim, takes about a minute once the machine is warm.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
