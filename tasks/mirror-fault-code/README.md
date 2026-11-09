# Read a fault code that faces the wall in an inspection mirror, then fetch the spare it names

A pump stands in front of a wall. Its controller shows a one-digit fault code on a small backlit display on the pump's
back, which faces the wall, so no camera can see it. The robot reads it the way a technician does, with an inspection
mirror: it carries the mirror round the pump, sets it down behind the pump facing it, reads the code in the glass, and
then stands the filter cartridge with that number on a green pad, leaving the three look-alikes alone. The FAA's
advisory circular on visual inspection, AC 43-204 (*Visual Inspection for Aircraft*, 1997): "The mirror can be used
with great effect in locations that cannot be viewed directly", and "The flashlight and the mirror are standard tools
which should accompany the inspector on all inspections." Done by: maintenance technicians and inspectors.

The hard part is seeing through a tool, then acting on what it showed. The display sits in a hood 3 cm deep, whose mouth
is 14.5 cm from the wall: with the mirror taken away, a wrist camera held over the gap, beside it and turned back toward
the pump never reads the code (seven poses traced; at most an edge of the display shows). The mirror has to go down in
that gap, flat and turned to the pump within 20 degrees, with its glass clear of the wall; then the right one of four
cartridges has to be picked. A wrong cartridge moved scores 0, so a guess wins one time in four.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder. The standard `yam_arena` embodiment runs it: importing
the task file builds the pump, the mirror and the cartridges (`parts.py`), so nothing needs installing.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=mirror_fault_code.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The pump's fault code shows on the display on its back, facing the wall. Set the inspection mirror on the table behind the pump, facing it, read the code in the mirror, then stand the filter cartridge with that number on the green pad. Leave the other cartridges where they are." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mirror-fault-code-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=mirror_fault_code.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The pump's fault code shows on the display on its back, facing the wall. Set the inspection mirror on the table behind the pump, facing it, read the code in the mirror, then stand the filter cartridge with that number on the green pad. Leave the other cartridges where they are." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mirror-fault-code-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/mirror-fault-code-demo/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. A trial scores 1.0 once the mirror is set, the right cartridge stands on the
pad and everything has been still for 1 s; otherwise it scores the last step, which tops out at 0.9.

## Scoring
The mirror is **set** when its whole footprint lies between the hood's mouth and the wall, its centre within 4 cm of
the pump's centre line, its base flat on the table within 10 degrees and its glass facing the pump within 20 degrees.
The code's cartridge is **on the pad** when its centre is within 2 cm of the pad's, upright within 10 degrees and
resting on the pad. A wrong cartridge counts as **taken** once it is 1.5 cm from its spot or tipped 15 degrees.

| State | Score |
|---|---|
| A wrong cartridge taken, or the pump moved 5 mm | 0 |
| Nothing done | 0 |
| Mirror set | 0.25 |
| Right cartridge out of its spot (3 cm or more) | +0.25 |
| Right cartridge on the pad | +0.5 instead |
| Mirror set, right cartridge on the pad, everything still | 0.9 |
| The same, held 1 s | 1.0 |

Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1.0. The start scores 0. On a real bench: a top photo of the pad and the
cartridges' spots, and one of the mirror behind the pump.

## Design notes
- **Where the pieces come from.** `mechanism.py` is copied unchanged from robocurve/benchmark-tasks pull request #223
  (MIT License; the notice is at the top of the file); `parts.py` needs its import hook. `parts.py` (ours) builds loose
  parts from boxes; a piece in its `MIRROR` colour is polished metal and one in `GLOW` lights itself.
- **The mirror.** A hand inspection mirror is tilted by the wrist; a top-down gripper can't, so the glass is fixed at 50
  degrees on a weighted base and the robot carries it by its post, sets it down and turns it. The glass is a common
  2-1/4 by 3-1/2 inch head.
- **The display** lights itself, as a backlit LCD does, so the code reads in the mirror under the station's lights. Its
  hood keeps it out of every direct view the arms can reach (traced, mirror removed).
- **The demo** plays fixed waypoints from the known layout and does not read the code: it carries the mirror round the
  pump's side, sets it down behind the pump and holds its wrist camera over it, then fetches cartridge 3. The code shows
  in the glass in the right wrist camera as the mirror goes down (the demo video's right half, from about 9.5 s). A
  policy has to read it there.

## Runtime and seeds
- Per trial on one L40S (inspect-robots 1.0): the scripted demo reaches 1.0 at step 549 (27.5 s of simulated time,
  about 3 min of wall clock with stored frames). Dream-1 used all 2,400 steps and scored 0.0: it reached toward the
  mirror with its right arm and toward the cartridges with its left, and moved neither.
- The demo with its first move 0.05 s and 0.10 s late: 1.0 and 1.0. With the mirror set 1 cm to the side: 1.0; set 1 cm
  closer to the pump, its base reaches into the hood's mouth and it does not count as set: 0.5. The demo fetching
  cartridge 1 instead of 3: 0.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- `test_mirror_fault_code.py` checks the scoring and layout without a GPU (`python -m pytest -q
  test_mirror_fault_code.py`): the start, every credit level, the 0.9 cap, a wrong cartridge moved, bumped or put on the
  pad, the pump moved, the mirror's zone, flatness, facing and resting limits either side, the pad's limits, motion and
  spin, the layout's clearances, and the demo's reach. A mutation check (22 deliberate breaks of the scorer, each run
  against the tests) catches 22 of 22.
