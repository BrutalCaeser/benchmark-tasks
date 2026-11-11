# Read a fault code that faces the wall in an inspection mirror, then fetch the spare it names

A pump stands in front of a wall. Its controller shows a one-digit fault code on a small backlit display on the pump's
back, which faces the wall, so no camera can see it. The robot reads it the way a technician does, with an inspection
mirror: it carries the mirror round the pump, sets it down behind the pump facing it, reads the code in the glass, and
then stands the filter cartridge with that number on a green pad, leaving the three look-alikes alone. The FAA's
*Aviation Maintenance Technician Handbook: General* (FAA-H-8083-30B, 2023, p. 8-11): "Sometimes the inspection areas are
obscured by structural members, equipment installations, or for some reason are awkward to check visually. [...]
Mirrors, borescopes, and fiber optics are useful in providing the means of observing obscure areas." Done by:
maintenance technicians and inspectors.

The hard part is seeing through a tool, then acting on what it showed. The display sits in a hood 5.5 cm deep, whose
mouth is 17 cm from the wall; the hood cuts every straight sight line to the display steeper than 36 degrees, and a
wrist camera pointing down behind the pump looks from above that. With the mirror taken away, the right wrist camera
shows none of the display at any of 28 such poses (four wrist turns, 6 to 15 cm up, traced), and the demo's reader finds
nothing in those frames. A policy that tilted its wrist camera down into the gap could look straight in; nothing in the
scoring forbids that, as nothing stops a technician with room to lean in. The mirror has to go down in the gap, flat and
turned to the pump within 20 degrees, clear of the hood's mouth and the wall; then the right one of four cartridges has
to be picked. A wrong cartridge moved scores 0, so a guess wins one time in four.

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
  degrees on a weighted base and the robot carries it by its post, sets it down and turns it. The glass's size (57 by 89
  mm) is illustrative. The demo squeezes the post hard: held lightly, the mirror swung 24 degrees on the post and landed
  off its mark.
- **The display** lights itself, as a backlit LCD does, so the code reads in the mirror under the station's lights. It
  is 6 by 3.6 cm, its digit 2.8 cm tall, with the hood's visor 4 mm above it.
- **The demo reads the code.** It carries the mirror round the pump's side by its post, sets it down behind the pump,
  then holds the right wrist camera over it for 2 s, turned so that the camera looks into the glass from the wall's
  side; the whole display shows in the glass, upright. `read_digit()` finds the display in that camera's frames (the
  only light green in view), splits it into a 6 by 9 grid of dark-pixel shares, and matches the grid against one
  reference per digit, measured from runs with each code; a frame counts only if one digit matches clearly, and the demo
  takes the majority of the 30 frames in the hold (all 120 frames of those runs read right). Until then the left arm
  waits at a standby pose, and it fetches nothing if nothing was read. yam_scripted plays `demo()` open loop, so
  `demo()` wraps the policy's `act()` to hand each observation to the reader; it uses nothing else from the simulator.
  The rest of the plan (where the mirror, the cartridges and the pad are) is the known layout, all in plain view of the
  top camera. The environment variable `MIRROR_FAULT_CODE` (1 to 4, default 3) changes the displayed code; it is only
  for checking the demo, never set in the official run.
- **Parts written side by side.** `parts.py` writes each part's USD file in a folder of the process's own. Before, two
  runs of this task started together on one machine used the same file paths, and one of them crashed twice; with a
  folder per process, runs side by side finished.

## Runtime and seeds
- Per trial on one L40S (inspect-robots 1.0): the scripted demo reads 3 in the mirror (30 of 30 frames) and reaches 1.0
  at step 581 (29 s of simulated time, about 2.5 min of wall clock). Dream-1 used all 2,400 steps and scored 0.0: its
  right arm reached to the mirror without moving it, and its left arm swept into the cartridge row and knocked several
  cartridges out of place.
- The demo reads the code, it does not know it. With the displayed code set to 1, 2, 3 and 4 (`MIRROR_FAULT_CODE`, test
  runs only), it read 1, 2, 3 and 4, each in 30 of 30 frames, and scored 1.0 each, at steps 581, 580, 581 and 580.
  Reading the code and then fetching cartridge 1 instead: 0.
- The demo with its first move 0.05 s and 0.10 s late: 1.0 and 1.0 (read 3; steps 582 and 583).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- `test_mirror_fault_code.py` checks the scoring, layout, reader and demo without a GPU (`python -m pytest -q
  test_mirror_fault_code.py`, 47 tests): the start, every credit level, the 0.9 cap, a wrong cartridge moved, bumped or
  put on the pad, the pump moved, the mirror's zone, flatness, facing and resting limits either side, the pad's limits,
  motion and spin, the hood's sight-line angle and clearances, the reader on each reference (also moved, and with its
  box 1 px bigger) and its refusals, the demo waiting for the read and fetching the cartridge read, and the demo's reach
  for every code. A mutation check (30 deliberate breaks of the scorer, the reader and the demo's hand-off, each run
  against the tests) catches 30 of 30.
