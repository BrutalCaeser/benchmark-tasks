# Sort machined blocks with a go/no-go gap gauge

Instruction given to the policy:
`Check each of the seven aluminium blocks with the gauge: set it on the green GO gap and, only if it drops in, then set it on the red NO-GO gap. A block that drops into GO but not into NO-GO is in tolerance: put it in the green tray. Put every other block in the red tray.`

Files: `task.json`, `gauge_sort.py` (scene, layouts, gauge checks, success, partial credit and the scripted demo),
`check_demo.py` (offline checks), `README.md`.

## Origin

This task builds on pull request #1849, whose hold lapsed without a revision. Its files are used here with the
changes listed under Claimed revision.

## Claimed revision

| Reviewer point (latest review) | What changed |
|---|---|
| The documented scorer still accepts poses within 0.5 mm of a gauge surface without establishing contact. | The 0.5 mm tolerance no longer decides support; it only names which surface (base or anvil tops) a supported block rests on. Support is now established by elimination for every step of the 0.3 s check: no gripper's tool centre point within 60 mm of the block, open or closed (out of reach of any finger); no other block within 5 mm; and the block still (under 1 cm/s and 0.5 rad/s each step, and moved at most 0.3 mm and turned at most 1 degree over the whole 0.3 s). A block that touches nothing but the gauge and does not move is held up by the gauge; unsupported, it would fall 12 mm in one control step. (`released()`, `clear_of_blocks()`, `still_since()` and `_update()` in `gauge_sort.py`.) |
| Update the scorer to verify gauge support. | As above. The earlier "open within 50 mm" release rule is replaced by "out of reach within 60 mm", since an open gripper next to a block could still prop or press it. |
| Test unsupported poses within that tolerance. | `check_demo.py` now puts blocks 0.4 mm off the GO anvils, off the base in GO, and off a wedged pose, and checks that each gets no reading when an open gripper is 20 mm above it, another block lies 1 mm from it, it is falling, or it is lowered slowly through the band (1.6 mm/s, under the rest limit); the first, second and fourth of these scored under the previous rule. The same poses held still, with nothing else in reach, are credited. Direct `gauge_reading()` cases cover an open gripper and a touching block; `still_since()` is checked on settling, drift and turning. |
| Rerun or rescore the full demo to confirm 1.0 within the action limit. | Rerun on this revision: 1.0, success after 3,970 of 6,000 steps (see Submitted runs). The demo plan is unchanged: after each let-go it backs off to 140 mm above the table, about 92 mm above the block, and holds there for 0.6 s (12 steps), so the new 60 mm reach rule leaves it at least the 6-step window by construction. |

## Environment

The program's Brev launchable (Isaac Lab-Arena, Isaac Sim, yam-arena), with the launchable's `yam.env` sourced
in every shell (`source <ARENA_DIR>/yam.env`). Versions used for the submitted runs:

- inspect-robots: 1.0.0
- yam-arena commit: 0059d02a1a91
- task file SHA-256, first 12 hex: see `eval.environment_revision` in the logs (it changes with every edit to
  `gauge_sort.py`)

## Layouts

The seven blocks are shuffled among seven fixed start places, so where a block starts says nothing about its
width. The layout number comes from the environment variable `GAUGE_LAYOUT_SEED` (a whole number, default 0),
read when the task file loads. It is not an `-E` option: the `yam_arena` embodiment accepts only its own `-E`
options, and it does not pass the run's `--seed` to task files. The task file is loaded, and the scene built,
before any seed is known. The layout number is printed at the end of the task description the embodiment
reports (`Layout number N.`).

Rule for any layout number N: order the place indices 0 to 6 (the columns below, left to right) by the SHA-256
hex digest of `gauge-layout:N:<index>`, and give them to blocks 1 to 7 in that order (`layout()` in
`gauge_sort.py`). Every place keeps x >= 0.16, and each arm's row keeps its four or three places. Block
widths: 1 30.5 mm, 2 34.5, 3 28.5, 4 25.5, 5 35.0, 6 29.5, 7 25.0 (1, 3 and 6 are in tolerance).

| Layout | x 0.16, y +0.30 | x 0.25, y +0.30 | x 0.34, y +0.30 | x 0.43, y +0.30 | x 0.16, y -0.30 | x 0.25, y -0.30 | x 0.34, y -0.30 |
|---|---|---|---|---|---|---|---|
| 0 | 7 | 5 | 2 | 1 | 4 | 6 | 3 |
| 1 | 5 | 3 | 6 | 1 | 4 | 7 | 2 |
| 2 | 3 | 1 | 7 | 5 | 6 | 4 | 2 |
| 3 | 7 | 1 | 3 | 4 | 6 | 5 | 2 |
| 4 | 5 | 3 | 4 | 6 | 2 | 1 | 7 |
| 5 | 2 | 5 | 7 | 1 | 6 | 4 | 3 |
| 6 | 5 | 3 | 7 | 4 | 2 | 1 | 6 |
| 7 | 3 | 4 | 1 | 6 | 2 | 7 | 5 |
| 8 | 3 | 4 | 7 | 2 | 6 | 1 | 5 |
| 9 | 3 | 6 | 2 | 1 | 7 | 5 | 4 |

The screenshot and the submitted runs use layout 3. For real-world trials, draw a layout number per trial (say,
a ten-sided die for 0 to 9) and set the blocks out by the numbers on their undersides.

## Scoring

A block earns credit only if it was checked with the gauge, in order, and ends in the right tray.

- **A valid check** of a block at a gap: for at least 0.3 s (6 control steps in a row at 20 Hz) the block is
  **supported by the gauge** (below), centred over the gap (within 15 mm), and either **seated** in the gap or
  **staying out** on that gap's anvils.
  - Seated: lying level (tilted 2 degrees or less) on the gauge base between the anvils, its lowest point
    within 0.5 mm of the base.
  - Stays out: clear of the base by 3 mm or more, and either lying level across both anvils (its lowest point
    within 0.5 mm of the anvil tops, its bottom spanning the gap) or tilted and touching both anvils within
    0.5 mm (wedged). Its highest point is no more than 3 mm above a block lying flat on them.
  - The reading must also be one the block's width allows: only a block narrower than the gap can seat, and
    only a wider one can rest on top. A narrow block held up on top of a gap, or partway down it, is no
    reading.
  - **Supported, not merely near.** The 0.5 mm tolerance above only says which surface the block rests on. Support
    itself is shown by elimination, at every step of the 0.3 s:
    - Released and out of reach: no gripper's tool centre point (between the finger pads) is within 60 mm of the
      block's bounding box, open or closed. A fully open finger pad reaches about 53 mm to the side of the tool
      centre point and 13 mm below it, so nothing of either gripper can hold, prop or press the block.
    - No other block within 5 mm of it.
    - Still: under 1 cm/s and 0.5 rad/s at every step, and over the whole 0.3 s moved at most 0.3 mm and turned
      at most 1 degree.
    A block that touches nothing but the gauge and does not move is held up by the gauge: a free block would
    fall 12 mm in one 50 ms control step. So a block hovering, falling or being lowered within 0.5 mm of a
    gauge surface, or held or propped there by a gripper (open or closed) or by another block, gives no
    reading, and so does a block kept still 1 mm or more above the anvils or the base. (Blocks that were let
    go lay within 0.02 mm of the base or the anvil tops in the simulator logs.)
  - How the simulator sees the gripper: the task file's hooks get only the objects' states, so the scorer
    reads both arms' finger joints and gripper link poses from the physics engine each step
    (`grip_reader()` in `gauge_sort.py`, through Isaac Lab's running `SimulationContext`). If that reading is
    unavailable (no simulator, a missing joint or link, any error), no check counts that step, so no credit,
    and the run prints a one-time warning. In a real trial the judge checks on the video that the gripper is
    out of reach and the block still (below). With `GAUGE_POSE_LOG` set, the pose log records each arm's pad
    gap and its distance to each block, and whether other blocks are clear, next to every check.
- **Order:** every block needs a valid GO check. A block seated in GO then needs a valid NO-GO check after it.
  A NO-GO reading before the block has passed GO doesn't count. A block that stays out of GO needs no NO-GO
  check. The first valid reading at each gap stands. The readings then call for a tray: stays out of GO, red;
  seated in GO and in NO-GO, red; seated in GO and stays out of NO-GO, green.
- **In a tray:** the block's whole footprint is inside the tray's walls (inside 80 x 220 mm), its bottom is on
  the tray floor or on top of another block (within 3 mm), and it is still (under 1 cm/s and 0.5 rad/s). A
  block held above the floor, lying on or across a rim, or anywhere else counts for nothing.
- **Score:** C counts blocks with valid checks, in order, that end in the right tray. F counts in-tolerance
  blocks in the red tray (false rejects) and E out-of-tolerance blocks in the green tray (escapes), checked or
  not. Score = (C - 2F - 2E) / 7, at least 0, and at most 0.25 if E >= 1.
- **Success:** all seven blocks credited, held for 1 s. `task_score` is 1.0 on success, otherwise `score()` at
  the end of the episode.

Sorting every block right without the checks scores 0. Every block in one tray scores 0. Checking every block
and then sorting at random scores about 0.052 on average.

**State between trials.** A task file gets no reset call, so the checks clear when `score()` sees a trial's
start: every block lying still on the table within 35 mm of its start place. `success()` and `score()` share
one update per control step.

**Real-world trials.** A camera records the gauge from above for the whole trial. A judge reviews the video
for each block. A check counts when the block is seen seated in the gap, or resting on that gap's anvils
without reaching the base, for at least 0.3 s with both grippers withdrawn (at least about 6 cm away, a hand's
width), no other block touching it, and the block not moving. That is the real-world form of the simulator's
support test: an untouched block that stays put is resting on the gauge. The judge also checks that the NO-GO
check, where needed, comes after the GO check. Two still photos per check, 0.3 s or more apart, showing the
block in the same place in or on the gap with the grippers withdrawn, can replace the video. The end state is read after the arms stop and open,
with a top photo and a tape.

## Offline check, no GPU

From the root of this repository:

```bash
python tasks/go-no-go-gauge-sort/check_demo.py     # about 4.5 min
```

- **Layouts 0 to 199:** every one is a permutation of the seven places. Every block starts at x >= 0.16, in
  its own arm's row, with room for the open fingers and at least 20 mm clear of the gauge and trays. Every
  block visits all seven places.
- **Scoring, on hand-built sequences of states, in layouts 0 and 3:**
  - Every block checked and sorted: 1.0 and success.
  - Sorted right without any check: 0. One block unchecked: 6/7.
  - Checks one step short of 0.3 s count for nothing.
  - A GO passer with no NO-GO check, or with NO-GO only before GO, gets no credit.
  - A narrow block held on top of GO gets no credit.
  - Held, never let go: a block kept still 1.5 to 2 mm above the GO anvils, above the base in GO or NO-GO, or
    tilted as if wedged but 1.5 mm clear of both anvils, for 0.9 s, then sorted right, gets no credit (6/7
    with the rest checked); the same block really wedged is credited. Every block hovered over its gaps and
    then sorted right scores 0.
  - Released: a block lying exactly as a let-go block would (seated in GO, then NO-GO; on the GO anvils) with a
    closed gripper on it, for 0.9 s, gets no credit; nor with that gripper open (wide, or 55 mm) but still at
    the block. Two arms: one gripper closed on the block while the other is open and far, either way round:
    no credit; both open 20 mm above it: no credit. A gripper, closed or wide open, 62 mm from the block is
    ignored; at 58 mm it is not.
  - Support within the 0.5 mm tolerance: a block 0.4 mm off the GO anvils, off the base in GO, or off a wedged
    pose gets no credit with an open gripper 20 mm above it, another block 1 mm from it, falling, or lowered
    through the band at 1.6 mm/s for 0.3 s; held still with nothing else in reach, the same block is credited
    (1.0 with the rest).
  - Fail closed: with the gripper reading unavailable (None, or a reader that raises), every block checked
    and sorted scores 0, and the warning prints once.
  - Checks clear at the next trial's start.
  - These don't count as in a tray: a block held 10 cm or 5 mm above the floor, on the near or side rim,
    straddling a wall, or still moving. A block one layer up with nothing beneath doesn't count either; a
    second layer on a block does.
  - Every block in one tray: 0, checked or not.
  - One escape: 0.25; two: 1/7. One false reject: 4/7. Only the in-tolerance blocks sorted: 3/7; only the
    others: 4/7.
  - Checked, then sorted at random over all 2^7 ways: expected score 0.052.
- **The gauge reading:** checked on blocks seated in a gap, on the anvils, jammed tilted (two wedge angles),
  held, partway down and over the other gap; each reading must follow the block's width. No reading for a
  block held still 1 to 2.9 mm above the anvils or the base (flat), or 1 to 2.9 mm clear of a wedged pose;
  for a block moving while on the gauge; for one level on the anvils but not spanning the gap; or one tipped
  5 degrees on the base. No reading for a seated or resting block with a gripper closed on it, an open
  gripper 20 mm above it, another block 1 mm from it, or with no gripper reading; a block 20 cm away does not
  matter. The drift test passes microns of settling and fails 0.4 mm of drift or a 2 degree turn. The real
  reader, run with no simulator, returns no reading.
- **The demo, replayed offline:** through `yam_arena.demo.ArmPlan` at 20 Hz, with `score()` running every step
  as in the simulator, but with no physics (the grippers' pad gaps and TCPs come from the plan's finger
  commands and forward kinematics): a block moves with the TCP while gripped and drops straight down
  when let go (over a gap, shifted by the release slide that `RELEASE_DY` corrects), and the scenario decides
  whether it seats. This checks the plan's logic, not that it works in the
  simulator; the demo has been run in the simulator on layout 3 only (see Submitted runs).
  - Assumed outcomes (a block drops into a gap exactly when it is narrower) for layouts 0 to 9. In each, the
    scorer registers every block's GO check, a NO-GO check exactly for the GO passers, and a final 1.0.
  - Every block staying out of GO, every block dropping through both gaps, and every block passing, on layout
    3. Between them, these take each arm through every branch.
  - 4,005 steps (200 s), with a 0.5 s hold with the fingers shut before every let-go over a gap.
  - Worst IK error 0.095 mm / 0.015 deg, every joint inside the URDF limits, and the arms' link-3/4/5,
    gripper and TCP frames never closer than 164 mm.

## Run and score one Dream-1 trial

```bash
GAUGE_LAYOUT_SEED=3 inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/go-no-go-gauge-sort/gauge_sort.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Check each of the seven aluminium blocks with the gauge: set it on the green GO gap and, only if it drops in, then set it on the red NO-GO gap. A block that drops into GO but not into NO-GO is in tolerance: put it in the green tray. Put every other block in the red tray." \
  --scorer task_score --max-steps 6000 --seed 0 --no-prompt --store-frames --log-dir logs/gauge-dream
```

`-E object_jitter=0` is required: the blocks are 40 mm apart along x, and the default 3 cm jitter can overlap
them. Scoring is automatic (see Scoring). `-E control_hz=20` matters: a check is 6 control steps, which is
0.3 s at 20 Hz. The CLI prints `task_score: <value>`; the log keeps it at `results.metrics.task_score`.

## Run and score the full-score demo

```bash
GAUGE_LAYOUT_SEED=3 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/go-no-go-gauge-sort/gauge_sort.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Check each of the seven aluminium blocks with the gauge: set it on the green GO gap and, only if it drops in, then set it on the red NO-GO gap. A block that drops into GO but not into NO-GO is in tolerance: put it in the green tray. Put every other block in the red tray." \
  --scorer task_score --max-steps 6000 --seed 0 --no-prompt --store-frames --log-dir logs/gauge-demo
```

The demo is `demo()`, a scripted (not learned) policy run through `yam_scripted`. It gauges every block and
sorts it by what the gauge did. It never reads the widths, but it does read the simulator's block poses (see
Looking). It takes its pick-up places from the layout in use. For each block:

1. One arm picks it up, sets it square over the GO gap, holds still for 0.5 s, lets go, backs off and holds
   clear for the check (0.6 s).
2. If the block is seated in GO, the arm grips it by its ends, lifts it out and checks it over the NO-GO gap
   the same way. If it stayed out of GO, it goes straight to the red tray, with no NO-GO check.
3. After a NO-GO check, a block seated in NO-GO goes to the red tray, and one that stayed out goes to the
   green tray.

At each gap, the block sits untouched from the let-go until the arm reaches for it again: several seconds,
well over the 0.3 s a check needs.

- **Looking:** the `yam_scripted` policy sees only images and joint angles, so the demo reads the simulator's
  pose of the block through the task file's `score()` hook, which runs after every control step. Each waypoint
  after a look works out its target when the arm reaches it.
- **Grip:**
  - Straight down, turned 90 degrees, so the fingers close along x on the block's 50 mm ends. The anvils are
    only 10 mm long along x, so the fingers stand clear of them and reach down beside a block that has
    dropped into a gap. A grip across the width would not fit, with 0.75 to 4 mm between the block and each
    anvil.
  - Each block is gripped with the pad tips 1 mm above its bottom.
  - The fingers approach and descend open 88 mm (19 mm clear of each end) and release at 68 mm, which fits
    inside the 80 mm trays.
- **Sharing the gauge:** the arms take turns, one block at a time, alternating left and right from each row's
  nearest block outward. The right arm parks after its third block. An arm holds its next block above its
  starting place until the other arm has finished its delivery. Each arm uses its own half of each tray; a
  third block in the same half goes on top of the first.
- **Settling and the release correction:** in the simulator the shut fingers hold each block off-centre along
  x, about 23 mm from the TCP. As the fingers open over a gap, the block slides those 23 mm back under the TCP
  before it drops, and its position across the gap (y) is set during that slide. In the layout-3 runs the
  block was released -0.8 to +1.4 mm off the gap's centre for the left arm and -1.3 to -2.6 mm off for the
  right. The in-tolerance blocks have only 1.25 to 2.25 mm a side in GO; at -2.6 mm, block 6 (29.5 mm, 1.75 mm
  a side) caught an anvil edge, rolled 28 degrees and wedged without seating, and the demo put it in the red
  tray. So each arm lets go at the gap's centre plus a fixed per-arm correction, the negative of its mean
  offset in those runs: -0.17 mm for the left arm, +1.80 mm for the right (`RELEASE_DY` in `gauge_sort.py`,
  the same for every block and both gaps). The right arm's correction comes from its three GO releases only;
  it made no NO-GO release in those runs. The arm still holds still for 0.5 s, fingers shut, before each
  let-go, so its own motion has stopped; that hold does not centre the block.
- About 4,005 of the 6,000 steps; the episode ends 1 s after the last block comes to rest in its tray.

## Run limit, runtime and seeds

- Run limit: 6,000 control steps at 20 Hz (5 minutes). The demo gauges all seven blocks one at a time in about
  4,005 steps.
- Seeds: `GAUGE_LAYOUT_SEED=3` (the layout), `--seed 0`, `-E object_jitter=0`. The demo is deterministic;
  Dream-1 is not seeded.

## Submitted runs

Layout 3, `object_jitter=0`, `--seed 0`, 20 Hz, scored by `task_score`:

Runs of this revision (gauge_sort.py SHA-256 prefix 8d3a0117129d), logs in `runs/`:

- Scripted demo: **1.0**, success after 3,970 of 6,000 steps (`runs/demo_log.json`).
- Dream-1: **0.0** over all 6,000 steps (`runs/dream_log.json`, trimmed: its per-step records are left out for size).
- `check_demo.py` (all offline checks, including the new support cases): passes.

The submitted videos are from these two runs (the demo video shows the top camera enlarged beside both wrist cameras).

- Before this revision, under the previous rule (open grippers within 50 mm allowed, no drift or block check),
  the scripted demo scored 1.0, success after 3,970 steps, and Dream-1 scored 0.0 over all 6,000 steps.
- The new rule only adds conditions to a valid check, so it cannot raise the Dream-1 score. The demo plan is
  unchanged, and its 0.6 s hold about 92 mm above each block after every let-go gives the 6-step window the
  new rule needs.

## Reset between trials

Nothing is consumed. Pick the next trial's layout number and place each block, by the number on its underside,
in that layout's start place, lying along x. Start a new overhead video for the trial's checks.
