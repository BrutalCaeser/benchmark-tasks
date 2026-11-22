# Pry a nailed 1x4 off two joists with a flat bar

This task builds on an earlier submission, whose 48-hour revision hold lapsed without a revision. This version answers that submission's review (TK-JBZFMC):

| Review point | What changed |
|---|---|
| "specify the make and model of pry bar" | The bar is now named: a DEWALT DWHT55160 15 in (380 mm) Spring Steel Flat Bar, with the maker's listing linked and the dimensions it states compared with the simulated bar (see "The pry bar for a real trial" below, and `task.json` setup). The simulated bar was widened from 32 to 35.6 mm to match the listed width (a one-line change in `layout.py`; the demo needed no retuning and still scores 1.0). It matches the listed length and width; its mass, computed from our estimated 6 mm thickness, is 0.63 kg against the listed 0.59 kg (7% heavier), and its far end is drawn as a curled claw (that end is never used). |

Instruction given to the policy: `Pry the board off the joists with the pry bar and lay it flat on the table.`

A 600 mm length of 1x4 pine is nailed across two 2x4 joists standing on edge (a piece of floor or deck framing, its
joists ending in a rim joist clamped to the table), one 3d box nail into each: a temporary cleat of the kind framers
nail across joists to hold their spacing until the subfloor goes on. A 15 in
flat pry bar lies on the table in front. The robot works the bar's blade into the joint where the board's eased
bottom edge meets a joist, rests the bar's bend on that joist and pushes the handle down: the bar rocks on the joist
and lifts the board's edge, pulling that nail. It does the same at the other joist, then lifts the loose board off
and lays it flat on the table.

## Why it matters

Stripping nailed lumber off framing with a flat bar (braces, cleats, form boards, blocking, salvage) is routine work
for construction laborers (about 1.1M US workers) and carpenters (about 0.7M). O*NET task statements:
construction laborers (47-2061.00) "Erect or dismantle scaffolding, shoring, braces, traffic barricades, ramps, or
other temporary structures" and "Raze buildings or salvage useful materials"; carpenters (47-2031.00) "Remove damaged
or defective parts or sections of structures and repair or replace, using hand tools".

## Why it is hard

1. **Starting the blade without a hammer.** The board sits tight on the joist; the only way in is the small V its
   eased bottom edge leaves (3 mm). The 1.6 mm edge of the blade has to go in there, the blade flat on the joist.
2. **Using the joist as the fulcrum, with real force.** Each nail holds about 150 N. The bar's bend (the heel) rests
   on the joist and the handle is pushed down: about 3.6:1 leverage where the demo holds the bar, so roughly 40 N at
   the hand, through a parallel-jaw grip on a flat bar that can turn in the fingers.
3. **Re-setting the blade.** With only the edge in the V, the blade slips out as the board rises. The first stroke
   lifts the board a few millimetres; the blade then has to go 2 to 3 cm under the board before the nail can be
   levered out.
4. **Two nails, one bar, then a loose board.** The other end needs the same work (the demo hands the bar to the
   other arm). Once both nails are out the board rests loose on the bar's blade: drawing the bar out drags it off the
   joists, so the demo's free hand lifts the board off first while the other keeps the bar still.

## Files

| File | What |
|---|---|
| `task.py` | The task file: the objects, the nails' rule (`_nails`), the scorer hooks, the demo entry point |
| `layout.py` | Every dimension, mass and threshold |
| `scoring.py` | The scorer and the nails' withdrawal model, pure Python |
| `demo.py` | The scripted demo (closed loop on the bar's and the board's poses) |
| `tests/test_scoring.py` | Offline tests (32), no simulator |
| `setup.png` | The setup image (top camera, arms at their ready pose) |
| `mechkit.py`, `rigstate.py`, `demokit.py` | Shared helpers, copied (see below) |

## The scene

| Object | What |
|---|---|
| Frame (fixed) | A piece of floor or deck framing: two kiln-dried SPF 2x4 joists (38.1 x 88.9 mm), 612 mm long, on edge along x from 25 to 86 cm out, 16 in (406.4 mm) apart on centre either side of the middle, and a 2x4 rim joist on edge across their back ends (444.5 mm, flush with their outer faces, face-nailed with two 16d nails at each end) |
| C-clamps | Two 6 in C-clamps hold the rim to the table's back edge, 12 cm either side of the middle (pad on the rim's top edge, fixed jaw under the table top). Nothing is screwed into the robot's table; while the bar levers, a joist is pushed down overall (the heel presses harder than the nail pulls), so the clamps only keep the frame in place |
| Board (nailed) | 600 mm of 1x4 common pine (19.05 x 88.9 mm, S4S, eased edges), 0.46 kg, flat across the joists' tops, centred 40 cm out, ends overhanging 78 mm |
| Nails | Two 3d box nails (1-1/4 in, 0.076 in shank, 7/32 in head, bright), one into each joist through the board's centre line, heads flush; 12.7 mm in the joist. The cleat is tacked lightly, as temporary cleats often are (about 150 N to pull a nail). Heavier nailing would be beyond what the arm can lever; the skill (seating the blade, levering on the joist, re-setting deeper) is the same |
| Pry bar (loose) | 15 in flat pry bar, 35.6 x 6 mm spring steel, 0.63 kg; flat chisel blade 48 mm long bent 15 deg, ground on its top face to a 1.6 mm edge; curved nail claw at the other end. For a real trial: a DEWALT DWHT55160 15 in spring steel flat bar (see below) |

Nothing else is on the table. Coordinates for every object are in `task.json` (setup) and `layout.py`.

## The pry bar for a real trial

Use a **DEWALT DWHT55160, 15 in (380 mm) Spring Steel Flat Bar**:
https://www.dewalt.com/product/dwht55160/15-380-mm-spring-steel-flat-bar (DEWALT's own product page; returned HTTP 200
to curl on 2026-11-21). The page shows a yellow, powder-coated spring steel flat bar with a bevelled chisel end
carrying a nail slot at one end and a wider, sharply bent prying end at the other, and lists these specifications:

| | DWHT55160 (as listed) | Simulated bar |
|---|---|---|
| Length | 15.0 in (381 mm) | 380 mm |
| Width | 1.4 in (35.6 mm) | 35.6 mm |
| Weight | 1.3 lb (0.59 kg) | 0.63 kg (computed from the shape and our estimated thickness) |
| Material, finish | spring steel, powder coated | spring steel, blue powder coat |
| Thickness, blade bend | not listed | 6 mm, 15 deg (our estimates) |

The bar's working end in this task is the chisel end: its blade goes into the joint and its bend rests on the joist.
Length and width agree. The simulated mass, 0.63 kg, is 7% over the listed 1.3 lb: it follows from our 6 mm
thickness estimate, so the real bar is probably somewhat thinner. Its far end is a wide bent prying end rather than
the drawn curled claw; that end is never used here, as the demo holds the bar by its shaft only. The thickness and
blade bend are not on the listing; ours are estimates of a typical 15 in flat bar.

## Scoring (scoring.py)

At the end of the trial, with a ruler, a photo and the trial video:

- **1.0**: both nails out of the joists and the board lying flat on the table (face within 20 deg of horizontal,
  lowest point within 10 mm of the table, highest within 104 mm: not propped on a joist or the bar), both grippers at
  least 3 cm away, the board still;
- **0.75**: both nails out, but the board not laid flat on the table, or still held;
- **0.5**: both nails loosened (the board's underside at least 6 mm off the joist at each nail);
- **0.25**: one nail loosened;
- **0** otherwise. A board split along its length scores at most 0.75 (cannot happen in the simulation).

In the simulator "out" is the rule's own record that a nail has slid 12.7 mm (it releases the board then); in a real
trial it is a board that lifts away freely. The simulator credits each millimetre a nail slides to the bar when, at
that step, the bar's blade is beside that nail's joist (within 6 cm of its centre line), its edge under the board's
footprint and no more than 25 mm over the joists' tops, with the heel on the robot's side of it
(`scoring.bar_at_nail`); this is reported, not scored.

**Why the bar is needed.** A fresh nail holds about 150 N. A parallel gripper pinching the planed board holds roughly
2 x 50 N x 0.5, about 50 N, before it slips, and the arm cannot lift 150 N, so a gripper cannot pull a nail. Nor can
the board be levered on a joist while both nails hold: popping the near nail by pressing an overhanging end down
would take on the order of 800 N against the far nail. So the first nail needs the bar. Once one end is free, working
the board up by that end pivots it on the other joist and can draw the second nail: a normal technique on site, and
it counts. (In the simulation the board's end-to-end tip is limited to 0.12 rad while nailed, so that second route is
not available there; the demo uses the bar on both.)

## The nails (a declared rule, `task.py` `_nails`)

Simulators cannot model a nail's grip in wood, so a rule stands in for it, and the robot still has to apply the
real lift at the nail to trigger it. The board is glued (a PhysX fixed joint) to a hidden carrier inside the frame
that can only rise straight up and tip end to end (about a line across the board through its middle, on the joists'
tops), so one end can rise while the other is still nailed. The carrier's two
springs are the two nails: each is stiff (75 kN/m, 2 mm of give at full load) until the board pulls on it harder
than its withdrawal force; then the rule lets it slide and holds it where it was left. The withdrawal force is
150 N with the whole 12.7 mm in, from the Wood Handbook's formula for a smooth nail freshly driven into side grain,
p = 54.12 G^2.5 D L (N, mm), with SPF's G = 0.42 and D = 1.93 mm, and it falls in proportion to the length still in.
At 12.7 mm a nail is out; with both out, the glue is released and the board is loose. The rule never moves the
board: only contacts do.

## Run it

From this folder, with the benchmark's environment (`source ~/IsaacLab-Arena/yam.env`):

    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
        --scorer task_score ...

The layout is fixed (`object_jitter=0`); there are no seeds to set. The scorer reads the end state (`SCORE_OVER =
"end"`, 1 s hold).

The scripted demo:

    python demokit.py task.py <out_dir>                   # final: records <out_dir>/full/demo.mp4 and setup.png
    python demokit.py task.py <out_dir> --fast            # physics on the CPU, no cameras: about 6 times faster

It reaches 1.0 in 2,024 steps (101 s of robot time) on the GPU. A trial with cameras takes about 5 minutes of wall
time on one RTX 4080 (16 GB); the demo's other modes are `idle` (the setup image) and `right` (the first nail only).

Offline tests (no simulator; `yam-isaac-lab-arena` on `PYTHONPATH` for `yam_arena.task_file`):

    python3 -m unittest discover -s tests -v

The Dream-1 run:

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Pry the board off the joists with the pry bar and lay it flat on the table." \
      --scorer task_score --max-steps 4800 --seed 0 --no-prompt --store-frames --log-dir <out_dir>

## Demo (demo.py)

Closed loop: every stage reads the bar's and the board's poses; it never reads the scorer or the nails' rule, and
nothing is teleported or edited.

1. The right arm picks the bar up by its shaft at its centre of mass (fingers across its 35.6 mm width), turns it
   blade forward and lays the blade flat on the right joist, its edge in the board's eased-edge V.
2. It levers in small commanded steps until the board is seen 4 mm up (the arm is soft, so the command runs well
   past the bar's angle), re-sets the blade about 2 cm under the raised board, and levers again until the board
   stands 14 mm off the joist: the nail is out.
3. It eases the handle back up so the board settles, draws the blade out, and sets the bar down in the middle; the
   left arm picks it up and does the same at the left joist.
4. With the board now loose, the left hand keeps the bar where it is (drawing it out from under a loose board would
   drag the board) while the right hand grips the board inside the bay, lifts it straight up off the joists and the
   blade, carries it toward the robot and lays it flat on the table, clear of the bar's handle, and lets go. Then the
   left arm draws the bar out and puts it down on the table outside the left joist.

The demo holds the bar only by its painted shaft: the grippers' pads never touch the ground blade edge or the claw.
The recorded demo ends when the success condition has held for 1 s (the harness then stops the episode): the board
lies flat on the table, still, with both grippers at least 3 cm from it. At that moment the left hand is still holding
the bar, clear of the board; in a real trial it would then put the bar down.

## Results

| Run | Score | Steps |
|---|---|---|
| Demo, GPU, recorded | 1.0 (right nail out at step 676, left at 1680) | 2,024 |
| Dream-1 (`--policy dream`, seed 0, full limit) | 0.0: it moved and turned the bar on the table with its hands but never seated the blade under the board or levered on a joist; neither nail moved and the board stayed nailed down | 4,800 |

## Third-party code and assets

No meshes, scans or other assets: every object is built from boxes and cylinders here. `mechkit.py`, `rigstate.py`
and `demokit.py` are shared helpers copied from accepted tasks in this repository
(`tasks/shim-sleeper-to-level/mechkit.py`, `tasks/casement-stay-last-hole/rigstate.py`,
`tasks/alaris-set-load/demokit.py`, the last under the MIT licence as that task states), unchanged except that a few angle
variables in `demokit.py` were renamed. The O*NET task statements
quoted above are from O*NET OnLine (USDOL/ETA), used under CC BY 4.0. The withdrawal formula is from the USDA Forest
Products Laboratory's Wood Handbook (FPL-GTR-282, chapter 8), a US government work.
