# Saw a clamped wooden strip to length

A robot closes a screw vise on a loose wooden strip, picks up a hand saw and
cuts through the strip at a measured location. Full credit requires a separate
offcut and a retained piece measuring 150 ± 1 mm. This is a regular carpentry
operation: the [O*NET carpenter task list](https://www.onetonline.org/link/details/47-2031.00)
names measuring and marking cut lines and cutting materials to specified
measurements with hand tools as core tasks.

The scene has a 300 × 20 × 20 mm clear, straight-grained, kiln-dried Scots
pine (Pinus sylvestris) strip, 80 mm vise jaws, a gents saw with a 200 mm blade
and a round, turned handle, and a 300 mm steel rule. The strip rests partly in
the open vise and partly on a low support. A pencil line crosses its top 150 mm
from the left end. The saw starts in a rack. The robot must turn the screw to
close the moving jaw, then hold the saw by its handle and reciprocate it at the
desired line. The saw has no guide.

The real-trial saw is the [Pax 8 in (200 mm) gents saw by Thomas Flinn](https://taytools.com/products/thomas-flinn-pax-8-gent-dovetail-saw)
(SKU PG8): a straight, turned walnut handle with a brass ferrule, a brass back,
a 0.5–0.56 mm plate with 41–42 mm cutting depth under the back, and 20 TPI
(1.25–1.3 mm pitch) teeth used as supplied, cutting on the push stroke. The
retailers checked give its kerf as 0.7–0.9 mm
([fine-tools](https://www.fine-tools.com/Pax-Gents-Saw-Blade-8-inch-200-mm-FLINN/303009),
[Carbatec](https://carbatec.com.au/products/pax-8-gents-saw)). The simulated
saw matches its form and scale: a 26 mm diameter, 110 mm round handle with a
30 mm ferrule, and a 200 × 40 × 0.6 mm plate with a 1 mm tooth strip. The brass
back is not modelled; it sits 41–42 mm above the teeth, so it stays above the
wood's top face throughout a 20 mm deep cut. A
[75 mm commercial mechanics vise](https://www.faithfulltools.com/p/FAIMV75SG/Mechanics-Vice-75mm-%283in%29-with-Magnetic-Jaws)
is the vise's scale and form reference. The wood and saw are trial protocol
choices, not calibrated cutting data. The simulated geometry is procedural
rather than a replica of either product.

## Measuring device

A [Shinwa 300 mm flush-ended stainless steel rule](https://www.dictum.com/en/shinwa-precision-ruler-with-flush-starts-300-mm/730817)
(300 × 30 × 1.0 mm, 0.5 mm graduations on the front, scales starting flush at
both ends, also sold by [Workshop Heaven](https://www.workshopheaven.com/shinwa-flush-ended-stainless-steel-rule-300mm))
lies loose and flat along x on a fixed 100 × 16 × 20 mm wooden riser, beside the
saw rack, in the area where the left arm already picks up the saw. Reach to
the rule has not yet been checked in simulation. Its ends overhang the riser by
100 mm and its long edges by 7 mm, so a gripper can close across its width or
pinch an overhanging end. The simulated rule is a 71 g rigid steel plate with
drawn 10 mm graduations.

To mark or check 150 mm, lift the rule onto the strip's top face along its
length, with a flush zero end level with the strip's left end face (5 mm of the
strip projects past the jaws, so the end face is free). The 150 mm graduation
then lies over the cut line: draw or confirm the pencil line there, square
across the 20 mm width. After the cut, butt the zero end against the retained
piece's left end in the same way and read its sawn end face on the rule. Using
the rule is optional for the robot: the pencil line is already drawn, and the
scorer measures the retained wood itself, never the rule's position.

The scripted demonstration aims the blade centre on the waste side of the
150 mm pencil line. This allows for the 1 mm tooth-strip width and the measured
lateral deflection of the hand-held saw; scoring still measures the actual
retained wood, not the commanded blade position.

## Run

With `yam-isaac-lab-arena` and the Inspect Robots embodiment installed, from
this folder:

```bash
inspect-robots run --no-prompt --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P rows_per_request=4 \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E device=cpu \
  -E cam_width=320 -E cam_height=176 \
  --scorer task_score --max-steps 3200 --seed 3 --store-frames \
  --instruction "Clamp the wooden strip in the vise, then saw it so the retained piece is 150 mm long, within 1 mm. Leave the saw clear of the wood when finished."
```

The scripted robot demonstration uses only the 14-D arm and gripper action
interface to move objects. It reads object poses and the simulated material
grid to align and feed the saw; these reads choose actions only. The task
physics decides material removal and scoring.

```bash
python demo.py --mode full --out demo-out
```

The same runner has `idle`, `grasp` and `clamp` diagnostic modes. It writes
`result.json`, `physics.json`, camera images and videos in the output folder.
Use `--mode partial` for a robot trial stopping at the half-depth stage and
`--check-reset` to verify that a second episode restores intact loose stock.
The runner saves the final material array as well as poses and measurements.

## Validation

Runs of this revision (task.py SHA-256 prefix dccdb9cb42d7, with the visible sawing-while-unclamped veto in `stock.py`), logs in `runs/`:

- Scripted robot run (`python demo.py --mode full`): **1.0 in 2,370 actions**, two physical pieces, the retained piece
  149.25 mm (0.75 mm off, within the 1 mm tolerance), clamp closed and the saw clear (`runs/demo_result.json`).
- Dream-1 (command above, seed 3): **0.0** at the 3,200-step limit (`runs/dream_log.json`, per-step records trimmed).
- `test_*.py`: 18 passed, including the new loose-stroke veto cases.

The submitted videos are from these two runs. The results below were recorded on the earlier revision with a
rectangular grip and are kept as the record of that revision.

On the earlier revision, the final-source scripted robot run reached **1.0 in 2,170 actions** on the
published SDK, with two physical pieces and retained end-face measurements
149.25–149.5 mm. The separate reset restored intact loose stock, removed the
offcut and cleared material/history state. An independent development run
reached 1.0 in 2,210 actions; its partial demonstration reached 0.5 in 1,890
actions while the stock remained connected. Both resets passed. All 21
simulator scoring fixtures and 17 CPU checks passed. Evaluator-placed fixtures
check scoring boundaries and are not robot demonstrations.

Dream-1 beta seed 3 completed all 3,200 actions and scored **0.0**,
with zero errored trials. The command above reproduces its configuration.
Seeds 1 and 2 stopped with an official adapter gripper-range error after 1,364
and 2,644 actions respectively; neither has a valid score. The third attempt
is the first complete trial and was retained regardless of its score. No SDK,
task, action bounds or instruction changed between attempts.

The model video contains the complete 3,200-action trial. The demo video
contains every recorded frame of the 2,170-action trial (108.5 seconds), ending
at the scored state; only the separate post-trial reset check is excluded from
that video. Raw reset evidence is retained separately.

An earlier cloud trial scored zero under an any-fractional-removal veto,
despite correct physical separation and length: an out-of-zone accumulator
reached 0.03263 mm without clearing any 0.5 mm material layer. The current rule
below instead measures 0.5 mm damage. That failed receipt is preserved as a
failure; the full-score result above is a fresh trial on the current source.
The demonstrated controller uses measured poses and material state to choose
14-D robot controls; it does not write object poses or material state.

## Starting heights

The tabletop is 747.5 mm above the floor. Relative to it, the wood bottom and
support top are 80 mm, wood top 100 mm, 25 mm-high jaw centres 82 mm, and screw
axis 62 mm. The saw's round handle is spawned with its axis at 90 mm and
settles onto its wooden handle pedestal with its axis at 88 mm; the tooth edge
is about 53 mm high. The pedestal is 40 × 60 × 75 mm, with two open blade slots
at y = 200 and 80 mm. Their uprights end at 75.5 mm, leaving the saw free to
lift upward; the blade in the slots stops the round handle rolling. The rule's
riser is centred at x 200 mm, y 170 mm; the rule lies on it from x 50 to
350 mm, y 155 to 185 mm, with its top 21 mm above the table.

## Scoring and physical model

The task scores once at the end, taking the highest eligible stage:

| Score | Numeric end condition |
| --- | --- |
| 0.25 | Correctly clamped stock; no off-target removal. Intact stock, a shallow groove, or a groove through only part of the width can earn this stage. |
| 0.5 | One connected groove reaches at least 10 mm below the original top across the entire 20 mm width. At that depth its retained-side wall lies 149–151 mm from the original left end in every width sample. Stock is still one piece. |
| 1.0 | Exactly two physical pieces remain; every sampled width/depth point on the retained end is 149–151 mm from the original left end. |

Every stage also requires all of these conditions: the jaw opening is
19.2–20.2 mm (inclusive), screw rotation is greater than 1.5 rad, the retained
wood origin is less than 3 mm from its starting coordinate on each axis, its
orientation is within 2 degrees of its start, and the entire saw (blade, teeth,
ferrule and handle) is at least 5 mm clear of both wood pieces. The saw may be held
or placed anywhere meeting this clearance; returning it to the rack is optional.
The simulator conservatively measures clearance using box bounds of each saw
part in each wood body's frame; the round handle and ferrule are bounded by the
26 × 110 × 28 mm and 30 × 10 × 32 mm boxes that enclose them. A close diagonal pose can fail conservatively.

Sawing while unclamped permanently gives zero, even if the vise is tightened
later: any stroke in which the loaded teeth travel along the stock and cut it
while the vise does not hold it. This is checked from the uninterrupted trial
video (teeth moving on the wood while the jaws are open or the strip shifts in
them), not by measuring depth. A saw resting on the wood without a stroke is not
sawing. Off-target damage means at least one
cleared 0.5 mm layer outside the 149–154 mm work zone and gives zero. Sub-layer
fractional accumulators alone are not removed physical geometry and cannot
trigger either damage veto or earn cut-depth credit. Repeated contacts at the
same column accumulate; spreading shallow scratches over unrelated columns does
not add their depths together. This 5 mm zone allows kerf width and alignment correction around the pencil
line; it is not a relaxed full-score length tolerance. A completely severed
wrong-length piece scores zero, even if it once had a qualifying deep groove.
An incomplete cut within the work zone but missing the half-depth wall-location,
width or connectivity criterion earns at most 0.25. Separate cuts cannot be
added together to earn 0.5. No eligible stage means zero. A saw left at the wood,
loose/shifted/rotated stock, and cutting elsewhere cannot earn credit.

The 10 mm threshold is half the initial thickness: a substantial, unfinished
cut rather than a scratch. Check it in a real trial using a thin marked depth
probe at 2 mm intervals across the width and at both edges, side photographs,
and a ruler/calipers for the retained-side wall position. The minimum probe
depth must be 10 mm, with a continuous groove between probes; a narrow isolated
hole or cuts covering separate width segments do not qualify. Check clamping
from jaw measurements, a screw index mark, and top/side video throughout the
cut. With the saw withdrawn, measure out-of-zone groove depth with a thin marked
probe or depth gauge against the original top; 0.5 mm is the damage threshold.
Review the top and side video for any sawing stroke made before the vise was
closed or while the strip had shifted out of it; one such stroke gives zero. Check saw clearance with a ruler and top/side images of both pieces.
Full credit still requires physical separation and the actual retained face
measurements, never a stroke count or elapsed time.

Each physics step uses the saw's measured pose relative to the freely moving
wood, its actual motion along the stroke and the measured saw–wood contact
force. The task removes only material under the tooth strip, in 0.25 mm by 2 mm
columns and 0.5 mm height increments. Collision shapes are rebuilt from the
remaining wood whenever a layer is removed. A separate rigid offcut is created when the remaining solid
disconnects across the board's width. The removal law is a reduced-order,
uncalibrated model of sawing; it does not resolve the specified individual teeth, stroke-direction asymmetry, grain,
splintering, bending or heat. The simulated saw is 200 mm long and 150 g; the
cited listings give the real saw's 8 in blade but not its weight. Its contact
strip is a rectangular proxy, and the round handle is held by friction alone,
so it can turn in the fingers under load where the earlier rectangular grip
could not.
The 6 mm-per-turn screw drive and its friction are approximations, with a 100 N
force cap against the reference vise's stated 750 N maximum. The box- and
cylinder-built vise, saw, rack, support, rule and riser use task-owned
procedural geometry.

`task.py` holds the scene and public scorer. `runtime.py` and
`cutting_runtime.py` operate the physical screw, contact sensor and dynamic
colliders. `stock.py` is the material and connected-solid model; `scoring.py` checks
stages, clamping and conservative whole-saw clearance. The cutting law and
the scripted approach are unchanged from the earlier revision; the scene adds
the round handle and the rule.
`partkit.py` is a task-local derivative of the existing component helper and
does not modify the installed SDK. One layout is supplied; use
`-E object_jitter=0` for the stated dimensions.

## Third-party assets and licenses

| Asset | Use | Source and revision | License | Bundled here? |
| --- | --- | --- | --- | --- |
| YAM arms, grippers and meshes | Robot embodiment | [i2rt](https://github.com/i2rt-robotics/i2rt), commit `120c3c81400171174604e503943f8d1ebc891058`, via the installed YAM SDK | MIT, as shipped in `yam_arena/assets/yam/LICENSE.i2rt` | No |
| Station table, floor, cameras and lights | Base scene generated by the installed YAM SDK | [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), tested at `0059d02a1a91bc6834c1d0846022891df0c78730` | SDK repository states that it has no license yet; its i2rt robot assets are MIT | No |
| Isaac Lab-Arena and PhysX | Simulation runtime | Installed simulator, Isaac Lab-Arena 0.3.1 | Apache 2.0 for Isaac Lab-Arena; PhysX supplied with Isaac Sim | No |
| Wood, vise, screw, saw, rack, rule, riser and task code | Task geometry and logic | This folder | MIT; see `LICENSE` | Yes |

No third-party object mesh, texture or photograph is bundled. The task does
not load any object from the YAM SDK's station library.

## Revision (review TK-8BXEV6)

| Reviewer point | What changed |
|---|---|
| The cumulative 0.5 mm loose-cut depth veto cannot be checked from the trial video and end measurements; use a visible sawing-while-unclamped veto | `stock.py` latches `loose_stroke` on any stroke that cuts the stock while the measured clamp condition fails, and that alone zeroes the score; the per-column loose depth is kept only as an audit record. The real-trial check is the uninterrupted video. Tests: a single cutting stroke on loose stock is vetoed below one layer of depth; a resting saw, an unloaded stroke and sawing the clamped stock are not. |
| Protect the shared gripper pads from the saw's teeth, including during failed grasps | Real trials: the pads wear clear cut-resistant sleeves (ANSI A4 or better) for the whole trial, and the saw rests in its rack teeth down and away from the grasp side, so a missed grasp closes on the handle, the ferrule or air. |

## Claimed revision

This task builds on pull request #1984, whose revision hold lapsed without a
revision. It answers the latest review and keeps the fixes made for the one
before it.

| Reviewer point | What changed |
| --- | --- |
| The saw should have a round handle | The rectangular 26 × 110 × 28 mm grip is now a round, turned 26 mm diameter × 110 mm handle (a cylinder on the saw's long axis) with a 30 mm round brass ferrule, after the Pax 8 in gents saw now named as the real-trial saw. The pedestal is 1 mm taller (75 mm), so the handle axis still settles at 88 mm and the tooth edge at 53 mm. The grip material moved to the new handle and ferrule shapes. The scripted grasp is unchanged: the fingers still close across 26 mm at the handle's axis, 15 mm from its ferrule end (`GRIP_LOCAL`, 40 mm from the handle centre). |
| There should be a measuring device available to use | A loose Shinwa 300 mm flush-ended steel rule lies on a fixed wooden riser within reach, with overhangs for grasping. The README says how a person or robot uses it to mark and check 150 mm. |
| Earlier: partial credit for clamping and a substantial cut | Kept: 0.25, 0.5 and 1.0 stages unchanged. |
| Earlier: allow the saw to be placed elsewhere clear of the wood | Kept: any pose at least 5 mm clear of both pieces. |
| Earlier: specify wood species and tooth pattern | Kept the Scots pine. The tooth pattern is now the named saw's own: 20 TPI as supplied, push stroke. |

The scorer, cutting law and material model are unchanged. The clearance boxes
in `scoring.py` already enclose the round handle and ferrule; only their
comment changed. No scoring fixture depended on the grip shape, so the tests
are unchanged. The revised scene was run in simulation: 1.0 in 2,370 actions (Validation).

## Origin

This task builds on pull request #1984 in `robocurve/benchmark-tasks`. The
code is MIT licensed; see `LICENSE`.
