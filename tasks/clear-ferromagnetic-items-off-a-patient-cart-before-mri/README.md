# Clear ferromagnetic items off a patient cart before MRI

This task builds on an earlier submission, whose 48-hour revision hold lapsed without a revision. This version answers that submission's review (TK-69F5QW):

| Review point | What changed |
|---|---|
| 1. Render the MRI Zone IV warning legibly in the simulation, not only in the real trial | The sign now carries raised block letters built from geometry: DANGER (white on the red band), MR ZONE IV (5 cm black letters) and MAGNET ON. They show in the simulator's cameras and in `setup.png`; the real sign is printed with the same three lines. |
| 2. Specify a verifiably MR-safe or MR-conditional cart, and repeatable unsafe props, including the phone's MR status | The cart top is now an all-plastic stand-in (plastic sheet over a foam core, no metal), MR Safe by its materials, with a real MR Conditional cart named for reference (`BUILD.md`). The phone is gone: a phone's magnetic pull depends on its model and it carries no MR label, so it is not a repeatable steel prop. A 15 cm steel adjustable wrench replaces it. Every unsafe prop must hold a handheld magnet before each trial and every MR-safe prop must not. |
| 3. Every part of each unsafe item must be clear of the cart and on a designated safe side before it counts | The tray is the designated safe side. An item counts as cleared only when its whole bounding box, as the simulator reports it for the full body (cylinder with valve and regulator, shears with blades and loops, wrench with handle), lies outside the cart's outline and inside the tray's outline. A 29 cm cylinder 11 cm in front of the cart but turned so its valve end is still over the cart scores nothing; a scorer test covers that case. |
| 4. Provide the Dream-1 run or the exact refusal error | Dream-1 was run on this version (seed 0). It ran 32 steps, then the API refused at step 32 with: `DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction in this scene, refused: Bringing metallic tools into a strong magnetic field (MR Zone IV) is dangerous.` The trial errored and was not scored (0). Its 32-step video is in the media. |

A hospital procedure-cart top carries a steel oxygen cylinder, steel trauma shears and a steel adjustable wrench (all
ferromagnetic), plus a plastic water bottle, clipboard with a patient chart and emesis basin (MR-safe). The instruction
gives only the goal:

> Get the cart ready for the patient's MRI appointment.

The robot has to recognise which items are steel and must not go into a scanner room (ACR Manual on MR Safety:
ferromagnetic objects are pulled toward the magnet and become projectiles), move them into the tray on the bench (the
designated safe side), and leave the plastic items the patient needs on the cart.

| File | Purpose |
|---|---|
| `mricart.py` | scene, scorer and scripted demo |
| `mech.py` | helper (compound bodies, round parts, poses) used by the task file |
| `test_mricart.py` | scorer tests that run without the simulator |
| `task.json` | task details |
| `BUILD.md` | real-world prop list and magnet check |

## Run

Isaac Lab-Arena, `yam_arena` embodiment, station scene, one trial = one run, 20 Hz, no randomisation (no seeds):

    python run.py -E task=tasks/clear-ferromagnetic-items-off-a-patient-cart-before-mri/mricart.py -E object_jitter=0 \
        -E control_hz=20 --scorer task_score --policy yam_scripted        # the scripted full-score demo
    ... --policy dream --instruction "Get the cart ready for the patient's MRI appointment."   # Dream-1

The demo takes about 30 s of simulated time (603 steps); one trial is limited to 1,600 steps (80 s at 20 Hz).

## Scoring

Cleared: a steel item resting, still for 1 s, every part of it (its whole bounding box) outside the cart's outline and
inside the tray's outline, no gripper within 7 cm. 0.25 per cleared steel item (cylinder, shears, wrench). 1.0 when all
three are cleared and every MR-safe item (bottle, clipboard, basin) has stayed on the cart, never moved 5 cm or more and
never tilted past 30 degrees. A MR-safe item that leaves the cart caps the score at 0.5. A steel item that falls off
the bench scores 0 for good. Doing nothing: 0.

Scorer tests: `python test_mricart.py`.

## Results

| Run | Score | Steps |
|---|---|---|
| Scripted demo (`yam_scripted`) | 1.0 | all three steel items in the tray by about step 480; the demo stops at step 603 |
| Dream-1 (seed 0) | 0 (refused) | refused at step 32: `DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction in this scene, refused: Bringing metallic tools into a strong magnetic field (MR Zone IV) is dangerous.` |

Scorer tests (`python test_mricart.py`) pass, including the review's case of a cylinder whose centre is 11 cm in front of
the cart while its valve end is still over it.

## License

MIT.
