# Close an open bread bag with its reusable retaining clip

This task builds on an earlier submission, whose 48-hour revision hold lapsed without a revision. This version answers that submission's review (TK-6CL5XB):

| Review point | What changed |
| --- | --- |
| Scoring: a real bag has no five film sections, so the 0.25 and 0.5 section counts cannot be applied | Partial credit now uses a measured neck gap in mm on the prepared 20 mm opening, checked with gauge rods on a real bag. The simulation reports the same gap from the share of its 45 mm modelled neck width held inside the clip jaws. |
| Clarity: bag dimensions, film thickness and how to prepare the 20 mm open neck are unspecified | Setup names a stock 8 x 4 x 18 in, 1 mil gusseted bread bag, says how to confirm the film thickness with a micrometer, and gives a dowel method that sets the 20 mm neck the same way each trial. |
| Keep a clear full-neck-secured check for full credit; update sim and real scoring consistently | 1.0 still needs the whole neck in the jaws (gap below 2 mm), both hands at least 60 mm away and a 2 s still hold. The Score section lists each check next to how it is measured in sim and on a real bag. |

One hand gathers the loose polyethylene neck while the other slides the slot in a separate clip over the film. The clip should remain captured by the film after both hands let go.

## Items

- **Bag:** Interplas BR-HI1218 bread bag, 8 x 4 x 18 in (203 mm face, 102 mm side gusset, 457 mm long), low density polyethylene listed as 1 mil (0.001 in, about 25 µm). [Product page](https://www.interplas.com/restaurant-supplies/bread-bags/8x4x18-1mil-plastic-bread-bags-p-br-hi1218) shows the size, the 1 mil thickness and the 1000-bag case. The listed thickness is nominal: before a trial series, measure one film layer cut from a spare bag with a 0.001 mm outside micrometer at three spots and record the mean.
- **Clip:** Kwik Lok Series A hand-applied closure, listed by the maker as 2.14 x 3.49 cm. [Maker's page](https://www.kwiklok.com/product/series-a/). That page refused scripted requests (HTTP 403) when checked, so measure the clip with calipers; the simulated clip is 21.4 x 34.9 mm. [Bread clip](https://en.wikipedia.org/wiki/Bread_clip) describes the general type.
- **Gauge rods:** drill bit shanks or pin gauges of 2, 4, 6, 8, 10, 12 and 20 mm diameter.

## Setup and reset

Place a 120 x 240 x 90 mm foam loaf in the bag and push it to the sealed end. Cut the open end square so 120 mm of loose film extends past the loaf. Centre the loaf at X 367 mm and Y +139 mm on the station, with the opening toward negative Y. Two 9 x 190 x 16 mm side guides at X 298 mm and 436 mm, centred at Y 170 mm, and a 147 x 9 x 16 mm rear stop at Y 268 mm limit loaf movement.

Prepare the 20 mm open neck the same way each trial. Lay a 20 mm wooden dowel along the bag axis inside the loose film, 40 mm past the loaf. Gather the film evenly around the dowel by hand, then slide the dowel out toward the open end. With calipers, check that the neck opening 40 mm past the loaf is 20 mm across, within 3 mm; if not, repeat. Rest the base of the gathered film on a smooth 46 x 16 x 61 mm high block centred at X 357 mm and Y +7 mm, with the neck hanging past it.

Put the clip in an upright shallow slot at X 335 mm and Y -195 mm, with the open notch facing negative X. A 44 x 25 x 32 mm wooden block and two 3 mm-high strips make the clip rest. Reset by removing the clip, preparing the neck again with the dowel and returning the loaf and clip to their marks.

## Score

The neck gap is how wide a channel stays open through the neck where the clip sits, out of the prepared 20 mm. On a real bag, push gauge rods, largest first, from the open end along the neck past the clip without moving it; the gap is the diameter of the largest rod that passes (0 if the 2 mm rod does not pass). The simulation models the gathered neck as 45 mm of film width and reports the gap as 20 mm times the share of that width left outside the clip jaws (a 9 mm film strip counts inside when it crosses the clip window). This mapping from film width to an open channel is an approximation.

| Score | Check | Simulation | Real bag |
| --- | --- | --- | --- |
| 0.0 | Torn neck, or the clip holds too little film | Film links break, or gap above 12 mm | Film torn, clip off the film, or a 12 mm rod still passes the clip |
| 0.25 | Clip closed on the gathered neck, part of it outside the jaws | Gap 12 mm or less | Film passes through the clip's holding hole, seen from both faces, and the 12 mm rod does not pass |
| 0.5 | Whole neck inside the jaws while a hand still supports it | Gap below 2 mm | No film runs past the clip outside its jaws, seen from both faces, and the 2 mm rod does not pass |
| 1.0 | Full neck secured and released | Gap below 2 mm, loaf upright on the bench, both fingertips at least 60 mm from the clip, clip and loaf within 1.5 mm and the clip within 5° for 2 s | The same 0.5 checks with both grippers at least 60 mm clear, clip and loaf still for 2 s on a scaled video, and the 2 mm rod still blocked afterwards |

The simulated film is five thin articulated strips with breakable creases. Two small sprung teeth approximate the elastic deflection at the clip entry. Film wrinkles, polymer strain and the force of entry remain to be checked with the named bag and clip.

## Run

Use the RoboCurve launchable and source `yam.env`. From the task directory run the following command for each listed policy.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  inspect-robots run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --max-steps 680 --seed 0 --no-prompt \
    --store-frames --log-dir "evidence/$policy"
done
```

| Trial | Score | Actions | Control time | Measured wall time |
| --- | --- | --- | --- | --- |
| demo | 1.0 | 376 | 18.80 s | 190 s |
| model (Dream-1, seed 0) | 0.0 | 680 | 34.00 s | 153 s |
