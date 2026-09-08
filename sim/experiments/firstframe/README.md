# First-frame target selection

Does the instruction decide which of two people the planner steers toward on
its very first step, before any temporal lock-in? Two Habitat humanoids from
the OmTrackVLA training set stand left and right of a Spot-jaw camera
(384x384, hfov 90, 0.65 m up), on a blank background and inside an HM3D room;
the frame is fed as the evaluator's step 0 (the same frame padded 31 times).

| script | runs in | does |
| --- | --- | --- |
| `render_pairs.py` | omtrackvla-dev (habitat_sim, no habitat-lab needed) | renders one case + JSON sidecar with each person's bearing |
| `firstframe_test.py` | trackvla-ros2-dev | replays each case through `OmTrackVLABackend` under six prompts, mirrored too, and scores the first step's bearing against the two people |
| `parity_upstream.py` | omtrackvla-dev | reruns the same frames through the upstream classes and diffs against ours |

Result, 2026-09-04, checkpoint OmTrackVLA-0.6B, identities female_31
("blue top and sandals") and male_36 ("brown leather armor and a blue shirt"):

    tau[1] over 96 runs:  x = +0.029 +/- 0.004   |y| <= 0.0011   |theta| <= 0.011
    instruction names a person, first step points at them:  34 / 64 = 53 %  (chance 50 %)
    same image side picked after mirroring:                 43 / 48
    upstream classes vs our backend, 96 comparisons:        max |diff| = 0.0

On a first frame the planner drives straight ahead regardless of the
instruction; the "pick" is decided by lateral noise of ~0.0005. The two
inference paths are bit-identical, so this is the model, not the plumbing.

## Real DT opening frames (2026-09-04)

`dtff_test.py` repeats the test on the first frame of four episodes the user
ran in Habitat (scene tQ5s4ShP627, results under
`omtrackvla_storage/eval_outputs/dt_distractor_collision_count_20260824_0148`),
three of which succeeded in the closed loop. Under each episode's own
instruction, the other episodes' instructions, a neutral prompt and nonsense,
mirrored too:

    40 runs:  tau[1].x mean +0.028   |y| <= 0.0010   |theta| <= 0.0097
    first-step bearing within +/-2 deg of the axis in every run
    target bearings: ep2 -15, ep3 +2, ep4 +8, ep6 -4 deg

The planner's first step is straight ahead on real episodes too. The three
successes came from the steps that followed, once the target had moved.

## Frames 0..30 as real history, prompt at frame 31 (2026-09-04)

`dtseq_test.py` feeds each episode's first 31 frames in order, exactly as
the Habitat evaluator saw them, and reads the prediction at frame 31 under the
episode's own instruction, the other episodes' instructions, a neutral prompt
and nonsense, plain and mirrored.

Temporal parity: under the own instruction the replay's tau[1]/dt has the same
sign and magnitude as the real run's base_velocity at every sampled step
(turns where it turned, reverses where it reversed); residual differences of
0.02-0.58 come from replaying H.264 video with the drawn overlay instead of the
renderer's frames.

Text effect with history, at frame 31:

    ep2  turns right toward the red woman   wz -0.38   under all five prompts
    ep3  near stop / reverse                             under all five prompts
    ep4  reverses (distractor blocking)     vx -0.12   under all five prompts
    ep6  hard right toward target at -43deg wz -0.87   under all five prompts,
         with a distractor in view at +35deg
    max |tau diff| between own prompt and any other: 0.003-0.021
    (tau[1] magnitudes 0.03-0.09; ep6's wz corresponds to tau theta -0.087)

With motion history the planner steers decisively, and what it steers toward
is set entirely by what it has been following; the words move the output by
an order of magnitude less than the action. Mirroring flips every sign, so
this is scene-driven, not a side bias.
