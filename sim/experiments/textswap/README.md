# Text ablation on the DT benchmark

Rerun the OmTrackVLA DT evaluation with the instruction rewritten, keeping the
episodes, the scoring and the environment identical, and see whether SR / CR /
TR move. If they do not, the words are not steering the policy in closed loop.

| script | does |
| --- | --- |
| `make_textswap_agent.py` | writes a shadow copy of `trained_agent.py` (inserted lines only, nothing removed) that rewrites each episode's instruction according to `OMTRACK_INSTRUCTION_MODE`, and, with `OMTRACK_RESUME=1`, skips episodes whose result file already exists; the OmTrackVLA worktree is never edited |
| `run_textswap.py` | launches `run_eval.py` with the shadow module first on `sys.path`; refuses any `--save-path` that already exists unless `--resume` is given, so archived results cannot be overwritten |
| `run_all_modes.sh` | the four modes below, sequentially, on the same split |
| `agg_eval.py` | SR / CR / TR / lost / distractor collisions per results dir, paired on the episodes common to all dirs |

Modes: `original` (dataset instruction), `distractor` (the canonical dataset
description of the episode's first distractor, i.e. a real person in the scene
who is not the target), `neutral` ("follow the person"), `nonsense`
("xyzzy plugh frobnicate"). `swap` (a description of someone not in the scene)
is implemented but was not run.

## Result, 2026-09-04

DT val, first 100 episodes (`--split-num 14 --split-id 0`), checkpoint
OmTrackVLA-0.6B, container `omtrackvla-dev` after the dependency restore. The
environment is exactly deterministic with itself (two identical reruns of
seven episodes were step-identical), so every difference below is caused by
the instruction text alone.

    mode          SR      CR      TR    lost    DC     vs original: SR flips (s->f / f->s)   McNemar p
    original    0.390   0.130   0.673   0.39   1.77
    distractor  0.380   0.100   0.662   0.40   2.12    11 / 10                                1.000
    neutral     0.390   0.120   0.682   0.38   1.86    11 / 11                                1.000
    nonsense    0.330   0.120   0.668   0.44   1.71    12 /  6                                0.238

    TR paired difference:  -0.011 (p=0.94)  +0.009 (p=0.85)  -0.005 (p=0.42)
    DC paired difference:  +0.35  (p=0.12)  +0.09  (p=0.44)  -0.06  (p=0.87)
    distractor mode, per-episode distractor collisions: more in 28, fewer in 19,
    same in 53 episodes (sign test p=0.24)

Describing the distractor instead of the target changes success on 21 of 100
episodes, but 11 go one way and 10 the other: the trajectory is chaotically
sensitive to the text embedding, the outcome is not steered by it. Nonsense
costs six points of SR, within what 100 episodes can resolve (p=0.24). No mode
raises distractor collisions significantly, which is what steering toward the
described distractor would have produced.

## Full DT rerun with the original instruction (`dt_full_restore_20260904`)

Started 2026-09-04 07:57 with the unmodified `eval_dt.sh` command into a new
dir. The host shut down at 21:07 that day with 923 of 1405 episodes written;
the run was resumed on 2026-09-08 with `run_textswap.py --mode original
--resume`, which skipped exactly those 923 and continued from episode 924.

On the 923 episodes finished before the interruption, compared with the
archived `dt/` run made in the original container:

    dt/ (2026-08-20)              923   SR 0.416   CR 0.100   TR 0.639
    dt_full_restore (2026-09-04)  923   SR 0.416   CR 0.100   TR 0.639
    episodes identical in steps, success and TR:   913 / 923  (98.9 %)
    sampled episodes identical step for step:       11 / 12
    success differs on                              2 / 923

So the restored container reproduces the archive. A 7-episode probe run
earlier the same day had diverged from step 1-2; it ran while other containers
held models on the same GPU, and the full run did not, so GPU contention, not
the rebuild, is the working explanation for that divergence. Comparisons
should still be paired within one environment and one GPU state.
