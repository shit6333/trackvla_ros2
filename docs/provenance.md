# Provenance

Everything this workspace was verified against, recorded so a result can be
reproduced or a regression attributed.

## Upstream source

| Item | Value |
| --- | --- |
| Repository | https://github.com/om-ai-lab/OmTrackVLA |
| Pinned commit | `e9cb1fbd57f8dbcf98c460914251cbcbe20f2f57` |
| Location | `third_party/OmTrackVLA` (Git submodule) |

The inference path used by the adapter (`model.py`, `cache_gridpool.py`,
`open_trackvla_hf/`) is unmodified upstream code. The evaluation-side fixes
made during the EVT-Bench work live in a separate working tree and are
deliberately **not** carried into this repository.

## Model checkpoint

| Item | Value |
| --- | --- |
| Hugging Face repository | `omlab/OmTrackVLA-0.6B` |
| Revision | `ed97c9f99386c64b3d6b3607b9319438e17daa27` |
| Parameters | 609.2 M |
| Weights dtype | float32 on disk; the LLM loads in bfloat16 |

Training configuration, from the checkpoint's own `checkpoint_meta.json`:

| Key | Value |
| --- | --- |
| `llm_name` | `Qwen/Qwen3-0.6B` |
| `freeze_llm` | true |
| `n_waypoints` | 8 |
| `history` | 31 |
| `vision_feat_dim` | 1536 |
| `alpha_xy` | 2.0 |
| `use_tanh_actions` | false |
| `epoch` / `step` | 1 / 439700 |
| `embodiment` | `wheeled` |

Two notes on these values:

- `history` is absent from `config.json` but present in
  `checkpoint_meta.json`, so `resolve_history_length` reads it from there
  rather than hard-coding it. The adapter reports which source it used.
- `embodiment: wheeled` appears nowhere in the upstream Python code. It is
  recorded metadata only and therefore does **not** settle whether lateral
  velocity should be executed. See D015.

## Frozen vision encoders

Not part of the checkpoint; downloaded separately and never trained.

| Model | Width |
| --- | --- |
| `facebook/dinov3-vits16-pretrain-lvd1689m` | 384 |
| `google/siglip-so400m-patch14-384` | 1152 |

Concatenated to the 1536-dimensional feature the projector consumes.

## Runtime

| Item | Value |
| --- | --- |
| Base image | `nvidia/cuda:12.8.1-cudnn-devel-ubuntu24.04` |
| ROS | Jazzy, via `ros2-apt-source` pinned to 1.2.0 |
| Python | 3.12.3, the system interpreter |
| PyTorch | 2.8.0+cu128 |
| GPU verified on | NVIDIA RTX PRO 6000 Blackwell (`sm_120`) |

Resolved dependency versions are in
[`requirements/inference-lock.txt`](../requirements/inference-lock.txt), and
the rationale for each deviation from the OmTrackVLA runtime is in
[`docs/environment.md`](environment.md).

## Measurements

| Measurement | Value | Conditions |
| --- | --- | --- |
| Backend inference, isolated | 39.1 ms | 240x320 input, full 31-frame history, idle machine |
| Backend inference, under test load | 55.1 ms | same, with the rest of the suite running |
| In-pipeline inference | 46.6 ms | measured through `vla_inference_node` |
| Max `abs(waypoint[0])` | 0.0018 | confirms waypoint 0 is the trajectory origin |
| Predicted forward command | approx. 0.49 of full speed | from `tau[0, 1]` at `dt = 0.1` |

The forward figure is a fraction of full speed, not a speed. OmTrackVLA was
trained on Habitat base commands, which that simulator clips to [-1, 1] and
multiplies by a per-axis speed constant, and the training labels integrate
those commands with a bookkeeping constant of 0.1 rather than the simulator's
real timestep of `1 / ctrl_freq`. Dividing a waypoint by `dt` therefore
inverts that integration and returns the command, and the planner's output
`tanh` bounds it to [-1, 1]. No metric speed can be inherited from the
simulator, whose configured maxima are not physically calibrated, so the
metric scale belongs to the target robot and is applied by the executor.

The inference figures vary with machine load; treat the isolated number as the
floor and the loaded one as a realistic upper bound rather than either being
"the" latency.

The predicted speed was measured on synthetic noise imagery, so only its
magnitude is meaningful, not its direction. It is the basis for the warning
that the default 0.2 m/s limit saturates continuously.
