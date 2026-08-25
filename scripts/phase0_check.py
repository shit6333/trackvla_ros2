#!/usr/bin/env python3
"""Phase 0 environment-compatibility gate.

Answers the single question that blocks the rest of the project: can the
OmTrackVLA inference stack and rclpy share one CPython 3.12 interpreter on a
CUDA-enabled ROS 2 Jazzy image, and does the checkpoint produce a usable
trajectory there?

Exit criteria checked here come from docs/implementation-plan.md Phase 0:

  1. rclpy and PyTorch import in the same interpreter
  2. the GPU is visible inside the container
  3. the backend returns finite [N, 3] waypoints and an explicit dt
  4. repeated reset/inference does not retain a prior task's temporal state

Run inside the container:

    python3 scripts/phase0_check.py
"""

from __future__ import annotations

import os
import sys
import traceback
from collections import deque
from typing import Callable

import numpy as np

# Mirrors trained_agent.py: coarse-token history length and the executor dt.
HISTORY = 31
DT = 0.1
N_WAYPOINTS = 8
ACTION_DIMS = 3
VISION_FEAT_DIM = 1536
IMAGE_HW = (480, 640)

results: list[tuple[str, bool, str]] = []


def stage(title: str) -> Callable:
    """Run one gate stage, record its verdict, and keep going on failure."""

    def decorator(fn: Callable) -> Callable:
        print(f"\n=== {title} ===")
        try:
            detail = fn() or ""
            results.append((title, True, detail))
            print(f"PASS  {title}" + (f" — {detail}" if detail else ""))
        except Exception as exc:  # noqa: BLE001 - the gate reports every failure
            traceback.print_exc()
            results.append((title, False, f"{type(exc).__name__}: {exc}"))
            print(f"FAIL  {title} — {type(exc).__name__}: {exc}")
        return fn

    return decorator


# --------------------------------------------------------------------------
# 1. Interpreter coexistence
# --------------------------------------------------------------------------
state: dict = {}


@stage("rclpy and torch share one interpreter")
def _check_interpreter() -> str:
    import rclpy
    import torch

    if sys.version_info[:2] != (3, 12):
        raise RuntimeError(f"expected CPython 3.12, found {sys.version.split()[0]}")

    # A ROS node must actually initialise, not merely import.
    rclpy.init(args=None)
    node = rclpy.create_node("phase0_check")
    node.destroy_node()
    rclpy.shutdown()

    state["torch"] = torch
    return (
        f"python {sys.version.split()[0]}, rclpy {rclpy.__file__.split('/')[-3]}, "
        f"torch {torch.__version__}"
    )


@stage("dependency versions")
def _check_versions() -> str:
    import transformers

    return (
        f"transformers {transformers.__version__}, numpy {np.__version__}, "
        f"pillow {__import__('PIL').__version__}"
    )


# --------------------------------------------------------------------------
# 2. GPU visibility
# --------------------------------------------------------------------------
@stage("CUDA device is visible and supported")
def _check_cuda() -> str:
    torch = state["torch"]

    if not torch.cuda.is_available():
        raise RuntimeError("torch.cuda.is_available() is False; there is no CPU fallback")

    major, minor = torch.cuda.get_device_capability(0)
    required = f"sm_{major}{minor}"
    arch_list = torch.cuda.get_arch_list()
    if required not in arch_list:
        raise RuntimeError(
            f"torch build lacks {required} kernels (has: {' '.join(arch_list)})"
        )

    state["device"] = torch.device("cuda")
    return f"{torch.cuda.get_device_name(0)} ({required}), cuda {torch.version.cuda}"


# --------------------------------------------------------------------------
# 3. Upstream inference modules import without Habitat
# --------------------------------------------------------------------------
@stage("OmTrackVLA inference modules import without Habitat")
def _check_upstream_import() -> str:
    src = os.environ.get("OMTRACKVLA_SRC")
    if not src or not os.path.isdir(src):
        raise RuntimeError(f"OMTRACKVLA_SRC is not a directory: {src!r}")
    if src not in sys.path:
        sys.path.insert(0, src)

    from cache_gridpool import (  # noqa: F401
        VisionCacheConfig,
        VisionFeatureCacher,
        grid_pool_tokens,
    )
    from open_trackvla_hf import OpenTrackVLAForWaypoint

    if "habitat" in sys.modules:
        raise RuntimeError("importing the inference path pulled in habitat")

    state["VisionCacheConfig"] = VisionCacheConfig
    state["VisionFeatureCacher"] = VisionFeatureCacher
    state["grid_pool_tokens"] = grid_pool_tokens
    state["OpenTrackVLAForWaypoint"] = OpenTrackVLAForWaypoint
    return f"from {src}"


# --------------------------------------------------------------------------
# 4. Vision encoders
# --------------------------------------------------------------------------
@stage("vision encoders load and encode one frame")
def _check_vision() -> str:
    torch = state["torch"]

    cfg = state["VisionCacheConfig"](
        image_size=384, batch_size=1, device=str(state["device"])
    )
    encoder = state["VisionFeatureCacher"](cfg)
    encoder.eval()

    from PIL import Image

    rng = np.random.default_rng(0)
    frame = rng.integers(0, 255, (*IMAGE_HW, 3), dtype=np.uint8)
    pil = Image.fromarray(frame)

    with torch.inference_mode():
        tok_dino, hp, wp = encoder._encode_dino([pil])
        tok_siglip = encoder._encode_siglip([pil], out_hw=(hp, wp))
        tokens = torch.cat([tok_dino, tok_siglip], dim=-1)
        fine = state["grid_pool_tokens"](tokens, hp, wp, out_tokens=64)[0].float()
        coarse = state["grid_pool_tokens"](tokens, hp, wp, out_tokens=4)[0].float()

    if coarse.shape != (4, VISION_FEAT_DIM):
        raise RuntimeError(f"coarse tokens are {tuple(coarse.shape)}, expected (4, {VISION_FEAT_DIM})")
    if fine.shape != (64, VISION_FEAT_DIM):
        raise RuntimeError(f"fine tokens are {tuple(fine.shape)}, expected (64, {VISION_FEAT_DIM})")

    state["encoder"] = encoder
    return f"dino+siglip grid {hp}x{wp} -> coarse {tuple(coarse.shape)}, fine {tuple(fine.shape)}"


# --------------------------------------------------------------------------
# 5. Checkpoint load
# --------------------------------------------------------------------------
@stage("planner checkpoint loads")
def _check_checkpoint() -> str:
    torch = state["torch"]

    model_dir = os.environ.get("HF_MODEL_DIR")
    if not model_dir or not os.path.isfile(os.path.join(model_dir, "config.json")):
        raise RuntimeError(f"HF_MODEL_DIR does not look like a checkpoint: {model_dir!r}")

    model = state["OpenTrackVLAForWaypoint"].from_pretrained(model_dir)
    model = model.to(state["device"]).eval()

    params = sum(p.numel() for p in model.parameters())
    state["model"] = model
    return f"{model_dir}, {params / 1e6:.1f}M parameters"


# --------------------------------------------------------------------------
# 6 + 7. Forward pass and temporal-state isolation
# --------------------------------------------------------------------------
def _encode(frame: np.ndarray):
    """Encode one RGB frame to (coarse[4, C], fine[64, C]) on CPU."""
    torch = state["torch"]
    from PIL import Image

    encoder = state["encoder"]
    pil = Image.fromarray(frame)
    with torch.inference_mode():
        tok_dino, hp, wp = encoder._encode_dino([pil])
        tok_siglip = encoder._encode_siglip([pil], out_hw=(hp, wp))
        tokens = torch.cat([tok_dino, tok_siglip], dim=-1)
        fine = state["grid_pool_tokens"](tokens, hp, wp, out_tokens=64)[0].float()
        coarse = state["grid_pool_tokens"](tokens, hp, wp, out_tokens=4)[0].float()
    return coarse.cpu(), fine


def _infer(history: deque, coarse, fine, instruction: str):
    """Reproduce trained_agent.py's history assembly and planner call."""
    torch = state["torch"]
    device = state["device"]

    history.append(coarse)
    frames = list(history)
    if len(frames) < HISTORY:
        frames = [frames[0]] * (HISTORY - len(frames)) + frames
    else:
        frames = frames[-HISTORY:]

    coarse_list, coarse_tidx = [], []
    for t, tok in enumerate(frames):
        tok = tok.to(device)
        coarse_list.append(tok)
        coarse_tidx.append(
            torch.full((tok.size(0),), fill_value=t, dtype=torch.long, device=device)
        )
    coarse_tokens = torch.cat(coarse_list, dim=0).unsqueeze(0)
    coarse_tidx = torch.cat(coarse_tidx, dim=0).unsqueeze(0)

    fine_tokens = fine.to(device).unsqueeze(0)
    fine_tidx = torch.full(
        (1, fine_tokens.size(1)), fill_value=HISTORY, dtype=torch.long, device=device
    )

    with torch.inference_mode():
        tau = state["model"](
            coarse_tokens, coarse_tidx, fine_tokens, fine_tidx, [instruction]
        )
    return tau.detach().float().cpu().numpy()


@stage("forward pass returns finite [1, 8, 3] waypoints")
def _check_forward() -> str:
    rng = np.random.default_rng(1)
    frame_a = rng.integers(0, 255, (*IMAGE_HW, 3), dtype=np.uint8)
    coarse_a, fine_a = _encode(frame_a)
    state["frame_a"] = (coarse_a, fine_a)

    history = deque(maxlen=HISTORY)
    tau = _infer(history, coarse_a, fine_a, "follow the person in the red shirt")

    expected = (1, N_WAYPOINTS, ACTION_DIMS)
    if tau.shape != expected:
        raise RuntimeError(f"trajectory shape is {tau.shape}, expected {expected}")
    if not np.all(np.isfinite(tau)):
        raise RuntimeError("trajectory contains non-finite values")

    state["tau_first"] = tau
    wp1 = tau[0, 1]
    return (
        f"shape {tau.shape}, dt={DT}s, tau[0,1]=[{wp1[0]:+.4f}, {wp1[1]:+.4f}, "
        f"{wp1[2]:+.4f}] -> vx={wp1[0] / DT:+.3f} vy={wp1[1] / DT:+.3f} "
        f"wz={wp1[2] / DT:+.3f}"
    )


@stage("reset clears temporal state")
def _check_reset() -> str:
    coarse_a, fine_a = state["frame_a"]
    rng = np.random.default_rng(2)
    frame_b = rng.integers(0, 255, (*IMAGE_HW, 3), dtype=np.uint8)
    coarse_b, fine_b = _encode(frame_b)

    tau_first = state["tau_first"]

    # Same task, second frame: the history now differs, so the output must too.
    history = deque(maxlen=HISTORY)
    _infer(history, coarse_a, fine_a, "follow the person in the red shirt")
    tau_second = _infer(history, coarse_b, fine_b, "follow the person in the red shirt")
    if np.allclose(tau_first, tau_second):
        raise RuntimeError(
            "appending a second frame did not change the prediction; "
            "the model may be ignoring its coarse history"
        )

    # New task: clearing the history must reproduce the very first prediction
    # exactly. Any residue from the previous task would perturb it.
    history.clear()
    tau_after_reset = _infer(history, coarse_a, fine_a, "follow the person in the red shirt")
    if not np.array_equal(tau_first, tau_after_reset):
        max_delta = float(np.max(np.abs(tau_first - tau_after_reset)))
        raise RuntimeError(
            f"prediction after reset differs from the first prediction "
            f"(max |delta| = {max_delta:.3e}); temporal state leaked across tasks"
        )

    return "second frame changes the output; reset restores it bit-for-bit"


# --------------------------------------------------------------------------
print("\n" + "=" * 72)
print("Phase 0 summary")
print("=" * 72)
for title, ok, detail in results:
    print(f"  [{'PASS' if ok else 'FAIL'}] {title}")
    if detail:
        print(f"         {detail}")

failed = [t for t, ok, _ in results if not ok]
if failed:
    print(f"\nPhase 0 GATE FAILED ({len(failed)}/{len(results)} stages)")
    sys.exit(1)

print(f"\nPhase 0 GATE PASSED ({len(results)}/{len(results)} stages)")
