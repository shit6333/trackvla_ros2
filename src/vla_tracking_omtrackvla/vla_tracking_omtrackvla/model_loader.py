"""
Resolution and loading of the OmTrackVLA planner checkpoint.

All knowledge of the upstream layout lives here: where the source checkout is,
where the weights are, and how the two are combined. The upstream package is
not installable, and its HuggingFace wrapper imports a top-level module named
`model`, so the checkout root must be placed on sys.path before the wrapper is
imported.
"""

import os
from pathlib import Path
import sys
from typing import Any, Mapping, Optional, Tuple

import torch

#: Fallback number of previous frames, used when the checkpoint does not say.
#:
#: The value is absent from config.json but is recorded in checkpoint_meta.json
#: under config_overrides.history, so resolve_history_length prefers that. This
#: constant matches the --history default in train.py and
#: make_tracking_data.py, and the value the upstream evaluator hard-codes.
CHECKPOINT_HISTORY_LENGTH = 31

#: Seconds between predicted waypoints, as used by the upstream evaluator.
CHECKPOINT_DT = 0.1

#: Square side the vision encoders are fed at.
CHECKPOINT_IMAGE_SIZE = 384

#: Vision feature width, DINOv3 (384) concatenated with SigLIP (1152).
CHECKPOINT_VISION_FEAT_DIM = 1536


class UpstreamNotFoundError(RuntimeError):
    """Raised when the OmTrackVLA checkout cannot be located."""


class CheckpointNotFoundError(RuntimeError):
    """Raised when the planner weights cannot be located."""


class CudaUnavailableError(RuntimeError):
    """Raised when CUDA is required but unavailable."""


def resolve_source_path(config: Mapping[str, Any]) -> Path:
    """
    Locate the upstream checkout, preferring configuration over environment.

    A developer home directory is never hard-coded; the path arrives from the
    ROS parameter or from OMTRACKVLA_SRC, which the container sets.
    """
    candidate = config.get('source_path') or os.environ.get('OMTRACKVLA_SRC')
    if not candidate:
        raise UpstreamNotFoundError(
            'set the source_path parameter or the OMTRACKVLA_SRC environment '
            'variable to the OmTrackVLA checkout'
        )
    path = Path(candidate)
    if not (path / 'model.py').is_file():
        raise UpstreamNotFoundError(
            f'{path} does not look like an OmTrackVLA checkout (no model.py)'
        )
    return path


def resolve_checkpoint_path(config: Mapping[str, Any]) -> Path:
    """Locate the planner weights, preferring configuration over environment."""
    candidate = config.get('model_dir') or os.environ.get('HF_MODEL_DIR')
    if not candidate:
        raise CheckpointNotFoundError(
            'set the model_dir parameter or the HF_MODEL_DIR environment '
            'variable to the planner checkpoint directory'
        )
    path = Path(candidate)
    if not (path / 'config.json').is_file():
        raise CheckpointNotFoundError(
            f'{path} has no config.json; it is not a checkpoint directory'
        )
    return path


def ensure_importable(source_path: Path) -> None:
    """Put the upstream checkout on sys.path if it is not already there."""
    entry = str(source_path)
    if entry not in sys.path:
        sys.path.insert(0, entry)


def select_device(require_cuda: bool = True) -> torch.device:
    """
    Choose the inference device, refusing to fall back to CPU silently.

    A CPU fallback would produce a pipeline that looks healthy but cannot keep
    up with a camera, which is worse than an explicit failure.
    """
    if torch.cuda.is_available():
        return torch.device('cuda')
    if require_cuda:
        raise CudaUnavailableError(
            'CUDA is unavailable and require_cuda is set; refusing to run '
            'inference on the CPU'
        )
    return torch.device('cpu')


def load_planner(
    config: Mapping[str, Any],
) -> Tuple[Any, torch.device, Path]:
    """
    Load the planner and report the device and checkpoint it came from.

    :returns: the model in eval mode, the device it lives on, and the
        checkpoint directory, so a caller can report provenance.
    """
    source_path = resolve_source_path(config)
    checkpoint_path = resolve_checkpoint_path(config)
    ensure_importable(source_path)
    device = select_device(bool(config.get('require_cuda', True)))

    # Imported here, not at module scope: the import only works once the
    # checkout is on sys.path.
    from open_trackvla_hf import OpenTrackVLAForWaypoint

    model = OpenTrackVLAForWaypoint.from_pretrained(str(checkpoint_path))
    model = model.to(device).eval()
    return model, device, checkpoint_path


def resolve_history_length(checkpoint_path: Path) -> Tuple[int, str]:
    """
    Read the training history length from the checkpoint, if it records one.

    The model's coarse history carries explicit time indices, so its length is
    a property of the weights rather than a free runtime choice. It is missing
    from config.json but present in checkpoint_meta.json, so prefer that over
    the constant and report which was used.

    :returns: the length and a short description of where it came from.
    """
    meta_path = checkpoint_path / 'checkpoint_meta.json'
    try:
        import json

        with open(meta_path, encoding='utf-8') as handle:
            meta = json.load(handle)
        recorded = meta.get('config_overrides', {}).get('history')
        if isinstance(recorded, int) and recorded > 0:
            return recorded, 'checkpoint_meta.json'
    except (OSError, ValueError):
        pass
    return CHECKPOINT_HISTORY_LENGTH, 'project default'


def describe_checkpoint(checkpoint_path: Path) -> Optional[dict]:
    """Read the checkpoint config for provenance reporting, if readable."""
    import json

    try:
        with open(checkpoint_path / 'config.json', encoding='utf-8') as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None
