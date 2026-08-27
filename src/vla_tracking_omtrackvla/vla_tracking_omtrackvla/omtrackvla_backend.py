"""
OmTrackVLA implementation of the vla_tracking backend contract.

The backend owns the model's temporal state. That state is not raw imagery: it
is the four-token coarse summary of each of the previous 31 frames, produced
once per frame by the frozen vision encoders. Handing the node a window of raw
images instead would force the encoders to re-run 31 times per step.
"""

from collections import deque
import time
from typing import Any, Mapping, Optional

import numpy as np
import torch

from vla_tracking.backend_interface import (
    BackendError,
    BackendNotConfiguredError,
    BackendNotReadyError,
    Observation,
    Prediction,
)

from vla_tracking_omtrackvla.model_loader import (
    CHECKPOINT_DT,
    CHECKPOINT_HISTORY_LENGTH,
    describe_checkpoint,
    load_planner,
    resolve_history_length,
)
from vla_tracking_omtrackvla.preprocessing import FrameEncoder

DEFAULT_FRAME_ID = 'base_link'


class OmTrackVLABackend:
    """Serves the released OmTrackVLA planner through the backend contract."""

    def __init__(self):
        """Create an unconfigured backend; no weights are touched yet."""
        self._model = None
        self._encoder: Optional[FrameEncoder] = None
        self._device: Optional[torch.device] = None
        self._instruction: Optional[str] = None
        self._history: deque = deque(maxlen=CHECKPOINT_HISTORY_LENGTH)
        self._history_length = CHECKPOINT_HISTORY_LENGTH
        self._dt = CHECKPOINT_DT
        self._frame_id = DEFAULT_FRAME_ID
        self._checkpoint_info: Optional[dict] = None
        self._checkpoint_path = None
        self._history_source = 'project default'
        self._last_inference_seconds = 0.0

    # -- contract ---------------------------------------------------------

    @property
    def name(self) -> str:
        """Return the identifier recorded in published trajectories."""
        return 'omtrackvla'

    def configure(self, config: Mapping[str, Any]) -> None:
        """Load the planner and the frozen vision encoders."""
        if self._model is not None:
            return

        model, device, checkpoint_path = load_planner(config)
        self._model = model
        self._device = device
        self._encoder = FrameEncoder(device, config)
        self._checkpoint_info = describe_checkpoint(checkpoint_path)
        self._checkpoint_path = checkpoint_path

        # The history length belongs to the weights, so it is taken from the
        # checkpoint when recorded there. An explicit override still wins, but
        # describe() reports which source was used so a mismatch is visible.
        recorded, source = resolve_history_length(checkpoint_path)
        override = config.get('history_length')
        if override:
            self._history_length = int(override)
            self._history_source = 'configuration override'
        else:
            self._history_length = recorded
            self._history_source = source
        if self._history_length < 1:
            raise ValueError('history_length must be at least 1')
        self._history = deque(maxlen=self._history_length)
        self._dt = float(config.get('dt', CHECKPOINT_DT))
        self._frame_id = str(config.get('frame_id', DEFAULT_FRAME_ID))

    def reset(self, instruction: str) -> None:
        """Start a new task, discarding the previous task's token history."""
        if self._model is None:
            raise BackendNotConfiguredError(
                'configure() must succeed before reset()'
            )
        self._instruction = instruction
        self._history.clear()
        self._last_inference_seconds = 0.0

    def infer(self, observation: Observation) -> Prediction:
        """Encode one frame, extend the history, and predict a trajectory."""
        if self._model is None or self._encoder is None:
            raise BackendNotConfiguredError(
                'configure() must succeed before infer()'
            )
        if self._instruction is None:
            raise BackendNotReadyError('reset() must be called before infer()')

        started = time.perf_counter()
        try:
            coarse, fine = self._encoder.encode(observation.rgb)
            self._history.append(coarse)
            warming_up = len(self._history) < self._history_length
            waypoints = self._predict(fine, observation.instruction)
        except Exception as exc:
            self._last_inference_seconds = time.perf_counter() - started
            raise BackendError(
                f'{type(exc).__name__}: {exc}'
            ) from exc
        self._last_inference_seconds = time.perf_counter() - started

        finite = bool(np.all(np.isfinite(waypoints)))
        if not finite:
            return Prediction(
                waypoints=waypoints,
                dt=self._dt,
                frame_id=self._frame_id,
                stamp_ns=observation.stamp_ns,
                valid=False,
                warming_up=False,
                status='planner produced non-finite waypoints',
            )

        return Prediction(
            waypoints=waypoints,
            dt=self._dt,
            frame_id=self._frame_id,
            stamp_ns=observation.stamp_ns,
            valid=not warming_up,
            warming_up=warming_up,
            status=(
                f'warming up, {len(self._history)}/{self._history_length} '
                f'frames' if warming_up else 'ok'
            ),
        )

    def describe(self):
        """Report device and checkpoint provenance for the startup log."""
        info = self._checkpoint_info or {}
        device = self._device
        if device is not None and device.type == 'cuda':
            index = device.index or 0
            device_text = f'{torch.cuda.get_device_name(index)} (cuda:{index})'
        else:
            device_text = str(device)
        return {
            'backend': 'omtrackvla',
            'device': device_text,
            'torch': torch.__version__,
            'checkpoint': str(self._checkpoint_path),
            'llm': str(info.get('llm_name', 'unknown')),
            'n_waypoints': str(info.get('n_waypoints', 'unknown')),
            'history_length': (
                f'{self._history_length} (from {self._history_source})'
            ),
            'dt': f'{self._dt:.3f} s',
        }

    def shutdown(self) -> None:
        """Release the model and encoders. Safe to call repeatedly."""
        self._history.clear()
        self._instruction = None
        self._model = None
        self._encoder = None
        if self._device is not None and self._device.type == 'cuda':
            torch.cuda.empty_cache()
        self._device = None

    # -- reporting --------------------------------------------------------

    @property
    def last_inference_seconds(self) -> float:
        """Wall-clock duration of the most recent infer() call."""
        return self._last_inference_seconds

    @property
    def checkpoint_info(self) -> Optional[dict]:
        """Return the checkpoint's config for provenance reporting."""
        return self._checkpoint_info

    # -- internals --------------------------------------------------------

    def _predict(self, fine: torch.Tensor, instruction: str) -> np.ndarray:
        """
        Assemble the token sequence and run the planner.

        Reproduces the upstream evaluator: a short history is left-padded with
        its earliest frame, each historical frame carries the time index of
        its position, and the current frame is indexed one past the history.
        """
        device = self._device
        horizon = self._history_length

        frames = list(self._history)
        if len(frames) < horizon:
            frames = [frames[0]] * (horizon - len(frames)) + frames

        coarse_list = []
        time_index_list = []
        for step, tokens in enumerate(frames):
            tokens = tokens.to(device)
            coarse_list.append(tokens)
            time_index_list.append(
                torch.full(
                    (tokens.size(0),),
                    fill_value=step,
                    dtype=torch.long,
                    device=device,
                )
            )
        coarse_tokens = torch.cat(coarse_list, dim=0).unsqueeze(0)
        coarse_tidx = torch.cat(time_index_list, dim=0).unsqueeze(0)

        fine_tokens = fine.unsqueeze(0)
        fine_tidx = torch.full(
            (1, fine_tokens.size(1)),
            fill_value=horizon,
            dtype=torch.long,
            device=device,
        )

        with torch.inference_mode():
            tau = self._model(
                coarse_tokens,
                coarse_tidx,
                fine_tokens,
                fine_tidx,
                [instruction],
            )
        return tau.detach().float().cpu().numpy()[0].astype(np.float64)
