"""
Conversion from an RGB frame to the token pair the planner consumes.

The vision encoders are frozen and are not part of the planner checkpoint:
DINOv3 supplies 384 channels and SigLIP 1152, concatenated to 1536. Each frame
is encoded exactly once and pooled to two granularities, a coarse summary that
is retained as history and a fine view of the current frame only.
"""

from typing import Any, Mapping, Tuple

import numpy as np
import torch

from vla_tracking_omtrackvla.model_loader import (
    CHECKPOINT_IMAGE_SIZE,
    CHECKPOINT_VISION_FEAT_DIM,
)

#: Tokens kept per historical frame.
COARSE_TOKENS = 4

#: Tokens kept for the current frame.
FINE_TOKENS = 64


class FrameEncoder:
    """Wraps the upstream vision cacher behind a small, typed surface."""

    def __init__(self, device: torch.device, config: Mapping[str, Any]):
        """Build the frozen DINOv3 and SigLIP encoders on the given device."""
        # Imported lazily: only valid once the checkout is on sys.path.
        from cache_gridpool import (
            VisionCacheConfig,
            VisionFeatureCacher,
            grid_pool_tokens,
        )

        self._grid_pool_tokens = grid_pool_tokens
        self._device = device
        cacher_config = VisionCacheConfig(
            image_size=int(config.get('image_size', CHECKPOINT_IMAGE_SIZE)),
            batch_size=1,
            device=str(device),
        )
        self._cacher = VisionFeatureCacher(cacher_config)
        self._cacher.eval()

    def encode(self, rgb: np.ndarray) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Encode one HxWx3 uint8 RGB frame.

        :returns: coarse tokens on the CPU, because they are retained as
            history and holding 31 frames of them on the GPU buys nothing,
            and fine tokens on the inference device.
        """
        from PIL import Image

        pil = Image.fromarray(rgb)
        with torch.inference_mode():
            tokens_dino, grid_h, grid_w = self._cacher._encode_dino([pil])
            tokens_siglip = self._cacher._encode_siglip(
                [pil], out_hw=(grid_h, grid_w)
            )
            tokens = torch.cat([tokens_dino, tokens_siglip], dim=-1)
            fine = self._grid_pool_tokens(
                tokens, grid_h, grid_w, out_tokens=FINE_TOKENS
            )[0].float()
            coarse = self._grid_pool_tokens(
                tokens, grid_h, grid_w, out_tokens=COARSE_TOKENS
            )[0].float()

        expected = (COARSE_TOKENS, CHECKPOINT_VISION_FEAT_DIM)
        if tuple(coarse.shape) != expected:
            raise RuntimeError(
                f'coarse tokens are {tuple(coarse.shape)}, expected {expected}'
            )
        return coarse.cpu(), fine.to(self._device)
