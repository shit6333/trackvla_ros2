#!/usr/bin/env python3
"""Peak GPU memory and per-frame latency of the OmTrackVLA backend on this GPU.

Loads the backend exactly as vla_inference_node does (configure, reset,
infer), feeds 384x384 frames until the 31-frame history is full and then
some, and prints torch's allocator peak plus what the driver holds for the
process, which is the number nvidia-smi shows and the one a small card runs
out of. Run inside the inference container:

    docker compose run --rm vla_tracking python3 scripts/measure_backend.py

Latency depends on what else is on the GPU: measure with the card otherwise
idle, or the figure describes the contention, not the card. Memory does not.
"""
import os
import subprocess
import time

import numpy as np
import torch

from vla_tracking.backend_interface import Observation
from vla_tracking_omtrackvla.omtrackvla_backend import OmTrackVLABackend

FRAMES = 60


def process_memory_mib() -> int:
    """GPU memory the driver attributes to this process, as nvidia-smi reports it."""
    try:
        out = subprocess.check_output(
            ['nvidia-smi', '--query-compute-apps=pid,used_memory',
             '--format=csv,noheader,nounits'], text=True)
    except (OSError, subprocess.CalledProcessError):
        return -1
    for line in out.splitlines():
        pid, used = [x.strip() for x in line.split(',')]
        if int(pid) == os.getpid():
            return int(used)
    return -1


def main() -> None:
    backend = OmTrackVLABackend()
    started = time.perf_counter()
    backend.configure({'model_dir': '', 'source_path': '', 'require_cuda': True})
    print(f'configure + warm-up   {time.perf_counter() - started:.1f} s')
    print(f'after load            torch reserved {torch.cuda.memory_reserved() / 2**20:.0f} MiB'
          f'   process {process_memory_mib()} MiB')

    backend.reset('follow the person')
    rng = np.random.default_rng(0)
    latencies = []
    for k in range(FRAMES):
        rgb = rng.integers(0, 255, (384, 384, 3), dtype=np.uint8)
        t = time.perf_counter()
        backend.infer(Observation(rgb=rgb, stamp_ns=int(time.time() * 1e9),
                                  instruction='follow the person'))
        torch.cuda.synchronize()
        latencies.append(time.perf_counter() - t)

    history = backend._history_length
    steady = np.array(latencies[history:]) * 1000
    print(f'device                {torch.cuda.get_device_name(0)}')
    print(f'inference, history full ({len(steady)} frames): '
          f'median {np.median(steady):.0f} ms   max {steady.max():.0f} ms')
    print(f'peak GPU memory       torch reserved {torch.cuda.max_memory_reserved() / 2**20:.0f} MiB'
          f'   process {process_memory_mib()} MiB')
    backend.shutdown()


if __name__ == '__main__':
    main()
