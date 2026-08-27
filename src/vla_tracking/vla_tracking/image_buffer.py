"""
Thread-safe holder for the most recent camera frame.

The node keeps one frame, not a queue. Inference is slower than a camera, so a
queue would only accumulate lag and eventually drive the base from imagery that
no longer describes the world. Dropping intermediate frames is the correct
behaviour here, and the temporal history that the model needs lives in the
backend instead (see D012).
"""

import threading
from typing import Optional, Tuple

import numpy as np


class LatestFrame:
    """Holds the newest frame, discarding any earlier one that was unused."""

    def __init__(self):
        """Create an empty holder."""
        self._lock = threading.Lock()
        self._rgb: Optional[np.ndarray] = None
        self._stamp_ns: int = 0
        self._dropped = 0
        self._received = 0

    def put(self, rgb: np.ndarray, stamp_ns: int) -> None:
        """Store a frame, counting the previous one as dropped if unused."""
        with self._lock:
            if self._rgb is not None:
                self._dropped += 1
            self._rgb = rgb
            self._stamp_ns = stamp_ns
            self._received += 1

    def take(self) -> Optional[Tuple[np.ndarray, int]]:
        """
        Remove and return the newest frame, or None when there is none.

        Taking rather than peeking is deliberate: it prevents a stalled camera
        from being silently re-inferred as though it were live.
        """
        with self._lock:
            if self._rgb is None:
                return None
            frame = (self._rgb, self._stamp_ns)
            self._rgb = None
            return frame

    def clear(self) -> None:
        """Discard any held frame, for example when a task is replaced."""
        with self._lock:
            self._rgb = None

    @property
    def counters(self) -> Tuple[int, int]:
        """Return the number of frames received and dropped unused."""
        with self._lock:
            return self._received, self._dropped
