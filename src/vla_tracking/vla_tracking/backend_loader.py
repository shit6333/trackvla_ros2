"""
Discovery of backend implementations by name.

Backends register themselves under the `vla_tracking.backends` entry point
group. Resolving them at runtime is what keeps this package free of any import
dependency on a model adapter: vla_tracking never names OmTrackVLA, and an
adapter can be installed or removed without touching the runtime.
"""

from importlib.metadata import entry_points
from typing import Dict, List

from vla_tracking.backend_interface import VLABackend

#: Entry point group that backend packages advertise themselves under.
BACKEND_ENTRY_POINT_GROUP = 'vla_tracking.backends'


class BackendNotFoundError(KeyError):
    """Raised when no installed package provides the requested backend."""


def available_backends() -> Dict[str, str]:
    """Map each installed backend name to the object it resolves to."""
    return {
        ep.name: ep.value
        for ep in entry_points(group=BACKEND_ENTRY_POINT_GROUP)
    }


def load_backend(name: str) -> VLABackend:
    """
    Instantiate the backend registered under the given name.

    :raises BackendNotFoundError: when no installed package provides it. The
        message lists what is installed, because the usual cause is that the
        adapter package was not built into the overlay.
    """
    matches: List = [
        ep for ep in entry_points(group=BACKEND_ENTRY_POINT_GROUP)
        if ep.name == name
    ]
    if not matches:
        installed = sorted(available_backends())
        raise BackendNotFoundError(
            f'no backend named {name!r}; installed backends: '
            f'{installed if installed else "none"}'
        )

    backend_class = matches[0].load()
    backend = backend_class()

    if not isinstance(backend, VLABackend):
        raise TypeError(
            f'backend {name!r} resolved to {backend_class!r}, which does not '
            f'implement the VLABackend protocol'
        )
    return backend
