"""Tests for entry-point based backend discovery."""

import pytest

from vla_tracking.backend_interface import VLABackend
from vla_tracking.backend_loader import (
    available_backends,
    BackendNotFoundError,
    load_backend,
)


def test_fake_backend_is_discoverable():
    """Check this package registers its own fake backend."""
    assert 'fake' in available_backends()


def test_load_backend_returns_a_conforming_instance():
    """Check a discovered backend implements the protocol."""
    backend = load_backend('fake')
    assert isinstance(backend, VLABackend)
    assert backend.name == 'fake'


def test_unknown_backend_names_what_is_installed():
    """
    Check the failure is diagnosable.

    The usual cause is that the adapter package was not built into the
    overlay, so the error must say which backends are actually present.
    """
    with pytest.raises(BackendNotFoundError) as excinfo:
        load_backend('does-not-exist')
    assert 'fake' in str(excinfo.value)
