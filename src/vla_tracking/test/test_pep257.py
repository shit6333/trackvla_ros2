"""Docstring convention check."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Report any pep257 violation as a test failure."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
