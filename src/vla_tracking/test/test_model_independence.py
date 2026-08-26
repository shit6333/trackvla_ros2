"""
Structural test that the runtime stays model-independent.

Decision D001 says OmTrackVLA-specific code lives in an adapter. That is easy
to violate by accident with a single convenience import, so it is checked
mechanically rather than by review.
"""

import ast
import pathlib

#: Modules the generic runtime must never import, directly or transitively
#: through its own source.
FORBIDDEN_ROOTS = {
    'torch',
    'torchvision',
    'transformers',
    'timm',
    'habitat',
    'habitat_sim',
    'cache_gridpool',
    'open_trackvla_hf',
    'vla_tracking_omtrackvla',
}

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parent.parent / 'vla_tracking'


def _imported_roots(path):
    """Collect the top-level module name of every import in a source file."""
    tree = ast.parse(path.read_text(encoding='utf-8'))
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                roots.add(alias.name.split('.')[0])
        elif isinstance(node, ast.ImportFrom):
            if node.level == 0 and node.module:
                roots.add(node.module.split('.')[0])
    return roots


def test_runtime_does_not_import_model_libraries():
    """Check no runtime module reaches for a model library or the adapter."""
    offenders = {}
    for path in sorted(PACKAGE_ROOT.rglob('*.py')):
        hits = _imported_roots(path) & FORBIDDEN_ROOTS
        if hits:
            offenders[path.name] = sorted(hits)
    assert not offenders, (
        f'generic runtime modules import model-specific packages: {offenders}'
    )
