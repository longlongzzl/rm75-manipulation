"""Only collect the maintained offline suite, never legacy hardware scripts."""
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent
TESTS = ROOT / "tests"


def pytest_configure(config):
    # testpaths alone does not protect an explicit `pytest legacy/test_*.py`.
    for argument in config.args:
        path = (Path(config.invocation_params.dir) / str(argument).split("::", 1)[0]).resolve()
        if path != ROOT and path != TESTS and TESTS not in path.parents:
            raise pytest.UsageError(
                "Offline test collection is restricted to tests/. "
                "Legacy scripts may connect to the robot during import."
            )


def pytest_ignore_collect(collection_path, config):
    path = Path(collection_path).resolve()
    if path != ROOT and path != TESTS and TESTS not in path.parents:
        return True
    return None
