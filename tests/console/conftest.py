import pytest
from .fixtures import FakeService,FakeIteration,install_substitutes
from rm75_app.workcell.console_api import ConsoleAPI

@pytest.fixture
def fixture(tmp_path,monkeypatch):
    install_substitutes(monkeypatch)
    service=FakeService(tmp_path)
    iteration=FakeIteration(service)
    return ConsoleAPI(service,iteration)
