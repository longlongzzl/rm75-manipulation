"""Worker resource ownership contract; stubs do not qualify native execution."""
from types import SimpleNamespace
import pytest
from rm75_app.swm import integration
from rm75_app.swm.skills import AtomicSkillRuntime


class RuntimeStub(AtomicSkillRuntime):
    def __init__(self, failure=None):
        self.world=SimpleNamespace(domain='physics')
        self.sync=SimpleNamespace(sync=lambda boundary: {'boundary':boundary})
        self.failure=failure
    def run(self, request):
        if self.failure:raise self.failure
        return SimpleNamespace(skill_verified=True,as_dict=lambda: {'fixture':True})


@pytest.mark.parametrize('failure',[None,RuntimeError('plan failure'),RuntimeError('observation failure'),KeyboardInterrupt()])
def test_worker_context_closes_resources_on_success_failure_and_cancel(monkeypatch,failure):
    closed=[]
    def build(stack):
        stack.callback(closed.append,'planner')
        stack.callback(closed.append,'simulator')
        return integration.RuntimeSession(RuntimeStub(failure),['grasp'],lambda snapshot: True)
    monkeypatch.setattr(integration,'_FACTORIES',{'pickplace':lambda *args:integration.RuntimeContext(build)})
    args=({'task':'pickplace','mode':'sim'},{'swm':{'enabled':True}},None,None,SimpleNamespace(check=lambda:None),None)
    if failure:
        with pytest.raises(type(failure)):
            integration.dispatch_if_enabled(*args)
    else:
        assert integration.dispatch_if_enabled(*args)['task_success'] is True
    assert closed==['simulator','planner']


def test_partial_initialization_cleanup_and_context_not_reusable():
    closed=[]
    def build(stack):
        stack.callback(closed.append,'first resource')
        raise RuntimeError('second acquisition failed')
    context=integration.RuntimeContext(build)
    with pytest.raises(RuntimeError,match='acquisition'):
        with context:pass
    assert closed==['first resource']
    with pytest.raises(RuntimeError,match='reused'):
        with context:pass


def test_preview_never_allocates_native_resources(monkeypatch):
    def forbidden(*args):raise AssertionError('must not initialize')
    monkeypatch.setattr(integration,'_FACTORIES',{'pickplace':forbidden})
    with pytest.raises(RuntimeError,match='compile-only'):
        integration.dispatch_if_enabled({'task':'pickplace','mode':'preview'}, {'swm':{'enabled':True}},None,None,None,None)
