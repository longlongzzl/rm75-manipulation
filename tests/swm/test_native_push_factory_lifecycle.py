"""Failure ownership checks; these are not native motion qualification."""
import subprocess
import sys
from pathlib import Path
import pytest
from rm75_app.swm.native_push_hypotheses import NativePushHypothesisFactory


@pytest.fixture
def factory(tmp_path):
    urdf=tmp_path/'unused.urdf';urdf.write_text('Not loaded by lifecycle fixtures')
    owner=NativePushHypothesisFactory({}, {}, urdf,python=sys.executable,directory=tmp_path)
    yield owner
    owner.close()


@pytest.mark.parametrize('error',[RuntimeError('failed native preparation'),KeyboardInterrupt()])
def test_partial_preparation_closes_transaction(factory,monkeypatch,error):
    directory=factory.directory;sibling=directory.parent/'unrelated';sibling.write_text('keep')
    def fail(*args):
        factory.key='partial';factory.candidates.append({'partial':True})
        (directory/'partial.json').write_text('{}')
        raise error
    monkeypatch.setattr(factory,'_prepare',fail)
    context=factory();other=factory()
    with pytest.raises(type(error)):
        context.solve_candidates(None,None,{})
    assert factory.closed and not directory.exists() and not factory.candidates
    assert sibling.read_text()=='keep'
    with pytest.raises(RuntimeError,match='closed'):factory()
    with pytest.raises(RuntimeError,match='closed'):other.solve_candidates(None,None,{})


def test_prediction_failure_discards_all_candidates(factory,monkeypatch):
    def prepared(*args):factory.candidates.append({'partial':False})
    def fail(*args):raise ValueError('native prediction rejected')
    monkeypatch.setattr(factory,'_prepare',prepared);monkeypatch.setattr(factory,'_predict',fail)
    with pytest.raises(ValueError,match='prediction rejected'):
        factory().solve_candidates(None,None,{})
    assert factory.closed and not factory.directory.exists() and not factory.candidates


def test_evaluation_failure_closes_transaction(factory,monkeypatch):
    factory.request=None
    def fail(*args):raise ValueError('audit scene rejected')
    monkeypatch.setattr(factory,'_prepare',fail)
    with pytest.raises(ValueError,match='audit scene rejected'):
        factory().evaluate(None,None,{})
    assert factory.closed and not factory.directory.exists()


def test_cancel_before_native_initialization_closes(factory):
    def stop():raise InterruptedError('requested cancellation')
    factory.check=stop
    with pytest.raises(InterruptedError):factory().solve_candidates(None,None,{})
    assert factory.closed and not factory.directory.exists()


def test_compile_cancel_reaps_owned_os_child(factory,monkeypatch):
    # Run an actual isolated OS child, not GPU/PhysX or a robot process.
    original_popen=subprocess.Popen;children=[]
    root=Path(__file__).resolve().parents[2]
    def launch(command,**kwargs):
        child=original_popen([sys.executable,str(root/'tools/run_network_isolated.py'),'--',
            sys.executable,'-c','import time; time.sleep(60)'],**kwargs)
        children.append(child);return child
    monkeypatch.setattr(subprocess,'Popen',launch)
    def stop():
        if children:raise InterruptedError('cancel owned compile child')
    factory.check=stop
    def compile_only(*args):factory._compile(0,{}, {})
    monkeypatch.setattr(factory,'_prepare',compile_only)
    with pytest.raises(InterruptedError,match='owned compile child'):
        factory().solve_candidates(None,None,{})
    assert len(children)==1 and children[0].poll() is not None
    assert factory.closed and not factory.directory.exists()
