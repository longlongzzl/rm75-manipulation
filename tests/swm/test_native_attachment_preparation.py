"""Preparation ownership/error contracts; fixtures do not qualify native fitting."""
import hashlib
from types import SimpleNamespace
import pytest
from rm75_app.swm.native_context import _prepare_pen_attachment, _PenCheckpointStore
from rm75_app.swm.scene import SceneInvalid
from rm75_app.workcell.events import StopToken, Cancelled


def make_case(tmp_path):
    path=tmp_path/'metric.obj'
    path.write_text('fixture metric asset')
    snapshot=dict(valid=True,observation_domain='physics',snapshot_id='measured',
        robot=dict(idle=True,holding='empty',joint_names=[f'joint_{i}' for i in range(1,8)],positions=[0.]*7),
        objects={'bi':dict(asset_id='pen'),'table':{}},assets={'pen':dict(mesh_path=str(path),
            mesh_sha256=hashlib.sha256(path.read_bytes()).hexdigest())})
    calls=[]
    backend=SimpleNamespace(_scene=SimpleNamespace(revision='measured',objects=[
        SimpleNamespace(name='bi',metadata=dict(visual_mesh_path=str(path),visual_mesh_scale=[1.,1.,1.])),
        SimpleNamespace(name='table')]),
        _attachment_fit_report=dict(object_name='bi',fit_type='morphit',cache_file='fixture.npz',fitted_spheres=4),
        attach_object=lambda *args:calls.append('attach'),detach_object=lambda *args:calls.append('detach'))
    def restore(observed):
        assert observed is snapshot
        calls.append('restore_complete_scene')
        return observed['snapshot_id']
    port=SimpleNamespace(backend=backend,apply_idle_snapshot=restore)
    events=SimpleNamespace(emit=lambda *args,**kwargs:calls.append('emit'))
    return snapshot,backend,port,StopToken(),events,calls


def test_prepare_uses_original_fitter_then_restores_before_reporting(tmp_path):
    snapshot,backend,port,stop,events,calls=make_case(tmp_path)
    _prepare_pen_attachment(backend,port,snapshot,stop,events)
    assert calls==['attach','detach','restore_complete_scene','emit']
    assert snapshot['robot']['holding']=='empty'


@pytest.mark.parametrize('failure',[RuntimeError('fit failed'),Cancelled('cancelled')])
def test_fit_failure_restores_without_emitting_success(tmp_path,failure):
    snapshot,backend,port,stop,events,calls=make_case(tmp_path)
    def fail(*args):
        calls.append('attach')
        raise failure
    backend.attach_object=fail
    with pytest.raises(type(failure)):
        _prepare_pen_attachment(backend,port,snapshot,stop,events)
    assert calls==['attach','detach','restore_complete_scene']


def test_mismatched_asset_rejected_before_model_changes(tmp_path):
    snapshot,backend,port,stop,events,calls=make_case(tmp_path)
    snapshot['assets']['pen']['mesh_sha256']='changed'
    with pytest.raises(SceneInvalid,match='same registered'):
        _prepare_pen_attachment(backend,port,snapshot,stop,events)
    assert calls==[]


def test_stop_before_preparation_never_changes_model(tmp_path):
    snapshot,backend,port,stop,events,calls=make_case(tmp_path)
    stop.request()
    with pytest.raises(Cancelled):
        _prepare_pen_attachment(backend,port,snapshot,stop,events)
    assert calls==[]


def test_wrong_restoration_acknowledgement_rejects(tmp_path):
    snapshot,backend,port,stop,events,calls=make_case(tmp_path)
    port.apply_idle_snapshot=lambda snapshot:'wrong'
    with pytest.raises(SceneInvalid,match='restoration'):
        _prepare_pen_attachment(backend,port,snapshot,stop,events)
    assert calls==['attach','detach']


def test_checkpoint_evidence_is_bounded(tmp_path):
    store=_PenCheckpointStore(tmp_path)
    world=SimpleNamespace(snapshot=lambda:dict(revision=store.count+1,valid=True))
    for _ in range(32):store.save(world)
    with pytest.raises(SceneInvalid,match='budget'):store.save(world)
