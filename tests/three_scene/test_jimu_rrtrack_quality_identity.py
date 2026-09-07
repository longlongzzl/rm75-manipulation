from types import SimpleNamespace as NS
import numpy as np
import pytest

from rm75_app.perception.rrtrack.scene_registry import build_scene_registry,resolve_active_result
from rm75_app.perception.rrtrack.tracker import RRTracker
from rm75_app.perception.rrtrack.models import FrameObservation,SegmentationPrediction,PoseEstimate


def test_twelve_same_class_instances_do_not_shift_identity_after_failed_detections():
    rows=[dict(object_name='orange_jimu_plate',ok=i not in (0,3,8),score=.9,
               T_cam_obj=np.eye(4).tolist(),run_dir=f'/tmp/instance_{i}') for i in range(12)]
    payload={'results':rows}
    for index,row in enumerate(rows):
        if not row['ok']:
            with pytest.raises(ValueError,match='initialization failed'):
                resolve_active_result(payload,'orange_jimu_plate',index)
            with pytest.raises(ValueError,match='initialization failed'):
                build_scene_registry(payload,'orange_jimu_plate',index)
        else:
            assert resolve_active_result(payload,'orange_jimu_plate',index) is row
            registry=build_scene_registry(payload,'orange_jimu_plate',index)
            assert registry['active_instance_id']==f'orange_jimu_plate:{index}'
            assert len(registry['objects'])==12
            assert registry['objects'][0]['state']=='initialization_failed'


def tracker_fixture():
    mask=np.zeros((20,30),bool);mask[5:15,10:20]=True;calls=[]
    def initialize(rgb,observed):
        calls.append('initialize');return SegmentationPrediction(observed,observed.astype(float))
    def refine(*args):calls.append('refine');return PoseEstimate(np.eye(4))
    def register(*args):calls.append('register');return PoseEstimate(np.eye(4),source='test_register')
    tracker=RRTracker(segmenter=NS(initialize=initialize,predict=lambda rgb:SegmentationPrediction(mask,mask.astype(float)),
        inject=lambda *a,**k:calls.append('inject'),clear_non_permanent_memory=lambda:calls.append('clear')),
        pose_refiner=NS(refine=refine,global_register=register),renderer=NS(render=lambda *a:mask.copy()))
    def frame(index,depth=None):return FrameObservation(np.zeros((20,30,3),np.uint8),
        np.ones((20,30),float) if depth is None else depth,np.eye(3),index)
    return tracker,mask,calls,frame


def test_low_depth_frame_does_not_refine_write_memory_or_accept_then_can_recover():
    tracker,mask,calls,frame=tracker_fixture();tracker.initialize(frame(0),mask,np.eye(4))
    calls.clear()
    lost=tracker.step(frame(1,np.zeros(mask.shape)))
    assert not lost.accepted and lost.event=='insufficient_depth'
    assert not calls and not lost.memory_updates
    waiting=tracker.step(frame(2,np.zeros(mask.shape)))
    assert not waiting.accepted and waiting.metadata['depth_rejected_candidates']==1
    assert not calls
    recovered=tracker.step(frame(3))
    assert recovered.accepted and recovered.event=='recovered'
    assert 'register' in calls


def test_under_supported_initialization_fails_before_cutie_memory_is_written():
    tracker,mask,calls,frame=tracker_fixture();depth=np.zeros(mask.shape)
    ys,xs=np.where(mask);depth[ys[:21],xs[:21]]=1.
    with pytest.raises(ValueError,match='insufficient depth'):
        tracker.initialize(frame(0,depth),mask,np.eye(4))
    assert not calls and tracker.pose is None
