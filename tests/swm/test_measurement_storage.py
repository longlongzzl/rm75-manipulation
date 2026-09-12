import copy
from types import SimpleNamespace
import json
import numpy as np
import pytest
from .conftest import pose
from .test_identification_program import transition,Replay,hypotheses
from rm75_app.swm.scene import SceneInvalid,digest
from rm75_app.swm.skills import SkillRequest,ExecutionReceipt
from rm75_app.swm.storage import SnapshotStore
from rm75_app.swm.measurements import bind_measured_transition
from rm75_app.swm.identification import (AdaptivePhysicsManager,IsolatedReplayPool,ParameterBounds,
    validate_transition,infer_posterior)


def recording(rig):
    def fixture_source(batch):
        for row in batch['objects']:row['source']='fixture'
    rig.source.mutate=fixture_source
    before=rig.sync.sync('before_push');t0=before['robot']['captured_at']
    start=rig.clock();rig.clock.tick(.8);end=rig.clock()
    rig.source.poses['a']=pose(.35,z=.03)
    after=rig.sync.sync('after_push');t1=after['objects']['a']['measured']['captured_at']
    trace=dict(schema='rm75_swm_action_recording_v1',actual_action_id='recorded-action',source='measured_feedback',
       domain='fixture',sensor_session='camera-session',calibration_id='extrinsics-v1',
       tool_captured_at=[t0,t0+.4,t1],T_world_tcp=[before['robot']['T_world_tcp'],pose(.3,z=.2),pose(.3,z=.2)],
       stages=['push','push','post_settle'],object_samples=[],support_id='table',
       initially_settled=True,settling_evidence='observed-static-fixture',intervened=False)
    return before,after,trace,ExecutionReceipt(True,start,end,True,'recorded-action',trace)


def test_sparse_measurements_use_actual_tool_feedback_through_final_frame(rig):
    before,after,trace,receipt=recording(rig)
    result=bind_measured_transition(SkillRequest('push','a',pose(.35,z=.03)),receipt,before,after,recording=trace)
    assert result['observation_mode']=='endpoint_only' and result['final_snapshot_id']==after['snapshot_id']
    assert result['T_world_object'][0]==before['objects']['a']['measured']['T_world_object']
    assert result['T_world_object'][-1]==after['objects']['a']['measured']['T_world_object']
    assert result['actual_action']['time_s'][-1]==result['object_time_s'][-1]
    assert validate_transition(result)==result  # canonicalization is idempotent


@pytest.mark.parametrize('fault',['commanded','horizon','clock','source','identity'])
def test_recording_cannot_be_filled_with_predicted_samples(rig,fault):
    before,after,trace,receipt=recording(rig)
    if fault=='commanded':trace['source']='planned_commands'
    if fault=='horizon':trace['tool_captured_at'][-1]=receipt.ended_at
    if fault=='clock':trace['tool_captured_at'][0]+=.1
    if fault=='source':trace['sensor_session']='another-camera-session'
    if fault=='identity':trace['actual_action_id']='another-action'
    with pytest.raises(ValueError):bind_measured_transition(SkillRequest('push','a',pose(.35,z=.03)),receipt,before,after,recording=trace)


def test_external_move_does_not_calibrate(rig):
    before,after,trace,receipt=recording(rig);trace['intervened']=True
    assert bind_measured_transition(SkillRequest('push','a',pose(.35)),receipt,before,after,recording=trace) is None


def test_no_free_body_inference_for_constraint_pull(rig):
    before,after,trace,receipt=recording(rig)
    assert bind_measured_transition(SkillRequest('pull','a',pose(.35),functional_pose_id='side'),receipt,before,after,recording=trace) is None


def test_physics_manager_updates_then_resamples_but_never_replays_old_plan(rig):
    before,after,trace,receipt=recording(rig)
    manager=AdaptivePhysicsManager(rig.world,IsolatedReplayPool(lambda r:Replay(r,[])),count=8,seed=12,
        transition_builder=lambda request,receipt,a,b:bind_measured_transition(request,receipt,a,b,recording=trace))
    result=manager.on_skill(SkillRequest('push','a',pose(.35,z=.03)),receipt,initial_snapshot=before,final_snapshot=after)
    assert result['posterior']['updated'] and rig.world.physics_revision==1
    assert result['replan_from_current_measurement'] and len(result['next_hypotheses'])==8
    assert not result['posterior']['unique_parameters_identified']
    with pytest.raises(SceneInvalid):manager.on_skill(SkillRequest('push','a',pose(.35)),receipt,initial_snapshot=before,final_snapshot=after)


def test_no_recorder_is_not_automatic_physics_success(rig):
    before,after,trace,receipt=recording(rig);events=[]
    manager=AdaptivePhysicsManager(rig.world,IsolatedReplayPool(lambda r:Replay(r,[])),emit=lambda **kw:events.append(kw))
    assert manager.on_skill(SkillRequest('push','a',pose(.35)),receipt,initial_snapshot=before,final_snapshot=after) is None
    assert rig.world.physics_revision==0 and events[0]['kind']=='swm_physics_fit_skipped'


def test_storage_restart_never_restores_a_live_execution_permission(rig,tmp_path):
    rig.sync.sync('before_grasp');rig.source.holding='a';rig.sync.sync('after_grasp')
    original=rig.world.snapshot();store=SnapshotStore(tmp_path/'swm.json');assert store.save(rig.world)==original['snapshot_id']
    restored=store.restore(rig.manifest);current=restored.snapshot()
    assert not current['valid'] and current['robot'] is None and current['sensor_session'] is None
    assert all(x['measured'] is None for x in current['objects'].values())
    assert json.loads(store.path.read_text())['snapshot']['robot']['holding']=='a'  # history retained only as evidence


def test_storage_belief_retained_only_for_same_models(rig,tmp_path):
    data=transition(rig);_,rows=IsolatedReplayPool(lambda r:Replay(r,[])).run(data,hypotheses())
    posterior=infer_posterior(data,hypotheses(),rows);rig.world.update_physics('a',posterior,expected_physics_revision=0)
    store=SnapshotStore(tmp_path/'swm.json');store.save(rig.world);restored=store.restore(rig.manifest)
    assert restored.physics_revision==1 and restored.snapshot()['physics']['a']==posterior
    altered=copy.deepcopy(rig.manifest);altered['calibration_id']='moved-camera'
    with pytest.raises(SceneInvalid):store.restore(altered)


def test_storage_modified_state_rejected(rig,tmp_path):
    rig.sync.sync('initial');store=SnapshotStore(tmp_path/'swm.json');store.save(rig.world)
    data=json.loads(store.path.read_text());data['snapshot']['objects']['a']['measured']['T_world_object'][0][3]+=1
    store.path.write_text(json.dumps(data))
    with pytest.raises(SceneInvalid):store.restore(rig.manifest)
