"""Bounded candidate generation through the original PushT native search."""
import numpy as np
from .scene import SceneInvalid,digest
from rm75_app.pusht.observation import Observation
from rm75_app.pusht.batch_search import rank_pushes


def generate_original_push_candidates(executor,snapshot,goal, *, count=2,proposal_limit=32):
    if not 1<=count<=4 or not count<=proposal_limit<=32:raise ValueError('Bounded original candidate budget required')
    canonical=dict(snapshot);sid=canonical.pop('snapshot_id')
    if digest(canonical)!=sid or not snapshot['valid'] or snapshot['robot']['idle'] is not True:
        raise SceneInvalid('Original candidate generation requires an immutable idle snapshot')
    dynamic=[oid for oid,obj in snapshot['objects'].items() if not obj['fixed']]
    if len(dynamic)!=1:raise SceneInvalid('Original planar candidate compiler requires exactly one moving target')
    measured=snapshot['objects'][dynamic[0]]['measured'];matrix=np.asarray(measured['T_world_object'])
    observation=Observation(snapshot['sensor_session'],measured['sequence'],measured['captured_at'],
        (float(matrix[0,3]),float(matrix[1,3]),float(np.arctan2(matrix[1,0],matrix[0,0]))),'simulation')
    mask=executor.contact_direction_mask(observation,executor.config)
    proposals=rank_pushes(observation.pose,goal,executor.config,limit=proposal_limit,contact_mask=mask)
    candidates=[];selected=set()
    while proposals and len(candidates)<count:
        executor.stop.check()
        index,prepared=executor.plan_push_candidates(proposals,observation)
        push=proposals[index][0]
        push_data=dict(contact=list(map(float,push.contact)),direction=list(map(float,push.direction)),
            normal=list(map(float,push.normal)),length_m=float(push.length_m),speed_mps=float(push.speed_mps))
        identity=digest(push_data)
        if identity in selected:raise SceneInvalid('Original candidate search repeated a selected action')
        selected.add(identity)
        if np.max(np.abs(prepared.start_q-np.asarray(snapshot['robot']['positions'])))>1e-5:
            raise SceneInvalid('New candidate starts from another measured joint state')
        candidate=dict(complete_chain=True,validation_success=True,execute_real=False,hardware_connected=False,
            source_snapshot_id=sid,source_observation=observation.as_dict(),selected_push=push_data,
            contact_binding=executor.last_contact_binding,
            stages=[dict(stage=name,positions=np.asarray(q).tolist(),times=np.asarray(t).tolist()) for name,q,t in prepared.stages],
            source='original_rank_pushes_and_native_plan_push_candidates',candidate_action_digest=identity,
            validation_scope='original_planner_candidate_not_complete_SWM_execution_audit',
            execution_authorized=False,physics_prediction_run=False)
        candidates.append(candidate)
        proposals=[row for row in proposals if row[0]!=push]
    if len(candidates)!=count:raise SceneInvalid('Insufficient distinct native candidates within original budget')
    return candidates
