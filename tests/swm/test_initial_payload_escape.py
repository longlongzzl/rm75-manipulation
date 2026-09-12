"""Original bounded support escape, never an unconditional collision waiver."""
from dataclasses import replace
import numpy as np
import pytest
from rm75_app.core.frames import MANISKILL_TABLE_COLLISION_NAME as TABLE
from rm75_app.planning.contracts import JointTrajectory, Pose
from rm75_app.swm.native_audit import CuroboNativeStageAuditor
from rm75_app.swm.native_skills import NativePrimitive, NativeStage, NativeStageState
from rm75_app.swm.scene import SceneInvalid
from rm75_app.swm.skills import PlannedSkill, REQUIRED_AUDITS
from .test_native_stage_audit import Model


def setup(fault=None):
    names=tuple(f'joint_{i}' for i in range(1,8)); identity=np.eye(4)
    snapshot=dict(snapshot_id='measured',objects={'a':dict(measured=dict(T_world_object=identity.tolist())),
        TABLE:dict(fixed=True), 'b':dict(fixed=False)},robot=dict(idle=True,gripper_closed=False,
        holding='empty',T_world_tcp=identity.tolist(),positions=[-.02]*7,joint_names=names))
    empty=NativeStageState(False,'empty'); held=NativeStageState(True,'a',identity)
    grasp=NativeStage('grasp',JointTrajectory(names,np.array([[-.02]*7,[0.]*7]),dt=.1),True,empty,held,('a',))
    lift=NativeStage('lift',JointTrajectory(names,np.array([[0.]*7,[.02]*7]),dt=.1),
        state_before=held,state_after=held,allow_start_contact_escape=True)
    if fault=='ignore_table': lift=replace(lift,contact_objects=(TABLE,))
    if fault=='discontinuous': lift=replace(lift,trajectory=replace(lift.trajectory,positions=lift.trajectory.positions+.001))
    primitive=NativePrimitive('grasp','a',(grasp,) if fault=='no_lift' else (grasp,lift))
    plan=PlannedSkill('request','measured',primitive.fingerprint(),primitive,identity,'fixture')
    class ContactModel(Model):
        def tool_pose_for_configuration(self,q,frame):
            return Pose([0,0,.01 if fault=='moved_payload' else 0.],[1,0,0,0])
        def _collision_diagnostics_for_states(self,planner,states,**kwargs):
            if self.held=='empty': return []
            rows=[]
            for index,q in enumerate(states['path']):
                depth=(.021 if fault=='excessive' else .0055)-float(q[0])
                if fault=='increasing' and q[0]>.001: depth=.009
                if fault=='uncleared': depth=.0055
                if depth>0:
                    rows.append(dict(candidate_index=index,collision_type='world',
                        robot_link='finger' if fault=='robot_link' else 'attached_object',
                        world_object='b' if fault=='other_object' else TABLE,penetration_m=depth))
            return rows
    model=ContactModel(snapshot)
    model.held = "empty"
    model.closed = False
    model.config.retreat_escape_contact_links=('attached_object','finger')
    model.config.retreat_start_contact_max_penetration_m=.020
    return snapshot,plan,model


def test_support_overlap_requires_complete_immediate_escape_proof():
    snapshot,plan,model=setup(); auditor=CuroboNativeStageAuditor(model)
    assert auditor(plan,snapshot).passed==REQUIRED_AUDITS
    assert auditor.last_evidence[0]['post_transition_contacts']
    assert auditor.last_evidence[0]['post_transition_escape_verified_by']=='lift'
    assert auditor.last_evidence[1]['escape'] is True
    assert model.held=='empty' and model.closed is False


@pytest.mark.parametrize('fault',['no_lift','excessive','robot_link','other_object','increasing',
    'uncleared','moved_payload','ignore_table','discontinuous'])
def test_invalid_contact_transition_never_returns_audit(fault):
    snapshot,plan,model=setup(fault)
    with pytest.raises(SceneInvalid): CuroboNativeStageAuditor(model)(plan,snapshot)
    assert model.held=='empty' and model.closed is False
