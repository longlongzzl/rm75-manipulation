"""Bind original native push feedback to existing SWM transition contracts."""
from dataclasses import replace
import time
import numpy as np
from rm75_app.swm.feedback import MeasuredFeedbackRecorder
from rm75_app.swm.skills import ExecutionReceipt,SkillRequest
from rm75_app.workcell.io import atomic_json
from .model import predict
from .native_tool_recording import bind_tool_motion


class PushTActionRecording:
    def __init__(self,session):
        self.session=session;self.handle=None;self.receipt=None
        self.native_rows=[];self.last_native_row=None
        self.recorder=MeasuredFeedbackRecorder(domain='physics',sensor_session=session.session,
            calibration_id=session.swm_capture.calibration)

    def sample(self,feedback=None):
        row=dict(self.session.measured_tool_feedback() if feedback is None else feedback)
        row['calibration_id']=self.session.swm_capture.calibration
        if row['stage']=='settle':
            row['native_stage']='settle';row['stage']='post_settle'
        self.recorder.append(row)
        native=dict(simulation_clock=row['simulation_clock'],T_world_tcp=row['T_world_tcp'],
            link_poses=row['native_tool_link_poses'],gripper_joint_positions=row['gripper_joint_positions'])
        self.last_native_row=native
        if self.handle is not None:self.native_rows.append(native)

    def begin(self,snapshot,push):
        if self.handle is not None:raise RuntimeError('Previous push recording still active')
        support=snapshot['objects'].get('simulation_table')
        if support is None or not support['fixed']:raise ValueError('Original measured support required')
        self.initial=snapshot
        pose=np.asarray(snapshot['objects']['dynamic_T']['measured']['T_world_object'])
        planar=[pose[0,3],pose[1,3],np.arctan2(pose[1,0],pose[0,0])]
        binding=self.session._prepared_contact_binding
        bound=replace(push,contact=tuple(binding['surface_contact_xyz'][:2]))
        goal=predict(planar,bound,self.session.config)
        target=np.eye(4);c,s=np.cos(goal[2]),np.sin(goal[2])
        target[:2,:2]=[[c,-s],[s,c]];target[:3,3]=[goal[0],goal[1],pose[2,3]]
        self.request=SkillRequest('push','dynamic_T',target=target)
        self.started=time.monotonic()
        self.handle=self.recorder.begin_action(self.session.feedback_action_id,self.started,
            support_id='simulation_table',initially_settled=True,
            settling_evidence=dict(source='native_idle_no_tool_contact_checkpoint',
                snapshot_id=snapshot['snapshot_id'],simulation_clock=snapshot['robot']['simulation_clock']))
        self.native_rows=[self.last_native_row]

    def finish_command(self):
        if self.handle is None or self.receipt is not None:
            raise RuntimeError('Push command recording is not awaiting completion')
        receipt=ExecutionReceipt(True,self.started,time.monotonic(),True,self.handle.actual_action_id)
        self.receipt=self.recorder.finish_command(self.handle,receipt)

    def complete(self,snapshot):
        transition=self.recorder.transition(self.request,self.receipt,self.initial,snapshot)
        self.handle=None;self.receipt=None
        if transition is None:raise RuntimeError('Completed native push was not eligible for transition binding')
        motion=bind_tool_motion(transition,self.session.native_tool_geometry,self.native_rows)
        atomic_json(self.session.swm_capture.directory/f'tool_motion_{transition["action_id"]}.json',motion)
        self.native_rows=[]
        path=self.session.swm_capture.directory/f'transition_{transition["action_id"]}.json'
        atomic_json(path,transition)
        self.session.events.emit('swm_native_push_transition',action_id=transition['action_id'],
            transition_digest=transition['transition_digest'],file=str(path),
            tool_samples=len(transition['actual_action']['time_s']),
            physics_fit_run=False,mirror_acknowledged=False)
        return transition

    def cancel(self):
        if self.handle is not None:self.handle.close()
        self.handle=None;self.receipt=None
        self.native_rows=[]

    def close(self):
        self.cancel();self.recorder.close()
