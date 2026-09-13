"""Software fixtures, not a native physics qualification."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.pusht.physics import PhysicsSession

class NativeArray:
    def __init__(self, value):self.value=np.asarray(value)
    def detach(self):return self
    def cpu(self):return self
    def numpy(self):return self.value

def session():
    value=PhysicsSession.__new__(PhysicsSession)
    value.full_arm=True;value.session='fixture';value.feedback_action_id='actual-action'
    q=np.arange(13,dtype=float)/100
    pose=np.eye(4);pose[0,3]=.123
    value.base=SimpleNamespace(
        robot=SimpleNamespace(get_qpos=lambda:NativeArray(q)),
        tcp=SimpleNamespace(pose=SimpleNamespace(to_transformation_matrix=lambda:NativeArray(pose))),
        arm_indices=list(range(7)),joint_names=[f'joint_{i+1}' for i in range(7)],
        gripper_indices={f'jaw_{i}':i+7 for i in range(6)},stage='push',
        commanded_q=np.ones(7)*99)
    value.simulation_clock=SimpleNamespace(read=lambda:dict(epoch='fixture',physical_time_s=.5))
    return value

def test_reads_native_tcp_and_joints_not_commands():
    row=session().measured_tool_feedback()
    assert row['joint_positions']==pytest.approx(np.arange(7)/100)
    assert row['T_world_tcp'][0][3]==.123
    assert len(row['gripper_joint_positions'])==6
    assert row['actual_action_id']=='actual-action'
    assert row['simulation_clock']['physical_time_s']==.5

def test_kinematic_surrogate_not_labeled_measured():
    value=session();value.full_arm=False
    assert value.measured_tool_feedback() is None

def test_changing_clock_rejects_read():
    value=session();clocks=iter([{'step':1},{'step':2}])
    value.simulation_clock.read=lambda:next(clocks)
    with pytest.raises(RuntimeError,match='changed during feedback'):
        value.measured_tool_feedback()

def test_nonfinite_native_feedback_rejects():
    value=session();value.base.robot.get_qpos=lambda:NativeArray([np.nan]*13)
    with pytest.raises(RuntimeError,match='Nonfinite'):
        value.measured_tool_feedback()
