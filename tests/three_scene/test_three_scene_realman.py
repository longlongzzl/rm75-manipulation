import types
import numpy as np
import pytest
from rm75_app.workcell.realman import RealManArm,time_parameterize
from rm75_app.workcell.events import EventLog,StopToken
class SDK:
    def __init__(self):self.q=[0.]*7;self.sent=[];self.stopped=0;self.fail=False;self.closed=False
    def rm_create_robot_arm(self,*a):return types.SimpleNamespace(id=1)
    def rm_get_arm_run_mode(self):return 0,1
    def rm_get_joint_degree(self):return 0,self.q
    def rm_movej_canfd(self,q,follow):self.sent.append((q,follow));self.q=q;return -1 if self.fail else 0
    def rm_set_arm_stop(self):self.stopped+=1;return 0
    def rm_delete_robot_arm(self):self.closed=True;return 0
def config():return {'hardware_reviewed':True,'qualification_id':'FAKE_ONLY','ip':'not-networked','joint_lower_rad':[-3]*7,'joint_upper_rad':[3]*7,'stream_hz':30}
def test_sdk_units_and_low_follow(tmp_path):
    sdk=SDK();arm=RealManArm(config(),StopToken(),EventLog(tmp_path),sdk=sdk);path=np.array([[0]*7,[.01]*7]);arm.execute(path,[0,.04],stage='fake');assert np.allclose(sdk.sent[-1][0],np.rad2deg(path[-1])) and sdk.sent[-1][1] is False;arm.close()
def test_failure_latches_and_stops_without_gripper_or_home(tmp_path):
    sdk=SDK();sdk.fail=True;arm=RealManArm(config(),StopToken(),EventLog(tmp_path),sdk=sdk)
    with pytest.raises(RuntimeError):arm.execute(np.array([[0]*7,[.01]*7]),[0,.04],stage='fake')
    assert sdk.stopped and arm.faulted
@pytest.mark.parametrize('speed',[.005,.01,.02])
def test_speed_mps_changes_timing_and_is_bounded(speed):
    path=np.array([[0]*7,[.015,0,0,0,0,0,0],[.03,.005,0,0,0,0,0]]);fk=lambda q:np.array([q[0],q[1],.2]);emitted,t=time_parameterize(path,fk,speed_mps=speed);cart=np.linalg.norm(np.diff(np.array([fk(q) for q in emitted]),axis=0),axis=1)/np.diff(t);assert cart.max()<=speed+1e-9 and np.allclose(emitted[-1],path[-1])
