import numpy as np
import pytest
from rm75_app.workcell.verification import wait_for_poses
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.events import StopToken
@pytest.mark.parametrize('offset,expected',[(0.,True),(.03,False)])
def test_final_verification_uses_measurements_not_expected_targets(tmp_path,offset,expected):
    import time
    target=np.eye(4);measured=target.copy();measured[0,3]=offset;path=tmp_path/'measurement.json';after=time.time()-.05;atomic_json(path,{'schema':'rm75_object_observations_v1','frame':'base_link','source':'live_tracker','session_id':'test','sequence':1,'captured_at':time.time(),'objects':{'block':{'T_base_object':measured.tolist(),'confidence':1}}});result=wait_for_poses({'target_T_base_objects':{'block':target.tolist()},'observation_file':str(path)},after=after,stop=StopToken());assert result['task_success'] is expected and result['magnetic_force_verified'] is False
def test_old_final_verification_is_not_retimestamped(tmp_path):
    import time
    path=tmp_path/'measurement.json';atomic_json(path,{'schema':'rm75_object_observations_v1','frame':'base_link','source':'live_tracker','session_id':'test','sequence':1,'captured_at':time.time()-10,'objects':{}});result=wait_for_poses({'target_T_base_objects':{'block':np.eye(4).tolist()},'observation_file':str(path),'timeout_s':.1},after=time.time(),stop=StopToken());assert result['task_success'] is None
