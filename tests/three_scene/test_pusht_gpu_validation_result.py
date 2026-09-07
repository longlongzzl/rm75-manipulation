import pytest
from tools.run_pusht_gpu_validation import validation_passed


def report():
    return dict(complete_chain=True,config={'speed_mps':.015},
        motion={'joint_speed_rad_s':.25,'joint_accel_rad_s2':.5},
        stages=[dict(stage=name,max_tcp_speed_mps=.014,max_joint_speed_rad_s=.2,
                     max_joint_accel_rad_s2=.4)
                for name in ('approach','descend','contact','push','retreat')])


def test_all_stages_and_requested_negative_gate_are_required():
    data=report();assert validation_passed(data)
    assert not validation_passed(data,audit_blocker=True)
    data['unrelated_obstacle_audit']={'rejected':True}
    assert validation_passed(data,audit_blocker=True)
    data['stages'].pop();assert not validation_passed(data)


def test_exception_after_planning_is_not_a_validation_success():
    data=report();data['error']='post-planning diagnostic failed'
    assert not validation_passed(data)


@pytest.mark.parametrize('key,value',[
    ('max_tcp_speed_mps',.016),('max_joint_speed_rad_s',.26),
    ('max_joint_accel_rad_s2',.51),('max_tcp_speed_mps',float('nan')),
    ('max_tcp_speed_mps',-1),('max_joint_speed_rad_s',float('inf'))])
def test_measured_dynamics_fail_closed(key,value):
    data=report();data['stages'][2][key]=value
    assert not validation_passed(data)
