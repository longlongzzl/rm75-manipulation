import pytest
import time
from types import SimpleNamespace
import numpy as np
from rm75_app.pusht.motion import CuroboPushExecutor, PreparedPush
from rm75_app.pusht.model import Config
from rm75_app.pusht.observation import Observation
from tools.run_pusht_gpu_validation import validation_passed, audit_execution_gates, EXECUTION_GATE_CASES


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


def gate_fixture():
    executor = CuroboPushExecutor.__new__(CuroboPushExecutor)
    executor.config = Config()
    executor.arm = object()
    executor.observer = object()
    executor.stop = SimpleNamespace(check=lambda: None)
    executor.events = SimpleNamespace(emit=lambda *args, **kwargs: None)
    observation = Observation('injected_unit_test', 1, time.time(), (.35, -.18, 0.), 'simulation')
    return executor, PreparedPush((), np.zeros(7)), observation


def test_injected_execution_gates_reject_before_any_execute_and_preserve_executor():
    executor, prepared, observation = gate_fixture()
    arm, observer = executor.arm, executor.observer
    rows = audit_execution_gates(executor, prepared, None, observation)
    assert [row['case'] for row in rows] == list(EXECUTION_GATE_CASES)
    assert all(row['passed'] and row['execute'] == 0 for row in rows)
    assert all(row['observe'] == row['plan'] == 1 for row in rows)
    assert executor.arm is arm and executor.observer is observer
    assert 'plan_push' not in executor.__dict__
    assert observation.source == 'simulation'
    np.testing.assert_array_equal(prepared.start_q, np.zeros(7))


@pytest.mark.parametrize('unexpected_execute', [False, True])
def test_gate_audit_does_not_count_unrelated_exception_or_execute_as_rejection(monkeypatch, unexpected_execute):
    executor, prepared, observation = gate_fixture()
    def broken(self, *args):
        if unexpected_execute:
            self.arm.execute()
        raise RuntimeError('unrelated failure')
    monkeypatch.setattr(CuroboPushExecutor, 'execute_push', broken)
    rows = audit_execution_gates(executor, prepared, None, observation)
    assert not any(row['passed'] or row['rejected'] for row in rows)
    assert all(row['execute'] == int(unexpected_execute) for row in rows)


def test_required_execution_gates_need_complete_unique_case_list_and_no_execute():
    data = report()
    assert not validation_passed(data, audit_gates=True)
    executor, prepared, observation = gate_fixture()
    data['execution_gate_audits'] = audit_execution_gates(executor, prepared, None, observation)
    assert validation_passed(data, audit_gates=True)
    data['execution_gate_audits'][-1]['execute'] = 1
    assert not validation_passed(data, audit_gates=True)
    data['execution_gate_audits'][-1]['execute'] = 0
    data['execution_gate_audits'][-1]['case'] = EXECUTION_GATE_CASES[0]
    assert not validation_passed(data, audit_gates=True)
