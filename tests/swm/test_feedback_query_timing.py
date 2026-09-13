"""No SDK, devices, commands or stop calls: pure shared feedback callback tests."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.execution.realman_executor import RealManTrajectoryExecutor, RealManHardwareError


def sink(times, observer, q=None):
    calls = []
    def read():
        calls.append('read')
        return np.full(7, .012) if q is None else q
    session = SimpleNamespace(config=SimpleNamespace(joint_names=tuple(f'joint_{i}' for i in range(1,8))),
        read_joint_radians=read)
    clock = iter(times)
    return RealManTrajectoryExecutor(session, clock_fn=lambda: next(clock), feedback_observer=observer), calls


def test_existing_single_query_records_host_timing_not_device_timestamp():
    rows = []
    executor, calls = sink([10., 10.02, 10.025], rows.append)
    executor._record_actual_feedback()
    assert calls == ['read']
    row = rows[0]
    assert row['positions'] == [.012]*7
    assert row['captured_at'] == 10.02
    assert row['query_duration_s'] == pytest.approx(.02)
    assert not row['device_sample_time_known']
    assert row['timestamp_semantics'] == 'host_query_completion_not_device_sample_time'
    assert executor.last_feedback_timing['observer_duration_s'] == pytest.approx(.005)
    assert executor.last_feedback_timing['synchronous_feedback_duration_s'] == pytest.approx(.025)
    assert not executor.last_feedback_timing['hardware_frequency_qualified']


def test_observer_failure_preserves_original_error_and_records_elapsed_time():
    def failed(row): raise RuntimeError('recorder unavailable')
    executor, calls = sink([1., 1.1, 1.2], failed)
    with pytest.raises(RuntimeError, match='recorder unavailable'): executor._record_actual_feedback()
    assert calls == ['read']
    assert executor.last_feedback_timing['observer_delivered'] is False
    assert executor.last_feedback_timing['observer_duration_s'] == pytest.approx(.1)


@pytest.mark.parametrize('times,q', [([2.,1.],np.zeros(7)), ([1.,2.],np.full(7,np.nan)),
                                    ([1.,2.],np.zeros(6))])
def test_invalid_feedback_or_query_clock_never_reaches_recorder(times,q):
    rows = []
    executor, calls = sink(times, rows.append,q)
    with pytest.raises(RealManHardwareError): executor._record_actual_feedback()
    assert not rows and calls == ['read']


def test_no_observer_means_no_added_query_or_clock_read():
    executor, calls = sink([],None)
    executor._record_actual_feedback()
    assert calls == [] and executor.last_feedback_timing is None
