from tools.run_workcell_native_validation import native_outcomes
import pytest


def test_prefetch_episode_success_is_not_native_cycle_or_full_chain_success():
    result=native_outcomes('legacy_episode_end command_success=True\ncycle 1 success = True\n',12)
    assert len(result['native_cycles'])==1 and not result['native_full_chain_passed']


@pytest.mark.parametrize('suffix,passed', [
    ('final success = True',True), ('final success = False',False), ('',False),
    ('final success = True\n[warn] post-place clearance planning failed after release; blocked',False),
])
def test_final_marker_and_clearance_warnings_are_independent_gates(suffix,passed):
    text='cycle 1 success = True\ncycle 2 success = True\n'+suffix
    assert native_outcomes(text,2)['native_full_chain_passed'] is passed


def test_duplicate_cycle_does_not_count_as_completion():
    assert not native_outcomes('cycle 1 success = True\ncycle 1 success = True\nfinal success = True',2)['native_full_chain_passed']


def test_original_failed_source_retry_is_retained_but_can_finish_same_cycle():
    result=native_outcomes('cycle 1 success = False\ncycle 1 success = True\ncycle 2 success = True\nfinal success = True',2)
    assert result['native_full_chain_passed'] and len(result['native_cycles'])==3
    assert result['native_cycles'][0]['success'] is False
    assert result['native_completed_cycles']==[1,2]


@pytest.mark.parametrize('line,count', [
    ('cycle 1 success = True/path/from/prefetch',1),
    ('cycle 1 success = True[prefetch] started',1),
    ('cycle 1 success = Trueish',0),
    ('debug: cycle 1 success = True',0),
])
def test_interleaved_native_log_marker_keeps_exact_boolean_boundary(line,count):
    assert len(native_outcomes(line,1)['native_cycles'])==count
