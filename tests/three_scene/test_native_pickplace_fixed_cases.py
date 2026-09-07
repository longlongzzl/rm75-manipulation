import io
import json
import pytest

from tools.run_native_pickplace import FIXED_CASES, ROOT, NativeOutcomeCapture,build_native_argv


def test_every_fixed_case_exists_and_contains_every_source():
    for scene, names in FIXED_CASES.values():
        data = json.loads((ROOT / scene).read_text())
        assert set(names) <= set(data['objects'])
    assert len(FIXED_CASES['current_table_all'][1]) == 7


def test_native_outcome_capture_handles_split_lines_and_preserves_stdout():
    stream = io.StringIO(); capture = NativeOutcomeCapture(stream)
    chunks = ['cycle 1 suc', 'cess = True\n', 'cycle 2 success = False\n',
              'final success = False\n', 'not final success = True\n']
    for chunk in chunks: capture.write(chunk)
    assert stream.getvalue() == ''.join(chunks)
    assert capture.cycles == [{'cycle': 1, 'success': True}, {'cycle': 2, 'success': False}]
    assert capture.final is False


def test_missing_native_outcome_remains_unverified():
    capture = NativeOutcomeCapture(io.StringIO())
    capture.write('planner success\n')
    assert capture.final is None and capture.cycles == []


def test_native_true_does_not_hide_missing_clearance():
    capture=NativeOutcomeCapture(io.StringIO())
    warning='[warn] post-place clearance planning failed after release; settling the object without moving the arm away first'
    capture.write(warning+'\ncycle 1 success = True\nfinal success = True\n')
    assert capture.final is True
    assert capture.clearance_failures==[warning]


@pytest.mark.parametrize('checked',[False,True])
def test_transport_validation_preserves_all_native_sources_and_has_no_motion_flag(tmp_path,checked):
    argv=build_native_argv('current_table_all',tmp_path,transport_world_checked=checked)
    assert '--execute-real' not in argv
    assert ('--no-fast-chain-cuda-graph-ik' in argv) is checked
    index=argv.index('--cycle-object-names')
    assert argv[index+1:index+8]==list(FIXED_CASES['current_table_all'][1])
    assert all(flag not in argv for flag in ('--no-curobo-self-collision','--no-curobo-table-collision',
                                            '--fast-chain-num-ik-seeds'))
