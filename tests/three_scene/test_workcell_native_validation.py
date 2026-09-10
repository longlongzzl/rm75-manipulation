import hashlib
import json
from tools.run_workcell_native_validation import native_outcomes,read_original_task_bundle,read_documented_jimu_start,jimu_lift_trial_args
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


def test_original_task_bundle_rejects_dependency_escape(tmp_path):
    (tmp_path/'manifest.json').write_text(json.dumps({'schema':'jimu_task_manifest_v1',
        'builder_scene_json':'../builder.json','sam6d_fixed_scene_result_file':'fixed.json'}))
    with pytest.raises(ValueError,match='inside its original task'):
        read_original_task_bundle(tmp_path)


def test_original_task_bundle_rejects_unknown_schema(tmp_path):
    (tmp_path/'manifest.json').write_text('{}')
    with pytest.raises(ValueError,match='schema'):read_original_task_bundle(tmp_path)


def test_task_bundle_reads_exact_three_file_closure_without_modification(tmp_path,design):
    manifest={'schema':'jimu_task_manifest_v1','builder_scene_json':'builder.json',
              'sam6d_fixed_scene_result_file':'fixed.json'}
    files={'manifest.json':manifest,'builder.json':design,'fixed.json':{'results':[{'object_name':'anchor'}]}}
    for name,data in files.items():(tmp_path/name).write_text(json.dumps(data))
    before={name:(tmp_path/name).read_bytes() for name in files}
    resolved,hashes=read_original_task_bundle(tmp_path)
    assert set(resolved)=={'manifest','builder','fixed_scene'}
    assert hashes=={key:hashlib.sha256(path.read_bytes()).hexdigest() for key,path in resolved.items()}
    assert before=={name:(tmp_path/name).read_bytes() for name in files}
    assert set(path.name for path in tmp_path.iterdir())==set(files)


def test_documented_start_reads_only_angles_without_other_command_flags(tmp_path):
    path=tmp_path/'original.md'
    path.write_text('python native.py --execute-real\n --jimu-sim-start-joints-deg 45 0 0 -90 0 -90 60 \\\n --real-control-hz 15\n')
    before=path.read_bytes();record=read_documented_jimu_start(path)
    assert record['joints_deg']==[45,0,0,-90,0,-90,60]
    assert record['source_sha256']==hashlib.sha256(before).hexdigest()
    assert record['read_only'] and path.read_bytes()==before
    assert 'execute-real' not in json.dumps(record)


@pytest.mark.parametrize('body',[
    'no start configuration',
    '--jimu-sim-start-joints-deg 45 0 0 -90 0 -90 nan',
    '--jimu-sim-start-joints-deg 45 0 0 -90 0 -90 60 --execute-real',
    '--jimu-sim-start-joints-deg 45 0 0 -90 0 -90 60\n--jimu-sim-start-joints-deg 90 0 0 -90 0 -90 60',
])
def test_missing_ambiguous_or_non_numeric_documented_start_fails_closed(tmp_path,body):
    path=tmp_path/'original.md';path.write_text(body)
    with pytest.raises(ValueError):read_documented_jimu_start(path)


@pytest.mark.parametrize('task',['pickplace','magnetic'])
def test_default_runner_does_not_change_any_native_lift(task):
    assert jimu_lift_trial_args(task,False)==[]


def test_jimu_trial_changes_only_failed_lift_by_exactly_three_mm():
    argv=jimu_lift_trial_args('magnetic',True)
    assert argv==['--joint-search-start-collision-lift-m','0.103']
    assert float(argv[1])-.100==pytest.approx(.003)
    # No override of independent lift, collision/seed/geometry or execution flags.
    assert len(argv)==2


def test_lift_trial_cannot_modify_pickplace():
    with pytest.raises(ValueError,match='Jimu SIM only'):
        jimu_lift_trial_args('pickplace',True)


def test_lift_cli_rejects_other_tasks_before_starting_service(monkeypatch,tmp_path):
    import tools.run_workcell_native_validation as runner
    monkeypatch.setattr(runner.sys,'argv',['runner','--task','pickplace','--jimu-lift-plus-3mm',
        '--output',str(tmp_path/'unused'),'--extensions',str(tmp_path)])
    monkeypatch.setattr(runner,'verify_snapshot',lambda *args:pytest.fail('Should reject before loading native code'))
    with pytest.raises(SystemExit) as exc:runner.main()
    assert exc.value.code==2 and not (tmp_path/'unused').exists()


def test_pickplace_lift_target_is_single_explicit_argv_pair():
    from tools.run_workcell_native_validation import pickplace_lift_target_args
    assert pickplace_lift_target_args('pickplace',None)==[]
    argv=pickplace_lift_target_args('pickplace',.12)
    assert argv==['--joint-search-start-collision-lift-m','0.120']
    assert len(argv)==2  # nothing else: same collisions, seeds and thresholds


def test_pickplace_lift_target_guards_task_and_range():
    from tools.run_workcell_native_validation import pickplace_lift_target_args
    with pytest.raises(ValueError,match='PickPlace SIM only'):
        pickplace_lift_target_args('magnetic',.12)
    for value in (.029,.201):
        with pytest.raises(ValueError,match='0.030..0.200'):
            pickplace_lift_target_args('pickplace',value)


def test_pickplace_lift_cli_rejects_bad_range_or_task_before_service(monkeypatch,tmp_path):
    import tools.run_workcell_native_validation as runner
    for argv in (['--task','magnetic','--pickplace-lift-target-m','0.12'],
                 ['--task','pickplace','--pickplace-lift-target-m','0.21']):
        monkeypatch.setattr(runner.sys,'argv',['runner',*argv,
            '--output',str(tmp_path/'unused'),'--extensions',str(tmp_path)])
        monkeypatch.setattr(runner,'verify_snapshot',lambda *args:pytest.fail('Should reject before loading native code'))
        with pytest.raises(SystemExit) as exc:runner.main()
        assert exc.value.code==2 and not (tmp_path/'unused').exists()
