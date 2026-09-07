import argparse
from types import SimpleNamespace as NS
import pytest
from rm75_app.workcell.legacy import native_entrypoint,build_native_argv,working_direct,ENTRYPOINTS,PICKPLACE_WORLD_ENTRY
from rm75_app.workcell.spec import validate_spec


def test_default_camera_entry_is_unchanged():
    assert native_entrypoint({'task':'pickplace','mode':'sim'},{})==ENTRYPOINTS['pickplace']
    assert native_entrypoint({'task':'pickplace','mode':'real'},{})==ENTRYPOINTS['pickplace']


def test_three_reviewed_module_layouts_use_same_direct_boundary():
    direct=NS(run_targeted_place_episode_curobo_direct=lambda:True)
    for module in (direct,NS(direct=direct),NS(portable=NS(direct=direct))):
        assert working_direct(module) is direct
    with pytest.raises(RuntimeError):working_direct(NS())


@pytest.mark.parametrize('task,mode',[('pickplace','real'),('pickplace','preview'),('magnetic','sim')])
def test_world_entry_cannot_be_used_for_real_or_magnetic(task,mode):
    with pytest.raises(PermissionError):native_entrypoint({'task':task,'mode':mode},
        {task:{'fixed_scene_format':'native_world','fixed_scene':'original.json'}})


def test_unknown_input_format_and_missing_world_input_fail_closed():
    spec={'task':'pickplace','mode':'sim'}
    with pytest.raises(ValueError):native_entrypoint(spec,{'pickplace':{'fixed_scene_format':'guess'}})
    with pytest.raises(ValueError):native_entrypoint(spec,{'pickplace':{'fixed_scene_format':'native_world'}})


def test_original_world_cli_does_not_claim_or_invoke_sam6d(tmp_path):
    scene=tmp_path/'original.json';scene.write_text('{"objects":{}}')
    def parser():
        p=argparse.ArgumentParser()
        p.add_argument('--auto-execute',action='store_true');p.add_argument('--skip-foundationpose',action='store_true')
        p.add_argument('--object-name');p.add_argument('--fixed-scene-pose-file')
        return p
    spec={'task':'pickplace','mode':'sim','parameters':{'object_name':'gluestick'}}
    profile={'pickplace':{'fixed_scene_format':'native_world','fixed_scene':str(scene)}}
    assert native_entrypoint(spec,profile)==PICKPLACE_WORLD_ENTRY
    argv=build_native_argv(NS(build_arg_parser=parser),spec,profile,tmp_path,tmp_path)
    assert '--fixed-scene-pose-file' in argv and '--skip-foundationpose' in argv
    assert '--execute-real' not in argv and '--sam6d-fixed-scene-result-file' not in argv


def test_browser_cannot_choose_native_entry_or_input_format(profile):
    for key in ('fixed_scene_format','entrypoint'):
        with pytest.raises(ValueError):validate_spec({'task':'pickplace','mode':'sim',
            'parameters':{'object_name':'gluestick',key:'native_world'}},profile)
