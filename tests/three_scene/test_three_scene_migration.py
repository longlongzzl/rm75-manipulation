import argparse,copy,subprocess,types
from pathlib import Path
import pytest
from rm75_app.workcell.migration import export_snapshot,verify_snapshot,selected,SOURCE_COMMIT
from rm75_app.workcell.install_patches import blob_sha,transform
from rm75_app.workcell.legacy import build_native_argv,install_progress_hooks
from rm75_app.workcell.events import StopToken,EventLog

def run_git(path,*args):return subprocess.run(['git','-C',str(path),*args],check=True,capture_output=True).stdout.decode().strip()
def test_committed_source_export_preserves_dirty_worktree_and_hashes(tmp_path):
    src=tmp_path/'old';src.mkdir();run_git(src,'init');run_git(src,'config','user.name','Test');run_git(src,'config','user.email','test@example.invalid')
    file=src/'pick_jiaobang/example.py';file.parent.mkdir();raw=b'VALUE = "/home/zhangzhao/Desktop/lerobot/pick_jiaobang/mesh.glb"\n';file.write_bytes(raw)
    asset=file.parent/'mesh.glb';asset.write_bytes(b'not_a_real_mesh_test_fixture');font=file.parent/'do_not_share.ttf';font.write_bytes(b'not_a_real_font')
    run_git(src,'add','.');run_git(src,'commit','-m','synthetic migration fixture');ref=run_git(src,'rev-parse','HEAD');file.write_text('UNCOMMITTED USER WORK\n')
    report=export_snapshot(src,tmp_path/'new',ref=ref,expected_blobs={'pick_jiaobang/example.py':blob_sha(raw)});root=tmp_path/'new/rm75_app/_vendor/working_snapshot'
    assert file.read_text()=='UNCOMMITTED USER WORK\n';assert report['file_count']==2 and report['source_commit']==ref;assert not (root/'pick_jiaobang/do_not_share.ttf').exists();assert str(root) in (root/'pick_jiaobang/example.py').read_text();assert verify_snapshot(root)['runtime_gpu_verified'] is False
    (root/'pick_jiaobang/example.py').write_text('tampered\n')
    with pytest.raises(RuntimeError):verify_snapshot(root)
    with pytest.raises(FileExistsError):export_snapshot(src,tmp_path/'new',ref=ref,expected_blobs={})
def test_migration_filters():
    assert selected('Beta_demo-codex-v0.9/magnetic_snap.py')
    assert selected('Beta_demo-codex-v0.9/jimu_portable_repro/assets/plate.glb')
    assert selected('lerobot-sim2real/lerobot_sim2real/config/real_robot.py')
    assert selected('lerobot/common/robots/realman_lerobot/realman_arm.py')
    assert not selected('lerobot-sim2real/lerobot_sim2real/rl/ppo_rgb.py')
    assert not selected('lerobot/common/policies/act/modeling_act.py')
    assert not selected('lerobot/common/optim/optimizers.py')
    assert not selected('pick_jiaobang/weights/model.pth')
    assert not selected('pick_jiaobang/failure_renders/image.png')
    assert not selected('../pick_jiaobang/unsafe.py')
    assert not selected('pick_jiaobang/a.woff2')
def test_task_integration_preserves_new_curobo_default():
    original='modes=("curobo2", "rrtrack", "openworld-geometry", "tabletop-refine"),\ndefault_mode="curobo2"\n        return command_for_mode(normalized.mode, normalized.args, python=python)';result=transform('rm75_app/tasks/pickplace.py',original);assert 'default_mode="curobo2"' in result and 'working-real' in result and 'task="pickplace"' in result
def parser():
    p=argparse.ArgumentParser()
    for flag in ['--auto-execute','--execute-real','--jimu-apriltag-anchor-localization']:p.add_argument(flag,action='store_true')
    for flag in ['--object-name','--jimu-builder-scene-json','--real-ip','--lerobot-root','--render-mode','--sam6d-fixed-scene-result-file']:p.add_argument(flag)
    return p
@pytest.mark.parametrize('task',['pickplace','magnetic'])
def test_original_cli_adapter_gets_correct_parameters_without_rewriting(profile,design,tmp_path,task):
    module=types.SimpleNamespace(build_arg_parser=parser,build_arg_parser_triangle=parser);spec={'task':task,'mode':'real','parameters':{'object_name':'carriot'} if task=='pickplace' else {'design':design}};profile['hardware']['ip']='TEST_IP';args=build_native_argv(module,spec,profile,tmp_path,tmp_path);parsed=parser().parse_args(args);assert parsed.execute_real and parsed.auto_execute and parsed.real_ip=='TEST_IP';assert task!='magnetic' or parsed.jimu_apriltag_anchor_localization
def test_no_hardware_flag_leak_from_native_profile(profile,tmp_path):
    profile['pickplace']['native_args']=['--execute-real']
    with pytest.raises(ValueError):build_native_argv(types.SimpleNamespace(build_arg_parser=parser),{'task':'pickplace','mode':'sim','parameters':{'object_name':'bi'}},profile,tmp_path,tmp_path)
def test_failed_episode_preserves_original_source_retry_semantics(tmp_path):
    calls=[]
    def original():calls.append(1);return False
    direct=types.SimpleNamespace(run_targeted_place_episode_curobo_direct=original);install_progress_hooks(types.SimpleNamespace(direct=direct),StopToken(),EventLog(tmp_path));assert direct.run_targeted_place_episode_curobo_direct() is False;assert direct.run_targeted_place_episode_curobo_direct() is False;assert len(calls)==2
def test_unknown_return_is_not_task_success(tmp_path):
    direct=types.SimpleNamespace(run_targeted_place_episode_curobo_direct=lambda:None);results=install_progress_hooks(types.SimpleNamespace(direct=direct),StopToken(),EventLog(tmp_path));assert direct.run_targeted_place_episode_curobo_direct() is None and results==[None]
def test_triangle_uses_actual_triangle_parser_not_unpatched_portable(profile,design,tmp_path):
    module=types.SimpleNamespace(build_arg_parser=lambda:(_ for _ in ()).throw(AssertionError()),build_arg_parser_triangle=parser);args=build_native_argv(module,{'task':'magnetic','mode':'sim','parameters':{'design':design}},profile,tmp_path,tmp_path);assert '--jimu-builder-scene-json' in args
