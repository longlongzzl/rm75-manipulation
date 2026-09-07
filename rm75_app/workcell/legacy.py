"""Working-engine adapters. All original imports/monkeypatches stay in one child.

This is deliberately an adapter migration, not a speculative rewrite of the
working grasp, partial-open release, source retry or magnetic capture algorithms.
"""
from __future__ import annotations
import argparse
import builtins
import uuid
import contextlib
import functools
import importlib.util
import inspect
import os
import sys
import time
from pathlib import Path
from .migration import verify_snapshot
from .io import atomic_json
from rm75_app.magnetic.design import validate_design

ENTRYPOINTS={
 'pickplace':'pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place_sam6d.py',
 'magnetic':'Beta_demo-codex-v0.9/rm75_jimu_triangle_roof_apriltag_portable.py',
}


def snapshot_root(app_root):
    return Path(app_root)/'rm75_app'/'_vendor'/'working_snapshot'


def import_working_entry(root,task):
    path=root/ENTRYPOINTS[task]
    os.environ['LEROBOT_ROOT']=str(root)
    # Match native script startup. Portable Jimu inserts pick_jiaobang itself;
    # pre-inserting it here prevents that insertion and lets Jimu's same-named
    # curobo_rm75_planner shadow the planner imported by the PickPlace wrapper.
    for directory in (root,path.parent):
        sys.path.insert(0,str(directory))
    spec=importlib.util.spec_from_file_location('_rm75_working_entry',path)
    module=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=module
    spec.loader.exec_module(module)
    return module


def original_parser(module, task):
    """Use the actual public builder from the pinned working entrypoint.

    Triangle's builder is named build_arg_parser_triangle. Calling portable's
    builder before install_patches() would silently omit triangle/builder flags.
    """
    name = 'build_arg_parser_triangle' if task == 'magnetic' else 'build_arg_parser'
    builder = getattr(module, name, None)
    if not callable(builder):
        raise RuntimeError(f'Working entrypoint is missing {name}; inspect source version')
    return builder()


def build_native_argv(module,spec,profile,run_dir,root):
    parser=original_parser(module,spec['task'])
    actions=parser._option_string_actions
    options=[]
    def add(names,value=None,required=True):
        names=(names,) if isinstance(names,str) else names
        flag=next((name for name in names if name in actions),None)
        if not flag:
            if required:
                raise RuntimeError(f'Working CLI contract missing {names}; do not guess a replacement')
            return
        options.append(flag)
        if value is not None:
            options.extend(str(x) for x in (value if isinstance(value,(tuple,list)) else [value]))
    machine_args=profile.get(spec['task'],{}).get('native_args',[])
    if not isinstance(machine_args,list) or not all(isinstance(x,str) for x in machine_args):
        raise ValueError('native_args must be a trusted machine-profile string list')
    forbidden={'--execute-real','--auto-execute','--jimu-builder-scene-json','--object-name'}
    if any(arg.split('=')[0] in forbidden for arg in machine_args):
        raise ValueError('Machine native_args must not override task identity/real mode')
    options.extend(machine_args)
    params=spec['parameters']
    add('--auto-execute')
    if spec['task']=='pickplace':
        add('--object-name',params['object_name'])
    else:
        design=validate_design(params['design'])
        design_path=run_dir/'builder_scene.json'
        atomic_json(design_path,design.payload)
        add('--jimu-builder-scene-json',str(design_path))
    add('--lerobot-root',str(root),required=False)
    mode=spec['mode']
    if mode=='real':
        add('--execute-real')
        hardware=profile['hardware']
        add(('--real-robot-ip','--real-ip','--realman-ip','--robot-ip','--ip'),hardware['ip'])
        port=int(hardware.get('port',8080))
        add(('--real-robot-port','--real-port','--realman-port','--robot-port'),port,required=port!=8080)
        if spec['task']=='magnetic':
            add('--jimu-apriltag-anchor-localization')
    else:
        fixed=profile.get(spec['task'],{}).get('fixed_scene')
        if fixed:
            fixed=Path(fixed)
            if not fixed.is_absolute():
                fixed=root/fixed
            if not fixed.is_file():
                raise FileNotFoundError(f'Missing fixed scene: {fixed}')
            add('--sam6d-fixed-scene-result-file',str(fixed))
    add('--render-mode',profile.get(spec['task'],{}).get('render_mode','human'),required=False)
    parsed=parser.parse_args(options)
    if bool(getattr(parsed,'execute_real',False)) != (mode=='real'):
        raise PermissionError('Parsed original CLI real flag does not match authorized task mode')
    if mode == 'real':
        for key in ('sam6d_fixed_scene_result_file', 'fixed_scene_file',
                    'cached_pose_result', 'skip_foundationpose'):
            if getattr(parsed, key, None):
                raise PermissionError(f'Real execution cannot use frozen observation: {key}')
    return options


def install_progress_hooks(module,stop,events):
    """Observe actual original episode/stage boundaries without changing values."""
    direct=getattr(module,'direct',None)
    if direct is None and getattr(module,'portable',None) is not None:
        direct=module.portable.direct
    if direct is None:
        raise RuntimeError('Working source no longer exposes the reviewed direct boundary')
    original=direct.run_targeted_place_episode_curobo_direct
    results=[]
    @functools.wraps(original)
    def episode(*args,**kwargs):
        stop.check();events.emit('legacy_episode_start',index=len(results))
        value=original(*args,**kwargs)
        known=None
        if isinstance(value,bool): known=value
        elif isinstance(value,dict) and type(value.get('success')) is bool: known=value['success']
        elif isinstance(value,tuple) and value and type(value[0]) is bool: known=value[0]
        results.append(known)
        events.emit('legacy_episode_end',index=len(results)-1,command_success=known,
                    return_type=type(value).__name__,task_success=None)
        stop.check();return value
    direct.run_targeted_place_episode_curobo_direct=episode
    original_stage=getattr(direct,'_profile_stage',None)
    if callable(original_stage):
        @functools.wraps(original_stage)
        def stage(*args,**kwargs):
            stop.check()
            name=args[1] if len(args)>1 and isinstance(args[1],str) else kwargs.get('stage_name','profile_call')
            events.emit('legacy_profile_call',stage=str(name)[:200])
            value=original_stage(*args,**kwargs)
            events.emit('legacy_profile_return',stage=str(name)[:200],return_type=type(value).__name__)
            return value
        direct._profile_stage=stage
    return results


def run_working(spec,profile,app_root,run_dir,stop,events):
    root=snapshot_root(app_root)
    provenance=verify_snapshot(root)
    events.emit('working_source_verified',commit=provenance['source_commit'],files=provenance['file_count'])
    old_argv=sys.argv[:];old_cwd=Path.cwd()
    from .input_bridge import install as install_input_bridge, install_subprocess_bridge
    import subprocess
    original_input=install_input_bridge(run_dir)
    old_input_dir=os.environ.get('RM75_WORKCELL_INPUT_DIR')
    original_popen=install_subprocess_bridge(run_dir)
    os.environ['RM75_WORKCELL_INPUT_DIR']=str(run_dir.resolve())
    try:
        sys.argv=[str(root/ENTRYPOINTS[spec['task']])]
        os.chdir(root)
        module=import_working_entry(root,spec['task'])
        argv=build_native_argv(module,spec,profile,run_dir,root)
        atomic_json(run_dir/'native_command.json',{'entrypoint':ENTRYPOINTS[spec['task']],
             'argv':argv,'source_commit':provenance['source_commit'],'mode':spec['mode']})
        results=install_progress_hooks(module,stop,events)
        from .contact_audit import install_contact_audit, StrictContactNotSupported
        if spec['task'] == 'magnetic':
            direct = getattr(module, 'direct', None) or module.portable.direct
            install_contact_audit(direct, lambda row: events.emit('contact_audit', evidence=row))
        sys.argv=[str(root/ENTRYPOINTS[spec['task']]),*argv]
        stop.check()
        try:
            return_value=module.main()
        except StrictContactNotSupported as exc:
            return {'command_success': False, 'task_success': None,
                    'verification': exc.code, 'status': exc.code,
                    'contact_evidence': exc.evidence,
                    'episode_command_results': results}
        except SystemExit as exc:
            if exc.code not in (None,0):
                raise RuntimeError(f'Working engine exit {exc.code}') from exc
            return_value=0
        if type(return_value) is int and return_value!=0:
            raise RuntimeError(f'Working engine returned {return_value}')
        stop.check()
        return {'command_success':True,'task_success':None,'verification':'not_observed',
                'source_commit':provenance['source_commit'],'episode_command_results':results,
                'original_algorithms_preserved':True,
                'note':'Normal process return is not proof of a real grasp or magnetic connection'}
    finally:
        builtins.input=original_input
        subprocess.Popen=original_popen
        if old_input_dir is None:
            os.environ.pop('RM75_WORKCELL_INPUT_DIR',None)
        else:
            os.environ['RM75_WORKCELL_INPUT_DIR']=old_input_dir
        sys.argv=old_argv;os.chdir(old_cwd)
