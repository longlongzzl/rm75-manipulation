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
    for directory in (root,root/'pick_jiaobang',path.parent):
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
    # Only server-side profile can select interpreter, model paths or safety flags.
    # Browser never supplies free-form command-line arguments.
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
        # Explicitly reject a native argv list that would enable hardware through
        # an alias. argparse destination is authoritative, not just string search.
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
        # Inspect argparse destinations as well as option strings, so aliases
        # cannot sneak a frozen scene into a real-observation run.
        for key in ('sam6d_fixed_scene_result_file', 'fixed_scene_file',
                    'cached_pose_result', 'skip_foundationpose'):
            if getattr(parsed, key, None):
                raise PermissionError(f'Real execution cannot use frozen observation: {key}')
    return options


def install_progress_hooks(module,stop,events):
    """Observe the actual original episode/stage boundaries; preserve their values.

    These callbacks are cooperative. Parent-controlled stop remains available
    while an original GPU/driver call blocks. Original False return values are
    returned unchanged: the working engine uses them for same-family source
    retries, so converting them into exceptions would change its algorithm.
    Dependency sequencing remains owned by the migrated working runtime