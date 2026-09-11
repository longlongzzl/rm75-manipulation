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
PICKPLACE_WORLD_ENTRY='pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py'


def native_entrypoint(spec,profile):
    """Match original input schema to its original entry; never convert poses."""
    section=profile.get(spec['task'],{})
    kind=section.get('fixed_scene_format','sam6d')
    if kind not in ('sam6d','native_world'):raise ValueError('Unknown fixed native scene format')
    if kind=='native_world':
        if spec['task']!='pickplace' or spec['mode']!='sim':
            raise PermissionError('Original fixed-world entry is PickPlace SIM only')
        if not section.get('fixed_scene'):raise ValueError('Native world SIM requires a fixed scene file')
        return PICKPLACE_WORLD_ENTRY
    return ENTRYPOINTS[spec['task']]


def working_direct(module):
    direct=getattr(module,'direct',None)
    if direct is None and getattr(module,'portable',None) is not None:direct=module.portable.direct
    if direct is None and callable(getattr(module,'run_targeted_place_episode_curobo_direct',None)):direct=module
    if direct is None:raise RuntimeError('Working source no longer exposes the reviewed direct boundary')
    return direct


def snapshot_root(app_root):
    return Path(app_root)/'rm75_app'/'_vendor'/'working_snapshot'


def import_working_entry(root,task,*,entrypoint=None):
    entrypoint=entrypoint or ENTRYPOINTS[task]
    if entrypoint not in {*ENTRYPOINTS.values(),PICKPLACE_WORLD_ENTRY}:
        raise ValueError('Native entrypoint is not in the reviewed allowlist')
    path=root/entrypoint
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


def native_contact_policy(spec,profile):
    """Only a trusted SIM profile can select the audited contact compatibility.

    Real workers always keep the strict rejection boundary; simulation evidence
    and permissions must never silently qualify a hardware contact exception.
    """
    default='strict' if spec['task']=='magnetic' else 'original'
    policy=profile.get(spec['task'],{}).get('simulation_contact_policy',default)
    if policy not in ('original','strict','transport_world_checked_compatibility'):
        raise ValueError('Unknown trusted native simulation contact policy')
    if spec['mode']=='real':return 'strict'
    if spec['task']=='magnetic' and policy=='original':
        raise ValueError('Broad native magnetic collision masks are compatibility-audit only')
    return policy


def build_native_argv(module,spec,profile,run_dir,root,*,frozen_contract=None):
    entrypoint=native_entrypoint(spec,profile)
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
    if native_contact_policy(spec,profile)=='transport_world_checked_compatibility':
        add('--no-fast-chain-cuda-graph-ik')
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
            if entrypoint==PICKPLACE_WORLD_ENTRY:
                from .native_frozen_world import read_contract
                contract=frozen_contract or read_contract(fixed,params['object_name'])
                if contract['path']!=fixed.resolve() or contract['source']!=params['object_name']:
                    raise ValueError('Frozen world contract differs from task input')
                add('--skip-foundationpose')
                add('--fixed-scene-pose-file',str(fixed))
                # Original loader/activation removes only the CURRENT source.
                # Retain all names across same-cycle source retries.
                add('--tracked-scene-object-names',list(contract['names']))
                if len(contract.get('source_order',()))>1:
                    add('--cycle-object-names',list(contract['source_order']))
                    add('--cycle-order-targets')
                    add('--repeat-count',len(contract['source_order']))
            else:add('--sam6d-fixed-scene-result-file',str(fixed))
    if profile.get(spec['task'],{}).get('record_sim_video') is True:
        from .sim_failure_video import require_sim
        require_sim(spec,profile)
        add('--dry-run-motion-window-scale',1.0)
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
    direct=working_direct(module)
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
    entrypoint=native_entrypoint(spec,profile)
    provenance=verify_snapshot(root)
    events.emit('working_source_verified',commit=provenance['source_commit'],files=provenance['file_count'])
    old_argv=sys.argv[:];old_cwd=Path.cwd()
    from .input_bridge import install as install_input_bridge, install_subprocess_bridge
    import subprocess
    original_input=install_input_bridge(run_dir)
    old_input_dir=os.environ.get('RM75_WORKCELL_INPUT_DIR')
    original_popen=install_subprocess_bridge(run_dir)
    os.environ['RM75_WORKCELL_INPUT_DIR']=str(run_dir.resolve())
    adapters = contextlib.ExitStack()
    try:
        from .pickplace_curobo_only import source_adapter
        adapters.enter_context(source_adapter(root))
        sys.argv=[str(root/entrypoint)]
        os.chdir(root)
        module=import_working_entry(root,spec['task'],entrypoint=entrypoint)
        direct=working_direct(module)
        if spec['task']=='pickplace':
            from .pickplace_level_release import install as install_level_release
            adapters.callback(install_level_release(direct,
                lambda row:events.emit('contact_audit',evidence=row)))
        if spec['task']=='magnetic':
            from .pickplace_curobo_only import install_jimu_binding
            install_jimu_binding(module.portable)
        from .pickplace_curobo_only import install
        clearance_audits=install(direct)
        events.emit('native_backend_selected',task=spec['task'],planner='curobo',mplib_fallback=False)
        frozen_validation=None
        if entrypoint==PICKPLACE_WORLD_ENTRY:
            from .native_frozen_world import FrozenWorldValidation,read_contract
            fixed=Path(profile[spec['task']]['fixed_scene'])
            if not fixed.is_absolute():fixed=root/fixed
            frozen_validation=FrozenWorldValidation(
                read_contract(fixed,spec['parameters']['object_name'],
                    profile['pickplace'].get('frozen_source_order')),run_dir,events)
        argv=build_native_argv(module,spec,profile,run_dir,root,
            frozen_contract=None if frozen_validation is None else frozen_validation.contract)
        atomic_json(run_dir/'native_command.json',{'entrypoint':entrypoint,
             'argv':argv,'source_commit':provenance['source_commit'],'mode':spec['mode']})
        results=install_progress_hooks(module,stop,events)
        if frozen_validation is not None:
            # Must be inside transport's refresh wrapper: observe effective args.
            frozen_validation.install(direct)
        from .contact_audit import install_contact_audit, StrictContactNotSupported
        from .pickplace_world_coverage import FrozenWorldIncomplete
        policy=native_contact_policy(spec,profile)
        if policy=='transport_world_checked_compatibility':
            from curobo_rm75_planner import RM75CuRoboPlanner
            from .transport_contact import install_transport_contact
            adapters.callback(install_transport_contact(direct,RM75CuRoboPlanner,
                lambda row:events.emit('contact_audit',evidence=row)))
            if spec['task']=='magnetic':
                from .transport_contact import (install_read_only_jimu_diagnostics,guard_jimu_near_ik,
                                                install_jimu_grasp_ik_contact)
                install_read_only_jimu_diagnostics(module.portable,
                    lambda row:events.emit('contact_audit',evidence=row))
                if profile.get('magnetic',{}).get('audit_roof_ik') is True:
                    if spec['mode']!='sim':raise ValueError('Roof IK diagnostics are SIM only')
                    from .jimu_roof_ik_diagnostics import install_roof_ik_diagnostics
                    install_roof_ik_diagnostics(direct,
                        lambda row:events.emit('contact_audit',evidence=row))
                install_jimu_grasp_ik_contact(direct,
                    lambda row:events.emit('contact_audit',evidence=row))
                guard_jimu_near_ik(module.portable,
                    lambda row:events.emit('contact_audit',evidence=row))
                from .jimu_return_diagnostics import install_return_diagnostics,install_release_execution_observer
                install_return_diagnostics(module.portable,
                    lambda row:events.emit('contact_audit',evidence=row),
                    limit=int(profile.get('magnetic',{}).get('jimu_return_diagnostic_limit',1)))
                from .jimu_release_execution import install_release_execution_guard
                install_release_execution_guard(module.portable,
                    lambda row:events.emit('contact_audit',evidence=row))
                install_release_execution_observer(module.portable,
                    lambda row:events.emit('contact_audit',evidence=row),synchronize=True)
                if profile.get('magnetic',{}).get('jimu_return_model_sync') is True:
                    from .jimu_return_model_sync import install_return_model_sync
                    install_return_model_sync(module.portable,
                        lambda row:events.emit('contact_audit',evidence=row))
        elif policy=='strict':
            install_contact_audit(direct, lambda row: events.emit('contact_audit', evidence=row))
        events.emit('native_contact_policy_selected',policy=policy,mode=spec['mode'],
                    hardware_contact_qualified=False)
        if profile.get('pickplace',{}).get('audit_current_table_failures') is True:
            if (spec['task']!='pickplace' or spec['mode']!='sim' or frozen_validation is None
                    or policy!='transport_world_checked_compatibility'):
                raise ValueError('Focused current-table diagnostics require checked frozen-world PickPlace SIM')
            from curobo_rm75_planner import RM75CuRoboPlanner
            from .pickplace_focused_diagnostics import install as install_focused
            adapters.callback(install_focused(direct,RM75CuRoboPlanner,
                lambda row:events.emit('contact_audit',evidence=row),
                requested_source='gluestick' if profile['pickplace'].get('frozen_source_order')
                    else spec['parameters']['object_name']))
        if profile.get('pickplace',{}).get('tennis_ik_review'):
            if (spec['task']!='pickplace' or spec['mode']!='sim' or frozen_validation is None
                    or 'tennis' not in profile['pickplace'].get('frozen_source_order',[spec['parameters']['object_name']])
                    or policy!='transport_world_checked_compatibility'):
                raise ValueError('Tennis IK review requires checked native frozen SIM')
            from curobo_rm75_planner import RM75CuRoboPlanner
            from .pickplace_ik_review import install as install_ik_review
            adapters.callback(install_ik_review(direct,RM75CuRoboPlanner,
                lambda row:events.emit('contact_audit',evidence=row),
                strategy=profile['pickplace']['tennis_ik_review']))
        if profile.get('pickplace',{}).get('failed_object_ik_seeds'):
            from .failed_object_ik_search import install as install_failed_search, require_search
            require_search(spec,profile)
            adapters.callback(install_failed_search(direct,
                lambda row:events.emit('contact_audit',evidence=row),
                num_seeds=profile['pickplace']['failed_object_ik_seeds']))
        if profile.get('pickplace',{}).get('render_ik_candidates') is True:
            from .ik_candidate_gallery import install as install_gallery, require_gallery
            require_gallery(spec,profile)
            adapters.callback(install_gallery(direct,run_dir/'ik_gallery',
                lambda row:events.emit('contact_audit',evidence=row)))
        if profile.get(spec['task'],{}).get('record_sim_video') is True:
            from .sim_failure_video import install as install_video, require_sim
            require_sim(spec,profile)
            adapters.callback(install_video(direct,run_dir/'sim_video',
                requested_source=(spec['parameters']['object_name'] if spec['task']=='pickplace'
                    and not profile['pickplace'].get('frozen_source_order') else None),
                portable=module.portable if spec['task']=='magnetic' else None))
        sys.argv=[str(root/entrypoint),*argv]
        from .native_outcome import NativeOutcomeCapture
        captured=NativeOutcomeCapture(sys.stdout)
        expected_cycles=(len(validate_design(spec['parameters']['design']).ordered_roles)
                         if spec['task']=='magnetic' else
                         len(frozen_validation.contract['source_order']) if frozen_validation else 1)
        stop.check()
        try:
            with contextlib.redirect_stdout(captured):
                return_value=module.main()
        except StrictContactNotSupported as exc:
            return {'command_success': False, 'task_success': None,
                    'verification': exc.code, 'status': exc.code,
                    'contact_evidence': exc.evidence,
                    'episode_command_results': results,**captured.report(expected_cycles)}
        except FrozenWorldIncomplete as exc:
            outcome=captured.report(expected_cycles)
            failure=frozen_validation.result(outcome,clearance_audits) if frozen_validation else {}
            return {**outcome,**failure,'command_success':False,'task_success':None,
                    'status':'frozen_world_incomplete','error':str(exc),
                    'episode_command_results':results}
        except SystemExit as exc:
            if exc.code not in (None,0):
                raise RuntimeError(f'Working engine exit {exc.code}') from exc
            return_value=0
        if type(return_value) is int and return_value!=0:
            raise RuntimeError(f'Working engine returned {return_value}')
        stop.check()
        outcome=captured.report(expected_cycles)
        result={'command_success':outcome['native_full_chain_passed'],
                'task_success':None,'verification':'not_observed',**outcome,
                'native_entrypoint':entrypoint,'fixed_scene_format':profile.get(spec['task'],{}).get('fixed_scene_format','sam6d'),
                'source_commit':provenance['source_commit'],'episode_command_results':results,
                'contact_policy':policy,'clearance_path_audits':clearance_audits,
                'clearance_selection_audits':getattr(direct,'_clearance_selection_audits',[]),
                'independent_clearance_execution_audit_observed':bool(clearance_audits),
                'jimu_release_execution_audits':getattr(direct,'_jimu_release_execution_audits',[]),
                'loaded_mplib_modules':[n for n in sys.modules if n=='mplib' or n.startswith('mplib.')],
                'original_algorithms_preserved':spec['task']!='pickplace',
                'added_task_pose_constraints':['tennis_level_pre_place_and_place'] if spec['task']=='pickplace' else [],
                'note':'Normal process return is not proof of a real grasp or magnetic connection'}
        if frozen_validation is not None:
            result.update(frozen_validation.result(outcome,clearance_audits))
            if not result['command_success']:result['status']='frozen_world_validation_failed'
        return result
    finally:
        adapters.close()
        builtins.input=original_input
        subprocess.Popen=original_popen
        if old_input_dir is None:
            os.environ.pop('RM75_WORKCELL_INPUT_DIR',None)
        else:
            os.environ['RM75_WORKCELL_INPUT_DIR']=old_input_dir
        sys.argv=old_argv;os.chdir(old_cwd)
