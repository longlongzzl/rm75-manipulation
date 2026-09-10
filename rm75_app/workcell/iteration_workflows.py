"""Connect the new UI contracts to the preserved native runtime, not a mock path."""
from __future__ import annotations
import contextlib
import copy
import functools
import hashlib
from pathlib import Path
from .io import atomic_json

PREFERRED_ORDER=('shuazi','bi','lvmukuai','carriot','tennis','gluestick','hongshupian')


def ordered_sources(names, automatic=False):
    if not automatic: return list(names)
    return sorted(names, key=lambda n: (PREFERRED_ORDER.index(n) if n in PREFERRED_ORDER else len(PREFERRED_ORDER), n))


def prepare_request(spec, profile, run_dir=None):
    """Only validated typed task fields modify a per-job copy of the profile."""
    spec=copy.deepcopy(spec); profile=copy.deepcopy(profile)
    section=profile.setdefault(spec['task'], {})
    if spec['task']=='pickplace' and 'object_names' in spec['parameters']:
        params=spec['parameters']
        if spec['mode']!='sim' or section.get('fixed_scene_format')!='native_world':
            raise PermissionError('Multi-object native sequence currently requires frozen-world SIM')
        names=ordered_sources(params['object_names'], params.get('automatic_order',False))
        # The native contract accepts a sequence only when it has >=2 sources.
        # One selection is a normal single-object task, not a one-item cycle.
        if len(names)>1:
            section['frozen_source_order']=names
        else:
            section.pop('frozen_source_order',None)
        spec['parameters']={'object_name':names[0]}
    elif spec['task']=='magnetic' and 'generation_proof' in spec['parameters']:
        from rm75_app.magnetic.generation import load_library,validate_generated_request
        recipe=validate_generated_request(load_library(profile), spec['parameters']['design'],
                                          spec['parameters']['generation_proof'])
        # The browser/model cannot supply recipe arguments or override calibration.
        native=list(section.get('native_args',[])); incoming=list(recipe.get('native_args',[]))
        manifest=recipe.get('task_manifest')
        if manifest is not None:
            if run_dir is None: raise ValueError('Generated native task requires a per-job manifest directory')
            manifest=copy.deepcopy(manifest)
            manifest['builder_scene_json']='builder_scene.json'
            manifest['sam6d_fixed_scene_result_file']=recipe.get('fixed_scene')
            manifest_path=Path(run_dir)/'original_base_manifest.json'
            atomic_json(manifest_path,manifest)
            incoming+=['--jimu-task-dir',str(manifest_path.resolve())]
            # Let this base's original manifest own the tray indices, which can
            # differ between the grid and the arc. Preserve planner/lift settings.
            remove={'--jimu-triangle-tray-slot-indices'}
            filtered=[];dropping=False
            for value in native:
                if value.startswith('--'): dropping=value.split('=')[0] in remove
                if not dropping: filtered.append(value)
            native=filtered
        incoming_flags={x.split('=')[0] for x in incoming if x.startswith('--')}
        kept=[]; dropping=False
        for value in native:
            if value.startswith('--'): dropping=value.split('=')[0] in incoming_flags
            if not dropping: kept.append(value)
        section['native_args']=kept+incoming
        if spec['mode']=='sim' and recipe.get('fixed_scene'):
            path=Path(recipe['fixed_scene'])
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=recipe.get('fixed_scene_sha256'):
                raise ValueError('Original base-specific scene input is absent or changed')
            section['fixed_scene']=str(path)
        spec['parameters']={'design':spec['parameters']['design']}
    return spec, profile


def completion_summary(result, requested):
    from .sequence_evidence import summarize_sequence
    return summarize_sequence(result, requested)


def run(spec, profile, app_root, run_dir, stop, events):
    from . import legacy
    original_spec=copy.deepcopy(spec)
    spec,profile=prepare_request(spec,profile,run_dir)
    atomic_json(Path(run_dir)/'effective_task_profile.json',profile)
    requested=profile.get('pickplace',{}).get('frozen_source_order',[spec['parameters'].get('object_name')])
    repair=profile.get('pickplace',{}).get('paired_endpoint_repair',{})
    with contextlib.ExitStack() as stack:
        if spec['task']=='pickplace' and repair.get('enabled',False):
            if (spec['mode']!='sim' or profile['pickplace'].get('fixed_scene_format')!='native_world'
                    or profile['pickplace'].get('simulation_contact_policy')!='transport_world_checked_compatibility'):
                raise PermissionError('Endpoint repair requires checked native frozen-world SIM')
            original_import=legacy.import_working_entry
            def importing(*args,**kwargs):
                module=original_import(*args,**kwargs)
                original_main=module.main
                @functools.wraps(original_main)
                def main(*pos,**kw):
                    from .paired_endpoint_repair import install
                    close=install(legacy.working_direct(module),
                        lambda row:events.emit('contact_audit',evidence=row),stop,
                        max_queries=repair.get('max_queries',12),budget_s=repair.get('budget_s',5.))
                    try:return original_main(*pos,**kw)
                    finally:close()
                module.main=main
                return module
            legacy.import_working_entry=importing
            stack.callback(setattr,legacy,'import_working_entry',original_import)
        result=legacy.run_working(spec,profile,app_root,run_dir,stop,events)
    if original_spec['task']=='pickplace' and 'object_names' in original_spec['parameters']:
        result['sequence_summary']=completion_summary(result,requested)
        result['requested_sequence_order']=requested
    if original_spec['task']=='magnetic' and 'generation_proof' in original_spec['parameters']:
        result['generation_proof']=original_spec['parameters']['generation_proof']
        result['generated_design_used_unchanged']=True
    return result
