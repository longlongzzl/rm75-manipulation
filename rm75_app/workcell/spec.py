"""Typed browser contract; interpreters/paths/robot addresses never come from HTML."""
from __future__ import annotations
import copy,re
from .io import dumps,finite
from .transforms import vector
from rm75_app.magnetic.design import validate_design
from rm75_app.pusht.model import Config,valid_pose


def validate_spec(value,profile):
    if not isinstance(value,dict) or set(value)-{'task','mode','parameters'}:
        raise ValueError('Expected task, mode, parameters only')
    task=value.get('task');mode=value.get('mode')
    if task not in ('pickplace','magnetic','pusht') or mode not in ('preview','sim','real'):
        raise ValueError('Unsupported task/mode')
    params=copy.deepcopy(value.get('parameters',{}))
    if not isinstance(params,dict):
        raise ValueError('parameters must be an object')
    if task=='pickplace':
        if set(params)-{'object_name','object_names','automatic_order'}:
            raise ValueError('PickPlace accepts only typed object identities and sequence order')
        allowed=profile.get('pickplace',{}).get('object_names',['lvmukuai','carriot','shuazi','gluestick','bi','tennis'])
        if 'object_names' in params:
            if mode=='real' or profile.get('pickplace',{}).get('fixed_scene_format')!='native_world':
                raise PermissionError('Multi-object sequence currently requires native-world SIM/preview')
            names=params['object_names']
            if (not isinstance(names,list) or not 1<=len(names)<=12 or
                any(not isinstance(n,str) for n in names) or len(set(names))!=len(names)):
                raise ValueError('Expected 1..12 distinct configured objects')
            if 'object_name' in params: raise ValueError('Do not mix single-object and sequence requests')
            if type(params.get('automatic_order',False)) is not bool:
                raise ValueError('automatic_order must be boolean')
        else:
            if 'automatic_order' in params: raise ValueError('Sequence order requires object_names')
            names=[params.get('object_name','')]
        if any(not isinstance(n,str) or n not in allowed or not re.fullmatch('[A-Za-z0-9_-]{1,80}',n) for n in names):
            raise ValueError('Unknown/unapproved PickPlace asset')
    elif task=='magnetic':
        if 'design' not in params or set(params)-{'design','generation_proof'}:
            raise ValueError('Magnetic task requires the original builder design JSON')
        design=validate_design(params['design'])
        for p in design.payload['pieces']:
            if not re.fullmatch('[A-Za-z0-9_-]{1,100}',str(p.get('role') or p.get('id'))):
                raise ValueError('Use path-safe original piece ids/roles')
        params['design']=design.payload
        if 'generation_proof' in params:
            if mode=='real':
                raise PermissionError('Generated structures need separate hardware qualification; use SIM first')
            from rm75_app.magnetic.generation import load_library,validate_generated_request
            validate_generated_request(load_library(profile),params['design'],params['generation_proof'])
    else:
        if set(params)-{'initial_pose','goal_pose','speed_mps','max_steps','simulation_backend',
                         'maximum_push_length_m','geometry_id','run_until_goal'}:
            raise ValueError('Unsupported PushT parameter; geometry/safety belong in the machine profile')
        target=vector(params.get('goal_pose'),3,'goal_pose').tolist()
        initial=vector(params.get('initial_pose',[.35,0,0]),3,'initial_pose').tolist()
        if ('run_until_goal' in params and type(params['run_until_goal']) is not bool):
            raise ValueError('run_until_goal must be boolean')
        if mode=='real' and ('run_until_goal' in params or 'geometry_id' in params):
            raise PermissionError('Interactive/alternate-geometry runtime is SIM-only pending hardware qualification')
        geometry=params.get('geometry_id','original')
        if not isinstance(geometry,str) or not re.fullmatch('[A-Za-z0-9_-]{1,80}',geometry):
            raise ValueError('Invalid configured T geometry identity')
        maximum=params.get('maximum_push_length_m')
        if maximum is not None:
            finite(maximum,'maximum_push_length_m',.006,.08)
            if mode=='real' and maximum>float(profile.get('pusht',{}).get('model',{}).get('maximum_push_length_m',.05)):
                raise PermissionError('Long push exceeds the qualified machine limit')
        from rm75_app.pusht.scenarios import configured_model
        from dataclasses import asdict
        cfg=asdict(configured_model(profile,geometry_id=geometry,maximum_push_length_m=maximum))
        if 'speed_mps' in params:
            speed=finite(params['speed_mps'],'speed_mps',.001,.05)
            if mode=='real' and speed>float(cfg.get('speed_mps',.015)):
                raise ValueError('Requested speed exceeds the qualified machine profile')
            cfg['speed_mps']=speed
        if 'max_steps' in params:
            cfg['max_steps']=params['max_steps']
        model=Config.from_dict(cfg)
        if not valid_pose(target,model) or mode!='real' and not valid_pose(initial,model):
            raise ValueError('T geometry crosses workspace/no-go objects')
        simulation=params.get('simulation_backend')
        if simulation is not None:
            if mode=='real' or simulation not in ('surrogate','tool_only_physics','full_arm_physics'):
                raise ValueError('Simulation backend is explicit and never accepted in real mode')
            if simulation!='surrogate' and simulation not in profile.get('pusht',{}).get('physics',{}).get('enabled_backends',[]):
                raise ValueError('Requested physics backend is not configured on this server')
        extra={k:params[k] for k in ('maximum_push_length_m','geometry_id','run_until_goal') if k in params}
        params={'initial_pose':initial,'goal_pose':target,'speed_mps':model.speed_mps,'max_steps':model.max_steps,**extra}
        if simulation is not None:params['simulation_backend']=simulation
    dumps(params)
    return {'task':task,'mode':mode,'parameters':params}
