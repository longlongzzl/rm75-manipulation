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
        if set(params)-{'object_name'}:
            raise ValueError('PickPlace reuses original place rules; only object_name is selected here')
        name=params.get('object_name','')
        allowed=profile.get('pickplace',{}).get('object_names',['lvmukuai','carriot','shuazi','gluestick','bi','tennis'])
        if name not in allowed or not re.fullmatch('[A-Za-z0-9_-]{1,80}',name):
            raise ValueError('Unknown/unapproved PickPlace asset')
    elif task=='magnetic':
        if set(params)!={'design'}:
            raise ValueError('Magnetic task requires the original builder design JSON')
        design=validate_design(params['design'])
        for p in design.payload['pieces']:
            if not re.fullmatch('[A-Za-z0-9_-]{1,100}',str(p.get('role') or p.get('id'))):
                raise ValueError('Use path-safe original piece ids/roles')
        params['design']=design.payload
    else:
        if set(params)-{'initial_pose','goal_pose','speed_mps','max_steps'}:
            raise ValueError('Unsupported PushT parameter; geometry/safety belong in the machine profile')
        target=vector(params.get('goal_pose'),3,'goal_pose').tolist()
        initial=vector(params.get('initial_pose',[.35,0,0]),3,'initial_pose').tolist()
        cfg=dict(profile.get('pusht',{}).get('model',{}))
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
        params={'initial_pose':initial,'goal_pose':target,'speed_mps':model.speed_mps,'max_steps':model.max_steps}
    # Reject NaN/duplicate etc even in unused preserved JSON metadata.
    dumps(params)
    return {'task':task,'mode':mode,'parameters':params}
