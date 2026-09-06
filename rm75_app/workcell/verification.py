"""Optional final verification from a real observation producer, not runtime caches.

Targets are provided by the trusted machine/task profile. The producer only
supplies measured poses; it cannot choose its own success thresholds/targets.
"""
from __future__ import annotations
import time
from pathlib import Path
import numpy as np
from .io import read_json,finite
from .transforms import rigid,rotation_error


def wait_for_poses(config,*,after,stop):
    deadline=time.monotonic()+finite(config.get('timeout_s',10),'timeout_s',.1,120)
    targets={k:rigid(v,f'target {k}') for k,v in config['target_T_base_objects'].items()}
    if not targets:
        raise ValueError('Verification requires explicit task targets')
    p_tol=finite(config.get('position_tolerance_m',.008),'position_tolerance_m',.0001,.03)
    r_tol=finite(config.get('rotation_tolerance_rad',.12),'rotation_tolerance_rad',.001,.3)
    while time.monotonic()<deadline:
        stop.check()
        try:
            raw=read_json(Path(config['observation_file']))
            if raw.get('schema')!='rm75_object_observations_v1' or raw.get('frame')!='base_link' or raw.get('source') not in ('live_tracker','live_sam6d','realsense_apriltag'):
                raise ValueError('Unqualified verification observation')
            timestamp=finite(raw.get('captured_at'),'captured_at',1)
            now=time.time()
            if not after<timestamp<=now+.25 or now-timestamp>2:
                stop.wait(.05);continue
            if not raw.get('session_id') or type(raw.get('sequence')) is not int:
                raise ValueError('Verification needs capture-session/sequence evidence')
            measured=raw['objects'];errors={}
            for key,target in targets.items():
                item=measured[key]
                if finite(item.get('confidence',0),'confidence',0,1)<.8:
                    raise ValueError('Low confidence in verification')
                actual=rigid(item['T_base_object'],f'observed {key}')
                errors[key]={'position_m':float(np.linalg.norm(actual[:3,3]-target[:3,3])),
                             'rotation_rad':rotation_error(actual[:3,:3],target[:3,:3])}
            success=all(e['position_m']<=p_tol and e['rotation_rad']<=r_tol for e in errors.values())
            return {'task_success':success,'verification':'fresh_measured_object_poses',
                    'captured_at':timestamp,'errors':errors,
                    'magnetic_force_verified':False}
        except (OSError,KeyError):
            pass
        stop.wait(.05)
    return {'task_success':None,'verification':'fresh_measurement_timeout'}
