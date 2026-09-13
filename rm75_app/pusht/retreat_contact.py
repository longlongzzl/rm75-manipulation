"""Native retreat contact escape using only the original permitted link set."""
import numpy as np
from .cartesian_ik import PushPathRejected

class RetreatContactRejected(PushPathRejected):
    """A valid contact sequence violates the unchanged retreat policy."""


def validate_contact_escape(samples,allowed):
    initial=None;previous={};count=0
    for rows in samples:
        current={};count+=1
        for row in rows:
            if (not isinstance(row,dict) or row.get('collision_type') not in ('world','self')
                    or not isinstance(row.get('robot_link'),str)
                    or (row['collision_type']=='world' and not isinstance(row.get('world_object'),str))):
                raise PushPathRejected('Invalid retreat contact identity')
            depth=float(row['penetration_m'])
            if not np.isfinite(depth) or depth<=0:raise PushPathRejected('Invalid retreat contact depth')
            if (row.get('collision_type')!='world' or
                    row.get('world_object') not in ('pusht_target_0','pusht_target_1') or
                    row.get('robot_link') not in allowed):
                raise RetreatContactRejected(f'Retreat forbidden contact: {row}')
            key=(row['robot_link'],row['world_object'])
            current[key]=max(current.get(key,0.),depth)
        if initial is None:initial=dict(current)
        else:
            for key,depth in current.items():
                if key not in previous:
                    raise RetreatContactRejected(f'Retreat introduced or renewed contact: {key}')
                # Numerical comparison allowance only, no collision margin change.
                if depth>previous[key]+1e-7 or depth>initial[key]+1e-7:
                    raise RetreatContactRejected(f'Retreat contact deepened: {key}')
        previous=current
    if count<2:raise PushPathRejected('Retreat audit needs at least two samples')
    if previous:raise RetreatContactRejected('Retreat ends in contact')
    return dict(samples=count,initial_contact_pairs=len(initial),final_contact_pairs=0,
                initial_max_penetration_m=max(initial.values(),default=0.))


def audit_retreat_contact_escape(executor,path):
    backend=executor.backend;planner=backend._ensure_planner();mods=backend._import_modules()
    path=np.asarray(path,dtype=float)
    if path.ndim!=2 or path.shape[1]!=7 or not 2<=len(path)<=100000 or not np.isfinite(path).all():
        raise PushPathRejected('Invalid retreat path')
    def samples():
        for offset in range(0,len(path),32):
            batch=path[offset:offset+32]
            state=mods['JointState'].from_position(mods['torch'].as_tensor(batch,
                device=planner.device_cfg.device,dtype=planner.device_cfg.dtype),
                joint_names=list(executor.names))
            rows=backend._collision_diagnostics_for_states(planner,{'path':state})
            grouped=[[] for _ in batch]
            for row in rows:
                index=row['candidate_index']
                if type(index) is not int or not 0<=index<len(batch):
                    raise PushPathRejected('Invalid native retreat sample identity')
                grouped[index].append(row)
            yield from grouped
    result=validate_contact_escape(samples(),executor.allowed)
    executor.events.emit('retreat_contact_escape_audited',**result,
        allowed_links=sorted(executor.allowed),new_contact_allowed=False,
        static_or_self_contact_allowed=False)
    return result
