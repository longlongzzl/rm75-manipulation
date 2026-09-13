"""Apply observed initial motion only to owned private prediction/replay actors."""
import numpy as np
from .scene import SceneInvalid,pose_error


def initial_velocities(obj, *, required):
    measured=obj['measured'];keys=('linear_velocity','angular_velocity')
    present=[key in measured for key in keys]
    if any(present) and not all(present):raise SceneInvalid('Partial initial object velocity evidence')
    if not all(present):
        if required and not obj['fixed']:raise SceneInvalid('Future prediction requires measured initial velocity')
        return np.zeros(3),np.zeros(3),'fixed_constraint' if obj['fixed'] else 'legacy_unspecified_rest_assumption'
    values=[np.asarray(measured[key],float) for key in keys]
    if any(v.shape!=(3,) or not np.isfinite(v).all() for v in values):
        raise SceneInvalid('Invalid initial object velocity')
    if obj['fixed'] and any(np.any(v!=0) for v in values):
        raise SceneInvalid('Fixed object has nonzero initial velocity')
    return *values,'snapshot_velocity_readback'


def apply_private_initial_state(snapshot,actors, *, require_velocities):
    from .native_body_mirror import native_body,native_pose,pose_matrix
    if set(actors)!=set(snapshot['objects']):raise SceneInvalid('Private initial actor coverage differs')
    evidence={}
    for oid,obj in snapshot['objects'].items():
        body=native_body(actors[oid]);linear,angular,source=initial_velocities(obj,required=require_velocities)
        body.entity.set_pose(native_pose(obj['measured']['T_world_object']))
        if not obj['fixed']:
            body.linear_velocity=linear;body.angular_velocity=angular
            actual_linear=np.asarray(body.linear_velocity,float);actual_angular=np.asarray(body.angular_velocity,float)
        else:actual_linear=np.zeros(3);actual_angular=np.zeros(3)
        p,r=pose_error(pose_matrix(body.entity.pose),obj['measured']['T_world_object'])
        if (p>1e-6 or r>1e-6 or not np.allclose(actual_linear,linear,atol=1e-7,rtol=1e-6) or
                not np.allclose(actual_angular,angular,atol=1e-7,rtol=1e-6)):
            raise SceneInvalid('Private initial pose/velocity readback differs')
        evidence[oid]=dict(velocity_source=source,linear_velocity=actual_linear.tolist(),
            angular_velocity=actual_angular.tolist(),position_error_m=p,rotation_error_rad=r)
    return dict(scope='owned_private_snapshot_initialization',objects=evidence,
        complete_velocity_evidence=all(r['velocity_source']!='legacy_unspecified_rest_assumption' for r in evidence.values()),
        observed_world_mutated=False)
