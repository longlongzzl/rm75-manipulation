"""Report missing/invalid PushT inputs without connecting to any device."""
from .model import Config
from rm75_app.workcell.transforms import rigid,vector
from rm75_app.workcell.io import finite


def planning_qualification(profile):
    section=profile.get('pusht',{})
    motion=section.get('motion',{})
    observer=section.get('observer',{})
    errors=[]
    def check(name,fn):
        try: fn()
        except (ValueError,TypeError,KeyError) as exc: errors.append({'field':name,'error':str(exc)})
    check('pusht.model',lambda:Config.from_dict(section.get('model',{})))
    for key,low,high in [('push_tcp_z_m',-1,2),('object_centroid_z_m',-1,2),
                         ('object_height_m',.001,.5),('hover_clearance_m',.02,.2)]:
        check('pusht.motion.'+key,lambda key=key,low=low,high=high:finite(motion.get(key),key,low,high))
    check('pusht.motion.tool_quaternion_wxyz',lambda:vector(motion.get('tool_quaternion_wxyz'),4,'tool_quaternion_wxyz'))
    for key in ('tool_frame','pusher_contact_links','static_collision_objects'):
        if not motion.get(key): errors.append({'field':'pusht.motion.'+key,'error':'missing'})
    if motion.get('tool_collision_geometry_verified') is not True:
        errors.append({'field':'pusht.motion.tool_collision_geometry_verified','error':'unqualified'})
    if observer.get('kind')=='realsense_apriltag':
        for key in ('T_base_camera','T_marker_object'):
            check('pusht.observer.'+key,lambda key=key:rigid(observer.get(key),key))
        check('pusht.observer.marker_size_m',lambda:finite(observer.get('marker_size_m'),'marker_size_m',.005,.2))
    elif observer.get('kind')=='json_live':
        if not observer.get('observation_file'):
            errors.append({'field':'pusht.observer.observation_file','error':'missing live base-frame tracker source'})
    else:
        errors.append({'field':'pusht.observer.kind','error':'unsupported observation source'})
    return {'planning_inputs_complete':not errors,'errors':errors,
            'integration_qualified':section.get('integration_qualified') is True,
            'hardware_reviewed':profile.get('hardware',{}).get('hardware_reviewed') is True,
            'motion_authorized':False}
