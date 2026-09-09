"""Planner-local participants in the existing GPU-locked world transaction.

No second global state store and no collision exemptions. Adapters register
their own capture/restore operations so their metadata follows the same world.
"""
import copy
import threading


def register(planner, name, participant):
    participants = getattr(planner, '_rm75_phase_participants', None)
    if participants is None:
        participants = {}; planner._rm75_phase_participants = participants
    if name in participants and participants[name] is not participant:
        raise RuntimeError('Duplicate phase-state participant: '+name)
    participants[name] = participant


def evidence(planner):
    present = {item.name for item in planner._world.objects}
    disabled = set(planner._disabled_world_obstacles)
    names = getattr(planner, 'world_collision_checker_obstacle_names', lambda: sorted(present | disabled))()
    raw = getattr(planner, 'robot_cfg_dict', {})
    kin = raw.get('robot_cfg', raw).get('kinematics', {})
    participants = getattr(planner, '_rm75_phase_participants', {})
    return dict(planner_identity=id(planner), thread_identity=threading.get_ident(),
        expected_world_objects=sorted(present), cache_rows=sorted(names),
        disabled_objects=sorted(disabled), required_objects_disabled=sorted(disabled & present),
        absent_cache_rows_disabled=sorted(disabled-present),
        attached=bool(getattr(planner, 'attached_object_active', False)),
        attached_sphere_count=int(getattr(planner, 'get_attached_sphere_count', lambda: 0)()),
        gripper_locks=copy.deepcopy(kin.get('lock_joints', {})),
        adapter_state={name:copy.deepcopy(p.capture()) for name,p in participants.items()})


def emit(planner, phase, label, **extra):
    for participant in getattr(planner, '_rm75_phase_participants', {}).values():
        participant.emit(dict(event='planner_phase_state', phase=phase, step_id=label,
                              **evidence(planner), **extra))
