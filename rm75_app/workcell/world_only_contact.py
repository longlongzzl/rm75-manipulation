"""Simulation-only, temporary world-cost filtering; never disable robot spheres.

Caller must hold the native GPU lock and qualify the segment before entry.
CUDA graph replay is deliberately unsupported: a Python scope cannot constrain
an already captured/replayed graph. Unrecognized native layouts fail closed.
"""
from contextlib import contextmanager
import threading


FINGER_LINKS = frozenset({
    'gripper_Left_1_Link', 'gripper_Left_2_Link', 'gripper_Left_Support_Link',
    'gripper_Right_1_Link', 'gripper_Right_2_Link', 'gripper_Right_Support_Link',
    'left_pad', 'right_pad',
})


class WorldOnlyContactUnsupported(BaseException):
    """Cannot be swallowed by native except Exception retry paths."""


@contextmanager
def world_only_fingers(planner):
    """Clone ONLY the world cost/constraint input; self collision sees originals."""
    if getattr(planner, '_cuda_graph_batch_ik_solvers', {}):
        raise WorldOnlyContactUnsupported('cached_cuda_graph_ik_present')
    if getattr(planner, '_disabled_collision_links', set()):
        raise WorldOnlyContactUnsupported('preexisting_disabled_link_spheres')
    if getattr(planner, '_disabled_world_obstacles', set()):
        raise WorldOnlyContactUnsupported('preexisting_disabled_world_objects')
    bindings = {}
    for owner in (planner.motion_gen, planner.ik_solver):
        if getattr(owner, 'use_cuda_graph', None) is not False:
            raise WorldOnlyContactUnsupported('cuda_graph_disabled_not_confirmed')
        for rollout in owner.get_all_rollout_instances():
            config = rollout.kinematics.kinematics_config
            mapping = config.link_sphere_idx_map
            names = config.link_name_to_idx_map
            indices = sorted({i for name in FINGER_LINKS if name in names
                              for i, link_id in enumerate(mapping.tolist()) if link_id == names[name]})
            if not indices:
                raise WorldOnlyContactUnsupported('finger_sphere_mapping_missing')
            self_checks = [getattr(rollout, name, None) for name in
                           ('robot_self_collision_cost', 'robot_self_collision_constraint')]
            if not any(check is not None and check.enabled for check in self_checks):
                raise WorldOnlyContactUnsupported('active_self_collision_not_confirmed')
            for name in ('primitive_collision_cost', 'primitive_collision_constraint'):
                cost = getattr(rollout, name, None)
                if cost is None or not cost.enabled:
                    continue
                signature = (tuple(indices), len(mapping))
                if id(cost) in bindings and bindings[id(cost)][2:] != signature:
                    raise WorldOnlyContactUnsupported('shared_cost_sphere_mapping_mismatch')
                bindings[id(cost)] = (cost, cost.forward, *signature)
    if not bindings:
        raise WorldOnlyContactUnsupported('world_costs_missing')
    calls = {'world_filter_calls': 0, 'world_cost_instances': len(bindings),
             'links': sorted(FINGER_LINKS), 'self_collision_input_modified': False}
    restored = []
    thread_id = threading.get_ident()
    try:
        for cost, original, indices, count in bindings.values():
            def forward(spheres, *args, _original=original, _indices=indices, _count=count, **kwargs):
                if threading.get_ident() != thread_id:
                    return _original(spheres, *args, **kwargs)
                if spheres.shape[-2:] != (_count, 4):
                    raise WorldOnlyContactUnsupported('unexpected_world_sphere_shape')
                filtered = spheres.clone()
                filtered[..., list(_indices), 3] = -100.0
                calls['world_filter_calls'] += 1
                return _original(filtered, *args, **kwargs)
            restored.append((cost, original))
            cost.forward = forward
        yield calls
    finally:
        for cost, original in reversed(restored):
            cost.forward = original
