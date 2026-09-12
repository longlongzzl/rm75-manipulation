"""Read the owned primary world into the EXISTING shared collision compiler.

No primary-world setters, execution, planner construction or alternate proxy
algorithm. This is idle collision synchronization, not task qualification.
"""
from __future__ import annotations

from dataclasses import replace
import hashlib
from pathlib import Path
import time

import numpy as np

from .native_bootstrap import _array
from .scene import SceneInvalid, digest, pose_error, transform


def _file_hash(path):
    return hashlib.sha256(Path(path).expanduser().read_bytes()).hexdigest()


def compile_primary_collision_scene(primary):
    from rm75_app.assets.object_specs import get_object_spec, resolve_object_spec_scales
    from rm75_app.core.frames import MANISKILL_TABLE_COLLISION_NAME
    from rm75_app.execution.maniskill_scene import robot_base_transform, _pose_matrix
    from rm75_app.execution.maniskill_task_bridge import _pose_matrix as actor_pose
    from rm75_app.orchestration.multi_object_executor import SceneObjectState, TaskSceneState
    from rm75_app.pickplace.atom_task_builder import AtomTaskBuilderConfig, FixedSceneAtomTaskBuilder
    from rm75_app.pickplace.cached_scene import matrix_to_quaternion_wxyz
    from rm75_app.planning.contracts import CollisionObject, PlanningScene, Pose
    import sapien

    measured = primary.read_state()
    if np.max(np.abs(measured['velocities'])) > .001:
        raise SceneInvalid('Collision synchronization requires measured idle joints')
    base = robot_base_transform(primary.env)
    if base is None or not np.allclose(base[:3, :3], np.eye(3), atol=1e-6):
        raise SceneInvalid('Original task builder requires a measured translation-only base calibration')
    registry = primary.demo._single_scene_object_registry
    if set(registry) != set(measured['objects']):
        raise SceneInvalid('Native inventory changed during collision synchronization')
    states, assets = {}, {}
    for oid, pose in measured['objects'].items():
        spec = get_object_spec(oid)
        if spec is None:
            raise SceneInvalid(f'No original shared asset for {oid}')
        native = registry[oid]['object_args']
        _, sim_scale = resolve_object_spec_scales(spec)
        actual_mesh = _file_hash(native.mesh_file)
        actual_sim = _file_hash(native.sim_asset_file)
        if (actual_mesh != _file_hash(spec.mesh_file)
                or actual_sim != _file_hash(spec.sim_asset_file or spec.mesh_file)
                or not np.isclose(float(native.sim_asset_scale), sim_scale, atol=1e-9, rtol=0)):
            raise SceneInvalid(f'Primary and shared compiler asset/scale differ for {oid}')
        states[oid] = SceneObjectState(oid, oid, np.asarray(pose))
        assets[oid] = dict(mesh_sha256=actual_mesh, simulation_sha256=actual_sim,
                          simulation_scale=float(sim_scale))
    builder = FixedSceneAtomTaskBuilder(config=AtomTaskBuilderConfig(
        robot_base_world_xyz_m=tuple(base[:3, 3]), include_maniskill_workspace_table=False))
    compiled = builder._planning_scene(TaskSceneState(states, revision=measured['sequence']))
    objects = list(compiled.objects)
    infrastructure = {}

    def add_box(name, world_pose, dimensions, source):
        world_pose = transform(world_pose)
        dimensions = np.asarray(dimensions, dtype=float)
        if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
            raise SceneInvalid('Invalid native infrastructure dimensions')
        planning_pose = np.linalg.inv(base) @ world_pose
        objects.append(CollisionObject(name, 'cuboid', Pose(planning_pose[:3, 3],
            matrix_to_quaternion_wxyz(planning_pose[:3, :3])), dimensions=dimensions.tolist(),
            metadata=dict(fixed=True, source=source, world_pose=world_pose.tolist())))
        infrastructure[name] = dict(source=source, world_pose=world_pose.tolist(),
                                     dimensions=dimensions.tolist())

    # Use actual native table collision geometry, not its visual mesh bounds or
    # an assumed world centre. The original table is one kinematic box actor.
    table = primary.env.unwrapped.table_scene.table
    if len(table._objs) != 1:
        raise SceneInvalid('Single primary table instance required')
    entity = table._objs[0]
    body = entity.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
    if body is None:
        body = entity.find_component_by_type(sapien.physx.PhysxRigidStaticComponent)
    if body is None or len(body.collision_shapes) != 1:
        raise SceneInvalid('Original table collision geometry changed')
    shape = body.collision_shapes[0]
    if not isinstance(shape, sapien.physx.PhysxCollisionShapeBox):
        raise SceneInvalid('Original table is no longer a collision box')
    add_box(MANISKILL_TABLE_COLLISION_NAME, actor_pose(table) @ _pose_matrix(shape.local_pose),
            2. * _array(shape.half_size), 'native_SAPIEN_table_collision_shape')

    native_actors = {str(actor.name): actor for actor in primary.env.unwrapped._scene_obstacle_actors}
    for item in primary.demo.scene_obstacles:
        oid = item['object_name']
        if oid in measured['objects']:
            continue
        if oid not in ('virtual_side_wall', 'virtual_top_wall'):
            raise SceneInvalid(f'Unbound original infrastructure: {oid}')
        actor = native_actors.get(item['actor_name'])
        if actor is None or item.get('planner_collision') is not True:
            raise SceneInvalid('Original planning guard actor/geometry missing')
        add_box(oid, actor_pose(actor), item['planner_box_size'],
                'original_virtual_wall_policy_and_native_actor_pose')
    names = [obj.name for obj in objects]
    if len(set(names)) != len(names):
        raise SceneInvalid('Duplicate native collision identity')
    evidence = dict(primary_sequence=measured['sequence'],
        capture_started_at=measured['capture_started_at'], capture_finished_at=time.monotonic(),
        input_sha256=primary.contract['sha256'], T_world_base=base.tolist(),
        object_ids=sorted(measured['objects']), asset_bindings=assets,
        infrastructure=infrastructure, primary_world_mutated=False)
    revision = digest(evidence)
    return PlanningScene(tuple(objects), revision=revision), evidence


def read_curobo_collision_ack(backend, scene):
    """Read actual GPU storage, including disabled/misplaced obstacle negatives."""
    from scipy.spatial.transform import Rotation

    checker = backend._ensure_planner().scene_collision_checker
    expected = backend._to_formal_curobo_scene_dict(scene)
    if not isinstance(expected, dict) or set(expected) - {'cuboid'}:
        raise SceneInvalid('Native mesh/multi-environment readback is not installed')
    expected = expected.get('cuboid', {})
    names = checker.get_obstacle_names()
    if len(names) != len(set(names)) or set(names) != set(expected):
        raise SceneInvalid('Native GPU scene identity/count differs from complete scene')
    data = checker.data.cuboids
    if data is None:
        raise SceneInvalid('Native cuboid GPU storage missing')
    count = int(data.count[0].item())
    dims = _array(data.dims[0, :count, :3])
    inv_poses = _array(data.inv_pose[0, :count, :7])
    enabled = _array(data.enable[0, :count])
    native_names = list(data.names[0][:count])
    if len(native_names) != len(expected) or set(native_names) != set(expected):
        raise SceneInvalid('Native GPU slot identity/count mismatch')

    def matrix(pose):
        pose = np.asarray(pose)
        out = np.eye(4)
        out[:3, :3] = Rotation.from_quat(pose[[4, 5, 6, 3]]).as_matrix()
        out[:3, 3] = pose[:3]
        return out

    rows = []
    for index, name in enumerate(native_names):
        reference = expected[name]
        actual_pose = np.linalg.inv(matrix(inv_poses[index]))
        p, r = pose_error(actual_pose, matrix(reference['pose']))
        if (enabled[index] != 1 or not np.allclose(dims[index], reference['dims'], atol=1e-6, rtol=0)
                or p > 1e-5 or r > 1e-3):
            raise SceneInvalid(f'Native GPU geometry/enable readback differs for {name}')
        rows.append(dict(name=name, enabled=True, dimensions=dims[index].tolist(),
                         T_base_proxy=actual_pose.tolist()))
    return dict(scene_revision=scene.revision, source='native_GPU_collision_tensors',
                coordinate_frame='base_link', count=count, obstacles=rows,
                robot_state_qualified=False, attachment_qualified=False)
