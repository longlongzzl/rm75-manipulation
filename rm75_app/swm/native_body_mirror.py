"""Copy primary native rigid-body geometry into the owned private robot world.

No source setters and no physics stepping. Planning proxy meshes are NOT used
as physical actor geometry. Unsupported native shapes fail closed.
"""
from __future__ import annotations

import copy

import numpy as np

from .native_bootstrap import _array
from .scene import SceneInvalid, digest, transform


BODY_SCALARS = ('linear_damping', 'angular_damping', 'max_linear_velocity',
    'max_angular_velocity', 'max_contact_impulse', 'max_depenetration_velocity',
    'sleep_threshold', 'solver_position_iterations', 'solver_velocity_iterations')
SHAPE_SCALARS = ('density', 'contact_offset', 'rest_offset', 'patch_radius', 'min_patch_radius')


def _apply_kinematic_mode(body, measured):
    if type(measured) is not bool:
        raise SceneInvalid('Measured kinematic mode must be boolean')
    # Native SAPIEN resets automatic mass even on a same-value assignment.
    if body.kinematic != measured:
        body.kinematic = measured


def native_pose(matrix):
    import sapien
    from rm75_app.pickplace.cached_scene import matrix_to_quaternion_wxyz
    matrix = transform(matrix)
    return sapien.Pose(matrix[:3, 3], matrix_to_quaternion_wxyz(matrix[:3, :3]))


def pose_matrix(pose):
    return _array(pose.to_transformation_matrix()).reshape(4, 4)


def native_body(actor):
    import sapien
    entities = getattr(actor, '_objs', ())
    if len(entities) != 1:
        raise SceneInvalid('One native primary entity per SWM instance is required')
    body = entities[0].find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
    if body is None:
        body = entities[0].find_component_by_type(sapien.physx.PhysxRigidStaticComponent)
    if body is None:
        raise SceneInvalid('Primary actor has no supported native rigid body')
    return body


def shape_state(shape, object_from_actor):
    kind = type(shape).__name__.removeprefix('PhysxCollisionShape')
    if kind == 'Box':
        geometry = dict(half_size=_array(shape.half_size).tolist())
    elif kind == 'Sphere':
        geometry = dict(radius=float(shape.radius))
    elif kind in ('Capsule', 'Cylinder'):
        geometry = dict(radius=float(shape.radius), half_length=float(shape.half_length))
    elif kind == 'ConvexMesh':
        vertices = _array(shape.vertices).reshape(-1, 3)
        vertices = vertices[np.lexsort(vertices.T[::-1])]
        geometry = dict(vertices=vertices.tolist(), scale=_array(shape.scale).tolist())
    elif kind == 'Plane':
        geometry = {}
    else:
        raise SceneInvalid(f'Unsupported original native collision shape: {kind}')
    material = shape.physical_material
    return dict(kind=kind, geometry=geometry,
        T_object_shape=(object_from_actor @ pose_matrix(shape.local_pose)).tolist(),
        material={key: float(getattr(material, key)) for key in ('static_friction', 'dynamic_friction', 'restitution')},
        collision_groups=list(shape.get_collision_groups()),
        properties={key: float(getattr(shape, key)) for key in SHAPE_SCALARS})


def body_state(body, actor_to_object=None):
    import sapien
    object_from_actor = np.linalg.inv(transform(np.eye(4) if actor_to_object is None else actor_to_object))
    dynamic = isinstance(body, sapien.physx.PhysxRigidDynamicComponent)
    state = dict(kind='dynamic' if dynamic else 'static',
        shapes=[shape_state(shape, object_from_actor) for shape in body.collision_shapes])
    if dynamic:
        state.update(kinematic=bool(body.kinematic), auto_compute_mass=bool(body.auto_compute_mass),
            mass=float(body.mass), inertia=_array(body.inertia).tolist(),
            T_object_cmass=(object_from_actor @ pose_matrix(body.cmass_local_pose)).tolist(),
            disable_gravity=bool(body.disable_gravity), locked_motion_axes=list(body.get_locked_motion_axes()),
            properties={key: getattr(body, key) for key in BODY_SCALARS})
    # Reject nonfinite native state instead of silently dropping physical fields.
    digest(state)
    return state


def physics_geometry_state(state):
    return dict(schema='rm75_native_physics_geometry_v1', frame='object',
        body_kind=state['kind'], shapes=[copy.deepcopy({key: shape[key]
            for key in ('kind', 'geometry', 'T_object_shape')}) for shape in state['shapes']])


def compare_native_state(expected, actual, path='body'):
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or set(expected) != set(actual):
            raise SceneInvalid(f'Native property coverage differs: {path}')
        for key in expected:
            compare_native_state(expected[key], actual[key], path + '.' + key)
    elif isinstance(expected, list):
        if not isinstance(actual, list) or len(expected) != len(actual):
            raise SceneInvalid(f'Native geometry count differs: {path}')
        for index, (left, right) in enumerate(zip(expected, actual)):
            compare_native_state(left, right, path + f'[{index}]')
    elif type(expected) in (int, bool):
        if type(actual) is not type(expected) or expected != actual:
            raise SceneInvalid(f'Native integer/boolean identity differs: {path}')
    elif type(expected) is float:
        if not np.isclose(expected, actual, atol=1e-6, rtol=1e-6):
            raise SceneInvalid(f'Native property differs: {path}')
    elif expected != actual:
        raise SceneInvalid(f'Native property differs: {path}')


class PrivateActor:
    """Small facade for the existing SapienScenePort, backed by native storage."""
    def __init__(self, entity, body):
        self._objs = [entity]
        self.entity, self.body = entity, body

    @property
    def pose(self):
        return self.entity.pose

    def set_pose(self, value):
        self.entity.set_pose(value)
        if hasattr(self.body, 'kinematic') and self.body.kinematic:
            self.body.kinematic_target = value

    def _velocity(self, name):
        return _array(getattr(self.body, name)) if hasattr(self.body, name) else np.zeros(3)

    def _set_velocity(self, name, value):
        value = np.asarray(value, dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise SceneInvalid('Finite measured object velocity required')
        if hasattr(self.body, name):
            setattr(self.body, name, value)
        elif np.any(value != 0):
            raise SceneInvalid('A native static actor cannot acquire velocity')

    linear_velocity = property(lambda self: self._velocity('linear_velocity'))
    angular_velocity = property(lambda self: self._velocity('angular_velocity'))

    def set_linear_velocity(self, value):
        self._set_velocity('linear_velocity', value)

    def set_angular_velocity(self, value):
        self._set_velocity('angular_velocity', value)


class NativeBodyMirror:
    def __init__(self, registration, robot_port, *, resources):
        import sapien
        from .native_robot_mirror import SapienRobotStatePort
        from .native_construction import recipe_for_actor
        if not isinstance(robot_port, SapienRobotStatePort) or robot_port.closed:
            raise SceneInvalid('Live owned private robot world is required')
        self.actors, self.expected, self.asset_bindings = {}, {}, {}
        self.robot_port = robot_port
        resources.callback(self.close)
        for oid, binding in registration.source.bindings.items():
            expected = body_state(native_body(binding.actor), binding.T_actor_object)
            asset_id = registration.world._objects[oid]['asset_id']
            asset = registration.world.assets[asset_id]
            geometry_id = digest(physics_geometry_state(expected))
            if asset.get('native_physics_geometry_sha256') != geometry_id:
                raise SceneInvalid('Native physical geometry differs from the registered asset')
            self.asset_bindings[oid] = geometry_id
            entity = sapien.Entity()
            entity.name = oid
            recipe = recipe_for_actor(registration.source.primary.construction_recipes, binding.actor)
            body = recipe.build_body(binding.T_actor_object,
                                     auto_compute_mass=expected.get("auto_compute_mass"))
            if len(body.collision_shapes) != len(expected['shapes']):
                raise SceneInvalid('Original collision loader omitted or added a native shape')
            for shape, state in zip(body.collision_shapes, expected['shapes']):
                shape.physical_material = sapien.physx.PhysxMaterial(**state['material'])
                shape.set_collision_groups(state['collision_groups'])
                for key, value in state['properties'].items():
                    if getattr(shape, key) != value:
                        setattr(shape, key, value)
            if expected['kind'] == 'dynamic':
                _apply_kinematic_mode(body, expected['kinematic'])
                body.disable_gravity = expected['disable_gravity']
                body.set_locked_motion_axes(expected['locked_motion_axes'])
                for key, value in expected['properties'].items():
                    setattr(body, key, value)
                if not expected['auto_compute_mass']:
                    body.mass = expected['mass']
                    body.inertia = expected['inertia']
                    body.cmass_local_pose = native_pose(expected['T_object_cmass'])
            entity.add_component(body)
            self.robot_port._scene.add_entity(entity)
            self.actors[oid] = PrivateActor(entity, body)
            self.expected[oid] = expected
        self.readback()

    def readback(self):
        rows = {}
        for oid, actor in self.actors.items():
            actual = body_state(actor.body)
            compare_native_state(self.expected[oid], actual, oid)
            geometry_id = digest(physics_geometry_state(actual))
            if geometry_id != self.asset_bindings[oid]:
                raise SceneInvalid('Private native physical geometry asset identity changed')
            rows[oid] = dict(native_physics_geometry_sha256=geometry_id,
                shape_count=len(actual['shapes']), body_kind=actual['kind'],
                shape_kinds=[shape['kind'] for shape in actual['shapes']],
                mass=actual.get('mass'), inertia=actual.get('inertia'),
                expected_native_digest=digest(self.expected[oid]), actual_native_digest=digest(actual))
        return dict(source='native_CPU_PhysX_shape_and_body_readback', objects=rows,
            original_body_geometry_and_properties_aligned=True, primary_world_mutated=False,
            attachment_qualified=False, robot_collision_policy_qualified=False, hardware_qualified=False)

    def close(self):
        for actor in self.actors.values():
            self.robot_port._scene.remove_entity(actor.entity)
        self.actors.clear()
        self.expected.clear()
        self.asset_bindings.clear()
