"""Retain original native collision construction recipes during owned startup.

Rebuilding cooked convex vertices changes geometry. Reuse the ORIGINAL builder
and file-based load path instead; native readback still checks every shape.
The builder's scene, render records and material objects are not shared.
"""
from __future__ import annotations

from contextlib import contextmanager, ExitStack
import copy
import hashlib
from pathlib import Path
import threading
from unittest.mock import patch

import numpy as np

from .scene import SceneInvalid, transform


DRIVE_SETTERS = tuple('set_limit_' + axis for axis in
    ('x', 'y', 'z', 'twist', 'cone', 'pyramid')) + tuple(
    'set_drive_property_' + axis for axis in ('x', 'y', 'z', 'twist', 'swing', 'slerp')) + (
    'set_drive_target', 'set_drive_velocity_target', 'set_inv_mass_scales',
    'set_inv_inertia_scales')


def _drive_argument(value):
    if hasattr(value, 'to_transformation_matrix'):
        return {'native_pose_matrix': transform(value.to_transformation_matrix()).tolist()}
    if isinstance(value, np.ndarray):
        return _drive_argument(value.tolist())
    if isinstance(value, (list, tuple)):
        return [_drive_argument(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    if value is None or type(value) in (str, bool, int, float):
        from .scene import digest
        digest(value)
        return value
    raise SceneInvalid('Unsupported original native drive argument')


def _restore_drive_argument(value):
    if isinstance(value, dict):
        from .native_body_mirror import native_pose
        return native_pose(value['native_pose_matrix'])
    if isinstance(value, list):
        return [_restore_drive_argument(item) for item in value]
    return value


class NativeConstraintRecipe:
    """Preserve original setters: limit getters do not expose locked motions."""
    def __init__(self, drive):
        from .native_body_mirror import pose_matrix
        self.drive = drive
        self.parent = drive.parent
        self.child_entity = drive.entity
        self.parent_pose = pose_matrix(drive.pose_in_parent).tolist()
        self.child_pose = pose_matrix(drive.pose_in_child).tolist()
        self.calls = []

    def build(self, scene, source_links, private_links):
        from .native_body_mirror import native_pose
        parent = [name for name, link in source_links.items() if link is self.parent]
        child = [name for name, link in source_links.items() if link.entity is self.child_entity]
        if len(parent) != 1 or len(child) != 1:
            raise SceneInvalid('Original constraint recipe has foreign endpoints')
        drive = scene.create_drive(private_links[parent[0]], native_pose(self.parent_pose),
                                   private_links[child[0]], native_pose(self.child_pose))
        for name, args, kwargs in self.calls:
            if name not in DRIVE_SETTERS:
                raise SceneInvalid('Unsupported native constraint operation')
            getattr(drive, name)(*[_restore_drive_argument(value) for value in args],
                **{key: _restore_drive_argument(value) for key, value in kwargs.items()})
        return drive


@contextmanager
def capture_drive_construction(scene_type, drive_type):
    owner = threading.get_ident()
    recipes = {}
    original_create = scene_type.create_drive

    def create(scene, *args, **kwargs):
        if threading.get_ident() != owner:
            raise SceneInvalid('Native drive capture is single-worker only')
        drive = original_create(scene, *args, **kwargs)
        recipes[id(drive)] = NativeConstraintRecipe(drive)
        return drive

    def recorder(name, original):
        def call(drive, *args, **kwargs):
            if threading.get_ident() != owner:
                raise SceneInvalid('Native drive capture is single-worker only')
            encoded = (name, [_drive_argument(value) for value in args],
                       {key: _drive_argument(value) for key, value in kwargs.items()})
            if id(drive) not in recipes:
                raise SceneInvalid('Native drive setter without captured construction')
            result = original(drive, *args, **kwargs)
            recipes[id(drive)].calls.append(encoded)
            return result
        return call

    with ExitStack() as stack:
        stack.enter_context(patch.object(scene_type, 'create_drive', create))
        for name in DRIVE_SETTERS:
            if hasattr(drive_type, name):
                stack.enter_context(patch.object(drive_type, name,
                    recorder(name, getattr(drive_type, name))))
        yield recipes


class NativeConstructionRecipe:
    def __init__(self, builder, actor):
        from .native_body_mirror import pose_matrix

        self.entities = tuple(actor._objs)
        self.prototype = copy.copy(builder)
        self.prototype.scene = None
        self.prototype.visual_records = []
        self.prototype.initial_pose = None
        self.prototype.scene_idxs = None
        self.prototype.collision_groups = copy.copy(builder.collision_groups)
        self.prototype._plane_collision_poses = set()
        self.records = []
        self.files = {}
        for source in builder.collision_records:
            record = copy.copy(source)
            record.scale = copy.deepcopy(source.scale)
            if hasattr(source, 'decomposition_params'):
                record.decomposition_params = copy.deepcopy(source.decomposition_params)
            if getattr(source, 'filename', None):
                path = Path(source.filename).expanduser().resolve(strict=True)
                record.filename = str(path)
                self.files[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
            material = source.material
            material_values = tuple(float(getattr(material, name))
                for name in ('static_friction', 'dynamic_friction', 'restitution'))
            matrix = pose_matrix(source.pose).copy()
            record.pose = None
            record.material = None
            self.records.append((record, matrix, material_values))
        self.prototype.collision_records = []

    def build_body(self, actor_to_object, *, auto_compute_mass=None):
        import sapien
        from .native_body_mirror import native_pose

        for filename, expected in self.files.items():
            if hashlib.sha256(Path(filename).read_bytes()).hexdigest() != expected:
                raise SceneInvalid('Original collision construction file changed')
        object_from_actor = np.linalg.inv(transform(actor_to_object))
        builder = copy.copy(self.prototype)
        if auto_compute_mass is not None:
            if type(auto_compute_mass) is not bool:
                raise SceneInvalid("Measured automatic-mass mode must be boolean")
            builder._auto_inertial = auto_compute_mass
        builder._plane_collision_poses = set()
        builder.collision_records = []
        for source, matrix, material in self.records:
            record = copy.copy(source)
            record.scale = copy.deepcopy(source.scale)
            if hasattr(source, 'decomposition_params'):
                record.decomposition_params = copy.deepcopy(source.decomposition_params)
            record.pose = native_pose(object_from_actor @ matrix)
            record.material = sapien.physx.PhysxMaterial(*material)
            builder.collision_records.append(record)
        # Reuse original native loaders and decomposition cache. Its legacy
        # skip-on-cook-error behavior is NOT a success acknowledgement: the
        # caller must compare complete native shape counts and geometry.
        return builder.build_physx_component()


@contextmanager
def capture_actor_construction(builder_type):
    owner = threading.get_ident()
    original = builder_type.build
    recipes = {}

    def build(builder, *args, **kwargs):
        if threading.get_ident() != owner:
            raise SceneInvalid('Native construction capture is single-worker only')
        actor = original(builder, *args, **kwargs)
        recipes[id(actor)] = NativeConstructionRecipe(builder, actor)
        return actor

    with patch.object(builder_type, 'build', build):
        yield recipes


def recipe_for_actor(recipes, actor):
    entities = tuple(actor._objs)
    matches = [recipe for recipe in recipes.values() if recipe.entities == entities]
    if len(matches) != 1:
        raise SceneInvalid('Exactly one original native construction recipe per actor is required')
    return matches[0]
