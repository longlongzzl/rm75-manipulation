"""Retain original native collision construction recipes during owned startup.

Rebuilding cooked convex vertices changes geometry. Reuse the ORIGINAL builder
and file-based load path instead; native readback still checks every shape.
The builder's scene, render records and material objects are not shared.
"""
from __future__ import annotations

from contextlib import contextmanager
import copy
import hashlib
from pathlib import Path
import threading
from unittest.mock import patch

from .scene import SceneInvalid


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

    def build_body(self):
        import sapien
        from .native_body_mirror import native_pose

        for filename, expected in self.files.items():
            if hashlib.sha256(Path(filename).read_bytes()).hexdigest() != expected:
                raise SceneInvalid('Original collision construction file changed')
        builder = copy.copy(self.prototype)
        builder._plane_collision_poses = set()
        builder.collision_records = []
        for source, matrix, material in self.records:
            record = copy.copy(source)
            record.scale = copy.deepcopy(source.scale)
            if hasattr(source, 'decomposition_params'):
                record.decomposition_params = copy.deepcopy(source.decomposition_params)
            record.pose = native_pose(matrix)
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
