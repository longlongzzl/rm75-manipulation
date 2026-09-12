"""Register original primary assets in SWM without changing their object frame.

Only private, newly-created artifact directories are written. Original meshes,
scales and shared collision proxies are reused, not shrunk or recentered.
Registration alone does not qualify a mirror, attachment, or atomic worker.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np

from .native_capture import NativePrimaryCapture, PrimaryActorBinding
from .scene import SceneInvalid, SceneWorldModel, digest, identifier, transform


def file_digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_metric_asset(directory, name, mesh, proxy, local, provenance):
    import trimesh

    name = identifier(name)
    local = transform(local)
    vertices = np.asarray(mesh.vertices)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices)
            or not np.isfinite(vertices).all() or not len(mesh.faces)):
        raise SceneInvalid('Finite nonempty metric mesh geometry required')
    if proxy.kind == 'cuboid':
        dimensions = np.asarray(proxy.dimensions, dtype=float)
        if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
            raise SceneInvalid('Shared cuboid dimensions must be positive')
        collision = trimesh.creation.box(extents=dimensions)
        shape = dict(collision_kind='cuboid', collision_dimensions_m=dimensions.tolist())
    elif proxy.kind == 'sphere':
        radius = float(proxy.radius)
        if not np.isfinite(radius) or radius <= 0:
            raise SceneInvalid('Shared sphere radius must be positive')
        collision = trimesh.creation.icosphere(subdivisions=2, radius=radius)
        shape = dict(collision_kind='sphere', collision_radius_m=radius)
    else:
        raise SceneInvalid('Unregistered original primary collision proxy kind')
    mesh_path = directory / (name + '.metric.ply')
    collision_path = directory / (name + '.collision.ply')
    if mesh_path.exists() or collision_path.exists():
        raise FileExistsError('Do not overwrite registered metric assets')
    mesh.export(mesh_path)
    collision.export(collision_path)
    exported = trimesh.load(mesh_path, force='mesh', process=False)
    if not np.allclose(exported.bounds, mesh.bounds, atol=1e-6, rtol=0):
        raise SceneInvalid('Metric export changed the original model frame or scale')
    return dict(source='legacy', units='m', metric_scale_verified=True,
        scale_evidence='Original shared scale applied without recentering; exported metric bounds read back',
        mesh_path=str(mesh_path), mesh_sha256=file_digest(mesh_path),
        collision_path=str(collision_path), collision_sha256=file_digest(collision_path),
        T_object_collision=local.tolist(), original_asset_name=name,
        provenance=provenance, physics_model_qualified=False, **shape)


@dataclass(frozen=True)
class PrimaryRegistration:
    world: SceneWorldModel
    source: NativePrimaryCapture
    original_scene: object
    evidence: dict


def register_primary_scene(primary, directory, *, sensor_session):
    import trimesh
    from rm75_app.assets.object_specs import get_object_spec, resolve_object_spec_scales
    from rm75_app.core.frames import MANISKILL_TABLE_COLLISION_NAME
    from rm75_app.execution.maniskill_task_bridge import _pose_matrix as actor_pose
    from .native_planning_scene import compile_primary_collision_scene

    directory = Path(directory).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=False)
    original, evidence = compile_primary_collision_scene(primary)
    proxies = {obj.name: obj for obj in original.objects}
    assets, objects, bindings = {}, [], {}
    for oid, actor in primary.actors.items():
        spec = get_object_spec(oid)
        if spec is None:
            raise SceneInvalid('Original registered object specification is required')
        mesh_scale, sim_scale = resolve_object_spec_scales(spec)
        source_path = Path(spec.mesh_file).expanduser().resolve()
        if file_digest(source_path) != evidence['asset_bindings'][oid]['mesh_sha256']:
            raise SceneInvalid('Original mesh changed during registration')
        mesh = trimesh.load(source_path, force='mesh', process=False)
        mesh.apply_scale(mesh_scale)
        proxy = proxies[oid]
        local = np.eye(4)
        local[:3, 3] = np.asarray(proxy.metadata['proxy_local_center'])
        asset = write_metric_asset(directory, oid, mesh, proxy, local, dict(
            source_mesh_path=str(source_path), source_mesh_sha256=file_digest(source_path),
            applied_uniform_scale=float(mesh_scale), original_simulation_scale=float(sim_scale),
            original_simulation_sha256=evidence['asset_bindings'][oid]['simulation_sha256'],
            proxy_source='shared_build_automatic_collision_proxy',
            proxy_scale=proxy.metadata['collision_proxy_scale']))
        assets[oid] = asset
        objects.append(dict(id=oid, name=oid, asset_id=oid, fixed=False))
        bindings[oid] = PrimaryActorBinding(actor, asset['mesh_sha256'], np.eye(4))
    native_obstacles = {str(actor.name): actor for actor in primary.env.unwrapped._scene_obstacle_actors}
    for oid, item in evidence['infrastructure'].items():
        proxy = proxies[oid]
        if oid == MANISKILL_TABLE_COLLISION_NAME:
            actor = primary.env.unwrapped.table_scene.table
        else:
            definition = next(row for row in primary.demo.scene_obstacles if row['object_name'] == oid)
            actor = native_obstacles[definition['actor_name']]
        # Infrastructure's registered object frame is its actual collision frame.
        actor_to_object = np.linalg.inv(actor_pose(actor)) @ transform(item['world_pose'])
        mesh = trimesh.creation.box(extents=np.asarray(item['dimensions']))
        asset = write_metric_asset(directory, oid, mesh, proxy, np.eye(4), dict(
            source=item['source'], original_dimensions_m=item['dimensions']))
        assets[oid] = asset
        objects.append(dict(id=oid, name=oid, asset_id=oid, fixed=True))
        bindings[oid] = PrimaryActorBinding(actor, asset['mesh_sha256'], actor_to_object)
    if set(bindings) != set(proxies):
        raise SceneInvalid('Full original collision inventory was not registered')
    calibration = digest(evidence['T_world_base'])
    manifest = dict(schema='rm75_swm_v1', world_frame='base_link', observation_domain='physics',
        calibration_id=calibration, assets=assets, objects=objects)
    world = SceneWorldModel(manifest)
    world.check_assets()
    (directory / 'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False))
    source = NativePrimaryCapture(primary, bindings, T_world_base=evidence['T_world_base'],
        calibration_id=calibration, sensor_session=sensor_session)
    return PrimaryRegistration(world, source, original, dict(
        original_compilation=evidence, manifest_sha256=file_digest(directory / 'manifest.json'),
        metric_asset_files_verified=True, full_mirror_qualified=False,
        task_semantics_qualified=False, physics_model_qualified=False, atomic_worker_qualified=False))
