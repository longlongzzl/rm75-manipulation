import copy
import hashlib
import json

import pytest

from rm75_app.swm.scene import SceneInvalid, SceneWorldModel
from rm75_app.swm.physics_replay import physical_replay, require_physical_replay_assets


def registered_manifest(rig, tmp_path):
    manifest = copy.deepcopy(rig.manifest)
    asset = manifest['assets']['asset']
    asset['collision_role'] = 'planning_proxy'
    for name in ('native_physics_geometry', 'native_physics_baseline'):
        path = tmp_path / (name + '.json')
        path.write_text(json.dumps({'fixture_role': name}))
        asset[name + '_path'] = str(path)
        asset[name + '_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    return manifest


def test_planning_proxy_requires_separate_physical_asset_records(rig):
    manifest = copy.deepcopy(rig.manifest)
    manifest['assets']['asset']['collision_role'] = 'planning_proxy'
    with pytest.raises(ValueError, match='native_physics_geometry_path'):
        SceneWorldModel(manifest)


@pytest.mark.parametrize('role', ['native_physics_geometry', 'native_physics_baseline'])
def test_changed_physical_record_invalidates_asset_check(rig, tmp_path, role):
    manifest = registered_manifest(rig, tmp_path)
    world = SceneWorldModel(manifest)
    world.check_assets()
    (tmp_path / (role + '.json')).write_text('changed native fixture state')
    with pytest.raises(SceneInvalid, match=role):
        world.check_assets()


def test_planning_mesh_hash_cannot_acknowledge_native_geometry(rig, tmp_path):
    manifest = registered_manifest(rig, tmp_path)
    asset = manifest['assets']['asset']
    asset['native_physics_geometry_sha256'] = asset['collision_sha256']
    world = SceneWorldModel(manifest)
    with pytest.raises(SceneInvalid, match='native_physics_geometry'):
        world.check_assets()


def test_physical_replay_rejects_planning_proxy_before_native_initialization():
    request = {'initial_snapshot': {'assets': {'a': {'collision_role': 'planning_proxy'}}}}
    with pytest.raises(ValueError, match='Native physical geometry replay adapter required'):
        physical_replay(request)


@pytest.mark.parametrize('role', [None, 'physical', 'unknown'])
def test_existing_physical_asset_role_is_retained_and_unknown_rejected(role):
    if role == 'unknown':
        with pytest.raises(ValueError, match='Unknown'):
            require_physical_replay_assets({'a': {'collision_role': role}})
    else:
        require_physical_replay_assets({'a': {'collision_role': role}})
