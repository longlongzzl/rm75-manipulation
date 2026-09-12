"""Metric artifact tests, independent of native-world qualification."""
from types import SimpleNamespace

import numpy as np
import pytest
import trimesh

from rm75_app.swm.native_registration import write_metric_asset
from rm75_app.swm.native_scene import planning_scene


@pytest.mark.parametrize('kind', ['cuboid', 'sphere'])
def test_metric_asset_retains_object_frame_and_shared_proxy_offset(tmp_path, kind):
    mesh = trimesh.creation.box(extents=[.1, .2, .3])
    mesh.apply_translation([.02, -.03, .04])
    local = np.eye(4); local[:3, 3] = mesh.bounds.mean(axis=0)
    proxy = SimpleNamespace(kind=kind, dimensions=[.1, .2, .3], radius=.15)
    asset = write_metric_asset(tmp_path, 'sample', mesh, proxy, local, {'source': 'fixture'})
    measured = np.eye(4); measured[:3, 3] = [.3, .2, .1]
    snapshot = dict(snapshot_id='fixture-snapshot', assets={'sample': asset},
        objects={'instance': dict(asset_id='sample', measured={'T_world_object': measured.tolist()})})
    scene = planning_scene(snapshot)
    collision = scene.objects[0]
    assert collision.kind == kind
    np.testing.assert_allclose(collision.pose.position, (measured @ local)[:3, 3])
    exported = trimesh.load(asset['mesh_path'], force='mesh', process=False)
    np.testing.assert_allclose(exported.bounds, mesh.bounds, atol=1e-7)
    assert asset['physics_model_qualified'] is False
    with pytest.raises(FileExistsError):
        write_metric_asset(tmp_path, 'sample', mesh, proxy, local, {})
