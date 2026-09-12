from types import SimpleNamespace

import pytest

from rm75_app.swm.native_scene import CuroboScenePort, SapienScenePort
from rm75_app.swm.scene import SceneInvalid


def test_native_planning_port_rejects_missing_independent_jaw_before_writes(rig):
    snapshot = rig.sync.sync('before_grasp')
    snapshot['observation_domain'] = 'physics'
    port = CuroboScenePort(SimpleNamespace())
    with pytest.raises(SceneInvalid, match='q/qdot'):
        port.apply_idle_snapshot(snapshot)
    assert port.applied_snapshot_id is None
    assert port.native_acknowledgement is None


def test_native_held_geometry_cannot_use_python_attachment_cache_as_ack(rig):
    rig.source.holding = 'a'
    snapshot = rig.sync.sync('after_grasp')
    snapshot['observation_domain'] = 'physics'
    port = CuroboScenePort(SimpleNamespace())
    with pytest.raises(SceneInvalid, match='attached geometry readback'):
        port.apply_idle_snapshot(snapshot)
    assert port.applied_snapshot_id is None


def test_native_simulator_without_private_robot_port_rejects_before_actor_writes(rig):
    snapshot = rig.sync.sync('before_grasp')
    snapshot['observation_domain'] = 'physics'
    writes = []
    port = SapienScenePort(
        {oid: SimpleNamespace(set_pose=lambda value: writes.append(value)) for oid in snapshot['objects']},
        asset_bindings={}, set_attachment=writes.append, read_attachment=lambda: None,
        apply_physics=writes.append, read_physics=lambda: {}, make_pose=lambda value: value)
    with pytest.raises(SceneInvalid, match='private robot state port'):
        port.apply_idle_snapshot(snapshot)
    assert writes == []
    assert port.applied_snapshot_id is None
