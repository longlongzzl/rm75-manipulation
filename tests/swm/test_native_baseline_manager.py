import pytest

from rm75_app.swm.native_body_mirror import NativeBodyMirror
from rm75_app.swm.scene import SceneInvalid


def manager(readback):
    port = object.__new__(NativeBodyMirror)
    port._baseline_applied = False
    port.readback = readback
    return port


def test_empty_belief_requires_actual_baseline_reads_on_apply_and_ack():
    reads = []
    port = manager(lambda: reads.append(True))
    with pytest.raises(SceneInvalid, match='not been acknowledged'):
        port.read_physics()
    port.apply_physics({})
    assert port.read_physics() == {}
    assert len(reads) == 2


def test_nonempty_posterior_does_not_masquerade_as_applied_native_parameters():
    port = manager(lambda: None)
    port.apply_physics({})
    with pytest.raises(SceneInvalid, match='adapter required'):
        port.apply_physics({'bi': {'density': 1000}})
    with pytest.raises(SceneInvalid, match='not been acknowledged'):
        port.read_physics()


def test_native_baseline_mismatch_propagates_without_acknowledgement():
    def failed():
        raise SceneInvalid('actual mass differs')
    port = manager(failed)
    with pytest.raises(SceneInvalid, match='actual mass'):
        port.apply_physics({})
    assert not port._baseline_applied
