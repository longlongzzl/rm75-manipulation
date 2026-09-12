from types import SimpleNamespace

from rm75_app.swm.native_scene import SapienScenePort


def test_old_attachment_is_removed_before_switching_native_tcp():
    state = {'tcp': 'old', 'holding': True}
    calls = []

    def detach(value):
        assert value is None
        assert state['tcp'] == 'old'
        calls.append('detach_old_relation')
        state['holding'] = False

    def apply(robot):
        assert not state['holding']
        state['tcp'] = robot['tcp']
        calls.append('switch_robot')
        return 'native_ack'

    port = SimpleNamespace(set_attachment=detach,
                           robot_port=SimpleNamespace(apply_idle_state=apply))
    assert SapienScenePort._switch_native_robot_state(port, {'tcp': 'new'}) == 'native_ack'
    assert calls == ['detach_old_relation', 'switch_robot']
