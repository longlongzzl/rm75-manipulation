from types import SimpleNamespace

import numpy as np

from rm75_app.validation.maniskill_gate import observe_gripper_boundary


def test_boundary_observation_reads_named_instances_without_physics_steps():
    poses = {"object_2": np.eye(4), "support_3": np.eye(4)}
    poses["support_3"][0, 3] = 0.4
    calls = []

    def observe(name):
        calls.append(name)
        return poses[name]

    q = np.zeros(7)
    velocity = np.array([[0.1, 0.2, 0.3]])
    gripper = {"finger": 0.2}
    adapter = SimpleNamespace(tcp_pose_matrix=lambda: np.eye(4),
                              current_arm_qpos=lambda: q,
                              current_arm_qvel=lambda: q,
                              current_arm_controller_target=lambda: q,
                              current_gripper_qpos=lambda: gripper)
    atom = SimpleNamespace(atom_id="sort_2", object_id="object_2", support_object_id="support_3")
    actor = SimpleNamespace(get_linear_velocity=lambda: velocity,
                            get_angular_velocity=lambda: velocity)
    row = observe_gripper_boundary(SimpleNamespace(observe_object_pose=observe,
                                                   actor=lambda name: actor), adapter, atom)
    assert calls == ["object_2", "support_3"]
    assert row["atom_id"] == "sort_2"
    assert row["support_pose"][0][3] == 0.4
    poses["object_2"][0, 3] = 2
    q[0] = 1
    gripper["finger"] = 1
    velocity[:] = 10
    assert row["object_pose"][0][3] == 0
    assert row["arm_qpos_rad"][0] == 0
    assert row["gripper_qpos_rad"]["finger"] == 0.2
    assert row["object_linear_velocity_m_s"] == [0.1, 0.2, 0.3]
    assert row["object_angular_velocity_rad_s"] == [0.1, 0.2, 0.3]
    assert row["arm_qvel_rad_s"][0] == 0


def test_boundary_observation_without_active_atom_does_not_query_objects():
    adapter = SimpleNamespace(tcp_pose_matrix=lambda: np.eye(4),
                              current_arm_qpos=lambda: np.zeros(7),
                              current_arm_qvel=lambda: np.zeros(7),
                              current_arm_controller_target=lambda: np.zeros(7),
                              current_gripper_qpos=lambda: {})
    row = observe_gripper_boundary(object(), adapter, None)
    assert row["atom_id"] is None
    assert row["object_pose"] is None
    assert row["support_pose"] is None
    assert row["object_linear_velocity_m_s"] is None


def test_adapter_velocity_uses_robot_indices_but_target_uses_controller_order():
    from rm75_app.validation.maniskill_gate import ManiSkillJointAdapter
    adapter = ManiSkillJointAdapter.__new__(ManiSkillJointAdapter)
    adapter.arm_indices = [2, 0]
    target = np.array([[0.3, 0.4]])
    agent = SimpleNamespace(robot=SimpleNamespace(get_qvel=lambda: np.array([[1., 2., 3.]])),
                            controller=SimpleNamespace(controllers={"arm": SimpleNamespace(_target_qpos=target)}))
    adapter.env = SimpleNamespace(unwrapped=SimpleNamespace(agent=agent))
    np.testing.assert_array_equal(adapter.current_arm_qvel(), [3., 1.])
    snapshot = adapter.current_arm_controller_target()
    target[:] = 9
    np.testing.assert_array_equal(snapshot, [0.3, 0.4])
