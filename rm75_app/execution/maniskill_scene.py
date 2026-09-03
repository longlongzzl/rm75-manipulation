"""Small ManiSkill scene helpers used by the Curobo2 replay adapter."""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any

import numpy as np

from rm75_app.assets.object_specs import get_object_spec


_RM75_PAD_ROTATION_IN_BASE = {
    "Left": np.asarray(
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
        dtype=np.float64,
    ),
    "Right": np.asarray(
        [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]],
        dtype=np.float64,
    ),
}


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def _initialize_rm75_stable_gripper(agent: Any, robot_class: type) -> None:
    """Load precise closures and preserve the RM75 parallel-pad kinematics."""

    import sapien

    super(robot_class, agent)._after_loading_articulation()
    link2_pin = [0.040367, 0.037539, 0.007696]
    support_pin = [-0.014463, 0.016458, 0.00053]
    # Drive-frame X is the free twist axis; rotate it onto the local Z hinge.
    frame_q = [np.sqrt(0.5), 0.0, -np.sqrt(0.5), 0.0]
    constraints = []
    for side in ("Right", "Left"):
        link2 = agent.robot.links_map.get(f"gripper_{side}_2_Link")
        support = agent.robot.links_map.get(f"gripper_{side}_Support_Link")
        if link2 is None or support is None:
            continue
        drive = agent.scene.create_drive(
            link2,
            sapien.Pose(p=link2_pin, q=frame_q),
            support,
            sapien.Pose(p=support_pin, q=frame_q),
        )
        for component in drive._objs:
            component.set_limit_x(0.0, 0.0)
            component.set_limit_y(0.0, 0.0)
            component.set_limit_z(0.0, 0.0)
            component.set_limit_twist(-np.pi, np.pi)
            component.set_limit_pyramid(0.0, 0.0, 0.0, 0.0)
        constraints.append(drive)
    get_link = agent.robot.links_map.get
    left_support = get_link("gripper_Left_Support_Link")
    right_support = get_link("gripper_Right_Support_Link")
    pad_parallel_constraints = []
    if left_support is not None and right_support is not None:
        # Keep the two mirrored pad frames parallel while leaving their
        # relative translation fully free for normal opening and closing.
        drive = agent.scene.create_drive(
            left_support,
            sapien.Pose(q=[0.0, 0.0, 1.0, 0.0]),
            right_support,
            sapien.Pose(),
        )
        for component in drive._objs:
            component.set_limit_twist(0.0, 0.0)
            component.set_limit_pyramid(0.0, 0.0, 0.0, 0.0)
            component.set_drive_property_slerp(
                stiffness=1.0e5,
                damping=650.0,
                mode="acceleration",
            )
        pad_parallel_constraints.append(drive)

    agent._rm75_planar_gripper_constraints = constraints
    agent._rm75_pad_parallel_constraints = pad_parallel_constraints

    gripper_links = (
        "gripper_base_link",
        "gripper_Left_1_Link",
        "gripper_Left_Support_Link",
        "gripper_Left_2_Link",
        "gripper_Right_1_Link",
        "gripper_Right_Support_Link",
        "gripper_Right_2_Link",
        "link_6",
        "link_7",
        "left_pad",
        "right_pad",
    )
    for link_name in gripper_links:
        link = get_link(link_name)
        if link is not None:
            link.set_collision_group_bit(group=2, bit_idx=31, bit=1)
    agent.finger1_link = get_link("gripper_Left_Support_Link")
    agent.finger2_link = get_link("gripper_Right_Support_Link")
    agent.finger1_pad = get_link("left_pad")
    agent.finger2_pad = get_link("right_pad")
    agent.tcp = get_link("gripper_tcp")


def _stabilize_rm75_parallel_gripper() -> None:
    """Apply the upstream agent's recommended closed-loop gripper setting."""

    module = importlib.import_module("mani_skill.agents.robots.realman")
    robot_class = getattr(module, "RM75Robot", None)
    if robot_class is not None:
        robot_class.disable_self_collisions = True
        if not bool(getattr(robot_class, "_rm75_planar_patch_installed", False)):
            def _after_loading_articulation_with_planar_constraints(self):
                _initialize_rm75_stable_gripper(self, robot_class)

            robot_class._after_loading_articulation = (
                _after_loading_articulation_with_planar_constraints
            )
            robot_class._rm75_planar_patch_installed = True


def ensure_pick_env_registered(env_id: str, extra_package_root: str) -> str:
    import gymnasium as gym

    if env_id in gym.envs.registry:
        _stabilize_rm75_parallel_gripper()
        return env_id
    package_root = Path(extra_package_root).expanduser()
    if package_root.name != "mani_skill" and (package_root / "mani_skill").exists():
        package_root = package_root / "mani_skill"
    if not package_root.exists():
        raise FileNotFoundError(f"extra ManiSkill package root not found: {package_root}")

    import mani_skill.agents.robots as robots_pkg
    import mani_skill.envs.tasks.digital_twins as digital_twins_pkg

    robots_path = str(package_root / "agents" / "robots")
    tasks_path = str(package_root / "envs" / "tasks" / "digital_twins")
    if robots_path not in robots_pkg.__path__:
        robots_pkg.__path__.append(robots_path)
    if tasks_path not in digital_twins_pkg.__path__:
        digital_twins_pkg.__path__.append(tasks_path)
    _stabilize_rm75_parallel_gripper()
    importlib.import_module(
        "mani_skill.envs.tasks.digital_twins.so101_arm_with_two_cameras.pick_jiaobang"
    )
    if env_id not in gym.envs.registry:
        raise gym.error.NameNotFound(f"ManiSkill environment {env_id!r} is not registered")
    return env_id


def _pose_matrix(pose: Any) -> np.ndarray | None:
    if pose is None:
        return None
    matrix = pose.to_transformation_matrix() if hasattr(pose, "to_transformation_matrix") else pose
    if hasattr(matrix, "detach"):
        matrix = matrix.detach().cpu().numpy()
    matrix = np.asarray(matrix)
    if matrix.ndim == 3:
        matrix = matrix[0]
    return matrix.astype(np.float32) if matrix.shape == (4, 4) else None


def gripper_pad_alignment_diagnostics(env: Any) -> dict[str, float] | None:
    """Measure pad tilt from ideal four-bar kinematics and mutual parallelism."""

    agent = getattr(env.unwrapped, "agent", None)
    robot = getattr(agent, "robot", None)
    links = getattr(robot, "links_map", {})
    base = _pose_matrix(getattr(links.get("gripper_base_link"), "pose", None))
    left = _pose_matrix(getattr(links.get("left_pad"), "pose", None))
    right = _pose_matrix(getattr(links.get("right_pad"), "pose", None))
    if base is None or left is None or right is None:
        return None

    base_rotation = base[:3, :3].astype(np.float64)
    observed = {
        "Left": base_rotation.T @ left[:3, :3],
        "Right": base_rotation.T @ right[:3, :3],
    }
    left_error = _rotation_angle_deg(
        _RM75_PAD_ROTATION_IN_BASE["Left"].T @ observed["Left"]
    )
    right_error = _rotation_angle_deg(
        _RM75_PAD_ROTATION_IN_BASE["Right"].T @ observed["Right"]
    )
    expected_pair = (
        _RM75_PAD_ROTATION_IN_BASE["Left"].T
        @ _RM75_PAD_ROTATION_IN_BASE["Right"]
    )
    observed_pair = observed["Left"].T @ observed["Right"]
    return {
        "left_pad_tilt_deg": left_error,
        "right_pad_tilt_deg": right_error,
        "pad_parallel_error_deg": _rotation_angle_deg(expected_pair.T @ observed_pair),
        "max_pad_tilt_deg": max(left_error, right_error),
    }


def robot_base_transform(env: Any) -> np.ndarray | None:
    robot = getattr(getattr(env.unwrapped, "agent", None), "robot", None)
    return _pose_matrix(getattr(robot, "pose", None))


def object_pose_matrix(env: Any) -> np.ndarray | None:
    return _pose_matrix(getattr(getattr(env.unwrapped, "obj", None), "pose", None))


def set_object_pose(env: Any, T_world_object: np.ndarray) -> None:
    import torch
    from mani_skill.utils.structs.pose import Pose
    from transforms3d.quaternions import mat2quat

    transform = np.asarray(T_world_object, dtype=np.float32).reshape(4, 4)
    obj = env.unwrapped.obj
    obj.set_pose(
        Pose.create_from_pq(
            p=transform[:3, 3],
            q=mat2quat(transform[:3, :3]).astype(np.float32),
        )
    )
    try:
        zero = np.zeros(3, dtype=np.float32)
        obj.set_linear_velocity(zero)
        obj.set_angular_velocity(zero)
    except Exception:
        pass
    if hasattr(env.unwrapped, "object_initial_height"):
        env.unwrapped.object_initial_height[:] = -1.0
    if hasattr(env.unwrapped, "get_obj_xy_shortest_edge_vector") and hasattr(
        env.unwrapped, "obj_xy_shortest_edge_vector"
    ):
        with torch.no_grad():
            env.unwrapped.obj_xy_shortest_edge_vector[:] = (
                env.unwrapped.get_obj_xy_shortest_edge_vector().detach()
            )


def apply_object_physics_profile(env: Any, object_name: str) -> None:
    import sapien

    spec = get_object_spec(object_name)
    if spec is None:
        return
    values = (
        spec.sim_static_friction,
        spec.sim_dynamic_friction,
        spec.sim_restitution,
        spec.sim_linear_damping,
        spec.sim_angular_damping,
    )
    if all(value is None for value in values):
        return
    obj = getattr(env.unwrapped, "obj", None)
    if obj is None:
        return
    if spec.sim_linear_damping is not None:
        obj.set_linear_damping(float(spec.sim_linear_damping))
    if spec.sim_angular_damping is not None:
        obj.set_angular_damping(float(spec.sim_angular_damping))
    actors = list(getattr(env.unwrapped, "_objs", []) or getattr(obj, "_objs", []) or [])
    for actor in actors:
        component = actor.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
        if component is None:
            continue
        for shape in component.collision_shapes:
            material = shape.physical_material
            if spec.sim_static_friction is not None:
                material.static_friction = float(spec.sim_static_friction)
            if spec.sim_dynamic_friction is not None:
                material.dynamic_friction = float(spec.sim_dynamic_friction)
            if spec.sim_restitution is not None:
                material.restitution = float(spec.sim_restitution)
