"""GPU physics gate for a complete manipulation plan."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np

from rm75_app.assets.object_specs import get_object_spec, resolve_object_spec_scales
from rm75_app.execution.maniskill_task_bridge import ManiSkillTaskBridge
from rm75_app.execution.maniskill_scene import gripper_pad_alignment_diagnostics
from rm75_app.execution.trajectory_executor import ManiSkillTrajectoryExecutor, sample_timed_joint_path
from rm75_app.orchestration.multi_object_executor import AtomExecution, TaskSceneState, load_task_scene
from rm75_app.planning.contracts import JointConfiguration
from rm75_app.runtime.curobo2_sim_replay import load_replay_events
from rm75_app.tasks.manipulation_plan import ManipulationPlan
from rm75_app.validation.contracts import GateResult, GateStatus


class ManiSkillJointAdapter:
    def __init__(self, env: Any, *, debug_viewer: bool = False, playback_hz: float = 20.0):
        self.env = env
        self.debug_viewer = bool(debug_viewer)
        self.playback_period_s = 1.0 / max(float(playback_hz), 1.0)
        robot = env.unwrapped.agent.robot
        names = [joint.get_name() for joint in robot.get_active_joints()]
        self.active_joint_names = tuple(names)
        self.arm_indices = [names.index(f"joint_{index}") for index in range(1, 8)]
        self.gripper_indices = [
            index for index, name in enumerate(names) if name.startswith("gripper_")
        ]
        self.arm_names = tuple(names[index] for index in self.arm_indices)
        self.action_dim = int(np.prod(env.action_space.shape))
        self.gripper_value = -1.0
        self.robot_link_names = set(robot.links_map)

    def current_arm_qpos(self) -> np.ndarray:
        qpos = self.env.unwrapped.agent.robot.get_qpos()
        if hasattr(qpos, "detach"):
            qpos = qpos.detach().cpu().numpy()
        return np.asarray(qpos, dtype=np.float64).reshape(-1)[self.arm_indices]

    def current_gripper_qpos(self) -> dict[str, float]:
        qpos = self.env.unwrapped.agent.robot.get_qpos()
        if hasattr(qpos, "detach"):
            qpos = qpos.detach().cpu().numpy()
        values = np.asarray(qpos, dtype=np.float64).reshape(-1)
        return {
            self.active_joint_names[index]: float(values[index])
            for index in self.gripper_indices
        }

    def current_gripper_alignment(self) -> dict[str, float] | None:
        return gripper_pad_alignment_diagnostics(self.env)

    def set_arm_qpos(self, arm_qpos: np.ndarray) -> None:
        robot = self.env.unwrapped.agent.robot
        qpos = robot.get_qpos()
        is_tensor = hasattr(qpos, "detach")
        raw = qpos.detach().cpu().numpy() if is_tensor else np.asarray(qpos)
        raw = np.asarray(raw, dtype=np.float32).copy()
        flat = raw.reshape(-1)
        flat[self.arm_indices] = np.asarray(arm_qpos, dtype=np.float32).reshape(7)
        robot.set_qpos(raw)

    def joint_configuration(self, scene: TaskSceneState | None = None) -> JointConfiguration:
        del scene
        return JointConfiguration(self.arm_names, self.current_arm_qpos())

    def tcp_pose_matrix(self) -> np.ndarray:
        link = self.env.unwrapped.agent.robot.links_map["gripper_tcp"]
        matrix = link.pose.to_transformation_matrix()
        if hasattr(matrix, "detach"):
            matrix = matrix.detach().cpu().numpy()
        value = np.asarray(matrix, dtype=np.float64)
        while value.ndim > 2:
            value = value[0]
        return value.reshape(4, 4)

    def compose_action(self, arm_target_q: np.ndarray, gripper_value: float) -> np.ndarray:
        self.gripper_value = float(gripper_value)
        action = np.zeros(self.action_dim, dtype=np.float32)
        action[:7] = np.asarray(arm_target_q, dtype=np.float32).reshape(-1)[:7]
        if self.action_dim > 7:
            action[7] = self.gripper_value
        return action

    def step_and_render(self, action: np.ndarray, tag: str = "") -> None:
        del tag
        self.env.step(action)
        self._render_debug_frame()

    def robot_contact_pairs(self) -> list[dict[str, Any]]:
        """Return current robot contact pairs with their aggregate impulse."""
        result = []
        for contact in self.env.unwrapped.scene.get_contacts():
            body_a = str(contact.bodies[0].entity.name)
            body_b = str(contact.bodies[1].entity.name)
            if body_a not in self.robot_link_names and body_b not in self.robot_link_names:
                continue
            impulse = np.zeros(3, dtype=np.float64)
            for point in contact.points:
                impulse += np.asarray(point.impulse, dtype=np.float64).reshape(3)
            result.append(
                {
                    "body_a": body_a,
                    "body_b": body_b,
                    "impulse_ns": float(np.linalg.norm(impulse)),
                }
            )
        return result

    def hold_current_and_set_gripper(self, value: float, steps: int = 20) -> None:
        action = self.compose_action(self.current_arm_qpos(), value)
        for _ in range(int(steps)):
            self.env.step(action)
            self._render_debug_frame()

    def hold_action(self) -> np.ndarray:
        return self.compose_action(self.current_arm_qpos(), self.gripper_value)

    def _render_debug_frame(self) -> None:
        if not self.debug_viewer:
            return
        viewer = getattr(self.env.unwrapped, "_viewer", None)
        if viewer is None or bool(getattr(viewer, "closed", False)):
            self.debug_viewer = False
            return
        self.env.unwrapped.render_human()
        time.sleep(self.playback_period_s)


def write_scene_spec(plan: ManipulationPlan, output_path: str | Path) -> tuple[Path, TaskSceneState]:
    state = load_task_scene(plan.scene_file)
    objects = []
    for item in state.objects.values():
        spec = get_object_spec(item.asset_name)
        if spec is None:
            raw = dict(item.metadata.get("source_scene_entry") or {})
            asset_file = raw.get("sim_asset_file") or raw.get("collision_mesh_path") or raw.get("mesh_file")
            scale = raw.get("sim_asset_scale") or raw.get("mesh_scale") or raw.get("scale") or 1.0
        else:
            _, sim_scale = resolve_object_spec_scales(spec)
            asset_file = spec.sim_asset_file or spec.mesh_file
            scale = sim_scale
        if not asset_file:
            raise KeyError(f"object {item.object_id!r} has no simulation asset")
        scale_array = np.asarray(scale, dtype=float).reshape(-1)
        if scale_array.size == 1:
            scale_array = np.repeat(scale_array, 3)
        objects.append(
            {
                "object_id": item.object_id,
                "asset_name": item.asset_name,
                "asset_file": str(Path(asset_file).expanduser().resolve()),
                "scale": scale_array.tolist(),
                "pose": item.pose.tolist(),
                "fixed": not item.movable,
                "physics": (
                    {}
                    if spec is None
                    else {
                        key: value
                        for key, value in {
                            "static_friction": spec.sim_static_friction,
                            "dynamic_friction": spec.sim_dynamic_friction,
                            "restitution": spec.sim_restitution,
                            "linear_damping": spec.sim_linear_damping,
                            "angular_damping": spec.sim_angular_damping,
                        }.items()
                        if value is not None
                    }
                ),
            }
        )
    path = Path(output_path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"objects": objects}, ensure_ascii=False, indent=2), encoding="utf-8")
    return path, state


def validate_timed_replay_events(events, observed: JointConfiguration, control_dt: float) -> float:
    """Validate the entire package before the first simulated action."""
    trajectories = [event["trajectory"] for event in events if event.get("type") == "trajectory"]
    if not trajectories:
        raise ValueError("no trajectories in timed replay package")
    previous = None
    for trajectory in trajectories:
        if tuple(trajectory.joint_names) != tuple(observed.names):
            raise ValueError("timed replay joint names do not match simulator")
        sample_timed_joint_path(trajectory, control_dt)
        if previous is not None:
            gap = float(np.max(np.abs(trajectory.positions[0] - previous)))
            if gap > 0.10:
                raise ValueError(f"timed replay stage gap {gap:.6f} rad exceeds 0.10")
        previous = trajectory.positions[-1]
    initial_gap = float(np.max(np.abs(trajectories[0].positions[0] - observed.positions)))
    if initial_gap > 0.12:
        raise ValueError(f"timed replay initial gap {initial_gap:.6f} rad exceeds 0.12; no teleport allowed")
    return initial_gap


def run_maniskill_gate(
    plan: ManipulationPlan,
    execution_file: str | Path,
    output_dir: str | Path,
    *,
    render_mode: str = "rgb_array",
    settle_steps: int | None = None,
    debug_viewer: bool = False,
    viewer: Any | None = None,
    stop_on_validation_failure: bool = True,
    strict_timed_replay: bool = False,
) -> GateResult:
    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    scene_spec, state = write_scene_spec(plan, output / "maniskill_scene.json")
    events = load_replay_events(execution_file)

    import gymnasium as gym
    from mani_skill.utils.wrappers.record import RecordEpisode
    import rm75_app.simulation.maniskill_multi_object_env  # noqa: F401

    if debug_viewer and viewer is None:
        from sapien.utils import Viewer

        # On this SAPIEN build the present-capable viewer must exist before
        # ManiSkill creates its offscreen render system.
        viewer = Viewer(resolutions=(960, 720))

    env = gym.make(
        "RM75-MultiObjectTask-v1",
        scene_spec_file=str(scene_spec),
        robot_uids="RM75",
        obs_mode="none",
        control_mode="pd_joint_pos",
        render_mode=render_mode,
        max_episode_steps=10000,
    )
    env = RecordEpisode(
        env,
        output_dir=str(output / "video"),
        save_trajectory=False,
        save_video=True,
        source_type="motionplanning",
        source_desc="RM75 three-gate multi-object physics validation",
        video_fps=20,
    )
    try:
        env.reset(seed=0)
        if viewer is not None:
            env.unwrapped._viewer = viewer
            env.unwrapped._setup_viewer()
        joint_adapter = ManiSkillJointAdapter(env, debug_viewer=debug_viewer)
        first_trajectory = next((item["trajectory"] for item in events if item.get("type") == "trajectory"), None)
        if first_trajectory is None:
            raise ValueError("Curobo2 package contains no trajectory")
        control_dt = None
        initial_gap = None
        if strict_timed_replay:
            control_dt = 1.0 / float(env.unwrapped.control_freq)
            observed = JointConfiguration(joint_adapter.arm_names, joint_adapter.current_arm_qpos())
            (output / "replay_initial_state.json").write_text(json.dumps({
                "joint_names": list(observed.names),
                "joint_positions": observed.positions.tolist(),
                "planned_joint_names": list(first_trajectory.joint_names),
                "planned_joint_positions": first_trajectory.positions[0].tolist(),
                "control_dt_s": control_dt,
                "source": "ManiSkill environment reset, no set_arm_qpos",
            }, indent=2), encoding="utf-8")
            initial_gap = validate_timed_replay_events(
                events,
                observed,
                control_dt,
            )
        else:
            joint_adapter.set_arm_qpos(first_trajectory.positions[0])
        if debug_viewer:
            env.unwrapped.render_human()
        state.set_joints(joint_adapter.arm_names, joint_adapter.current_arm_qpos())
        bridge = ManiSkillTaskBridge(
            env,
            env.unwrapped.task_actors,
            settle_steps=settle_steps,
            hold_action=joint_adapter.hold_action,
        )
        motion_executor = ManiSkillTrajectoryExecutor(joint_adapter, control_dt=control_dt)
        atoms = {atom.atom_id: atom for atom in plan.atoms}
        active_atom = None
        checks = []
        stage_trace = []
        failed = False
        for event in events:
            event_type = event.get("type")
            if event_type == "atom_start":
                active_atom = atoms[str(event["atom_id"])]
            elif event_type == "trajectory":
                motion_executor.execute_trajectory(str(event["stage"]), event["trajectory"])
                commanded = np.asarray(event["trajectory"].positions[-1], dtype=np.float64)
                actual = joint_adapter.current_arm_qpos()
                tcp_pose = joint_adapter.tcp_pose_matrix()
                object_pose = (
                    None
                    if active_atom is None
                    else bridge.observe_object_pose(active_atom.object_id)
                )
                trace = {
                    "atom_id": None if active_atom is None else active_atom.atom_id,
                    "stage": str(event["stage"]),
                    "max_joint_error_rad": float(np.max(np.abs(commanded - actual))),
                    "rms_joint_error_rad": float(np.sqrt(np.mean((commanded - actual) ** 2))),
                    "tcp_xyz_m": tcp_pose[:3, 3].tolist(),
                    "object_xyz_m": None if object_pose is None else object_pose[:3, 3].tolist(),
                    "gripper_qpos_rad": joint_adapter.current_gripper_qpos(),
                    "gripper_alignment": joint_adapter.current_gripper_alignment(),
                    "robot_contacts": motion_executor.last_contact_summary,
                }
                stage_trace.append(trace)
                if debug_viewer:
                    print(
                        "[maniskill-debug] "
                        f"stage={trace['stage']} max_q_err={trace['max_joint_error_rad']:.5f}rad "
                        f"tcp={np.round(tcp_pose[:3, 3], 4).tolist()} "
                        f"object={None if object_pose is None else np.round(object_pose[:3, 3], 4).tolist()}",
                        flush=True,
                    )
            elif event_type == "gripper":
                motion_executor.set_gripper(bool(event["closed"]))
            elif event_type == "atom_end":
                if active_atom is None or active_atom.atom_id != str(event["atom_id"]):
                    raise ValueError("trajectory package has mismatched atom markers")
                observed_pose = bridge.observe_object_pose(active_atom.object_id)
                execution = AtomExecution(
                    True,
                    final_object_pose=observed_pose,
                    joint_names=joint_adapter.arm_names,
                    joint_positions=joint_adapter.current_arm_qpos(),
                )
                validation = bridge.validate_atom(active_atom, execution, state)
                checks.append(
                    {
                        "atom_id": active_atom.atom_id,
                        "status": "passed" if validation.success else "failed",
                        "position_error_m": validation.position_error_m,
                        "orientation_error_deg": validation.orientation_error_deg,
                        "observed_pose": observed_pose.tolist(),
                        "message": validation.message,
                    }
                )
                committed_pose = (
                    observed_pose
                    if validation.observed_object_pose is None
                    else validation.observed_object_pose
                )
                state.commit_object_pose(active_atom.object_id, committed_pose)
                state.set_joints(joint_adapter.arm_names, joint_adapter.current_arm_qpos())
                if not validation.success:
                    failed = True
                    if stop_on_validation_failure:
                        break
                active_atom = None
        result_file = output / "maniskill_gate_result.json"
        result_file.write_text(
            json.dumps(
                {
                    "success": not failed,
                    "strict_timed_replay": strict_timed_replay,
                    "initial_gap_rad": initial_gap,
                    "control_dt_s": control_dt,
                    "checks": checks,
                    "stage_trace": stage_trace,
                    "final_scene": state.as_dict(),
                },
                ensure_ascii=False,
                indent=2,
                default=str,
            ),
            encoding="utf-8",
        )
        gate_result = GateResult(
            "maniskill",
            GateStatus.FAILED if failed else GateStatus.PASSED,
            "physics execution did not satisfy the task" if failed else "physics execution and settled poses passed",
            tuple(checks),
            artifacts={"scene_spec": str(scene_spec), "result_file": str(result_file), "video_dir": str(output / "video")},
        )
        if viewer is not None and not bool(getattr(viewer, "closed", False)):
            print("[maniskill-debug] 回放结束；关闭 SAPIEN 窗口以结束第三级验证。", flush=True)
            while not bool(getattr(viewer, "closed", False)):
                env.unwrapped.render_human()
                time.sleep(1.0 / 60.0)
        return gate_result
    finally:
        for method in ("flush_video", "flush", "close"):
            try:
                getattr(env, method)()
            except Exception:
                pass
        if viewer is not None:
            try:
                viewer.close()
            except Exception:
                pass
