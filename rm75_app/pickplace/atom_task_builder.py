"""Compile one resolved manipulation atom into a PickPlaceTask."""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
import time
from typing import Callable

import numpy as np
import trimesh

from rm75_app.assets.object_specs import (
    discrete_orientation_symmetry_rotations,
    get_object_spec,
    resolve_object_spec_scales,
)
from rm75_app.assets.collision_proxy import (
    DEFAULT_SCENE_PROXY_SCALE,
    build_automatic_collision_proxy,
)
from rm75_app.core.frames import (
    MANISKILL_TABLE_COLLISION_NAME,
    MANISKILL_TABLE_DIMENSIONS_M,
    MANISKILL_TABLE_WORLD_CENTER_M,
)
from rm75_app.orchestration.multi_object_executor import TaskSceneState
from rm75_app.pickplace.cached_scene import matrix_to_quaternion_wxyz
from rm75_app.pickplace.coordinator import PickPlaceTask
from rm75_app.planning.contracts import (
    CollisionObject,
    JointConfiguration,
    PlanningScene,
    Pose,
    PoseCandidate,
)
from rm75_app.tasks.manipulation_plan import ManipulationAtom, ManipulationPrimitive


JointStateProvider = Callable[[TaskSceneState], JointConfiguration]


@dataclass(frozen=True)
class AtomTaskBuilderConfig:
    # RM75 gripper TCP -> midpoint of the two pad collision spheres along
    # local +Z, measured from cuRobo2 FK with the configured locked joints.
    grasp_z_offset_m: float = 0.012777
    # World-Z offsets from the radial, robot-facing closing-axis baseline.
    # Closing-axis symmetry makes [0, 180) sufficient.
    grasp_yaw_offsets_deg: tuple[float, ...] | None = None
    grasp_tilt_toward_base_deg: tuple[float, ...] | None = None
    # Keep a fixed 8 x 8 relation lattice for one batch64 IK query.  Yaw is
    # physically interchangeable for a sphere but changes arm kinematics;
    # shortlist IK-feasible rows before trajectory planning to bound GPU memory.
    sphere_grasp_yaw_offsets_deg: tuple[float, ...] | None = None
    sphere_grasp_tilt_toward_base_deg: tuple[float, ...] | None = None
    # Prefer a planner-native continuous orientation freedom when the object
    # capability declares one. Disable to reproduce the saved discrete flow.
    use_continuous_grasp_freedom: bool = True
    # Legacy fallback for axial assets that do not yet declare their own
    # placement capability in ObjectSpec.
    axial_place_spin_deg: tuple[float, ...] = (0.0, -90.0, 90.0, 180.0)
    grasp_approach_offset_m: float = -0.08
    lift_height_m: float = 0.08
    place_clearance_m: float = 0.08
    minimum_retreat_clearance_m: float = 0.04
    maximum_retreat_clearance_m: float = 0.18
    retreat_above_object_margin_m: float = 0.02
    # The pads/support fingers extend below the TCP. Clearance based on the TCP
    # alone can therefore leave a released object trapped under one finger.
    gripper_below_tcp_clearance_m: float = 0.025
    # Permit a small jaw-height mismatch for reachability, while rejecting the
    # visibly side-on releases that make long objects roll off one finger.
    # With an 80 mm finger span, 15 degrees is about 21 mm height difference.
    # This still rejects the visibly side-on 19-35 degree solutions observed
    # for the cube and brush, while retaining enough reachability for carrots.
    max_tabletop_closing_axis_world_z: float = float(np.sin(np.deg2rad(15.0)))
    tabletop_level_refinement_offsets_deg: tuple[float, ...] = (
        0.0,
        -5.0,
        5.0,
        -10.0,
        10.0,
    )
    # Extra world-Z release heights searched for tabletop settling.
    tabletop_release_height_offsets_m: tuple[float, ...] = (0.0, 0.005)
    max_attempts: int = 3
    scene_collision_proxy_scale: float = DEFAULT_SCENE_PROXY_SCALE
    sphere_release_tilt_toward_base_deg: tuple[float, ...] = (0.0, 15.0, 30.0, 45.0)
    sphere_release_yaw_offsets_deg: tuple[float, ...] = (
        0.0,
        45.0,
        90.0,
        135.0,
    )
    sphere_release_lift_m: float = 0.03
    # Multi-object/ManiSkill scenes use a world frame whose RM75 base is at
    # x=-0.615 m. Real perception inputs are already base-referenced and keep
    # this unset. cuRobo itself always plans in robot-base coordinates.
    robot_base_world_xyz_m: tuple[float, float, float] | None = None
    # The multi-object physics environment always builds ManiSkill's canonical
    # table. Enable its exact collision box for simulation planning.
    include_maniskill_workspace_table: bool = False


class FixedSceneAtomTaskBuilder:
    """Build collision, grasp, and paired place candidates from task state."""

    def __init__(
        self,
        joint_state_provider: JointStateProvider | None = None,
        config: AtomTaskBuilderConfig | None = None,
    ):
        self.joint_state_provider = joint_state_provider
        self.config = config or AtomTaskBuilderConfig()

    def __call__(self, atom: ManipulationAtom, scene: TaskSceneState) -> PickPlaceTask:
        candidate_build_started = time.perf_counter()
        if atom.object_id not in scene.objects:
            raise KeyError(atom.object_id)
        current = self._joint_state(scene)
        planning_scene = self._planning_scene(scene)
        T_world_object = scene.objects[atom.object_id].pose
        T_planning_object = self._to_planning_pose(T_world_object)
        spec = get_object_spec(atom.object_asset)
        if spec is None:
            raise KeyError(f"object asset {atom.object_asset!r} is not registered")
        mesh_scale, _ = resolve_object_spec_scales(spec)
        loaded_asset = trimesh.load(Path(spec.mesh_file), force="scene", process=False)
        grasps = self._grasp_candidates(atom.object_asset, T_planning_object)
        place_by_grasp: dict[str, tuple[PoseCandidate, ...]] = {}
        all_places: dict[str, PoseCandidate] = {}
        T_world_goal_object = np.asarray(atom.target_pose, dtype=np.float64).reshape(4, 4)
        T_planning_goal_object = self._to_planning_pose(T_world_goal_object)
        T_planning_release_object = T_planning_goal_object.copy()
        support_spec = None
        if atom.support_object_id is not None:
            support_state = scene.objects.get(atom.support_object_id)
            if support_state is not None:
                support_spec = get_object_spec(support_state.asset_name)
        releases_to_tabletop = (
            atom.support_object_id is None
            or (
                support_spec is not None
                and support_spec.support_surface_kind == "tabletop"
            )
        )
        release_clearance_m = (
            float(spec.tabletop_release_clearance_m)
            if releases_to_tabletop
            else 0.0
        )
        T_planning_release_object[2, 3] += release_clearance_m
        shared_sphere_places: tuple[PoseCandidate, ...] | None = None
        if spec.orientation_symmetry == "spherical":
            shared_sphere_places = self._sphere_release_candidates(
                grasps[0],
                T_planning_object,
                T_planning_goal_object,
                T_world_goal_object=T_world_goal_object,
                loaded_asset=atom.object_asset,
                atom_id=atom.atom_id,
            )
        for grasp in grasps:
            T_world_tcp = _pose_to_matrix(grasp.pose)
            T_tcp_object = np.linalg.inv(T_world_tcp) @ T_planning_object
            if shared_sphere_places is not None:
                places = shared_sphere_places
            elif spec.orientation_symmetry == "cubic":
                places = self._discrete_symmetry_release_candidates(
                    grasp,
                    T_tcp_object,
                    T_planning_release_object,
                    T_world_goal_object,
                    rotations=discrete_orientation_symmetry_rotations("cubic"),
                    symmetry="cubic",
                    atom_id=atom.atom_id,
                    release_clearance_m=release_clearance_m,
                )
            elif spec.orientation_symmetry in {
                "axial", "axial_bidirectional", "orthotropic"
            }:
                axial_samples = (
                    spec.placement_axial_rotation_samples_deg
                    or self.config.axial_place_spin_deg
                )
                if (
                    releases_to_tabletop
                    and spec.orientation_symmetry in {"axial", "axial_bidirectional"}
                    and abs(
                        float(
                            np.dot(
                                _normalized(
                                    T_world_goal_object[:3, :3]
                                    @ np.asarray(
                                        spec.symmetry_axis_local or (0.0, 1.0, 0.0),
                                        dtype=np.float64,
                                    )
                                ),
                                np.asarray([0.0, 0.0, 1.0]),
                            )
                        )
                    )
                    < 1.0 - 1.0e-6
                ):
                    axial_samples = tuple(
                        dict.fromkeys(
                            float(sample) + float(offset)
                            for sample in axial_samples
                            for offset in self.config.tabletop_level_refinement_offsets_deg
                        )
                    )
                places = self._axial_release_candidates(
                    grasp,
                    T_tcp_object,
                    T_planning_release_object,
                    T_world_goal_object,
                    symmetry_axis_local=spec.symmetry_axis_local,
                    spin_samples_deg=axial_samples,
                    bidirectional=(
                        spec.axis_direction_equivalent
                        or spec.orientation_symmetry in {"axial_bidirectional", "orthotropic"}
                    ),
                    atom_id=atom.atom_id,
                    release_clearance_m=release_clearance_m,
                    release_height_offsets_m=(
                        spec.tabletop_release_height_offsets_m
                        if spec.tabletop_release_height_offsets_m is not None
                        else self.config.tabletop_release_height_offsets_m
                    ),
                )
                if (
                    spec.orientation_symmetry == "orthotropic"
                    and atom.success.orientation_tolerance_deg is not None
                ):
                    yaw_limit_deg = min(
                        15.0, 0.75 * float(atom.success.orientation_tolerance_deg)
                    )
                    yaw_fallbacks: list[PoseCandidate] = []
                    for yaw_deg in (-yaw_limit_deg, yaw_limit_deg):
                        world_yaw = _axis_angle_rotation(
                            np.asarray([0.0, 0.0, 1.0]),
                            np.deg2rad(yaw_deg),
                        )
                        adjusted_planning_release = T_planning_release_object.copy()
                        adjusted_planning_release[:3, :3] = (
                            world_yaw @ adjusted_planning_release[:3, :3]
                        )
                        adjusted_world_goal = T_world_goal_object.copy()
                        adjusted_world_goal[:3, :3] = (
                            world_yaw @ adjusted_world_goal[:3, :3]
                        )
                        adjusted_places = self._axial_release_candidates(
                            grasp,
                            T_tcp_object,
                            adjusted_planning_release,
                            adjusted_world_goal,
                            symmetry_axis_local=spec.symmetry_axis_local,
                            spin_samples_deg=axial_samples,
                            bidirectional=True,
                            atom_id=atom.atom_id,
                            release_clearance_m=release_clearance_m,
                            release_height_offsets_m=(
                                spec.tabletop_release_height_offsets_m
                                if spec.tabletop_release_height_offsets_m is not None
                                else self.config.tabletop_release_height_offsets_m
                            ),
                        )
                        for candidate in adjusted_places:
                            metadata = dict(candidate.metadata)
                            metadata["canonical_target_object_pose"] = (
                                T_world_goal_object.tolist()
                            )
                            metadata["target_yaw_adjustment_deg"] = float(yaw_deg)
                            metadata["search_tier"] = max(
                                2, int(metadata.get("search_tier", 0))
                            )
                            yaw_fallbacks.append(
                                replace(
                                    candidate,
                                    candidate_id=(
                                        f"{candidate.candidate_id}__target_yaw_{yaw_deg:+.0f}"
                                    ),
                                    score=float(candidate.score) - 0.001,
                                    metadata=metadata,
                                )
                            )
                    places = tuple(places) + tuple(yaw_fallbacks)
            else:
                T_world_goal_tcp = T_planning_release_object @ np.linalg.inv(T_tcp_object)
                places = (
                    PoseCandidate(
                        f"place_for_{grasp.candidate_id}",
                        _matrix_to_pose(T_world_goal_tcp),
                        score=grasp.score,
                        metadata={
                            "paired_grasp_id": grasp.candidate_id,
                            "T_tcp_object": T_tcp_object.tolist(),
                            "target_object_pose": T_world_goal_object.tolist(),
                            "planning_target_object_pose": T_planning_goal_object.tolist(),
                            "planning_release_object_pose": T_planning_release_object.tolist(),
                            "tabletop_release_clearance_m": release_clearance_m,
                            "atom_id": atom.atom_id,
                        },
                    ),
                )
                if (
                    atom.success.orientation_tolerance_deg is not None
                    and spec.orientation_symmetry == "none"
                ):
                    yaw_limit_deg = min(
                        15.0, 0.75 * float(atom.success.orientation_tolerance_deg)
                    )
                    yaw_fallbacks: list[PoseCandidate] = []
                    for yaw_deg in (-yaw_limit_deg, yaw_limit_deg):
                        world_yaw = _axis_angle_rotation(
                            np.asarray([0.0, 0.0, 1.0]),
                            np.deg2rad(yaw_deg),
                        )
                        adjusted_release = T_planning_release_object.copy()
                        adjusted_release[:3, :3] = (
                            world_yaw @ adjusted_release[:3, :3]
                        )
                        adjusted_world_goal = T_world_goal_object.copy()
                        adjusted_world_goal[:3, :3] = (
                            world_yaw @ adjusted_world_goal[:3, :3]
                        )
                        adjusted_tcp = adjusted_release @ np.linalg.inv(T_tcp_object)
                        yaw_fallbacks.append(
                            PoseCandidate(
                                f"place_for_{grasp.candidate_id}__target_yaw_{yaw_deg:+.0f}",
                                _matrix_to_pose(adjusted_tcp),
                                score=float(grasp.score) - 0.001,
                                metadata={
                                    "paired_grasp_id": grasp.candidate_id,
                                    "T_tcp_object": T_tcp_object.tolist(),
                                    "target_object_pose": adjusted_world_goal.tolist(),
                                    "canonical_target_object_pose": T_world_goal_object.tolist(),
                                    "planning_target_object_pose": adjusted_release.tolist(),
                                    "planning_release_object_pose": adjusted_release.tolist(),
                                    "tabletop_release_clearance_m": release_clearance_m,
                                    "target_yaw_adjustment_deg": float(yaw_deg),
                                    "search_tier": 2,
                                    "atom_id": atom.atom_id,
                                },
                            )
                        )
                    places = tuple(places) + tuple(yaw_fallbacks)
            safe_places: list[PoseCandidate] = []
            for place in places:
                metadata = dict(place.metadata)
                if releases_to_tabletop:
                    rotation = _pose_to_matrix(place.pose)[:3, :3]
                    closing_axis_world_z = float(rotation[2, 1])
                    max_closing_axis_world_z = float(
                        self.config.max_tabletop_closing_axis_world_z
                    )
                    # Stability is a release property, not a grasp-candidate
                    # property. Every bare-tabletop release keeps both pads at
                    # approximately the same world-Z height; insertion and
                    # support-object relations retain their requested pose.
                    level_jaws_required = True
                    if (
                        level_jaws_required
                        and abs(closing_axis_world_z)
                        > max_closing_axis_world_z + 1.0e-8
                    ):
                        continue
                    release_object_pose = np.asarray(
                        metadata.get(
                            "planning_release_object_pose",
                            metadata.get("planning_target_object_pose"),
                        ),
                        dtype=np.float64,
                    ).reshape(4, 4)
                    object_top_z = float(
                        _world_aabb_bounds(
                            loaded_asset, mesh_scale, release_object_pose
                        )[1, 2]
                    )
                    release_tcp_z = float(place.pose.position[2])
                    required_retreat = float(
                        np.clip(
                            object_top_z
                            + float(self.config.retreat_above_object_margin_m)
                            + float(self.config.gripper_below_tcp_clearance_m)
                            - release_tcp_z,
                            float(self.config.minimum_retreat_clearance_m),
                            float(self.config.maximum_retreat_clearance_m),
                        )
                    )
                    metadata.update(
                        {
                            "closing_axis_world_z": closing_axis_world_z,
                            "max_closing_axis_world_z": (
                                max_closing_axis_world_z
                                if level_jaws_required
                                else 1.0
                            ),
                            "level_jaws_required": level_jaws_required,
                            "estimated_jaw_height_difference_m": (
                                abs(closing_axis_world_z) * 0.08
                            ),
                            "release_object_top_z_m": object_top_z,
                            "release_tcp_z_m": release_tcp_z,
                            "gripper_below_tcp_clearance_m": float(
                                self.config.gripper_below_tcp_clearance_m
                            ),
                            "required_retreat_clearance_m": required_retreat,
                            "retreat_height_policy": "object_top_plus_margin",
                        }
                    )
                safe_places.append(
                    replace(
                        place,
                        metadata={
                            **metadata,
                            "place_position_tolerance_m": float(
                                atom.success.position_tolerance_m
                            ),
                            "place_orientation_tolerance_deg": float(
                                atom.success.orientation_tolerance_deg or 0.0
                            ),
                            "continuous_place_manifold": bool(
                                atom.success.orientation_tolerance_deg is not None
                            ),
                        },
                    )
                )
            places = tuple(safe_places)
            place_by_grasp[grasp.candidate_id] = places
            for place in places:
                all_places[place.candidate_id] = place
        return PickPlaceTask(
            object_name=atom.object_id,
            current=current,
            grasp_candidates=grasps,
            place_candidates=tuple(all_places.values()),
            place_candidates_by_grasp=place_by_grasp,
            scene=planning_scene,
            placement_mode=atom.placement_mode.value,
            grasp_approach_offset=float(self.config.grasp_approach_offset_m),
            lift_height=float(self.config.lift_height_m),
            max_attempts=int(self.config.max_attempts),
            place_contact_object_name=atom.support_object_id,
            pregrasp_fallback_configuration=self._initial_joint_state(scene),
            candidate_build_time_s=time.perf_counter() - candidate_build_started,
        )

    def _joint_state(self, scene: TaskSceneState) -> JointConfiguration:
        if self.joint_state_provider is not None:
            return self.joint_state_provider(scene)
        if not scene.joint_names or scene.joint_positions is None:
            raise ValueError("task scene has no robot joint state and no joint_state_provider was configured")
        return JointConfiguration(scene.joint_names, scene.joint_positions)

    @staticmethod
    def _initial_joint_state(scene: TaskSceneState) -> JointConfiguration | None:
        names = tuple(scene.metadata.get("task_initial_joint_names", ()))
        positions = scene.metadata.get("task_initial_joint_positions")
        if not names or positions is None:
            return None
        return JointConfiguration(names, positions)

    def _planning_scene(self, scene: TaskSceneState) -> PlanningScene:
        objects: list[CollisionObject] = []
        for state in scene.objects.values():
            spec = get_object_spec(state.asset_name)
            if spec is None:
                raise KeyError(f"scene asset {state.asset_name!r} is not registered")
            proxy = build_automatic_collision_proxy(
                state.object_id,
                spec,
                self._to_planning_pose(state.pose),
                global_scale=float(self.config.scene_collision_proxy_scale),
                metadata={
                    "movable": state.movable,
                    "support_surface_kind": spec.support_surface_kind,
                },
            )
            objects.append(proxy)
        if self.config.include_maniskill_workspace_table:
            T_world_table = np.eye(4, dtype=np.float64)
            T_world_table[:3, 3] = np.asarray(
                MANISKILL_TABLE_WORLD_CENTER_M, dtype=np.float64
            )
            T_planning_table = self._to_planning_pose(T_world_table)
            objects.append(
                CollisionObject(
                    MANISKILL_TABLE_COLLISION_NAME,
                    "cuboid",
                    Pose(
                        T_planning_table[:3, 3],
                        (1.0, 0.0, 0.0, 0.0),
                    ),
                    dimensions=MANISKILL_TABLE_DIMENSIONS_M,
                    metadata={
                        "fixed": True,
                        "source": "mani_skill.TableSceneBuilder",
                        "world_pose": T_world_table.tolist(),
                        "support_surface_kind": "tabletop",
                    },
                )
            )
        return PlanningScene(tuple(objects), revision=f"task-scene-{scene.revision}")

    def _to_planning_pose(self, T_world: np.ndarray) -> np.ndarray:
        """Convert a world pose to cuRobo's robot-base planning frame."""

        transform = np.asarray(T_world, dtype=np.float64).reshape(4, 4).copy()
        base_xyz = self.config.robot_base_world_xyz_m
        if base_xyz is not None:
            transform[:3, 3] -= np.asarray(base_xyz, dtype=np.float64).reshape(3)
        return transform

    def _grasp_candidates(
        self,
        asset_name: str,
        T_world_object: np.ndarray,
    ) -> tuple[PoseCandidate, ...]:
        spec = get_object_spec(asset_name)
        if spec is None:
            raise KeyError(f"object asset {asset_name!r} is not registered")
        mesh_scale, _ = resolve_object_spec_scales(spec)
        loaded = trimesh.load(Path(spec.mesh_file), force="scene", process=False)
        extents = (np.asarray(loaded.bounds[1]) - np.asarray(loaded.bounds[0])) * mesh_scale
        capability = spec.grasp_capability
        continuous_freedom = bool(
            self.config.use_continuous_grasp_freedom
            and capability.free_rotation_axis_local
        )
        grasp_axis = np.zeros(3, dtype=np.float64)
        if capability.orientation_policy == "robot_radial":
            yaw_offsets = (
                self.config.sphere_grasp_yaw_offsets_deg
                or capability.closing_axis_yaw_offsets_deg
            )
            tilt_offsets = (
                self.config.sphere_grasp_tilt_toward_base_deg
                or capability.approach_tilt_samples_deg
            )
            axis_shift_offsets = (0.0,)
        else:
            local_long_axis = np.asarray(
                capability.primary_axis_local or (0.0, 0.0, 0.0),
                dtype=np.float64,
            )
            if np.linalg.norm(local_long_axis) < 1e-9:
                local_long_axis[int(np.argmax(extents))] = 1.0
            local_long_axis = _normalized(local_long_axis)
            long_axis = np.asarray(T_world_object[:3, :3]) @ local_long_axis
            yaw_offsets = (
                self.config.grasp_yaw_offsets_deg
                or capability.closing_axis_yaw_offsets_deg
            )
            tilt_offsets = (
                self.config.grasp_tilt_toward_base_deg
                or capability.approach_tilt_samples_deg
            )
            if continuous_freedom and self.config.grasp_tilt_toward_base_deg is None:
                tilt_offsets = capability.free_rotation_seed_angles_deg
            half_length = 0.5 * float(np.max(extents))
            axis_shift_offsets = tuple(
                float(np.clip(value, -0.70 * half_length, 0.70 * half_length))
                for value in capability.contact_axis_shift_samples_m
            )
            grasp_axis = _normalized(long_axis)
        approaching = np.asarray([0.0, 0.0, -1.0])
        yaw_variants = [(float(value), False) for value in yaw_offsets]
        if capability.closing_axis_bidirectional:
            yaw_variants.extend(
                (float(value) + 180.0, True) for value in yaw_offsets
            )
        # FoundationPose returns the asset frame origin, which is not
        # guaranteed to be the geometric center.  Compute the grasp point
        # from the mesh's AABB after transforming it into the world frame.
        # This also makes the vertical grasp height follow world Z when an
        # elongated object is lying down or its asset frame is rotated.
        center = _world_aabb_center(loaded, mesh_scale, T_world_object)
        grasp_depth = float(spec.grasp_z_offset or self.config.grasp_z_offset_m)
        position = center - approaching * grasp_depth
        if capability.orientation_policy == "robot_radial":
            base_world_yaw_deg = _robot_facing_closing_yaw_deg(
                np.asarray(T_world_object, dtype=np.float64)[:3, 3]
            )
            yaw_basis = "robot_radial"
        else:
            long_xy = np.asarray([long_axis[0], long_axis[1], 0.0])
            if np.linalg.norm(long_xy) < 1e-8:
                base_world_yaw_deg = _robot_facing_closing_yaw_deg(
                    np.asarray(T_world_object, dtype=np.float64)[:3, 3]
                )
                yaw_basis = "robot_radial_fallback"
            else:
                long_xy = _normalized(long_xy)
                closing = np.asarray([-long_xy[1], long_xy[0], 0.0])
                base_world_yaw_deg = float(
                    np.rad2deg(np.arctan2(closing[1], closing[0]))
                )
                yaw_basis = "world_object_long_axis_perpendicular"
        output: list[PoseCandidate] = []
        index = 0

        def append_candidate(
            yaw_deg: float,
            tilt_deg: float,
            axis_shift_m: float,
            *,
            search_tier: int | None = None,
            refinement_parent_id: str | None = None,
            closing_axis_flipped: bool = False,
        ) -> PoseCandidate:
            nonlocal index
            world_yaw_deg = base_world_yaw_deg + float(yaw_deg)
            angle = np.deg2rad(world_yaw_deg)
            c, s = np.cos(angle), np.sin(angle)
            tilt_rad = np.deg2rad(float(tilt_deg))
            # World-frame convention shared by every asset:
            #   yaw  = absolute azimuth of local +Y (finger opening axis);
            #   tilt = inclination of local +Z away from world -Z.
            # The tilt plane is perpendicular to the closing axis, therefore
            # local +Y remains exactly horizontal and both fingers stay at the
            # same world height for every yaw/tilt pair.
            closing = np.asarray([c, s, 0.0], dtype=np.float64)
            tilt_direction = np.asarray([-s, c, 0.0], dtype=np.float64)
            approach = _normalized(
                np.cos(tilt_rad) * approaching
                + np.sin(tilt_rad) * tilt_direction
            )
            orthogonal = _normalized(np.cross(closing, approach))
            T_world_tcp = np.eye(4)
            T_world_tcp[:3, :3] = np.stack([orthogonal, closing, approach], axis=1)
            candidate_position = center - approach * grasp_depth
            if abs(float(axis_shift_m)) > 1e-9:
                candidate_position += grasp_axis * float(axis_shift_m)
            T_world_tcp[:3, 3] = candidate_position
            label = f"grasp_{index:02d}_{yaw_deg:+g}deg"
            if abs(float(tilt_deg)) > 1e-6:
                label += f"_tilt_{tilt_deg:+g}deg"
            if abs(float(axis_shift_m)) > 1e-9:
                label += f"_axis_{axis_shift_m * 1000:+.0f}mm"
            if refinement_parent_id is not None:
                label += "_refined"
            candidate = PoseCandidate(
                label,
                _matrix_to_pose(T_world_tcp),
                score=1.0
                - 0.002 * abs(float(yaw_deg))
                - 0.001 * abs(float(tilt_deg))
                - 0.1 * abs(float(axis_shift_m)),
                metadata={
                    "relation_index": index,
                    "yaw_offset_deg": float(yaw_deg),
                    "tilt_toward_base_deg": float(tilt_deg),
                    "world_yaw_deg": float(world_yaw_deg),
                    "world_yaw_offset_deg": float(yaw_deg),
                    "world_yaw_basis": yaw_basis,
                    "tilt_from_world_z_deg": float(tilt_deg),
                    "orientation_frame": "world_z",
                    "grasp_capability": capability.name,
                    "free_rotation_axis_local": (
                        capability.free_rotation_axis_local
                        if continuous_freedom
                        else None
                    ),
                    "min_downward_approach_cosine": float(
                        capability.min_downward_approach_cosine
                    ),
                    "closing_axis_constraint": (
                        "free"
                        if capability.orientation_policy == "robot_radial"
                        else "perpendicular_to_primary_axis"
                    ),
                    "closing_axis_flipped": bool(closing_axis_flipped),
                    "grasp_depth_m": grasp_depth,
                    "axis_shift_m": float(axis_shift_m),
                    "search_tier": (
                        int(search_tier)
                        if search_tier is not None
                        else int(
                            abs(float(tilt_deg)) > 20.0
                            or abs(float(axis_shift_m)) > 0.0201
                        )
                    ),
                    "refinement_parent_id": refinement_parent_id,
                    "source": "task_scene_known_mesh",
                },
            )
            output.append(candidate)
            index += 1
            return candidate

        coarse_candidates: list[PoseCandidate] = []
        for yaw_deg, closing_axis_flipped in yaw_variants:
            for tilt_deg in tilt_offsets:
                for axis_shift_m in axis_shift_offsets:
                    coarse_candidates.append(
                        append_candidate(
                            yaw_deg,
                            tilt_deg,
                            axis_shift_m,
                            closing_axis_flipped=closing_axis_flipped,
                        )
                    )

        if capability.refine_contact_edges and not continuous_freedom:
            half_length = 0.5 * float(np.max(extents))
            seen = {
                (
                    round(float(item.metadata["yaw_offset_deg"]), 6),
                    round(float(item.metadata["tilt_toward_base_deg"]), 6),
                    round(float(item.metadata["axis_shift_m"]), 6),
                )
                for item in coarse_candidates
            }
            for parent in coarse_candidates:
                parent_shift = float(parent.metadata["axis_shift_m"])
                if abs(parent_shift) < 0.035:
                    continue
                parent_yaw = float(parent.metadata["yaw_offset_deg"])
                parent_tilt = float(parent.metadata["tilt_toward_base_deg"])
                parent_axis_flipped = bool(
                    parent.metadata.get("closing_axis_flipped", False)
                )
                toward_center = parent_shift - np.sign(parent_shift) * float(
                    capability.refinement_axis_step_m
                )
                toward_center = float(
                    np.clip(toward_center, -0.70 * half_length, 0.70 * half_length)
                )
                variants = (
                    (parent_yaw, parent_tilt, toward_center),
                    (
                        parent_yaw,
                        parent_tilt - capability.refinement_tilt_step_deg,
                        toward_center,
                    ),
                    (
                        parent_yaw,
                        parent_tilt + capability.refinement_tilt_step_deg,
                        toward_center,
                    ),
                )
                if capability.refinement_yaw_step_deg > 0.0:
                    variants += (
                        (
                            parent_yaw - capability.refinement_yaw_step_deg,
                            parent_tilt,
                            toward_center,
                        ),
                        (
                            parent_yaw + capability.refinement_yaw_step_deg,
                            parent_tilt,
                            toward_center,
                        ),
                    )
                for yaw_deg, tilt_deg, axis_shift_m in variants:
                    if abs(float(tilt_deg)) >= 90.0:
                        continue
                    key = (
                        round(float(yaw_deg), 6),
                        round(float(tilt_deg), 6),
                        round(float(axis_shift_m), 6),
                    )
                    if key in seen:
                        continue
                    seen.add(key)
                    append_candidate(
                        yaw_deg,
                        tilt_deg,
                        axis_shift_m,
                        search_tier=2,
                        refinement_parent_id=parent.candidate_id,
                        closing_axis_flipped=parent_axis_flipped,
                    )
        return tuple(output)

    def _sphere_release_candidates(
        self,
        grasp: PoseCandidate,
        T_planning_object: np.ndarray,
        T_planning_goal_object: np.ndarray,
        *,
        T_world_goal_object: np.ndarray,
        loaded_asset: str,
        atom_id: str,
    ) -> tuple[PoseCandidate, ...]:
        """Generate world-Z yaw/tilt release poses for a sphere.

        A sphere's observed and target rotations are physically irrelevant.
        Keeping a rigid object-frame transform here unnecessarily forces a
        vertical wrist pose that RM75 cannot reach near the rear-left bin.
        Preserve the held center distance while expanding only release TCP
        orientation, matching the proven legacy tennis release strategy.
        """
        spec = get_object_spec(loaded_asset)
        if spec is None:
            raise KeyError(loaded_asset)
        mesh_scale, _ = resolve_object_spec_scales(spec)
        loaded = trimesh.load(Path(spec.mesh_file), force="scene", process=False)
        source_center = _world_aabb_center(loaded, mesh_scale, T_planning_object)
        goal_center = _world_aabb_center(loaded, mesh_scale, T_planning_goal_object)
        grasp_transform = _pose_to_matrix(grasp.pose)
        tcp_center_distance = float(
            np.linalg.norm(source_center - grasp_transform[:3, 3])
        )
        down = np.asarray([0.0, 0.0, -1.0])
        base_world_yaw_deg = _robot_facing_closing_yaw_deg(
            np.asarray(T_planning_goal_object, dtype=np.float64)[:3, 3]
        )
        output: list[PoseCandidate] = []
        for tilt_deg in self.config.sphere_release_tilt_toward_base_deg:
            tilt_rad = np.deg2rad(abs(float(tilt_deg)))
            for yaw_offset_deg in self.config.sphere_release_yaw_offsets_deg:
                world_yaw_deg = base_world_yaw_deg + float(yaw_offset_deg)
                angle = np.deg2rad(world_yaw_deg)
                c, s = np.cos(angle), np.sin(angle)
                closing = np.asarray([c, s, 0.0], dtype=np.float64)
                tilt_direction = np.asarray([-s, c, 0.0], dtype=np.float64)
                approach = _normalized(
                    np.cos(tilt_rad) * down
                    + np.sin(tilt_rad) * tilt_direction
                )
                orthogonal = _normalized(np.cross(closing, approach))
                tcp_position = goal_center - approach * tcp_center_distance
                tcp_position[2] += float(self.config.sphere_release_lift_m)
                transform = np.eye(4, dtype=np.float64)
                transform[:3, :3] = np.stack(
                    [orthogonal, closing, approach], axis=1
                )
                transform[:3, 3] = tcp_position
                output.append(
                    PoseCandidate(
                        f"sphere_place_yawoff_{yaw_offset_deg:+.0f}_tilt_{tilt_deg:+.0f}",
                        _matrix_to_pose(transform),
                        score=grasp.score
                        - 0.001 * abs(float(tilt_deg))
                        - 0.00001 * abs(float(yaw_offset_deg)),
                        metadata={
                            "paired_grasp_id": grasp.candidate_id,
                            "sphere_release": True,
                            "release_tilt_toward_base_deg": float(tilt_deg),
                            "release_world_yaw_deg": float(world_yaw_deg),
                            "release_world_yaw_offset_deg": float(yaw_offset_deg),
                            "orientation_frame": "world_z",
                            "tcp_center_distance_m": tcp_center_distance,
                            "sphere_release_lift_m": float(
                                self.config.sphere_release_lift_m
                            ),
                            "target_object_pose": T_world_goal_object.tolist(),
                            "planning_target_object_pose": T_planning_goal_object.tolist(),
                            "search_tier": int(abs(float(tilt_deg)) > 15.0),
                            "atom_id": atom_id,
                        },
                    )
                )
        return tuple(output)

    def _discrete_symmetry_release_candidates(
        self,
        grasp: PoseCandidate,
        T_tcp_object: np.ndarray,
        T_planning_goal_object: np.ndarray,
        T_world_goal_object: np.ndarray,
        *,
        rotations: tuple[np.ndarray, ...],
        symmetry: str,
        atom_id: str,
        release_clearance_m: float = 0.0,
    ) -> tuple[PoseCandidate, ...]:
        """Preserve the grasp relation while searching exact shape symmetries."""

        output: list[PoseCandidate] = []
        for index, rotation in enumerate(rotations):
            equivalent_local = np.eye(4, dtype=np.float64)
            equivalent_local[:3, :3] = rotation
            planning_target = T_planning_goal_object @ equivalent_local
            world_target = T_world_goal_object @ equivalent_local
            T_planning_tcp = planning_target @ np.linalg.inv(T_tcp_object)
            output.append(
                PoseCandidate(
                    f"place_for_{grasp.candidate_id}__{symmetry}_{index:02d}",
                    _matrix_to_pose(T_planning_tcp),
                    score=float(grasp.score) - 0.00005 * index,
                    metadata={
                        "paired_grasp_id": grasp.candidate_id,
                        "orientation_symmetry": symmetry,
                        "discrete_symmetry_index": index,
                        "T_tcp_object": T_tcp_object.tolist(),
                        "target_object_pose": world_target.tolist(),
                        "canonical_target_object_pose": T_world_goal_object.tolist(),
                        "planning_target_object_pose": planning_target.tolist(),
                        "planning_release_object_pose": planning_target.tolist(),
                        "tabletop_release_clearance_m": float(release_clearance_m),
                        "search_tier": max(
                            int(grasp.metadata.get("search_tier", 0)),
                            int(index > 0),
                        ),
                        "atom_id": atom_id,
                    },
                )
            )
        return tuple(output)

    def _axial_release_candidates(
        self,
        grasp: PoseCandidate,
        T_tcp_object: np.ndarray,
        T_planning_goal_object: np.ndarray,
        T_world_goal_object: np.ndarray,
        *,
        symmetry_axis_local: tuple[float, float, float] | None,
        spin_samples_deg: tuple[float, ...],
        bidirectional: bool,
        atom_id: str,
        release_clearance_m: float = 0.0,
        release_height_offsets_m: tuple[float, ...] | None = None,
    ) -> tuple[PoseCandidate, ...]:
        """Expand only the physically irrelevant spin around an object's axis."""

        axis = np.asarray(
            symmetry_axis_local or (0.0, 1.0, 0.0), dtype=np.float64
        ).reshape(3)
        axis = _normalized(axis)
        output: list[PoseCandidate] = []
        seen_rotations: set[tuple[float, ...]] = set()
        flip_axis = _perpendicular_axis(axis)
        flip_options = (False, True) if bidirectional else (False,)
        world_axis = _normalized(T_world_goal_object[:3, :3] @ axis)
        axial_semantics = (
            "world_yaw"
            if abs(float(np.dot(world_axis, np.asarray([0.0, 0.0, 1.0]))))
            >= 1.0 - 1e-6
            else "object_axis"
        )
        for flipped in flip_options:
            flip_rotation = (
                _axis_angle_rotation(flip_axis, np.pi)
                if flipped
                else np.eye(3, dtype=np.float64)
            )
            for spin_deg in spin_samples_deg:
                spin_abs = abs(float(spin_deg))
                if flipped:
                    symmetry_search_tier = 3
                elif spin_abs <= 1e-6:
                    symmetry_search_tier = 0
                elif spin_abs <= 90.0 + 1e-6:
                    symmetry_search_tier = 1
                else:
                    symmetry_search_tier = 2
                equivalent_local = np.eye(4, dtype=np.float64)
                equivalent_local[:3, :3] = (
                    _axis_angle_rotation(axis, np.deg2rad(float(spin_deg)))
                    @ flip_rotation
                )
                rotation_key = tuple(
                    np.round(equivalent_local[:3, :3], decimals=8).reshape(-1)
                )
                if rotation_key in seen_rotations:
                    continue
                seen_rotations.add(rotation_key)
                planning_target = T_planning_goal_object @ equivalent_local
                world_target = T_world_goal_object @ equivalent_local
                T_planning_tcp = planning_target @ np.linalg.inv(T_tcp_object)
                flip_label = "_flip" if flipped else ""
                output.append(
                    PoseCandidate(
                        f"place_for_{grasp.candidate_id}__axial_{spin_deg:+.0f}{flip_label}",
                        _matrix_to_pose(T_planning_tcp),
                        score=float(grasp.score)
                        - 0.0001 * abs(float(spin_deg))
                        - 0.00005 * int(flipped),
                        metadata={
                            "paired_grasp_id": grasp.candidate_id,
                            "orientation_symmetry": (
                                "axial_bidirectional" if bidirectional else "axial"
                            ),
                            "axial_spin_deg": float(spin_deg),
                            "axis_flipped": bool(flipped),
                            "axis_direction_equivalent": bool(bidirectional),
                            "axial_rotation_semantics": axial_semantics,
                            "axial_rotation_world_axis": world_axis.tolist(),
                            "refinement_parent_id": grasp.metadata.get(
                                "refinement_parent_id"
                            ),
                            "T_tcp_object": T_tcp_object.tolist(),
                            "target_object_pose": world_target.tolist(),
                            "canonical_target_object_pose": T_world_goal_object.tolist(),
                            "planning_target_object_pose": planning_target.tolist(),
                            "planning_release_object_pose": planning_target.tolist(),
                            "tabletop_release_clearance_m": float(release_clearance_m),
                            "total_release_height_m": float(release_clearance_m),
                            "search_tier": max(
                                int(grasp.metadata.get("search_tier", 0)),
                                symmetry_search_tier,
                            ),
                            "atom_id": atom_id,
                        },
                    )
                )
        if release_clearance_m > 0.0:
            baseline = tuple(output)
            offsets = (
                self.config.tabletop_release_height_offsets_m
                if release_height_offsets_m is None
                else release_height_offsets_m
            )
            for extra_height in offsets:
                if float(extra_height) <= 1.0e-9:
                    continue
                for candidate in baseline:
                    transform = _pose_to_matrix(candidate.pose)
                    transform[2, 3] += float(extra_height)
                    metadata = dict(candidate.metadata)
                    for key in (
                        "planning_target_object_pose",
                        "planning_release_object_pose",
                    ):
                        value = metadata.get(key)
                        if value is not None:
                            shifted = np.asarray(value, dtype=np.float64).reshape(4, 4).copy()
                            shifted[2, 3] += float(extra_height)
                            metadata[key] = shifted.tolist()
                    metadata["additional_release_height_m"] = float(extra_height)
                    total_release_height = float(release_clearance_m) + float(
                        extra_height
                    )
                    metadata["total_release_height_m"] = total_release_height
                    metadata["search_tier"] = max(
                        2, int(metadata.get("search_tier", 0))
                    )
                    output.append(
                        PoseCandidate(
                            f"{candidate.candidate_id}__release_total_{total_release_height * 1000:.0f}mm",
                            _matrix_to_pose(transform),
                            score=float(candidate.score) - 0.0001 * float(extra_height),
                            metadata=metadata,
                        )
                    )
        return tuple(output)



def _world_aabb_center(
    loaded: trimesh.Scene,
    mesh_scale: float,
    T_world_object: np.ndarray,
) -> np.ndarray:
    world_mesh = loaded.copy()
    scale_transform = np.eye(4, dtype=np.float64)
    scale_transform[:3, :3] *= float(mesh_scale)
    world_mesh.apply_transform(
        np.asarray(T_world_object, dtype=np.float64).reshape(4, 4) @ scale_transform
    )
    bounds = np.asarray(world_mesh.bounds, dtype=np.float64).reshape(2, 3)
    return 0.5 * (bounds[0] + bounds[1])


def _world_aabb_bounds(
    loaded: trimesh.Scene,
    mesh_scale: float,
    T_world_object: np.ndarray,
) -> np.ndarray:
    world_mesh = loaded.copy()
    scale_transform = np.eye(4, dtype=np.float64)
    scale_transform[:3, :3] *= float(mesh_scale)
    world_mesh.apply_transform(
        np.asarray(T_world_object, dtype=np.float64).reshape(4, 4) @ scale_transform
    )
    return np.asarray(world_mesh.bounds, dtype=np.float64).reshape(2, 3)


def _normalized(value: np.ndarray) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError("cannot normalize a zero-length grasp axis")
    return vector / norm


def _robot_facing_closing_yaw_deg(position: np.ndarray) -> float:
    """World-Z yaw whose corresponding wrist tilt leans toward the robot base."""

    point = np.asarray(position, dtype=np.float64).reshape(3)
    radial_to_base = np.asarray([-point[0], -point[1], 0.0], dtype=np.float64)
    if np.linalg.norm(radial_to_base) < 1e-9:
        radial_to_base = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
    radial_to_base = _normalized(radial_to_base)
    closing = np.cross(radial_to_base, np.asarray([0.0, 0.0, -1.0]))
    return float(np.rad2deg(np.arctan2(closing[1], closing[0])))


def _axis_angle_rotation(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    x, y, z = _normalized(axis)
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    one_c = 1.0 - c
    return np.asarray(
        [
            [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
            [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
            [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
        ],
        dtype=np.float64,
    )


def _perpendicular_axis(axis: np.ndarray) -> np.ndarray:
    axis = _normalized(axis)
    seed = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(float(np.dot(axis, seed))) > 0.9:
        seed = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    return _normalized(seed - float(np.dot(seed, axis)) * axis)


def _matrix_to_pose(transform: np.ndarray) -> Pose:
    matrix = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    return Pose(matrix[:3, 3], matrix_to_quaternion_wxyz(matrix[:3, :3]))


def _pose_to_matrix(pose: Pose) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    w, x, y, z = np.asarray(pose.quaternion_wxyz, dtype=np.float64)
    transform[:3, :3] = np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    transform[:3, 3] = pose.position
    return transform
