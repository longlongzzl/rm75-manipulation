"""Use already compiled PickPlace/Jimu atoms at real grasp/place boundaries.

The original compiler owns generation proof, ordering, inventory identity,
support rules and builder-to-world conversion. This bridge only converts its
world frame to the original builder's base frame and refreshes measured state.
It neither invokes the legacy episode nor accepts browser Python callbacks.
"""
from __future__ import annotations
from dataclasses import replace
import copy
import numpy as np

from .scene import SceneInvalid, transform, pose_error, digest
from .skills import SkillRequest, SkillVerification


class CompiledNativeTask:
    def __init__(self, task, compiled_plan, template_scene, task_builder, *, structure_verifier=None):
        from rm75_app.tasks.manipulation_plan import ManipulationPlan

        if task not in ('pickplace', 'magnetic') or not isinstance(compiled_plan, ManipulationPlan):
            raise TypeError('Original typed manipulation plan required')
        if not compiled_plan.atoms:
            raise ValueError('Empty native task')
        if task == 'magnetic' and not callable(structure_verifier):
            raise NotImplementedError('Independent measured structure verification is required')
        self.task = task
        self.plan = compiled_plan
        self.template = template_scene.copy()
        self.builder = task_builder
        self.structure_verifier = structure_verifier
        base_builder = getattr(task_builder, 'base', task_builder)
        config = getattr(base_builder, 'config', None)
        from rm75_app.core.frames import explicit_robot_base_transform
        self.T_world_base = explicit_robot_base_transform(
            world_xyz=getattr(config, 'robot_base_world_xyz_m', None),
            world_transform=getattr(config, 'robot_base_world_transform', None))
        self.T_base_world = np.linalg.inv(self.T_world_base)
        # Retain the informational translation attribute, never use it to drop rotation.
        self.base_offset = self.T_world_base[:3, 3].copy()
        self._active = None
        self._iterated = False
        # Preserve the original container-relative inside goal. Support metadata
        # alone does not make other task targets relative.
        self._relative_targets = {}
        for atom in self.plan.atoms:
            if atom.semantic_operator != 'inside':
                continue
            reference = atom.support_object_id
            if reference not in self.template.objects or reference == atom.object_id:
                raise SceneInvalid('Inside target requires its original observed container')
            local = np.linalg.inv(transform(self.template.objects[reference].pose)) @ transform(atom.target_pose)
            self._relative_targets[atom.atom_id] = (reference, local)

    def _base_target(self, atom, snapshot=None):
        if atom.atom_id in self._relative_targets and snapshot is not None:
            reference, local = self._relative_targets[atom.atom_id]
            return transform(SkillRequest('place', atom.object_id, local,
                target_reference_id=reference).resolve(snapshot).target)
        return self.T_base_world @ transform(atom.target_pose)

    def requests(self):
        if self._iterated:
            raise SceneInvalid('Do not replay an already consumed task program')
        self._iterated = True
        for atom in self.plan.atoms:
            self._active = atom
            position_tolerance = min(.006, atom.success.position_tolerance_m or .006)
            angle_tolerance = min(.10, np.deg2rad(atom.success.orientation_tolerance_deg or np.rad2deg(.10)))
            yield SkillRequest('grasp', atom.object_id, position_tolerance_m=position_tolerance,
                               rotation_tolerance_rad=float(angle_tolerance))
            reference, target = self._relative_targets.get(atom.atom_id, (None, self._base_target(atom)))
            yield SkillRequest('place', atom.object_id, target,
                position_tolerance_m=position_tolerance, rotation_tolerance_rad=float(angle_tolerance),
                target_reference_id=reference, goal_predicate="native_relation")
        self._active = None

    def measured_native_scene(self, snapshot):
        from rm75_app.orchestration.multi_object_executor import ObjectLifecycle

        if not snapshot['valid'] or set(snapshot['objects']) != set(self.template.objects):
            raise SceneInvalid('Original inventory and complete measured SWM must agree')
        scene = self.template.copy()
        for oid, state in scene.objects.items():
            obj = snapshot['objects'][oid]
            asset = snapshot['assets'][obj['asset_id']]
            if asset.get('native_asset_name') != state.asset_name:
                raise SceneInvalid('Explicit native asset/instance identity binding required')
            measured = self.T_world_base @ transform(obj['measured']['T_world_object'])
            state.pose = measured
            state.lifecycle = ObjectLifecycle.HELD if obj['lifecycle'] == 'held' else ObjectLifecycle.AVAILABLE
            state.movable = not obj['fixed']
            state.metadata['swm_observation'] = copy.deepcopy(obj['measured'])
            if asset.get('native_infrastructure') is True:
                if obj['fixed'] is not True or asset['collision_kind'] != 'cuboid':
                    raise SceneInvalid('Native infrastructure must retain fixed cuboid geometry')
                state.metadata['swm_native_infrastructure'] = dict(
                    dimensions_m=copy.deepcopy(asset['collision_dimensions_m']),
                    T_object_collision=copy.deepcopy(asset['T_object_collision']))
        scene.revision = snapshot['revision']
        scene.backend_revision = snapshot['snapshot_id']
        scene.joint_names = tuple(snapshot['robot']['joint_names'])
        scene.joint_positions = np.asarray(snapshot['robot']['positions'], dtype=float)
        return scene

    def build_task(self, request, snapshot):
        atom = self._active
        if atom is None or request.object_id != atom.object_id or request.skill not in ('grasp', 'place'):
            raise SceneInvalid('Request is not the active original compiled atom')
        if request.skill == 'place':
            p, r = pose_error(request.resolve(snapshot).target, self._base_target(atom, snapshot))
            if p > 1e-9 or r > 1e-7:
                raise SceneInvalid('Atomic target differs from the trusted compiled target')
        # Call the ORIGINAL FixedSceneAtomTaskBuilder or its magnetic wrapper.
        # Candidate support metadata, release clearances and collision proxies
        # remain owned by that implementation.
        if atom.atom_id in self._relative_targets:
            atom = replace(atom, target_pose=self.T_world_base @ self._base_target(atom, snapshot))
        task = self.builder(atom, self.measured_native_scene(snapshot))
        if {obj.name for obj in task.scene.objects} != set(snapshot['objects']):
            raise SceneInvalid('Implicit native table/obstacles must be registered and observed in SWM')
        objects = tuple(replace(obj, dimensions=None if obj.dimensions is None else np.asarray(obj.dimensions).tolist(), metadata={**obj.metadata,
            'swm_object_pose': snapshot['objects'][obj.name]['measured']['T_world_object']}) for obj in task.scene.objects)
        return replace(task, scene=replace(task.scene, objects=objects, revision=snapshot['snapshot_id']))

    def _validate_atom(self, atom, snapshot, scene, *, request=None):
        from rm75_app.orchestration.multi_object_executor import AtomExecution, validate_target_pose
        from rm75_app.execution.maniskill_task_bridge import validate_inside_relation

        current = replace(atom, target_pose=self.T_world_base @ self._base_target(atom, snapshot))
        actual = scene.objects[atom.object_id].pose
        if atom.success.relation.strip().lower() == 'inside':
            support = scene.objects.get(atom.support_object_id)
            if support is None:
                raise SceneInvalid('Native inside verification requires measured container')
            # Only the original pure geometric predicate: no settling, actor
            # writes or extra observation outside the synchronized checkpoint.
            return validate_inside_relation(current, actual, support.pose, support.asset_name)
        if request is not None:
            current = replace(current, success=replace(current.success,
                position_tolerance_m=min(request.position_tolerance_m,
                    current.success.position_tolerance_m or request.position_tolerance_m),
                orientation_tolerance_deg=min(float(np.rad2deg(request.rotation_tolerance_rad)),
                    current.success.orientation_tolerance_deg or float(np.rad2deg(request.rotation_tolerance_rad)))))
        return validate_target_pose(current, AtomExecution(True, final_object_pose=actual))

    def verify_skill(self, request, snapshot):
        atom = self._active
        if (atom is None or request.skill != 'place' or request.object_id != atom.object_id
                or request.goal_predicate != 'native_relation'):
            raise SceneInvalid('Native verification is not bound to the active original place atom')
        reference, local = self._relative_targets.get(atom.atom_id, (None, self._base_target(atom)))
        # Bind the semantic program, not a cached resolved pose. A moved
        # container must affect both the target and the measured predicate.
        if request.target_reference_id != reference:
            raise SceneInvalid('Native verification target reference differs from compiled atom')
        p, r = pose_error(request.target, local)
        if p > 1e-9 or r > 1e-7:
            raise SceneInvalid('Native verification target differs from compiled atom')
        scene = self.measured_native_scene(snapshot)
        result = self._validate_atom(atom, snapshot, scene, request=request)
        reached = bool(result.success and snapshot['robot']['holding'] == 'empty')
        return SkillVerification(digest(request.as_dict()), snapshot['snapshot_id'], reached,
            dict(relation=atom.success.relation, message=result.message,
                 position_error_m=result.position_error_m,
                 orientation_error_deg=result.orientation_error_deg,
                 geometry=copy.deepcopy(result.diagnostics)))

    def verify_goals(self, snapshot):
        if snapshot['robot']['holding'] != 'empty':
            return False
        scene = self.measured_native_scene(snapshot)
        if self.task == 'magnetic':
            result = self.structure_verifier(self.plan, scene)
            if type(result) is not bool:
                raise TypeError('Measured structure verifier must return a boolean')
            return result
        final_atoms = {atom.object_id: atom for atom in self.plan.atoms}
        return all(self._validate_atom(atom, snapshot, scene).success for atom in final_atoms.values())
