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

from .scene import SceneInvalid, transform, pose_error
from .skills import SkillRequest


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
        offset = getattr(config, 'robot_base_world_xyz_m', None)
        self.base_offset = np.zeros(3) if offset is None else np.asarray(offset, dtype=float).reshape(3)
        if not np.isfinite(self.base_offset).all():
            raise ValueError('Invalid original robot-base calibration')
        self._active = None
        self._iterated = False

    def _base_target(self, atom):
        target = transform(atom.target_pose)
        target[:3, 3] -= self.base_offset
        return target

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
            yield SkillRequest('place', atom.object_id, self._base_target(atom),
                position_tolerance_m=position_tolerance, rotation_tolerance_rad=float(angle_tolerance))
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
            measured = transform(obj['measured']['T_world_object'])
            measured[:3, 3] += self.base_offset
            state.pose = measured
            state.lifecycle = ObjectLifecycle.HELD if obj['lifecycle'] == 'held' else ObjectLifecycle.AVAILABLE
            state.movable = not obj['fixed']
            state.metadata['swm_observation'] = copy.deepcopy(obj['measured'])
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
            p, r = pose_error(request.target, self._base_target(atom))
            if p > 1e-9 or r > 1e-7:
                raise SceneInvalid('Atomic target differs from the trusted compiled target')
        # Call the ORIGINAL FixedSceneAtomTaskBuilder or its magnetic wrapper.
        # Candidate support metadata, release clearances and collision proxies
        # remain owned by that implementation.
        task = self.builder(atom, self.measured_native_scene(snapshot))
        if {obj.name for obj in task.scene.objects} != set(snapshot['objects']):
            raise SceneInvalid('Implicit native table/obstacles must be registered and observed in SWM')
        objects = tuple(replace(obj, dimensions=None if obj.dimensions is None else np.asarray(obj.dimensions).tolist(), metadata={**obj.metadata,
            'swm_object_pose': snapshot['objects'][obj.name]['measured']['T_world_object']}) for obj in task.scene.objects)
        return replace(task, scene=replace(task.scene, objects=objects, revision=snapshot['snapshot_id']))

    def verify_goals(self, snapshot):
        from rm75_app.orchestration.multi_object_executor import AtomExecution, validate_target_pose

        if snapshot['robot']['holding'] != 'empty':
            return False
        scene = self.measured_native_scene(snapshot)
        if self.task == 'magnetic':
            result = self.structure_verifier(self.plan, scene)
            if type(result) is not bool:
                raise TypeError('Measured structure verifier must return a boolean')
            return result
        # Each object's final compiled target wins for repeated-object tasks.
        final_atoms = {atom.object_id: atom for atom in self.plan.atoms}
        return all(validate_target_pose(atom, AtomExecution(True,
            final_object_pose=scene.objects[oid].pose)).success for oid, atom in final_atoms.items())
