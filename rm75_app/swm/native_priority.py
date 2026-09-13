"""Bounded cached-IK closure heuristics, never execution or audit permission."""
from __future__ import annotations

from pathlib import Path
import numpy as np

from rm75_app.pickplace.coordinator import _cached_configuration, _pose_matrix
from .native_closure import NativeClosureRejected, reject_predicted_closure
from .scene import SceneInvalid, digest, pose_error


class CachedClosurePriority:
    """Prioritize, but never eliminate, original relations using private physics.

    Cached IK has measured millimetre-scale FK residuals. A coarse veto therefore
    only lowers priority; the solved trajectory endpoint MUST be checked again.
    A new snapshot resets the bounded heuristic cache, not a motion budget.
    """
    def __init__(self, planner, primary, registration, urdf_path, *, directory, emit,
                 max_predictions=64):
        if type(max_predictions) is not int or not 1 <= max_predictions <= 64:
            raise ValueError('Bounded original coarse-batch prediction budget required')
        self.planner, self.primary = planner, primary
        self.registration, self.urdf_path = registration, urdf_path
        self.directory, self.emit = Path(directory), emit
        self.limit = max_predictions
        self._snapshot_id = None
        self._cache = {}
        self._spent = 0

    def __call__(self, task, candidates, snapshot):
        if (not isinstance(snapshot, dict) or task.scene.revision != snapshot.get('snapshot_id')
                or snapshot.get('valid') is not True or snapshot.get('observation_domain') != 'physics'):
            raise SceneInvalid('Cached closure priority requires the same valid physics snapshot')
        if snapshot['snapshot_id'] != self._snapshot_id:
            self._snapshot_id = snapshot['snapshot_id']
            self._cache.clear()
            self._spent = 0
        ids = [candidate.candidate_id for candidate in candidates]
        if len(ids) != len(set(ids)):
            raise SceneInvalid('Duplicate cached-priority candidate identity')
        result = {}
        for candidate in candidates:
            configuration = _cached_configuration(self.planner, candidate, task.current)
            if configuration is None:
                raise SceneInvalid('Original cached IK unavailable for closure priority')
            q = np.asarray(configuration.positions, dtype=float)
            if (tuple(configuration.names) != tuple(task.current.names)
                    or q.shape != (7,) or not np.isfinite(q).all()):
                raise SceneInvalid('Cached IK has wrong joint identity or nonfinite state')
            actual_fk = self.planner.tool_pose_for_configuration(configuration, task.tool_frame)
            p_error, r_error = pose_error(_pose_matrix(actual_fk), _pose_matrix(candidate.pose))
            key = digest(dict(snapshot_id=self._snapshot_id, candidate_id=candidate.candidate_id,
                joint_names=list(configuration.names), positions=q.tolist(),
                candidate_pose=candidate.pose.as_curobo_list()))
            hit = key in self._cache
            output = self.directory / (key + '.json')
            if hit:
                priority = self._cache[key]
            elif self._spent >= self.limit:
                priority = 1  # Explicitly unpredicted; NOT a pass or a veto.
            else:
                output.parent.mkdir(parents=True, exist_ok=True)
                self._spent += 1
                try:
                    reject_predicted_closure(self.primary, self.registration, self.urdf_path,
                        target=task.object_name, emit=self.emit, output=output,
                        candidate_snapshot=snapshot, candidate_configuration=configuration)
                except NativeClosureRejected:
                    priority = 2
                else:
                    priority = 0
                self._cache[key] = priority
            result[candidate.candidate_id] = priority
            self.emit(kind='swm_native_cached_closure_priority', snapshot_id=self._snapshot_id,
                candidate_id=candidate.candidate_id, cached_joint_names=list(configuration.names),
                cached_positions=q.tolist(), candidate_pose=candidate.pose.as_curobo_list(),
                fk_position_error_m=p_error, fk_rotation_error_rad=r_error,
                priority=priority, classification=(
                    'no_forbidden_contact_predicted' if priority == 0 else
                    'not_predicted_budget_exhausted' if priority == 1 else 'predicted_physical_rejection'),
                cache_hit=hit, predictions_spent=self._spent, prediction_limit=self.limit,
                evidence_path=None if priority == 1 else str(output),
                exact_endpoint_recheck_required=True, execution_authorized=False,
                model_qualified=False)
        return result
