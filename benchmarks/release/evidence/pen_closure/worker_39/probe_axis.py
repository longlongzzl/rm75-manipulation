"""Diagnostic only. Run with repository PYTHONPATH and isolated worker CLI args."""
import json
from rm75_app.swm.native_skills import PickPlaceNativePhases
from rm75_app.swm.native_robot import read_primary_drive_state
from rm75_app.swm.scene import SceneInvalid
from rm75_app.pickplace.coordinator import _CONTACT_ENDPOINT_COLLISION_LINKS, _rebuild_places_for_resolved_grasp
original = PickPlaceNativePhases._grasp_relations


def probe_after_exhaustion(self, task, jaw):
    yield from original(self, task, jaw)
    primary = self.coordinator.executor.primary
    before = primary.read_state()
    drives = read_primary_drive_state(primary)
    report = dict(domain='native_continuous_axis_candidate_diagnostic',
        original_count=len(task.grasp_candidates), axis_enabled=task.enable_axis_fallback,
        whole_episode_called=False, new_primary_actions=False, model_qualified=False,
        skill_verified=False)
    try:
        resolved = tuple(self.coordinator.planner.resolve_axis_constrained_pose_candidates(
            tuple(task.grasp_candidates), task.scene, tool_frame=task.tool_frame,
            ignore_object_names=(task.object_name,), disable_collision_links=_CONTACT_ENDPOINT_COLLISION_LINKS))
        originals = {candidate.candidate_id: candidate for candidate in task.grasp_candidates}
        report['resolved_count'] = len(resolved)
        report['eligible_relations'] = []
        for candidate in resolved:
            source_id = str(candidate.metadata.get('source_grasp_candidate_id', candidate.candidate_id))
            if source_id not in originals:
                raise SceneInvalid('Resolved candidate has foreign source identity')
            places = _rebuild_places_for_resolved_grasp(originals[source_id], candidate,
                task.places_for_grasp(source_id))
            report['eligible_relations'].append(dict(candidate_id=candidate.candidate_id,
                source_id=source_id, rebuilt_places=len(places),
                position=candidate.pose.position.tolist(), quaternion_wxyz=candidate.pose.quaternion_wxyz.tolist()))
    except BaseException as error:
        report['error'] = repr(error)
        raise
    finally:
        final = primary.read_state()
        report['primary_unchanged'] = all(final[key] == before[key]
            for key in ('positions', 'velocities', 'objects')) and read_primary_drive_state(primary) == drives
        print('SWM_AXIS_DIAGNOSTIC ' + json.dumps(report), flush=True)
        if not report['primary_unchanged']:
            raise SceneInvalid('Candidate diagnostic changed primary state')


PickPlaceNativePhases._grasp_relations = probe_after_exhaustion
from rm75_app.workcell.worker import main
raise SystemExit(main())
