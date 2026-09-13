"""Read-only diagnostic; use repository PYTHONPATH and isolated worker CLI args."""
import json
from rm75_app.pickplace.coordinator import PickPlaceCoordinator, _cached_configuration, _pose_matrix
from rm75_app.swm.native_robot import read_primary_drive_state
from rm75_app.swm.scene import SceneInvalid, pose_error
original = PickPlaceCoordinator.rank_grasp_relations


def inspect_axis_cache(self, task, candidates, scores):
    if task.enable_axis_fallback:
        return original(self, task, candidates, scores)
    primary = self.executor.primary
    before = primary.read_state()
    drives = read_primary_drive_state(primary)
    report = dict(domain='native_original_cached_IK_read_only', relation_count=len(candidates),
        full_motion_budget=task.max_motion_candidates, policy_modified=False, rows=[])
    try:
        for candidate in candidates:
            configuration = _cached_configuration(self.planner, candidate, task.current)
            row = dict(candidate_id=candidate.candidate_id, cache_available=configuration is not None)
            if configuration is not None:
                measured_fk = self.planner.tool_pose_for_configuration(configuration, task.tool_frame)
                position_error, rotation_error = pose_error(_pose_matrix(measured_fk), _pose_matrix(candidate.pose))
                row.update(joint_names=list(configuration.names), positions=configuration.positions.tolist(),
                    candidate_pose=candidate.pose.as_curobo_list(),
                    fk_position_error_m=position_error, fk_rotation_error_rad=rotation_error)
            report['rows'].append(row)
    except BaseException as error:
        report['error'] = repr(error)
        raise
    finally:
        final = primary.read_state()
        report['primary_unchanged'] = all(final[key] == before[key]
            for key in ('positions', 'velocities', 'objects')) and read_primary_drive_state(primary) == drives
        print('SWM_CACHED_IK_DIAGNOSTIC ' + json.dumps(report), flush=True)
        if not report['primary_unchanged']:
            raise SceneInvalid('Cache diagnostic changed primary state')
    raise SceneInvalid('Read-only cache diagnostic complete; additional motion planning not executed')


PickPlaceCoordinator.rank_grasp_relations = inspect_axis_cache
from rm75_app.workcell.worker import main
raise SystemExit(main())
