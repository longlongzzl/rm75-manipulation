"""Read-only diagnostic; run with worker CLI arguments under network isolation."""
import json
from rm75_app.pickplace.coordinator import PickPlaceCoordinator
original = PickPlaceCoordinator.rank_grasp_relations


def observed_rank(self, task, candidates, scores):
    rows = original(self, task, candidates, scores)
    print('SWM_CANDIDATE_BUDGET ' + json.dumps(dict(
        original_count=len(task.grasp_candidates),
        screened_count=len(candidates),
        ranked_count=len(rows),
        max_motion_candidates=task.max_motion_candidates,
        screened_ids=[item.candidate_id for item in candidates],
        ranked_ids=[item.candidate_id for item in rows],
        policy_modified=False)), flush=True)
    return rows


PickPlaceCoordinator.rank_grasp_relations = observed_rank
from rm75_app.workcell.worker import main
raise SystemExit(main())
