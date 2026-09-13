"""Original PushT goal scoring, gated by exact predicted-scene audit identity."""
from dataclasses import asdict
import numpy as np
from .scene import digest,SceneInvalid
from .future_collision_samples import future_collision_samples
from rm75_app.pusht.model import error,reached


def score_audited_future_push(motion,prediction,audit,snapshot,object_id,goal_xyyaw,config):
    expected=dict(source_snapshot_id=snapshot['snapshot_id'],source_plan_digest=motion['source_plan_digest'],
        future_motion_digest=motion['future_motion_digest'],prediction_digest=digest(prediction),
        parameters_digest=digest(prediction['parameters']),model_digest=digest(asdict(config)),object_id=object_id)
    if (audit.get('source')!='original_CuroboPushExecutor_dynamic_scene_collision_audit' or
            any(audit.get(key)!=value for key,value in expected.items()) or
            audit.get('collision_samples_checked') is not True or audit.get('original_joint_limits_checked') is not True or
            audit.get('retreat',{}).get('final_contact_pairs')!=0):
        raise SceneInvalid('Candidate goal score requires its exact original collision audit')
    samples=future_collision_samples(motion,prediction,snapshot,object_id)
    if audit.get('samples')!=len(samples):raise SceneInvalid('Candidate audit coverage changed')
    def planar(matrix):
        matrix=np.asarray(matrix,float)
        return [float(matrix[0,3]),float(matrix[1,3]),float(np.arctan2(matrix[1,0],matrix[0,0]))]
    initial=planar(samples[0]['T_world_object']);final=planar(samples[-1]['T_world_object'])
    before=error(initial,goal_xyyaw,config);after=error(final,goal_xyyaw,config)
    return dict(source='original_pusht_model.error_and_reached',goal_xyyaw=list(goal_xyyaw),
        goal_digest=digest(list(goal_xyyaw)),initial_cost=before,cost=after,predicted_improvement=before-after,
        predicted_goal_reached=bool(reached(final,goal_xyyaw,config)),
        prediction_digest=expected['prediction_digest'],audit_digest=digest(audit),
        source_plan_digest=expected['source_plan_digest'],ranking_only=True,
        verified_task_goal_reached=False,full_primitive_audit_issued=False,execution_authorized=False)
