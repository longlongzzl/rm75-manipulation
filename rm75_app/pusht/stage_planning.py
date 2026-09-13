"""Measured-state stage replanning using the original PushT motion bridge."""
import numpy as np
from rm75_app.planning.contracts import BatchPlanningRequest,JointConfiguration,PoseCandidate
from rm75_app.workcell.realman import time_parameterize
from .cartesian_ik import PushPathRejected,plan_cartesian_line
from .model import Push
from .physics_replay import STAGES


def replan_stage(executor,observation,request):
    stage=request['stage']
    if stage not in STAGES:raise ValueError('Unknown measured stage')
    names=tuple(executor.backend._ensure_planner().joint_names)
    if names!=tuple(f'joint_{index}' for index in range(1,8)):
        raise ValueError('Measured stage requires original ordered seven arm joints')
    executor.names=names
    q=np.asarray(executor.arm.read_joints(),dtype=float)
    goal_q=np.asarray(request['goal_q'],dtype=float)
    if (q.shape!=(7,) or goal_q.shape!=(7,) or not np.isfinite(q).all()
            or not np.isfinite(goal_q).all()):
        raise ValueError('Invalid measured stage joints')
    push=Push(**request['push'])
    scene=executor._scene(observation);executor.backend.update_scene(scene)
    executor.backend.set_gripper_collision_state(closed=True)
    if stage=='retreat':
        path,times=executor._plan_retreat(q,push,scene,request['contact_binding'])
        # The old generator suspends checks. Recheck with measured target and
        # static scene after restoration, never inherit that exemption.
        executor._audit(path,contact=False)
    else:
        goal=executor._fk(goal_q);start=executor._fk(q)
        straight=stage!='approach';contact=stage in ('contact','push')
        candidate=PoseCandidate(stage,goal)
        planning=BatchPlanningRequest(JointConfiguration(executor.names,q),(candidate,),
            scene=scene,tool_frame=executor.tool_frame,prefer_direct_tcp_path=straight)
        saved={name:executor.backend._obstacle_enabled(name)
               for name in ('pusht_target_0','pusht_target_1')}
        def audit(edge):
            executor._audit_tcp(edge,start,goal.position,straight=straight,stage=stage,
                                check_endpoint=False)
            executor._audit(edge,contact=contact)
        try:
            if contact:
                for name in saved:executor.backend._set_obstacle_enabled(name,False)
            if straight:path,_=plan_cartesian_line(executor.backend,planning,audit)
            else:
                plan=executor.backend.plan_candidates(planning).best((candidate,))
                if plan is None or plan.trajectory is None:
                    raise PushPathRejected('Measured approach planning failed')
                path=plan.trajectory.positions
        finally:
            for name,enabled in saved.items():executor.backend._set_obstacle_enabled(name,enabled)
        executor._audit_tcp(path,start,goal.position,straight=straight,stage=stage)
        path,times=time_parameterize(path,lambda joints:executor._fk(joints).position,
            speed_mps=push.speed_mps,hz=executor.arm.hz,
            joint_speed_rad_s=executor.profile.get('joint_speed_rad_s',.25),
            joint_accel_rad_s2=executor.profile.get('joint_accel_rad_s2',.5))
        executor._audit_tcp(path,start,goal.position,straight=straight,stage=stage)
        executor._audit(path,contact=contact)
    if np.max(abs(path[0]-q))>1e-5:raise PushPathRejected('Measured stage start mismatch')
    return dict(stage=stage,positions=path.tolist(),times=times.tolist(),
                stage_validated=True,measured_start_q=q.tolist(),
                source_observation=observation.as_dict(),swm_scene_audit_verified=False)
