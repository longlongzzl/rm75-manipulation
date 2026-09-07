"""Join full-seed GPU IK along the original line; audit every joint edge."""
from dataclasses import replace
import numpy as np

from rm75_app.planning.contracts import JointConfiguration,Pose,PoseCandidate


class PushPathRejected(RuntimeError):
    """A candidate fails an unchanged geometric/collision gate."""


def plan_cartesian_line(backend,request,validate,*,step_m=.005,joint_step_rad=.02):
    if len(request.candidates)!=1:
        raise ValueError('Cartesian line requires exactly one requested endpoint')
    if not (np.isfinite(step_m) and step_m>0 and np.isfinite(joint_step_rad) and joint_step_rad>0):
        raise ValueError('Cartesian/joint sampling steps must be finite and positive')
    candidate=request.candidates[0];q=np.array(request.current.positions,copy=True)
    start=backend.tool_pose_for_configuration(request.current,request.tool_frame)
    count=max(1,int(np.ceil(np.linalg.norm(candidate.pose.position-start.position)/step_m)))
    points=np.linspace(start.position,candidate.pose.position,count+1)[1:]
    output=[q.copy()];records=[]
    for index,xyz in enumerate(points):
        sub=PoseCandidate(f'{candidate.candidate_id}:ik_{index+1:03}',Pose(xyz,candidate.pose.quaternion_wxyz))
        variants=backend.solve_pose_ik_variants(replace(request,
            current=JointConfiguration(request.current.names,q),candidates=(sub,),current_by_candidate={}))
        variants=sorted(variants,key=lambda goal:float(np.linalg.norm(goal.positions-q)))
        row=dict(waypoint=index+1,total_waypoints=count,successful_ik_variants=len(variants),rejections=[])
        selected=None
        for rank,goal in enumerate(variants):
            # All successful seeds remain available; no branch can jump over
            # an unaudited intermediate state to reach a feasible endpoint.
            intervals=max(1,int(np.ceil(np.max(np.abs(goal.positions-q))/joint_step_rad)))
            edge=np.linspace(q,goal.positions,intervals+1)
            try:
                validate(edge)
            except PushPathRejected as exc:
                row['rejections'].append(dict(rank=rank,reason=str(exc)))
                continue
            row.update(selected_rank=rank,joint_samples=len(edge));selected=edge
            break
        records.append(row)
        if selected is None:
            raise PushPathRejected(f'cartesian_ik_failed:{candidate.candidate_id}: waypoint={index+1}/{count}; '
                                   f'ik_variants={len(variants)}; rejections={row["rejections"][:3]}')
        output.extend(selected[1:]);q=selected[-1]
    return np.asarray(output),records
