"""Private marker FK only; run from repository root under network isolation."""
import json, sys
from contextlib import ExitStack
from pathlib import Path
import numpy as np
sys.path.append('/home/zhangzhao/anaconda3/envs/realman/lib/python3.11/site-packages')
from rm75_app.swm.native_robot_mirror import SapienRobotStatePort
from rm75_app.swm.scene import SceneInvalid
inputs = sorted(Path('runtime_data/swm_release_pen_worker_43/swm_native/closure_priority').resolve().glob('*.json'))
report = dict(domain='native_private_marker_FK_diagnostic', builder_default_offset_m=.012777,
    physics_steps=0, primary_world_created=False, contact_calibration_qualified=False, rows=[])
with ExitStack() as resources:
    port = SapienRobotStatePort('/home/zhangzhao/.maniskill/data/robots/RM75_gripper/RM75-B/urdf/RM75-B.urdf', resources=resources)
    report['urdf_path'] = str(port.urdf_path)
    names = [joint.name for joint in port._robot.get_active_joints()]
    links = {link.name: link for link in port._robot.get_links()}
    if not {'gripper_tcp', 'left_pad', 'right_pad'} <= set(links):
        raise SceneInvalid('Original marker link inventory is incomplete')
    for path in inputs:
        evidence = json.loads(path.read_text())
        opening = evidence['candidate_open_preparation']
        incoming_names = opening['joint_names']
        if set(incoming_names) != set(names) or len(incoming_names) != len(names):
            raise SceneInvalid('Recorded joint identity differs from original native model')
        for stage, values in [('open_prepared', opening['positions']), ('last_rejected_state', evidence['steps'][-1]['q'])]:
            q = np.asarray([values[incoming_names.index(name)] for name in names], dtype=np.float32)
            port._robot.set_qpos(q)
            if np.max(np.abs(np.asarray(port._robot.get_qpos())-q)) > 1e-6:
                raise SceneInvalid('Private FK state failed native readback')
            tcp = links['gripper_tcp'].entity.pose.to_transformation_matrix()
            left = np.asarray(links['left_pad'].entity.pose.p, dtype=float)
            right = np.asarray(links['right_pad'].entity.pose.p, dtype=float)
            midpoint = np.linalg.inv(tcp) @ np.r_[.5*(left+right), 1.]
            report['rows'].append(dict(source_file=str(path), stage=stage,
                pad_midpoint_in_tcp_m=midpoint[:3].tolist(),
                pad_marker_span_m=float(np.linalg.norm(left-right)),
                delta_from_builder_default_m=float(midpoint[2]-.012777)))
report['private_robot_closed'] = port.closed
print('SWM_PAD_FK_DIAGNOSTIC ' + json.dumps(report), flush=True)
