"""Fit actual native support convex volumes without changing deployed geometry."""
import json
import sys
from pathlib import Path
from contextlib import ExitStack
import numpy as np
sys.path.append('/home/zhangzhao/anaconda3/envs/realman/lib/python3.11/site-packages')
from rm75_app.swm.native_robot_mirror import SapienRobotStatePort
from rm75_app.swm.native_body_mirror import shape_state
from rm75_app.planning.convex_sphere_cover import cover_convex_volume, SphereCoverBudgetExceeded

report = dict(domain='actual_native_convex_volume_fit', physics_steps=0,
    primary_world_created=False, hardware_connected=False, deployed=False, links={})
with ExitStack() as resources:
    port = SapienRobotStatePort('assets/robot_models/RM75_gripper/RM75-B/urdf/RM75-B.urdf', resources=resources)
    report['urdf_sha256'] = port.urdf_sha256
    links = {link.name: link for link in port._robot.get_links()}
    for name in ('gripper_Left_Support_Link', 'gripper_Right_Support_Link'):
        results = []
        for shape in links[name].collision_shapes:
            state = shape_state(shape, np.eye(4))
            if state['kind'] != 'ConvexMesh': raise ValueError('Original convex support mesh required')
            points = np.asarray(state['geometry']['vertices']) * np.asarray(state['geometry']['scale'])
            points = (np.c_[points, np.ones(len(points))] @ np.asarray(state['T_object_shape']).T)[:, :3]
            try:
                result = cover_convex_volume(points, max_halfspace_excess_m=.0005, max_spheres=8192)
            except SphereCoverBudgetExceeded as error:
                result = error.evidence
            results.append(result)
        report['links'][name] = results
report['private_robot_closed'] = port.closed
print(json.dumps(report, indent=2), flush=True)
