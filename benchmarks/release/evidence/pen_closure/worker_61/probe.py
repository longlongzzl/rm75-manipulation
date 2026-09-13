"""One private feedback-close hypothesis; never authorize primary execution."""
import copy
import json
from pathlib import Path
from dataclasses import replace
import numpy as np
from rm75_app.assets.object_specs import OBJECT_SPECS
from rm75_app.swm import native_drive_commands, native_closure
from rm75_app.swm.native_bootstrap import _array
from rm75_app.swm.scene import SceneInvalid

ROOT = Path.cwd().resolve()
OUT = ROOT / 'runtime_data/swm_release_pen_worker_61'
original_spec = OBJECT_SPECS['bi']
if original_spec.grasp_z_offset != .012777:
    raise SceneInvalid('Frozen pen depth changed')
OBJECT_SPECS['bi'] = replace(original_spec, grasp_z_offset=.016777)
original_apply = native_drive_commands.apply_private_control
original_predict = native_closure.reject_predicted_closure
state = dict(command=-1., contact_mode=False, closed_calls=0, trace=[])
owned = None


def feedback_control(robot, physics, command, groups):
    global owned
    from mani_skill.utils.sapien_utils import compute_total_impulse
    if owned is None:
        owned = robot
    if robot is not owned:
        raise SceneInvalid('Diagnostic permits one private articulation only')
    roots = ('gripper_Left_1_Joint', 'gripper_Right_1_Joint')
    indices = [command['joint_names'].index(name) for name in roots]
    requested = [command['position_targets'][index] for index in indices]
    if requested == [0., 0.]:
        return original_apply(robot, physics, command, groups)
    if not np.allclose(requested, [.91, .91], atol=1e-7, rtol=0):
        raise SceneInvalid('Original normalized closed endpoint required')
    state['closed_calls'] += 1
    if state['closed_calls'] > 220:
        raise SceneInvalid('Original close plus hold diagnostic budget exhausted')
    fingers = ('gripper_Left_Support_Link', 'gripper_Right_Support_Link')
    vectors = {name: np.zeros(3) for name in fingers}
    for contact in physics.get_contacts():
        names = [body.entity.name for body in contact.bodies]
        if 'bi' not in names:
            continue
        for name in fingers:
            if name in names:
                vectors[name] += compute_total_impulse([(contact, names[0] == name)]) / physics.timestep
    forces = [float(np.linalg.norm(vectors[name])) for name in fingers]
    if max(forces) > .1:
        state['contact_mode'] = True
    in_band = min(forces) >= .5 and max(forces) <= 2.
    if not state['contact_mode']:
        state['command'] = min(1., state['command'] + .04)
    elif not in_band:
        error = 1. - (max(forces) if max(forces) > 4. else min(forces))
        angle = (state['command'] + 1.) * .91 / 2. + float(np.clip(error * .0005, -.0005, .0005))
        state['command'] = float(np.clip(2. * angle / .91 - 1., -1., 1.))
    adaptive = copy.deepcopy(command)
    for index in indices:
        adaptive['position_targets'][index] = (state['command'] + 1.) * .91 / 2.
    names = [joint.name for joint in robot.get_active_joints()]
    q = _array(robot.get_qpos()).reshape(-1)
    arm = [names.index(f'joint_{i}') for i in range(1, 8)]
    target = [command['position_targets'][command['joint_names'].index(f'joint_{i}')] for i in range(1, 8)]
    state['trace'].append(dict(control_tick=state['closed_calls'],
        command=state['command'], native_root_targets=[adaptive['position_targets'][i] for i in indices],
        pre_step_forces_n=forces, in_force_band=in_band,
        pre_step_arm_endpoint_error_rad=float(np.max(np.abs(q[arm] - target))),
        holding_qualified=False))
    return original_apply(robot, physics, adaptive, groups)


def one_prediction(*args, **kwargs):
    try:
        original_predict(*args, **kwargs)
    finally:
        evidence = dict(hypothesis='private_feedback_close_not_production_action',
            original_depth_m=.012777, hypothesis_depth_m=.016777,
            physical_parameters_changed=False, thresholds_changed=False,
            force_band_n=[.5, 2.], force_target_n=1.,
            source_execution_authorized=False, holding_qualified=False,
            note='Base prediction nominal closed targets are not this adaptive action; use trace.', **state)
        (OUT / 'feedback_hypothesis.json').write_text(json.dumps(evidence, indent=2))
    raise SceneInvalid('Private feedback hypothesis complete; source execution is prohibited')

native_drive_commands.apply_private_control = feedback_control
native_closure.reject_predicted_closure = one_prediction
try:
    from rm75_app.workcell.worker import main
    raise SystemExit(main())
finally:
    native_drive_commands.apply_private_control = original_apply
    native_closure.reject_predicted_closure = original_predict
    OBJECT_SPECS['bi'] = original_spec
