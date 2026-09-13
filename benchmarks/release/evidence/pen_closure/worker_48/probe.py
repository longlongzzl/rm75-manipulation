"""Original initial arm hold baseline; intentionally does not execute a skill."""
import json
import numpy as np
from rm75_app.swm.native_bootstrap import _array
from rm75_app.swm.native_execution import NativePrimaryExecutor
from rm75_app.swm.scene import SceneInvalid
from rm75_app.workcell.worker import main

original_execute = NativePrimaryExecutor.execute_trajectory
original_plan_depth = None
# Reach the same audited pre-execution boundary used by workers 44-47. The
# hypothesis affects only planning; its trajectory is never executed here.
from dataclasses import replace
from rm75_app.assets.object_specs import OBJECT_SPECS
original_spec = OBJECT_SPECS['bi']
if original_spec.grasp_z_offset != .012777:
    raise SceneInvalid('Frozen baseline pen depth changed')
OBJECT_SPECS['bi'] = replace(original_spec, grasp_z_offset=.016777)

def hold_baseline(self, stage, trajectory):
    if stage != 'approach':
        raise SceneInvalid('Initial hold diagnostic must intercept first approach')
    primary = self.primary
    _, _, initial_q, _ = self._read()
    target = primary.actors['bi']
    agent = primary.env.unwrapped.agent
    others = [(oid, actor, 'object') for oid, actor in primary.actors.items() if oid != 'bi']
    others.append(('__maniskill_workspace_table__', primary.env.unwrapped.table_scene.table, 'table'))
    others.extend((name, link, 'robot_link') for name, link in agent.robot.links_map.items())
    rows = []
    for index in range(200):
        primary.stop.check()
        action = self.demo.compose_action(initial_q, self._gripper_value)
        self.demo.step_and_render(action, tag='diagnostic_initial_hold')
        contacts = []
        for name, other, kind in others:
            force = _array(agent.scene.get_pairwise_contact_forces(target, other)).reshape(-1)
            if force.shape != (3,):
                raise SceneInvalid('Invalid native target force')
            if np.any(force != 0.):
                contacts.append(dict(other_id=name, other_kind=kind, force_on_target_n=force.tolist()))
        linear = _array(target.linear_velocity).reshape(-1)
        angular = _array(target.angular_velocity).reshape(-1)
        if linear.shape != (3,) or angular.shape != (3,):
            raise SceneInvalid('Invalid native target velocity')
        _, _, actual_q, _ = self._read()
        rows.append(dict(tick=index+1, linear_velocity_world_m_s=linear.tolist(),
            angular_velocity_world_rad_s=angular.tolist(),
            idle=bool(np.linalg.norm(linear) <= .001 and np.linalg.norm(angular) <= .005),
            initial_arm_target_rad=initial_q.tolist(), measured_arm_positions_rad=actual_q.tolist(),
            target_position_world_m=_array(target.pose.p).reshape(-1).tolist(), contacts=contacts))
    print('SWM_INITIAL_HOLD_BASELINE ' + json.dumps(dict(
        control_ticks=200, source='original_primary_controller_and_native_readback',
        grasp_trajectory_executed=False, q_or_object_setters_used=False,
        scene_or_physics_parameters_changed=False, hardware_authorized=False,
        coverage='control_boundaries_only', rows=rows)), flush=True)
    raise SceneInvalid('Initial hold diagnostic complete; atomic trajectory intentionally not executed')

NativePrimaryExecutor.execute_trajectory = hold_baseline
try:
    raise SystemExit(main())
finally:
    NativePrimaryExecutor.execute_trajectory = original_execute
    OBJECT_SPECS['bi'] = original_spec
