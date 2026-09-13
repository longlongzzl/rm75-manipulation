"""Observe target motion/contact at every original primary control boundary."""
import json
import time
from dataclasses import replace
import numpy as np
from rm75_app.assets.object_specs import OBJECT_SPECS
from rm75_app.swm.native_bootstrap import _array
from rm75_app.swm.native_execution import NativePrimaryExecutor
from rm75_app.swm.scene import SceneInvalid
from rm75_app.workcell.worker import main

original_spec = OBJECT_SPECS['bi']
original_hook = NativePrimaryExecutor._after_control_step
if original_spec.grasp_z_offset != .012777:
    raise SceneInvalid('Frozen baseline pen depth changed')
OBJECT_SPECS['bi'] = replace(original_spec, grasp_z_offset=.016777)
samples = 0

def control_contacts(self, stage):
    global samples
    original_hook(self, stage)
    samples += 1
    if samples > 1600:
        raise SceneInvalid('Bounded primary contact diagnostic exhausted')
    primary = self.primary
    target = primary.actors['bi']
    agent = primary.env.unwrapped.agent
    others = [(oid, actor, 'object') for oid, actor in primary.actors.items() if oid != 'bi']
    others.append(('__maniskill_workspace_table__', primary.env.unwrapped.table_scene.table, 'table'))
    others.extend((name, link, 'robot_link') for name, link in agent.robot.links_map.items())
    contacts = []
    for name, other, kind in others:
        primary.stop.check()
        force = _array(agent.scene.get_pairwise_contact_forces(target, other)).reshape(-1)
        if force.shape != (3,):
            raise SceneInvalid('Invalid native target contact force shape')
        if np.any(force != 0.):
            contacts.append(dict(other_id=name, other_kind=kind, force_on_target_n=force.tolist()))
    linear = _array(target.linear_velocity).reshape(-1)
    angular = _array(target.angular_velocity).reshape(-1)
    if linear.shape != (3,) or angular.shape != (3,):
        raise SceneInvalid('Invalid native target velocity shape')
    print('SWM_TARGET_CONTROL_CONTACTS ' + json.dumps(dict(
        sample=samples, stage=stage, captured_at=time.monotonic(),
        linear_velocity_world_m_s=linear.tolist(), angular_velocity_world_rad_s=angular.tolist(),
        target_position_world_m=_array(target.pose.p).reshape(-1).tolist(),
        contacts=contacts, source='native_primary_actor_and_pairwise_force_readback',
        coverage='every_original_control_boundary_not_every_physics_substep',
        observation_only=True, checkpoint_accepted=False)), flush=True)

NativePrimaryExecutor._after_control_step = control_contacts
print('SWM_DEPTH_HYPOTHESIS ' + json.dumps(dict(baseline_depth_m=.012777,
    hypothesis_depth_m=.016777, production_default_changed=False,
    collision_thresholds_changed=False, settle_budget_changed=False,
    diagnostic_sample_limit=1600, original_guard_called_first=True,
    hardware_authorized=False)), flush=True)
try:
    raise SystemExit(main())
finally:
    NativePrimaryExecutor._after_control_step = original_hook
    OBJECT_SPECS['bi'] = original_spec
