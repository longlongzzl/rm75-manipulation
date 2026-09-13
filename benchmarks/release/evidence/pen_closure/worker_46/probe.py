"""Bounded read-only target contact diagnosis during the original formal worker."""
import json
from dataclasses import replace
import numpy as np
from rm75_app.assets.object_specs import OBJECT_SPECS
from rm75_app.swm.native_bootstrap import _array
from rm75_app.swm.native_capture import NativePrimaryCapture
from rm75_app.swm.scene import SceneInvalid
from rm75_app.workcell.worker import main

original_spec = OBJECT_SPECS['bi']
original_read = NativePrimaryCapture.read_settle_state
if original_spec.grasp_z_offset != .012777:
    raise SceneInvalid('Frozen original pen depth changed')
OBJECT_SPECS['bi'] = replace(original_spec, grasp_z_offset=.016777)
samples = 0

def read_contacts(self):
    global samples
    observed = original_read(self)
    samples += 1
    if samples > 256:
        raise SceneInvalid('Bounded target contact diagnostic exhausted')
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
            raise SceneInvalid('Target contact diagnostic has invalid native force shape')
        if np.any(force != 0.):
            contacts.append(dict(other_id=name, other_kind=kind, force_on_target_n=force.tolist()))
    print('SWM_TARGET_SETTLE_CONTACTS ' + json.dumps(dict(
        sample=samples, objects=observed['objects'], idle=observed['idle'],
        capture_started_at=observed['capture_started_at'], contacts=contacts,
        source='native_pairwise_contact_force_readback',
        coverage='control_boundary_resultant_forces_not_substep_or_sweep',
        observation_only=True, checkpoint_accepted=False)), flush=True)
    return observed

NativePrimaryCapture.read_settle_state = read_contacts
print('SWM_DEPTH_HYPOTHESIS ' + json.dumps(dict(baseline_depth_m=.012777,
    hypothesis_depth_m=.016777, production_default_changed=False,
    collision_thresholds_changed=False, settle_budget_changed=False,
    diagnostic_sample_limit=256, hardware_authorized=False)), flush=True)
try:
    raise SystemExit(main())
finally:
    NativePrimaryCapture.read_settle_state = original_read
    OBJECT_SPECS['bi'] = original_spec
