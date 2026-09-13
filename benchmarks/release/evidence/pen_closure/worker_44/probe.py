"""Single bounded planning-depth hypothesis through the unchanged formal worker."""
import json
from dataclasses import replace
from rm75_app.assets.object_specs import OBJECT_SPECS
from rm75_app.workcell.worker import main

original = OBJECT_SPECS['bi']
if original.grasp_z_offset != .012777:
    raise RuntimeError('Frozen baseline pen depth changed; do not apply hypothesis')
OBJECT_SPECS['bi'] = replace(original, grasp_z_offset=.016777)
print('SWM_DEPTH_HYPOTHESIS ' + json.dumps(dict(
    baseline_depth_m=original.grasp_z_offset, hypothesis_depth_m=.016777,
    delta_along_negative_approach_m=.004, pure_world_z_shift=False,
    production_default_changed=False, scene_changed=False,
    collision_thresholds_changed=False, motion_budget_changed=False,
    formal_worker_main=True, hardware_authorized=False)), flush=True)
try:
    raise SystemExit(main())
finally:
    OBJECT_SPECS['bi'] = original
