"""Hypothesis: omit only exact native-equal target writes; retain controller updates."""
import json
from pathlib import Path
import runpy
import numpy as np
from rm75_app.swm.native_bootstrap import _array
from rm75_app.swm.native_execution import NativePrimaryExecutor
from rm75_app.swm.native_robot import read_primary_drive_state
from rm75_app.swm.scene import SceneInvalid

original_init = NativePrimaryExecutor.__init__
restorations = []
counts = {}

def install(self, primary, *args, **kwargs):
    original_init(self, primary, *args, **kwargs)
    controllers = primary.env.unwrapped.agent.controller.controllers
    if set(controllers) != {'arm', 'gripper', 'gripper_passive'}:
        raise SceneInvalid('Original arm/gripper controller identity required')
    from mani_skill.agents.controllers.passive_controller import PassiveController
    if not isinstance(controllers['gripper_passive'], PassiveController):
        raise SceneInvalid('Original passive gripper controller required')
    for name in ('arm', 'gripper'):
        controller = controllers[name]
        if controller.config.interpolate or controller.config.use_delta:
            raise SceneInvalid('Diagnostic requires original absolute noninterpolating control')
        names = [joint.name for joint in controller.joints]
        original = controller.set_drive_targets
        counts[name] = dict(skipped_exact_native_equal=0, original_writes=0)
        def targets(value, *, _names=names, _original=original, _name=name):
            before = read_primary_drive_state(primary)
            incoming = _array(value).reshape(-1)
            if len(set(_names)) != len(_names) or not set(_names) <= set(before['joint_names']):
                raise SceneInvalid('Controller/native joint identity mismatch')
            if incoming.shape != (len(_names),):
                raise SceneInvalid('Controller target shape mismatch')
            actual = np.asarray([before['position_targets'][before['joint_names'].index(j)] for j in _names])
            if np.array_equal(incoming, actual):
                counts[_name]['skipped_exact_native_equal'] += 1
                if read_primary_drive_state(primary) != before:
                    raise SceneInvalid('Native drive state changed during identical-write diagnostic')
                return
            counts[_name]['original_writes'] += 1
            return _original(value)
        controller.set_drive_targets = targets
        restorations.append((controller, original))

NativePrimaryExecutor.__init__ = install
print('SWM_IDENTICAL_DRIVE_HYPOTHESIS ' + json.dumps(dict(
    comparison='exact_requested_vs_native_readback_no_tolerance',
    controller_set_action_and_cache_updates_retained=True,
    contact_offsets_or_physical_parameters_changed=False,
    production_controller_changed=False, hardware_authorized=False)), flush=True)
try:
    runpy.run_path(str(Path('benchmarks/release/evidence/pen_closure/worker_49/probe.py').resolve()), run_name='__main__')
finally:
    for controller, original in reversed(restorations):
        controller.set_drive_targets = original
    NativePrimaryExecutor.__init__ = original_init
    print('SWM_IDENTICAL_DRIVE_COUNTS ' + json.dumps(counts), flush=True)
