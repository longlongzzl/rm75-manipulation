"""Owned primary PhysX substep contacts and sleep state; no control changes."""
import json
import time
from dataclasses import replace
import numpy as np
from rm75_app.assets.object_specs import OBJECT_SPECS
from rm75_app.swm.native_bootstrap import _array
from rm75_app.swm.native_body_mirror import native_body
from rm75_app.swm.native_execution import NativePrimaryExecutor
from rm75_app.swm.scene import SceneInvalid
from rm75_app.workcell.worker import main

original_init = NativePrimaryExecutor.__init__
original_spec = OBJECT_SPECS['bi']
restorations = []
if original_spec.grasp_z_offset != .012777:
    raise SceneInvalid('Frozen baseline pen depth changed')
OBJECT_SPECS['bi'] = replace(original_spec, grasp_z_offset=.016777)

def install(self, primary, *args, **kwargs):
    original_init(self, primary, *args, **kwargs)
    scene = primary.env.unwrapped.scene
    if type(scene.px).__name__ != 'PhysxCpuSystem':
        raise SceneInvalid('Diagnostic requires original CPU primary physics')
    target = primary.actors['bi']
    body = native_body(target)
    target_entity = body.entity
    original_step = scene.step
    original_control = primary.demo.step_and_render
    state = dict(control=0, substep=0, stage=None, total=0)
    robot_entities = {link.name for link in primary.env.unwrapped.agent.robot.links_map.values()}

    def observed():
        sleeping = getattr(body, 'is_sleeping', None)
        if sleeping is None:
            sleeping = getattr(body, 'sleeping', None)
        sleeping = sleeping() if callable(sleeping) else sleeping
        if type(sleeping) is not bool:
            raise SceneInvalid('Native target sleeping readback unavailable')
        return dict(sleeping=sleeping,
            linear_velocity_world_m_s=_array(target.linear_velocity).reshape(-1).tolist(),
            angular_velocity_world_rad_s=_array(target.angular_velocity).reshape(-1).tolist(),
            target_position_world_m=_array(target.pose.p).reshape(-1).tolist())

    def emit(kind, **row):
        print('SWM_TARGET_SUBSTEP ' + json.dumps(dict(kind=kind,
            control=state['control'], substep=state['substep'], stage=state['stage'],
            captured_at=time.monotonic(), **row)), flush=True)

    def step():
        primary.stop.check()
        state['total'] += 1
        state['substep'] += 1
        if state['total'] > 8000:
            raise SceneInvalid('Native substep diagnostic budget exhausted')
        before = observed()
        original_step()
        after = observed()
        contacts = []
        for contact in scene.px.get_contacts():
            if not any(item.entity == target_entity for item in contact.bodies):
                continue
            names = [item.entity.name for item in contact.bodies]
            points = []
            for point in contact.points:
                separation = float(point.separation)
                if not np.isfinite(separation):
                    raise SceneInvalid('Nonfinite native separation')
                points.append(dict(separation_m=separation,
                    position_world_m=_array(point.position).reshape(-1).tolist(),
                    normal_world=_array(point.normal).reshape(-1).tolist(),
                    impulse_ns=_array(point.impulse).reshape(-1).tolist()))
            contacts.append(dict(body_names=names, robot_contact=any(name in robot_entities for name in names),
                points=points))
        emit('physics_substep', before=before, after=after, contacts=contacts,
            physics_timestep_s=float(scene.px.timestep))

    def control(*args, **kwargs):
        state['control'] += 1
        state['substep'] = 0
        state['stage'] = kwargs.get('tag', 'unspecified')
        emit('control_begin', measured=observed())
        result = original_control(*args, **kwargs)
        emit('control_end', measured=observed())
        return result

    scene.step = step
    primary.demo.step_and_render = control
    restorations.append((scene, original_step, primary.demo, original_control))
    emit('installed', observation_only=True, scene_or_physics_parameters_changed=False,
        production_depth_changed=False, hypothesis_depth_m=.016777,
        robot_entity_names=sorted(robot_entities), target_entity_name=target_entity.name,
        initial=observed())

NativePrimaryExecutor.__init__ = install
try:
    raise SystemExit(main())
finally:
    for scene, step, demo, control in reversed(restorations):
        scene.step = step
        demo.step_and_render = control
    NativePrimaryExecutor.__init__ = original_init
    OBJECT_SPECS['bi'] = original_spec
