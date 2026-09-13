"""Read articulation sleep and complete feedback during both-group write elision."""
import json
from pathlib import Path
import runpy
from rm75_app.swm.native_execution import NativePrimaryExecutor
from rm75_app.swm.native_bootstrap import _array
from rm75_app.swm.scene import SceneInvalid

original_read = NativePrimaryExecutor._read
count = 0

def read(self):
    global count
    result = original_read(self)
    count += 1
    if count > 1600:
        raise SceneInvalid('Bounded articulation readback diagnostic exhausted')
    owned = self.primary.env.unwrapped.agent.robot._objs
    if len(owned) != 1:
        raise SceneInvalid('One original native articulation required')
    articulation = owned[0]
    native = articulation.root
    if native is None or not native.is_root or native.articulation != articulation:
        raise SceneInvalid('Original articulation root identity mismatch')
    sleeping = getattr(native, 'sleeping', None)
    if sleeping is None:
        sleeping = getattr(native, 'is_sleeping', None)
    sleeping = sleeping() if callable(sleeping) else sleeping
    if type(sleeping) is not bool:
        raise SceneInvalid('Original articulation sleeping readback unavailable')
    links = articulation.get_links()
    if len(links) != 20 or len({link.name for link in links}) != 20 or any(link.articulation != articulation for link in links):
        raise SceneInvalid('Complete original articulation link identity required')
    link_state = [dict(name=link.name, sleeping=bool(link.sleeping),
        linear_velocity_world_m_s=_array(link.linear_velocity).reshape(-1).tolist(),
        angular_velocity_world_rad_s=_array(link.angular_velocity).reshape(-1).tolist()) for link in links]
    raw, _, _, _ = result
    print('SWM_ARTICULATION_SLEEP ' + json.dumps(dict(sample=count,
        link_state=link_state, sleeping=sleeping, joint_names=raw['joint_names'],
        positions=raw['positions'], velocities=raw['velocities'],
        primary_sequence=raw['sequence'], captured_at=raw['capture_started_at'],
        stage_previous_feedback=None if self.last_settle_evidence is None else self.last_settle_evidence['stage'],
        native_velocities_not_replaced=True)), flush=True)
    return result

NativePrimaryExecutor._read = read
try:
    runpy.run_path(str(Path('benchmarks/release/evidence/pen_closure/worker_51/probe.py').resolve()), run_name='__main__')
finally:
    NativePrimaryExecutor._read = original_read
