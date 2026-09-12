"""Primary execution contract fixtures, not simulation qualification."""
from types import SimpleNamespace
import time
import numpy as np
import pytest
from rm75_app.swm.native_bootstrap import FrozenPrimaryWorld
from rm75_app.swm.native_execution import NativePrimaryExecutor
from rm75_app.swm.scene import SceneInvalid
from rm75_app.planning.contracts import JointTrajectory
from rm75_app.workcell.events import StopToken, Cancelled


def make_primary(*,moving=False,cancel_after=None):
    stop=StopToken(); state=dict(q=np.zeros(13),steps=0,seq=0)
    class Demo:
        def compose_action(self,q,gripper): return np.asarray(q).copy()
        def step_and_render(self,action,tag):
            state['steps']+=1; state['q'][:7]=action
            if state['steps']==cancel_after: stop.request()
    primary=FrozenPrimaryWorld(SimpleNamespace(unwrapped=SimpleNamespace(control_freq=50)),
        Demo(),SimpleNamespace(),{}, {},stop)
    names=[f'joint_{i}' for i in range(1,8)]+[f'jaw_{i}' for i in range(6)]
    def read():
        state['seq']+=1
        return dict(domain='physics',source='native_actor_and_joint_readback',sequence=state['seq'],
            capture_started_at=time.monotonic(),joint_names=names,positions=state['q'].tolist(),
            velocities=[.01 if moving else 0.]*13)
    primary.read_state=read
    return primary,state


def path():
    return JointTrajectory(tuple(f'joint_{i}' for i in range(1,8)),np.array([[0.]*7,[.1]*7]),dt=.04)


def test_timed_path_settles_on_actual_feedback_not_command_cache():
    primary,state=make_primary(); executor=NativePrimaryExecutor(primary,max_settle_steps=5)
    executor.execute_trajectory('approach',path())
    assert state['steps']==5
    assert executor.last_settle_evidence['stable_steps']==3
    state['q'][0]=.09
    assert executor.feedback()['positions'][0]==pytest.approx(.09)


def test_nonidle_feedback_rejects_after_bounded_real_steps():
    primary,state=make_primary(moving=True); executor=NativePrimaryExecutor(primary,max_settle_steps=3)
    with pytest.raises(SceneInvalid,match='settle budget'): executor.execute_trajectory('approach',path())
    assert state['steps']==5
    assert not executor.feedback()['idle']


def test_stop_is_checked_inside_path_not_only_between_stages():
    primary,state=make_primary(cancel_after=1); executor=NativePrimaryExecutor(primary)
    with pytest.raises(Cancelled): executor.execute_trajectory('approach',path())
    assert state['steps']==1


def test_gripper_without_preceding_endpoint_never_commands():
    primary,state=make_primary(); executor=NativePrimaryExecutor(primary)
    with pytest.raises(SceneInvalid,match='preceding audited'): executor.set_gripper(True)
    assert state['steps']==0


def contact_primary(*, contact_object='table', contact_link='left', force=(0., 0., 1.), after=1):
    primary, state = make_primary()
    links = {name: SimpleNamespace(name=name) for name in ('left', 'right', 'wrist')}
    actors = {name: object() for name in ('bi', 'neighbor')}
    table = object()
    objects = dict(actors, table=table)
    def query(link, actor):
        if state['steps'] >= after and link.name == contact_link and actor is objects[contact_object]:
            return np.asarray(force)
        return np.zeros(3)
    primary.actors = actors
    primary.env.unwrapped.agent = SimpleNamespace(robot=SimpleNamespace(links_map=links),
        finger1_link=links['left'], finger2_link=links['right'],
        scene=SimpleNamespace(get_pairwise_contact_forces=query))
    primary.env.unwrapped.table_scene = SimpleNamespace(table=table)
    executor = NativePrimaryExecutor(primary)
    executor.closure_target = 'bi'
    executor._last_commanded_target = np.zeros(7)
    return executor, state


@pytest.mark.parametrize('object_id,link', [('table','left'), ('neighbor','right'), ('bi','wrist')])
def test_closure_forbidden_contact_stops_before_second_command(object_id, link):
    executor, state = contact_primary(contact_object=object_id, contact_link=link)
    with pytest.raises(SceneInvalid, match='forbidden contact'):
        executor.set_gripper(True)
    assert state['steps'] == 1


def test_contact_guard_remains_active_during_closed_settling():
    executor, state = contact_primary(after=21)
    with pytest.raises(SceneInvalid, match='forbidden contact'):
        executor.set_gripper(True)
    assert state['steps'] == 21


def test_original_target_finger_contact_is_not_holding_verification():
    executor, state = contact_primary(contact_object='bi')
    events = []
    executor.emit = lambda **row: events.append(row)
    executor.set_gripper(True)
    assert state['steps'] == 23
    assert all(not row['skill_verified'] for row in events if row['kind']=='swm_primary_closure_contacts')


@pytest.mark.parametrize('force', [(float('nan'),0.,0.), (0.,0.), (1e-20,0.,0.)])
def test_invalid_or_arbitrarily_small_forbidden_force_is_not_ignored(force):
    executor, state = contact_primary(force=force)
    with pytest.raises(SceneInvalid): executor.set_gripper(True)
    assert state['steps'] == 1


def test_unbound_closure_target_rejects_before_motion():
    executor, state = contact_primary()
    executor.closure_target = None
    with pytest.raises(SceneInvalid, match='bound target'): executor.set_gripper(True)
    assert state['steps'] == 0
