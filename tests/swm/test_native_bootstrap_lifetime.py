"""Constructor ownership fixtures only; no native world or asset writes occur."""
from contextlib import ExitStack
from types import SimpleNamespace
import uuid

import pytest

from rm75_app.swm.native_bootstrap import initialize_frozen_primary
from rm75_app.swm.scene import SceneInvalid
from rm75_app.workcell.events import Cancelled, StopToken


def setup_case(tmp_path):
    closed, emitted, created = [], [], []
    env = SimpleNamespace(close=lambda: closed.append('env'))

    def make(*args, **kwargs):
        created.append(env)
        return env

    gym = SimpleNamespace(make=make)
    args = SimpleNamespace(execute_real=False, skip_foundationpose=True,
        foundationpose_refine_after_render=False, bridge_script_path='fixture_bridge',
        pick_script_path='fixture_setup', srdf_path=None)
    demo = SimpleNamespace(foundationpose_runtime=None, _freeze_active_object_before_grasp=False)

    def create_demo(args, bridge, planner, scene_capture_cache):
        return gym.make(), demo

    base = SimpleNamespace(gym=gym,
        make_cycle_args=lambda args, source: (SimpleNamespace(**vars(args)), None),
        load_module_from_path=lambda *args: SimpleNamespace(resolve_planning_artifact_paths=lambda *a: ('UNSAFE', 'UNSAFE')), create_demo=create_demo)
    direct = SimpleNamespace(targeted=SimpleNamespace(base=base),
        _single_scene_build_actor_registry=lambda *args: {
            'bi': {'actor': object()}, 'holder': {'actor': object()}})
    contract = dict(source='bi', names=('bi', 'holder'), sha256='fixture_input')
    events = SimpleNamespace(emit=lambda kind, **data: emitted.append((kind, data)))
    return SimpleNamespace(direct=direct, base=base, args=args, contract=contract,
        stop=StopToken(), events=events, env=env, demo=demo, original_make=make,
        closed=closed, emitted=emitted, created=created, artifacts=tmp_path / uuid.uuid4().hex)


def initialize(case, resources):
    return initialize_frozen_primary(resources, case.direct, case.args, case.contract,
        stop=case.stop, events=case.events, artifact_directory=case.artifacts)


@pytest.mark.parametrize('changes', [
    {'execute_real': True}, {'skip_foundationpose': False},
    {'foundationpose_refine_after_render': True},
])
def test_offline_gate_rejects_before_any_environment_creation(changes, tmp_path):
    case = setup_case(tmp_path)
    for key, value in changes.items():
        setattr(case.args, key, value)
    with ExitStack() as resources:
        with pytest.raises(PermissionError, match='offline simulation only'):
            initialize(case, resources)
    assert case.created == [] and case.closed == []


@pytest.mark.parametrize('error', [RuntimeError('reset failed'), Cancelled('cancelled'), KeyboardInterrupt()])
def test_environment_is_owned_before_constructor_can_fail(error, tmp_path):
    case = setup_case(tmp_path)

    def fail(*args, **kwargs):
        case.base.gym.make()
        raise error

    case.base.create_demo = fail
    with pytest.raises(type(error)):
        with ExitStack() as resources:
            initialize(case, resources)
    assert case.closed == ['env']
    assert case.base.gym.make is case.original_make
    assert [kind for kind, _ in case.emitted] == ['swm_primary_acquired', 'swm_primary_closed']


def test_complete_initialization_closes_once_and_disallows_late_reads(tmp_path):
    case = setup_case(tmp_path)
    with ExitStack() as resources:
        world = initialize(case, resources)
        assert set(world.actors) == {'bi', 'holder'}
        assert world.args.selected_obstacle_object_names == ['holder']
        assert world.args.fixed_scene_strict is True
        assert world.args.freeze_active_object_before_grasp is False
        assert world.args.next_cycle_plan_prefetch is False
        assert not world.closed and case.closed == []
    assert world.closed and case.closed == ['env']
    world.close()
    assert case.closed == ['env']
    with pytest.raises(SceneInvalid, match='closed'):
        world.read_state()


def test_incomplete_native_registry_fails_and_closes_environment(tmp_path):
    case = setup_case(tmp_path)
    case.direct._single_scene_build_actor_registry = lambda *args: {'bi': {'actor': object()}}
    with pytest.raises(SceneInvalid, match='omitted'):
        with ExitStack() as resources:
            initialize(case, resources)
    assert case.closed == ['env']


def test_missing_actor_handle_fails_even_when_names_match(tmp_path):
    case = setup_case(tmp_path)
    case.direct._single_scene_build_actor_registry = lambda *args: {
        'bi': {'actor': object()}, 'holder': {'actor': None}}
    with pytest.raises(SceneInvalid, match='omitted'):
        with ExitStack() as resources:
            initialize(case, resources)
    assert case.closed == ['env']


def test_live_runtime_or_persistent_freeze_never_accepted(tmp_path):
    for key, value in [('foundationpose_runtime', object()), ('_freeze_active_object_before_grasp', True)]:
        case = setup_case(tmp_path)
        setattr(case.demo, key, value)
        with pytest.raises(SceneInvalid):
            with ExitStack() as resources:
                initialize(case, resources)
        assert case.closed == ['env']


def test_second_world_creation_is_rejected_and_first_closed(tmp_path):
    case = setup_case(tmp_path)

    def double_create(*args, **kwargs):
        case.base.gym.make()
        case.base.gym.make()
        raise AssertionError('second creation should have been blocked')

    case.base.create_demo = double_create
    with pytest.raises(SceneInvalid, match='extra/background'):
        with ExitStack() as resources:
            initialize(case, resources)
    assert len(case.created) == 1 and case.closed == ['env']


def test_requested_stop_does_not_acquire_an_environment(tmp_path):
    case = setup_case(tmp_path)
    case.stop.request()
    with pytest.raises(Cancelled):
        with ExitStack() as resources:
            initialize(case, resources)
    assert case.created == [] and case.closed == []


def test_artifact_paths_are_private_and_source_srdf_is_not_modified(tmp_path):
    case = setup_case(tmp_path)
    original_srdf = tmp_path / 'original.srdf'
    original_srdf.write_text('original read-only input')
    case.args.srdf_path = str(original_srdf)
    paths = []
    def create(args, bridge, planner, **kwargs):
        paths.extend(planner.resolve_planning_artifact_paths('/asset/robot.urdf', args))
        assert args.srdf_path == str(case.artifacts / 'robot.permissive.srdf')
        return case.base.gym.make(), case.demo
    case.base.create_demo = create
    with ExitStack() as resources:
        initialize(case, resources)
    assert paths == [str(case.artifacts / 'robot.planning.tiny.urdf'),
                     str(case.artifacts / 'robot.permissive.srdf')]
    assert original_srdf.read_text() == 'original read-only input'
    assert (case.artifacts / 'robot.permissive.srdf').read_text() == original_srdf.read_text()


def test_existing_artifact_directory_is_never_reused(tmp_path):
    case = setup_case(tmp_path)
    case.artifacts.mkdir()
    marker = case.artifacts / 'robot.planning.tiny.urdf'
    marker.write_text('existing user file')
    with ExitStack() as resources:
        with pytest.raises(FileExistsError):
            initialize(case, resources)
    assert marker.read_text() == 'existing user file'
    assert case.created == []
