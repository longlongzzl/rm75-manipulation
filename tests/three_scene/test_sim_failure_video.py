import json
from types import SimpleNamespace as NS

import pytest

from rm75_app.workcell import sim_failure_video as video


@pytest.mark.parametrize('spec,profile', [
    ({'task': 'pickplace', 'mode': 'real'}, {'pickplace': {'fixed_scene': 'frozen'}}),
    ({'task': 'pusht', 'mode': 'sim'}, {'pusht': {'fixed_scene': 'frozen'}}),
    ({'task': 'pickplace', 'mode': 'sim'}, {'pickplace': {}}),
])
def test_recording_refuses_real_or_live_input(spec, profile):
    with pytest.raises(ValueError, match='frozen native SIM'):
        video.require_sim(spec, profile)


def test_empty_recording_is_not_fabricated_success(tmp_path):
    direct = NS(_current_source_object_name=lambda options: options.source)
    recorder = video.Recorder(direct, tmp_path / 'video', requested_source='tennis')
    assert not recorder.eligible(NS(source='tennis', execute_real=True))
    assert not recorder.eligible(NS(source='tennis', _planning_prefetch_capture_only=True))
    assert not recorder.eligible(NS(source='bi'))
    recorder.close()
    recorder.close()
    result = json.loads((tmp_path / 'video/recording.json').read_text())
    assert result['frames'] == 0 and result['render_complete'] is False
    assert result['live_camera_used'] is False and result['physical_success'] is None


def test_wrappers_keep_original_results_failures_and_do_not_execute_rejected_paths(tmp_path):
    records, calls = [], []
    direct = NS(_current_source_object_name=lambda options: options.source)
    class FakeRecorder(video.Recorder):
        def capture(self, demo, options, label, repeat=1):
            records.append(label)
        def close(self):
            pass
    options = NS(source='tennis', execute_real=False)
    def segment(*args, **kwargs):
        calls.append('rejected_query')
        return None
    direct._plan_constrained_linear_segment = segment
    direct._render_dry_run_motion_frame = lambda *args: calls.append('original_frame')
    direct._play_dry_run_motion_window = lambda demo, bridge, label, start, path, options: direct._render_dry_run_motion_frame(demo, bridge, options)
    sentinel = object()
    def episode(demo, bridge, real_exec, options):
        direct._play_dry_run_motion_window(demo, bridge, 'original_accepted_stage', [0], [[1]], options)
        assert direct._plan_constrained_linear_segment(label='post_place_clearance') is None
        return sentinel
    direct.run_targeted_place_episode_curobo_direct = episode
    original = dict(vars(direct))
    close = video.install(direct, tmp_path / 'video', requested_source='tennis', recorder_factory=FakeRecorder)
    assert direct.run_targeted_place_episode_curobo_direct(None, None, None, options) is sentinel
    assert calls == ['original_frame', 'rejected_query']
    assert any('REJECTED (not executed)' in label for label in records)
    close()
    assert vars(direct) == original


@pytest.mark.parametrize('accepted', [True, False])
def test_jimu_replays_only_original_accepted_path_and_restores_endpoint(tmp_path, accepted):
    import numpy as np
    calls=[]
    direct=NS(_current_source_object_name=lambda options:options.source)
    demo=NS(q=np.zeros(7))
    demo.current_arm_qpos=lambda:demo.q.copy()
    def sync(demo,q):
        demo.q=np.array(q)
    direct.targeted=NS(base=NS(sync_demo_arm_qpos=sync))
    direct._plan_constrained_linear_segment=lambda *args,**kwargs:None
    direct._render_dry_run_motion_frame=lambda *args:None
    def play(demo,bridge,label,start,path,options):
        calls.append((label,start.copy(),np.asarray(path)))
        demo.q=np.asarray(path[-1])
    direct._play_dry_run_motion_window=play
    result=(accepted,np.ones(7) if accepted else None)
    def execute(demo,bridge_mod,real_exec,label,pose,q_path,gripper_pos,args):
        if accepted:demo.q=np.ones(7)
        return result
    portable=NS(_jimu_execute_pose_path_stage_base=execute,
                _jimu_start_collision_diagnosis=lambda:dict(valid=False))
    options=NS(source='right_wall')
    def episode(demo,bridge,real_exec,options):
        return portable._jimu_execute_pose_path_stage_base(demo,bridge,real_exec,'lift',None,
            [[0.]*7,[1.]*7],.5,options)
    direct.run_targeted_place_episode_curobo_direct=episode
    class FakeRecorder(video.Recorder):
        def capture(self,*args,**kwargs):pass
        def close(self):self.closed=True
    close=video.install(direct,tmp_path/'video',portable=portable,recorder_factory=FakeRecorder)
    assert direct.run_targeted_place_episode_curobo_direct(demo,None,None,options) is result
    assert len(calls)==int(accepted)
    assert np.array_equal(demo.q,np.ones(7) if accepted else np.zeros(7))
    close()


def test_rejected_static_view_restores_sim_state_without_solver_or_physics(tmp_path):
    import threading
    import numpy as np
    state={'q':np.zeros((1,9)), 'v':np.ones((1,9))}
    robot=NS(get_qpos=lambda:state['q'].copy(),get_qvel=lambda:state['v'].copy(),
        set_qpos=lambda q:state.update(q=np.asarray(q).copy()),
        set_qvel=lambda v:state.update(v=np.asarray(v).copy()))
    demo=NS(robot=robot,arm_indices=np.arange(7))
    direct=NS(_CUROBO_GPU_LOCK=threading.RLock(),_current_source_object_name=lambda args:'front_wall')
    recorder=video.Recorder(direct,tmp_path/'video')
    recorder.context.set((demo,NS(execute_real=False)))
    observations=[]
    def capture(*args,**kwargs):
        observations.append((state['q'].copy(),args[2]))
    recorder.capture=capture
    recorder.inspect_rejected_q([.1]*7)
    assert np.allclose(observations[0][0][0,:7],.1)
    assert 'STATIC' in observations[0][1] and 'NOT execution' in observations[0][1]
    assert np.array_equal(state['q'],np.zeros((1,9)))
    assert np.array_equal(state['v'],np.ones((1,9)))
    with pytest.raises(ValueError,match='finite'):
        recorder.inspect_rejected_q([float('nan')]*7)
