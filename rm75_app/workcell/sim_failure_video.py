"""Opt-in recording of the ORIGINAL native SIM motion-window frames.

No camera device, robot SDK, solver, candidate generation or collision changes.
Rejected configurations are not animated as if they had been executed.
"""
import contextvars
import functools
import inspect
from pathlib import Path
import time

from .io import atomic_json


def require_sim(spec, profile):
    if (spec.get('mode') != 'sim' or spec.get('task') not in ('pickplace', 'magnetic')
            or not profile.get(spec['task'], {}).get('fixed_scene')):
        raise ValueError('Video recording requires frozen native SIM, never live devices')


class Recorder:
    def __init__(self, direct, directory, requested_source=None, fps=10, max_frames=12000):
        self.direct = direct
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.requested_source = requested_source
        self.fps, self.max_frames = fps, max_frames
        self.context = contextvars.ContextVar('native_sim_video', default=None)
        self.stage = contextvars.ContextVar('native_sim_video_stage', default='scene')
        self.writer = None
        self.frames = 0
        self.records, self.errors = [], []
        self.last_image = None
        self.closed = False

    def eligible(self, options):
        return (not getattr(options, 'execute_real', False)
                and not getattr(options, '_planning_prefetch_capture_only', False)
                and (self.requested_source is None or
                     self.direct._current_source_object_name(options) == self.requested_source))

    def capture(self, demo, options, label, repeat=1):
        if self.closed or not self.eligible(options) or self.frames >= self.max_frames:
            return
        try:
            import imageio.v2 as imageio
            import numpy as np
            from PIL import Image, ImageDraw
            from mani_skill.utils import sapien_utils
            with self.direct._CUROBO_GPU_LOCK:
                base = self.direct.targeted.base
                overview = base.capture_failure_render_image(demo, options)
                if overview is None:
                    raise RuntimeError('Native SIM did not return a render image')
                position = np.asarray(demo.get_obj_pose()[0], dtype=float).reshape(-1)[:3]
                camera = sapien_utils.look_at(position + np.array([.35, -.35, .30]), position)
                detail = base.capture_failure_render_image(demo, options, camera_pose=camera)
                if detail is None:
                    raise RuntimeError('Native SIM detail camera unavailable')
            canvas = Image.new('RGB', (1024, 600), (17, 22, 30))
            canvas.paste(Image.fromarray(overview).convert('RGB').resize((512, 512)), (0, 76))
            canvas.paste(Image.fromarray(detail).convert('RGB').resize((512, 512)), (512, 76))
            draw = ImageDraw.Draw(canvas)
            source = self.direct._current_source_object_name(options)
            draw.text((12, 7), 'NATIVE SIM ONLY | original path playback | NO ROBOT / NO LIVE CAMERA', fill='white')
            draw.text((12, 26), f'{source} | {label}'[:155], fill=(255, 210, 110))
            draw.text((12, 45), 'Rejected IK/path is NOT executed. Planning waits omitted; 10 fps is not real-time qualification.', fill='white')
            draw.text((12, 61), 'Overview', fill='white')
            draw.text((524, 61), 'Active-object close-up (render camera only)', fill='white')
            image = np.asarray(canvas)
            if self.writer is None:
                self.writer = imageio.get_writer(self.directory / 'native_sim.mp4', fps=self.fps,
                    codec='libx264', pixelformat='yuv420p', macro_block_size=8, quality=8)
                canvas.save(self.directory / 'first.png')
            count = min(repeat, self.max_frames - self.frames)
            for _ in range(count):
                self.writer.append_data(image)
            self.records.append(dict(first_frame=self.frames, frames=count, source=source,
                label=str(label), host_monotonic_s=time.monotonic()))
            if source=='tennis':
                from .transforms import quaternion_matrix
                rotation=quaternion_matrix(base.flatten_np(demo.tcp.pose.q))
                self.records[-1]['actual_tcp_down_error_rad']=float(np.arccos(np.clip(-rotation[2,2],-1,1)))
            self.frames += count
            self.last_image = canvas
        except Exception as exc:
            # Recording failures cannot promote a failed planner outcome.
            if len(self.errors) < 20:
                self.errors.append(type(exc).__name__ + ': ' + str(exc))

    def mark(self, label, repeat=15):
        context = self.context.get()
        if context is not None:
            self.capture(*context, label, repeat=repeat)

    def inspect_rejected_q(self, joints):
        """A labelled STATIC renderer inspection, never a path/physics step.

        Only the SIM articulation qpos is temporarily posed. Do not call native
        sync helpers (which update planner/attachments), env.step or any solver.
        Scene objects remain in their current SIM poses, not predicted grasp
        poses. Restore the exact articulation state before native retry resumes.
        """
        import numpy as np
        context = self.context.get()
        if context is None or self.closed:
            return
        demo, options = context
        q = np.asarray(joints, dtype=float)
        if q.shape != (7,) or not np.isfinite(q).all() or not self.eligible(options):
            raise ValueError('Rejected view requires seven finite SIM joints')
        def array(value):
            if hasattr(value, 'detach'):
                value = value.detach().cpu().numpy()
            return np.asarray(value).copy()
        with self.direct._CUROBO_GPU_LOCK:
            saved_q = array(demo.robot.get_qpos())
            saved_v = array(demo.robot.get_qvel())
            posed = saved_q.copy().reshape(-1)
            posed[demo.arm_indices] = q
            try:
                demo.robot.set_qpos(posed.reshape(saved_q.shape))
                self.capture(demo, options,
                    'STATIC rejected robot pose / objects at current SIM poses / NOT execution', repeat=40)
            finally:
                demo.robot.set_qpos(saved_q)
                demo.robot.set_qvel(saved_v)
                if not np.array_equal(array(demo.robot.get_qpos()), saved_q):
                    raise RuntimeError('Rejected-view SIM articulation was not restored')

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.writer is not None:
            self.writer.close()
        if self.last_image is not None:
            self.last_image.save(self.directory / 'last.png')
        atomic_json(self.directory / 'recording.json', dict(
            schema='rm75.native_sim_video/v1', frames=self.frames, fps=self.fps,
            render_complete=bool(self.frames and not self.errors and self.frames < self.max_frames),
            frame_cap_reached=self.frames >= self.max_frames, errors=self.errors,
            execute_real=False, robot_connected=False, live_camera_used=False,
            physical_success=None, planning_waits_omitted=True,
            interpretation='Original native SIM motion windows; rejected paths not executed; not wall-clock video.',
            records=self.records))


def install(direct, directory, *, requested_source=None, portable=None, recorder_factory=Recorder):
    recorder = recorder_factory(direct, directory, requested_source=requested_source)
    restores = []

    def replace(owner, name, wrapper):
        original = getattr(owner, name)
        setattr(owner, name, wrapper(original))
        restores.append((owner, name, original))

    def episode_wrapper(original):
        @functools.wraps(original)
        def episode(demo, bridge_mod, real_exec, options, *args, **kwargs):
            if real_exec is not None or not recorder.eligible(options):
                return original(demo, bridge_mod, real_exec, options, *args, **kwargs)
            token = recorder.context.set((demo, options))
            try:
                recorder.mark('episode start / current SIM state')
                result = original(demo, bridge_mod, real_exec, options, *args, **kwargs)
                recorder.mark(f'episode return={result}; worker verdict remains separate', repeat=20)
                return result
            except BaseException:
                recorder.mark('STOP / exception: candidate NOT executed; see worker failure', repeat=20)
                raise
            finally:
                recorder.context.reset(token)
        return episode

    def motion_wrapper(original):
        @functools.wraps(original)
        def motion(demo, bridge_mod, label, start, path, options):
            token = recorder.stage.set(str(label))
            try:
                return original(demo, bridge_mod, label, start, path, options)
            finally:
                recorder.stage.reset(token)
        return motion

    def frame_wrapper(original):
        @functools.wraps(original)
        def frame(demo, bridge_mod, options):
            result = original(demo, bridge_mod, options)
            if recorder.context.get() is not None:
                recorder.capture(demo, options, recorder.stage.get())
            return result
        return frame

    def segment_wrapper(original):
        @functools.wraps(original)
        def segment(*args, **kwargs):
            result = original(*args, **kwargs)
            label = str(kwargs.get('label', ''))
            if result is None and ('lift' in label or 'post_place_clearance' in label):
                recorder.mark('REJECTED (not executed): ' + label, repeat=20)
            return result
        return segment

    replace(direct, 'run_targeted_place_episode_curobo_direct', episode_wrapper)
    replace(direct, '_play_dry_run_motion_window', motion_wrapper)
    replace(direct, '_render_dry_run_motion_frame', frame_wrapper)
    replace(direct, '_plan_constrained_linear_segment', segment_wrapper)
    if portable is not None:
        # Jimu intentionally keeps its own execution sink; it does not call the
        # direct motion-window wrapper. Replay only a path the original guarded
        # sink has ALREADY accepted, using the same SIM-only native player.
        # This is a kinematic rendering pass, not another execution/physics test.
        def jimu_execution_wrapper(original):
            signature = inspect.signature(original)
            @functools.wraps(original)
            def execute(*args, **kwargs):
                import numpy as np
                values = signature.bind(*args, **kwargs).arguments
                options, demo = values['args'], values['demo']
                eligible = (values['real_exec'] is None and recorder.eligible(options)
                            and recorder.context.get() is not None and not recorder.closed)
                start = np.asarray(demo.current_arm_qpos()).copy() if eligible else None
                result = original(*args, **kwargs)
                if eligible and isinstance(result, tuple) and result[0] is True:
                    path = np.asarray(values['q_path'], dtype=float)
                    end = np.asarray(demo.current_arm_qpos()).copy()
                    if (path.ndim != 2 or path.shape[1] != 7 or not len(path)
                            or not np.isfinite(path).all() or not np.allclose(end, path[-1], atol=1e-6, rtol=0)):
                        recorder.mark('accepted stage: no equivalent playback endpoint; NOT animated')
                        return result
                    try:
                        direct._play_dry_run_motion_window(demo, values['bridge_mod'],
                            'accepted-path replay: ' + str(values['label']), start, path.tolist(), options)
                    finally:
                        direct.targeted.base.sync_demo_arm_qpos(demo, end)
                return result
            return execute
        replace(portable, '_jimu_execute_pose_path_stage_base', jimu_execution_wrapper)
        def diagnosis_wrapper(original):
            @functools.wraps(original)
            def diagnose(*args, **kwargs):
                result = original(*args, **kwargs)
                if (not recorder.closed and recorder.context.get() is not None
                        and isinstance(result, dict) and result.get('valid') is False):
                    recorder.mark('WORLD COLLISION / failed candidate NOT executed', repeat=20)
                    recorder.inspect_rejected_q(result['diagnosed_q'])
                    # Seal the first-failure clip before the service cancels its
                    # SIM process. The runner waits for this recording metadata.
                    recorder.close()
                return result
            return diagnose
        replace(portable, '_jimu_start_collision_diagnosis', diagnosis_wrapper)

    def close():
        for owner, name, original in reversed(restores):
            setattr(owner, name, original)
        recorder.close()
    return close
