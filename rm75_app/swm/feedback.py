"""Bounded actual-feedback recorder; collection extends past visual exposure.

The existing executor/simulator supplies samples. This module never polls a
robot or substitutes commanded trajectory points for feedback.
"""
from __future__ import annotations
import copy
import threading
from dataclasses import dataclass, replace
import numpy as np
from .scene import SceneInvalid, transform
from .measurements import bind_measured_transition


@dataclass(frozen=True)
class MeasuredRecordingHandle:
    """Owned recording reference, never a fabricated measured trace."""
    actual_action_id: str
    recorder: object

    def close(self):
        self.recorder.cancel_action(self)


def close_receipt_recording(receipt):
    handle = None if receipt is None else receipt.observations
    if isinstance(handle, MeasuredRecordingHandle):
        handle.close()


class MeasuredFeedbackRecorder:
    def __init__(self, *, domain, sensor_session, calibration_id, max_samples=100000):
        if domain not in ('real', 'physics', 'fixture') or not sensor_session or not calibration_id:
            raise ValueError('Explicit recorder provenance required')
        if type(max_samples) is not int or not 2 <= max_samples <= 100000:
            raise ValueError('Bounded recorder storage required')
        self.provenance = dict(domain=domain, sensor_session=sensor_session, calibration_id=calibration_id)
        self.max_samples = max_samples
        self._rows = []
        self._actions = {}
        self._seen_actions = set()
        self._lock = threading.RLock()
        self._handles = {}
        self._started = {}
        self._completed = {}
        self._closed = False

    def append(self, sample):
        row = copy.deepcopy(sample)
        if row.get('source') != 'measured_feedback':
            raise ValueError('Commanded targets are not measured feedback')
        if any(row.get(k) != v for k, v in self.provenance.items()):
            raise SceneInvalid('Feedback clock/session/calibration changed')
        stamp = row['captured_at']
        if not isinstance(stamp, (int, float)) or not np.isfinite(stamp):
            raise ValueError('Finite capture timestamp required')
        row['T_world_tcp'] = transform(row['T_world_tcp']).tolist()
        if row.get('stage') not in ('approach', 'descend', 'contact', 'push', 'retreat', 'post_settle'):
            raise ValueError('Measured push stage required')
        with self._lock:
            if self._closed:
                raise SceneInvalid('Feedback recorder is closed')
            if self._rows and stamp <= self._rows[-1]['captured_at']:
                raise SceneInvalid('Repeated/reordered tool feedback')
            if len(self._rows) >= self.max_samples:
                if self._actions:
                    raise SceneInvalid('Feedback budget exhausted; cannot drop action start')
                # Idle sampling is a bounded rolling window. Never evict rows
                # during an active action or while waiting for its after frame.
                del self._rows[:max(1, self.max_samples // 2)]
            self._rows.append(row)

    def bind_action(self, receipt, *, support_id, initially_settled, settling_evidence, intervened=False):
        """Bind receipt while KEEPING collection active until the after frame."""
        with self._lock:
            if self._closed:
                raise SceneInvalid('Feedback recorder is closed')
            if receipt.actual_action_id in self._seen_actions:
                raise SceneInvalid('Duplicate recorded action')
            self._seen_actions.add(receipt.actual_action_id)
            self._actions[receipt.actual_action_id] = dict(support_id=support_id,
                initially_settled=initially_settled, settling_evidence=settling_evidence, intervened=intervened)

    def begin_action(self, actual_action_id, started_at, *, support_id,
                     initially_settled, settling_evidence, intervened=False):
        """Begin BEFORE commands; existing sampling must already cover exposure."""
        if not actual_action_id or not np.isfinite(started_at):
            raise ValueError('Actual action identity and finite start required')
        with self._lock:
            if self._closed or self._actions:
                raise SceneInvalid('Recorder closed or previous action still awaiting its after frame')
            if actual_action_id in self._seen_actions:
                raise SceneInvalid('Duplicate recorded action')
            if not self._rows or self._rows[-1]['captured_at'] > started_at:
                raise SceneInvalid('Measured sampling must start before execution')
            handle = MeasuredRecordingHandle(actual_action_id, self)
            self._seen_actions.add(actual_action_id)
            self._handles[actual_action_id] = handle
            self._started[actual_action_id] = float(started_at)
            self._actions[actual_action_id] = dict(support_id=support_id,
                initially_settled=initially_settled, settling_evidence=settling_evidence,
                intervened=intervened)
            return handle

    def finish_command(self, handle, receipt):
        """Keep collecting after command completion; bind this exact receipt."""
        with self._lock:
            aid = handle.actual_action_id
            if self._handles.get(aid) is not handle or aid in self._completed:
                raise SceneInvalid('Unknown or already completed recording handle')
            if (receipt.actual_action_id != aid or receipt.command_success is not True
                    or receipt.motion_known_complete is not True
                    or receipt.started_at != self._started[aid]
                    or not np.isfinite(receipt.ended_at) or receipt.ended_at < receipt.started_at):
                raise SceneInvalid('Recording requires this actual completed command')
            self._completed[aid] = (receipt.started_at, receipt.ended_at)
            return replace(receipt, observations=handle)

    def _drop_action(self, aid):
        self._actions.pop(aid, None)
        self._handles.pop(aid, None)
        self._started.pop(aid, None)
        self._completed.pop(aid, None)
        if not self._actions and self._rows:
            # The last true sample may bracket the next exposure. Everything
            # older belongs to the completed/cancelled window, not future fits.
            self._rows[:] = self._rows[-1:]

    def cancel_action(self, handle):
        with self._lock:
            active = self._handles.get(handle.actual_action_id)
            if active is None:
                return
            if active is not handle:
                raise SceneInvalid('Recording handle belongs to another owner')
            self._drop_action(handle.actual_action_id)

    def close(self):
        with self._lock:
            self._closed = True
            self._rows.clear()
            self._actions.clear()
            self._handles.clear()
            self._started.clear()
            self._completed.clear()
            self._seen_actions.clear()

    def transition(self, request, receipt, initial_snapshot, final_snapshot):
        with self._lock:
            if receipt.actual_action_id not in self._actions:
                raise SceneInvalid('No recorder bound to this receipt')
            aid = receipt.actual_action_id
            if aid in self._handles:
                if (receipt.observations is not self._handles[aid]
                        or self._completed.get(aid) != (receipt.started_at, receipt.ended_at)):
                    raise SceneInvalid('Receipt is not bound to the completed recording')
            recording = dict(schema='rm75_swm_action_recording_v1', source='measured_feedback',
                actual_action_id=receipt.actual_action_id, **self.provenance, **self._actions[receipt.actual_action_id],
                tool_captured_at=[r['captured_at'] for r in self._rows],
                T_world_tcp=[r['T_world_tcp'] for r in self._rows], stages=[r['stage'] for r in self._rows])
            result = bind_measured_transition(request, receipt, initial_snapshot, final_snapshot, recording=recording)
            self._drop_action(receipt.actual_action_id)
            return result
