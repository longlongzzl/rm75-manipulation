"""Bounded actual-feedback recorder; collection extends past visual exposure.

The existing executor/simulator supplies samples. This module never polls a
robot or substitutes commanded trajectory points for feedback.
"""
from __future__ import annotations
import copy
import threading
import numpy as np
from .scene import SceneInvalid, transform
from .measurements import bind_measured_transition


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
            if self._rows and stamp <= self._rows[-1]['captured_at']:
                raise SceneInvalid('Repeated/reordered tool feedback')
            if len(self._rows) >= self.max_samples:
                raise SceneInvalid('Feedback budget exhausted; cannot drop action start')
            self._rows.append(row)

    def bind_action(self, receipt, *, support_id, initially_settled, settling_evidence, intervened=False):
        """Bind receipt while KEEPING collection active until the after frame."""
        with self._lock:
            if receipt.actual_action_id in self._seen_actions:
                raise SceneInvalid('Duplicate recorded action')
            self._seen_actions.add(receipt.actual_action_id)
            self._actions[receipt.actual_action_id] = dict(support_id=support_id,
                initially_settled=initially_settled, settling_evidence=settling_evidence, intervened=intervened)

    def transition(self, request, receipt, initial_snapshot, final_snapshot):
        with self._lock:
            if receipt.actual_action_id not in self._actions:
                raise SceneInvalid('No recorder bound to this receipt')
            recording = dict(schema='rm75_swm_action_recording_v1', source='measured_feedback',
                actual_action_id=receipt.actual_action_id, **self.provenance, **self._actions[receipt.actual_action_id],
                tool_captured_at=[r['captured_at'] for r in self._rows],
                T_world_tcp=[r['T_world_tcp'] for r in self._rows], stages=[r['stage'] for r in self._rows])
            result = bind_measured_transition(request, receipt, initial_snapshot, final_snapshot, recording=recording)
            del self._actions[receipt.actual_action_id]
            return result
