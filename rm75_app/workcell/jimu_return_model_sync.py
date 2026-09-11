"""Sync the planner gripper model to the SIM at the return-check boundary.

The pre-release return check runs while the simulation gripper may already be
partially opened while the planner model still carries the nominal lock joints;
the stale model makes the return START state collide with the just-placed
piece. This is a bounded model-correctness sync, never a collision exemption:
no sphere, buffer, candidate or success condition changes.
"""
import functools
import inspect

from .pickplace_gripper_state import sync_gripper_to_demo
from .jimu_return_diagnostics import _state


def install_return_model_sync(portable, emit):
    direct = portable.direct
    original = direct._plan_return_to_start_joint_curobo
    signature = inspect.signature(original)

    @functools.wraps(original)
    def returning(planner, demo, args, start_q, goal_q, *, label, **kwargs):
        if (getattr(args, 'execute_real', False)
                or getattr(args, '_planning_prefetch_capture_only', False)):
            return original(planner, demo, args, start_q, goal_q, label=label, **kwargs)
        with direct._CUROBO_GPU_LOCK:
            row = dict(event='jimu_return_model_sync', step_id=label,
                       source=direct._current_source_object_name(args),
                       diagnostic_only=True, execution_guard=False,
                       sync_applied=False)
            before = None
            try:
                before = _state(planner)
                result = sync_gripper_to_demo(planner, demo, args)
                row.update(sync_applied=bool(result.get('synced', False)),
                           locks_before=result.get('before'), locks_after=result.get('after'))
            except BaseException as exc:
                row.update(error_type=type(exc).__name__, error=str(exc))
                raise
            finally:
                if row.get('sync_applied'):
                    # A successful sync changes the lock model BY DESIGN; the
                    # unchanged marker covers only no-op observations.
                    row['model_state_unchanged'] = None
                elif before is not None:
                    row['model_state_unchanged'] = (_state(planner) == before)
                emit(row)
        return original(planner, demo, args, start_q, goal_q, label=label, **kwargs)

    direct._plan_return_to_start_joint_curobo = returning
    return signature
