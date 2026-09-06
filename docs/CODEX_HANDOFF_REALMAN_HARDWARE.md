# Codex Handoff — RM75 Hardware Integration

State: `CODEX_IMPLEMENTED` (ChatGPT implementation; local validation required)  
Branch: `chatgpt/realman-hardware-integration`  
Base: `chatgpt/unified-three-scenarios`

## Scope implemented

- restored the physically used RealMan SDK boundary without restoring the old monolithic task scripts;
- one `RealManTrajectoryExecutor` for sorting, magnetic assembly, and Push-T;
- actual joint feedback before stage execution and endpoint feedback after it;
- explicit disarmed/armed lifecycle;
- controller-side stop discovery/use;
- historical Modbus or hand-follow gripper paths;
- no-motion preflight CLI;
- guarded `/api/robot/*` lifecycle API and unified web entrypoint;
- fake-SDK and Flask lifecycle tests.

## Required local validation — no robot motion

Run first in the normal repository environment:

```bash
git fetch origin
git checkout chatgpt/realman-hardware-integration
git status --short

PYTHONPATH=. python -m compileall -q \
  rm75_app/execution/realman_executor.py \
  rm75_app/web/realman_hardware_api.py \
  rm75_app/web/unified_control_panel.py \
  tools/realman_preflight.py

PYTHONPATH=. python -m pytest -q \
  tests/test_realman_executor.py \
  tests/test_realman_hardware_api.py

PYTHONPATH=. python -m pytest tests -q
```

Do not change thresholds, joint units, gripper polarity, or stop semantics merely
to make tests pass.  Report any mismatch with the installed RealMan SDK.

## Required robot-PC validation — still no motion

Only with the physical E-stop available and human operator present:

```bash
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python \
  tools/realman_preflight.py \
  --ip 192.168.101.20 \
  --port 8080 \
  --require-ready
```

If the current arm IP differs, use the actual IP; do not edit the code default just
to hide the mismatch.  Save the JSON output.

Then verify the web lifecycle API without arming:

```bash
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python \
  -m rm75_app.web.unified_control_panel --host 127.0.0.1 --port 5000
```

Check `/api/robot/status`, then connect through `/api/robot/connect`.  Do not call
`/api/robot/arm` in this validation step.

## SDK points to verify on the installed robot PC

Report exact function availability/signatures for:

```text
rm_create_robot_arm
rm_get_joint_degree
rm_get_current_arm_state
rm_movej_follow
rm_set_arm_stop / rm_set_arm_slow_stop / rm_set_arm_pause
rm_set_modbus_mode
rm_write_registers
rm_write_single_register
rm_read_holding_registers
rm_set_hand_follow_pos
```

Only one gripper backend needs to work for the current hardware; state which one.

## First physical-motion ladder — do NOT execute until user approves

After ChatGPT reviews the no-motion handoff:

1. one reduced-speed free-space trajectory stage;
2. compare planned/actual q and endpoint error;
3. isolated open/close gripper away from objects;
4. one PickPlace atom;
5. two atoms with RRTrack checkpoint;
6. then sorting / magnetic / Push-T task ladders.

No complete task trial in this handoff.

## Required handoff block

```text
Commit tested:
Python environment:
compileall:
Focused pytest:
Full pytest:
Installed RealMan SDK/version:
Robot IP used:
No-motion preflight JSON path/result:
Available stop API:
Gripper backend verified:
Unexpected API/signature differences:
Changed files (if any):
Open questions for ChatGPT:
```
