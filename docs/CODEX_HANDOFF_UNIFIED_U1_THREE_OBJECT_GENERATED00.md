# Three-object compiled program and strict physics replay

Task: advance full-program sorting validation after the approved IK cache fix, using an independently frozen generated scene and all three historical assignments.
State: NEEDS_REVIEW.
Commit: containing commit (`git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_THREE_OBJECT_GENERATED00.md`).
Parent commit: 24754d5.

Changed files / reasons:

- `benchmarks/unified_scenarios/fixtures/sorting_historical_three_objects_generated00.json`: separate diagnostic request from the original three-object fixture with only request id, scene path and provenance metadata changed. No assignments, target poses, tolerances, or obstacles removed. Frozen before compiling.
- `benchmarks/unified_scenarios/u1_three_object_generated00_summary.json`: compact compilation, task failure, safety observations and exact raw artifact paths.
- This handoff: review record.

Commands run:
```bash
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/benchmark_sorting_program.py --request benchmarks/unified_scenarios/fixtures/sorting_historical_three_objects_generated00.json --initial-joints /tmp/rm75_u1_strict_replay_002/replay_initial_state.json --output-dir /tmp/rm75_u1_cache_three_generated00_001 > /tmp/rm75_u1_cache_three_generated00_001.log 2>&1
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/replay_sorting_program.py --compiled-dir /tmp/rm75_u1_cache_three_generated00_001 --output-dir /tmp/rm75_u1_cache_three_generated00_replay_001 > /tmp/rm75_u1_cache_three_generated00_replay_001.log 2>&1
git diff --check
```

Test environment: unchanged curobo2 Python 3.11 and realman Python 3.11.14 / ManiSkill / SAPIEN, RM75-MultiObjectTask-v1, pd_joint_pos, 20 Hz control. Explicit initial joints are previously recorded simulator reset zeros, not real measured joints. No runtime code changed in this commit; prior full regression is 219 passed at parent.

Before metrics: original current-table three-object compile failed at gluestick, with carrot unattempted; not overwritten or relabeled. Generated00's single-atom glue full-chain passed after cache fix, but its three-atom program/physics behavior had not been verified.
After metrics: 3/3 atoms fully compile in 18.7018 s, 18 motion stages, zero initial gap and maximum inter-stage gap 1.1921e-7 rad. Strict no-teleport time-preserving physics executes all 18 stages. Tennis passes inside; gluestick passes target_pose at 17.291 mm / 5.737 degrees. Carrot fails target_pose at settled 138.048 mm / 33.146 degrees. Overall replay exit code 2 / failed.

Correctness/safety: no planner calls during replay, no partial compilation executed, no teleport, same frozen portable program, no goal/tolerance/geometry weakening. Host software idle P95 1.956 ms, maximum 3.221 ms after excluding declared dwell/checkpoints. Among 464 control-boundary samples, seven arm joints show no limit excess; gripper_Left_Support_Joint shows 0.00133544 rad excess during close. These samples are not full physics-substep collision/limit verification. No robot commands and no physics identification.

Failures/raw statuses: carrot is close to target at place endpoint, then moves during opening/retreat and further settling. Retreat reports right gripper-support/carrot contact, maximum net impulse 0.005222 Ns. This is evidence of contact, not sufficient proof that retreat alone caused the final failure: opening has no object-pose boundary trace yet. Do not change placement tolerances to hide the failure. Surface pose checks are not a separate on_top stability validator.

Raw logs and committed summary: all exact paths in `benchmarks/unified_scenarios/u1_three_object_generated00_summary.json`. Local raw logs are not automatically available remotely.

Unexpected observations: a complete three-object plan is now available without relaxing constraints, but complete planning is not physical task success. Post-release instability rather than lack of a full program is the next diagnostic focus for this fixture.

Open questions for ChatGPT: review release/retreat diagnostics and the existing surface target contract. Measured 20-scene inputs still missing, full SIM gate still failed, real trials forbidden until prerequisites and explicit approval. Push remains blocked by destination/payload authorization from the previous turn; no retry without new authorization. Do not merge main.
