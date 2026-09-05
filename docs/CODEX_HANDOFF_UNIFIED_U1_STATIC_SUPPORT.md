# Isolated support stability diagnostic

Task: test whether the failed carrot release is explained simply by an unstable pre-open object/support pose.
State: NEEDS_REVIEW.
Commit: containing commit (`git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_STATIC_SUPPORT.md`).
Parent commit: 23a5207.

Changed files / reasons:

- `tools/diagnose_static_support.py`: standalone local physics diagnostic, not a replay gate. Deep-copies the source simulator scene, changes only carrot/brush initial poses to recorded pre-open observations, initializes zero velocity, and holds the robot at reset for 3 seconds. Retains all scene objects, meshes, scales, friction and fixed flags. Writes exclusive output directory with scene and 61 pose/contact samples.
- `tests/test_static_support_diagnostic.py`: verifies inputs remain unchanged, unrelated objects/physics remain unchanged, and missing supports are rejected.
- Summary JSON and this handoff: evidence and explicit non-equivalence to dynamic replay.

Commands run:
```bash
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/diagnose_static_support.py --scene-spec /tmp/rm75_u1_release_boundary_replay_001/maniskill_scene.json --replay-result /tmp/rm75_u1_release_boundary_replay_001/maniskill_gate_result.json --atom-id sort_03_carriot --object-id carriot --support-id shuazi --output-dir /tmp/rm75_u1_static_carrot_support_001 > /tmp/rm75_u1_static_carrot_support_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_static_support_tests.log 2>&1
git diff --check
```

Test environment: unchanged CPU Python 3.12.7; realman Python 3.11.14, ManiSkill/SAPIEN, RM75-MultiObjectTask-v1, pd_joint_pos at 20 Hz, seed 0. No real robot sink. Static initialization is explicitly not the strict no-teleport trajectory replay and is never labeled a gate pass.

Before metrics: opening in the failed three-object replay displaced carrot 55.7217 mm. Read-only contact audit of third opening's 20 control-boundary samples shows nonzero brush/carrot impulses at all 20; nonzero right gripper-support/carrot impulses appear only at samples 13-19. This cannot rule out contacts between samples.
After metrics: 223 tests pass, zero failed/skipped, 19.33 s. Isolated carrot displacement is 1.90638 mm at both 1 and 3 seconds, unchanged from 0.5 seconds onward. No sampled nonzero carrot/robot contacts. This observation contradicts a sufficient explanation based solely on the recorded static support pose.

Correctness/safety: no production planning, execution, collision model or task-criterion change. No friction tuning, robot teleport during replay, object deletion, or claimed successful grasp/release. Source scene and failed replay artifacts untouched. Unit tests enforce exact object/support presence and preserve other scene records.

Failures/raw statuses: original three-object replay remains failed at carrot. Static tool completion means diagnostic execution finished, not robot-task success. Robot configuration, initial velocities and other objects' dynamic poses differ from original opening; the experiment does not isolate which of those differences caused stability.

Raw paths: all exact paths in `benchmarks/unified_scenarios/u1_static_support_summary.json`.

Unexpected observations: despite original opening-time drift, the recorded pre-open pose can settle on the brush in isolation. Do not optimize target geometry or friction based on the earlier unsupported assumption that this pose itself must be unstable.

Open questions / next work: isolate release velocity and arm/gripper contact dynamics while retaining original failed replay. Measured 20-scene sorting data and later real/calibration gates remain outstanding. Push is still blocked by explicit destination/payload authorization; do not retry absent new approval. No main merge, no physics identification.
