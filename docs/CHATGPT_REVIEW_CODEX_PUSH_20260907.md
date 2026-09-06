# ChatGPT Review — Codex Three-Scene Push (2026-09-07)

Branch: `chatgpt/three-scene-software-closeout`

## Review conclusion

Codex stopped at the correct boundary. The previous remote delivery had two concrete software defects: the workcell frontend entry HTML was missing and the migration command documented in the handoff did not match the shipped CLI. Both defects are now fixed on this branch.

Codex also found behavior-changing dirty edits in the historical `lerobot-realman` working directory. Because PickPlace and magnetic Jimu are already-completed user workflows, those edits must not be silently discarded. Conversely, the entire dirty/untracked tree must not be copied into the new repository.

The migration contract is therefore:

```text
fixed committed source 7aaff9d
  + read-only dependency-closure audit
  + explicit SHA256-addressed approved dirty overlay
  + rerun software/GPU/simulation validation
```

## Source files to audit first

These paths are priority candidates, not automatic approvals:

```text
pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py
pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place_sam6d.py
pick_jiaobang/rm75_jiaobang_pick_real_with_foundationpose.py
pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo.py
pick_jiaobang/curobo_rm75_planner.py
Beta_demo-codex-v0.9/curobo_rm75_planner.py
Beta_demo-codex-v0.9/jimu_sam6d_pose_provider.py
Beta_demo-codex-v0.9/rm75_jimu_four_wall_direct_sam6d.py
Beta_demo-codex-v0.9/rm75_jimu_four_wall_portable.py
Beta_demo-codex-v0.9/rm75_jimu_triangle_roof_apriltag_portable.py
lerobot/common/robots/realman_lerobot/realman_arm.py
FoundationPose/run_demo_scale.py
```

Trace imports, dynamic module loads, configured paths and actual launch commands from those files. A dirty file should enter the approved overlay only when the audit can state what final validated behavior it preserves and why the fixed baseline is insufficient.

Untracked directories/assets such as `jimu_builder_frontend/`, `jimu_tasks/`, plate assets or `Demo_Triangle/` are not approved by name. Include individual files only if the dependency closure proves the migrated runtime needs them.

## Next Codex sequence

1. Pull the branch and record exact HEAD.
2. Run `compileall` and `pytest -q tests/three_scene`; all frontend routes must return a real page rather than 404.
3. Perform the old-repository read-only dependency audit. Do not reset, clean, checkout, stash or modify it.
4. Build `runtime_data/three_scene/approved_worktree_overlay.json` with exact source-head, status SHA256, file SHA256 and one reason per approved file.
5. Install the fixed `7aaff9d` snapshot with `tools/migrate_working_sources.py`.
6. Apply the approved overlay with `tools/apply_audited_worktree_overlay.py` if and only if the manifest contains audited files.
7. Re-verify and compile the vendored snapshot. Run non-motion original PickPlace/Jimu import/help/preview smoke tests and verify dynamic dependencies are present.
8. Run PickPlace/Jimu migrated GPU/simulation regression and PushT CPU/GPU checks according to the main handoff.
9. Do not start physical motion in this pass. No-motion hardware preflight may be recorded if the operator and machine are available.
10. Write `docs/CODEX_THREE_SCENE_RESULTS_20260907.md`, preserving every failure and `NOT_RUN`, then push.

## Acceptance for this pass

This pass is complete when:

- branch software compiles;
- `tests/three_scene` is green;
- all three frontend routes render;
- source audit and dependency closure are recorded;
- fixed snapshot and any overlay are content-addressed and reproducible;
- old repository status is byte-for-byte unchanged before/after migration;
- migrated PickPlace and magnetic entrypoints can be loaded/run in non-motion smoke mode without missing local dependencies;
- PushT CPU and available GPU no-motion planning checks are reported;
- no claim of physical task success is made without a real outcome observation.

Do not trade correctness for a green report by reducing candidate sets, disabling collision/self-collision/freshness checks, loosening success definitions, or silently falling back to a different task implementation.
