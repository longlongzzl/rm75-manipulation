# ChatGPT Review — Three-Scene Validation Push (2026-09-08)

Branch: `chatgpt/three-scene-software-closeout`  
Reviewed remote HEAD: `4c741184e0ec32595e8c08efd916cd2c8e52bb62`

## 1. Review conclusion

The latest Codex push is materially better than the previous closeout. The important progress is real, but **the branch is not yet a three-real-demo release**.

Accepted as genuine software progress:

- PickPlace is now exercised through the actual `WorkcellService -> worker -> original cuRobo1 native SIM` path with full frozen-world coverage and requested-source identity checks, rather than only through an isolated diagnostic runner.
- The worker correctly refuses to count a different source object's success, a background prefetch success, or native `final=True` as success for the requested object.
- The full-world gate keeps the other eight frozen objects loaded while removing only the active source.
- Jimu four-wall and standard triangle-roof control runs retain strict transport/return auditing with no transport world exemptions.
- PushT has real cuRobo2 GPU planning evidence for complete approach -> descend -> contact -> push -> retreat chains, speed retiming, blocker rejection, and execution-precondition rejection.
- The unrelated PPO / policy vendor trees were removed from the three-scene migration boundary.

Still not accepted as complete:

- PickPlace actual workcell requests are 6/11 on the current frozen request matrix, not a completed robust object set.
- Jimu's original full builder remains 8/12; the four roof pieces have not been recovered under the exact old working geometry.
- tagless Jimu RRTrack has only a single-piece camera experiment; multi-instance identity and assembly precision are not qualified.
- PushT's actual physical pusher/TCP/contact geometry and real observation profile are not qualified.
- RealMan SDK no-motion and physical motion ladders remain NOT_RUN.

## 2. Decision: approve the two missing Jimu triangle assets

The read-only audit has established that the reviewed old triangle entrypoint explicitly prefers the following files when present, and that the current snapshot falls back to a geometrically different mesh when they are absent:

```text
Demo_Triangle/red_triangle_74x135x6p5.glb
sha256 = fac68dee16a4f8d5b17d7de0f4007ac2445472c70d338c46dbb98908e38f4175

Demo_Triangle/red_triangle_74x135x6p5.glb.coacd.ply
sha256 = f6d655ec58ef7db661241935c6d7f9ee3d0ac35c6807e0d2d1b57f9772582a12
```

**These two files are approved for migration as final-runtime dependencies of the already-completed Jimu workflow.**

Approval is limited to exactly these paths and hashes. It does not approve the rest of `Demo_Triangle/`, alternate launch scripts, reduced-candidate profiles, or another triangle algorithm.

Codex should add these two files to the single audited overlay manifest, rebuild the snapshot from fixed `7aaff9d` + the complete approved overlay, verify the manifest, and rerun the exact old triangle/full-builder controls before changing any roof planning algorithm.

Reason: the old entrypoint code is the same, but `exists()` selects a different 135 mm model in the user's working setup. The current fallback is about 120 mm and has the opposite local tip orientation in the builder path. Until the exact old asset branch is restored, changing IK, grasps, collision allowances, builder orientation, or roof target poses would be premature.

## 3. PickPlace: stop broad diagnostics; close the three real request failures

Current actual workcell frozen requests:

- legacy gluestick: PASS
- jitter gluestick: PASS
- current-table lvmukuai: PASS
- current-table carriot: PASS
- current-table shuazi: PASS
- current-table bi: PASS
- yaw gluestick: FAIL
- swap gluestick: FAIL
- current-table hongshupian: FAIL
- current-table gluestick: FAIL
- current-table tennis: FAIL at post-release retreat

For the demo/release path, prioritize the **real current-table failures** (`hongshupian`, `gluestick`, `tennis`) before spending more time on synthetic yaw/swap perturbations.

Rules for the next pass:

1. Keep production `lazy_place + primary_only` unchanged.
2. Do not reduce grasp candidates, seeds, world objects, self-collision, payload collision, or frozen-world coverage.
3. Do not increase penetration/contact tolerance just to make tennis pass.
4. Do not count native fallback to another source as requested-object success.
5. Diagnose each failure to one first failing stage and one physical/model reason before changing code.

### Tennis

The current failure is specific enough: the selected release-retreat path increases target-local penetration by about 0.135 mm even though endpoint IK and the sampled Cartesian line otherwise pass.

Preferred repair order:

- first check whether the **existing** reverse-incoming / existing alternate retreat candidate decreases contact monotonically under the same collision model;
- if such an existing candidate is already generated, select it only after full world/self/target audit;
- if no existing safe retreat exists, return failure and report it before inventing a new tolerance or collision exemption.

A safe candidate-selection fix is acceptable; weakening the contact gate is not.

### Gluestick / hongshupian

Separate failures caused by:

- object/world geometry mismatch,
- attached-payload/base self model,
- grasp-to-lift relation failure,
- placement relation failure,
- or actual IK reachability.

If a collision model is shown to differ from the real calibrated geometry, update the model from measured/known geometry and preserve an explicit before/after audit. Do not shrink spheres/buffers solely from planner failure evidence.

Acceptance for PickPlace software release: the selected demo object set must pass the actual workcell worker from a complete frozen world with requested-source identity, transport audit, post-place clearance audit, and no hidden alternate-source success. Synthetic yaw/swap may remain robustness failures if explicitly reported; they are not allowed to overwrite current-table results.

## 4. Jimu: restore old geometry first, then rerun full builder

After installing the two approved triangle assets, run in this order:

1. snapshot verify + compileall;
2. no-motion import/parser contract;
3. four-wall control;
4. standard triangle-roof control;
5. original `tag1_standard_three_layer` full builder;
6. compare roof model branch, dimensions, local tip direction, target edge gaps, first failing stage, and final 12-piece outcome against the pre-restoration run.

Keep all intermediate failures in the denominator.

If full builder becomes 12/12 with the restored old geometry, do **not** add further roof IK/planning modifications.

If it remains 8/12, then isolate the new first failure under the restored 135 mm model before considering any algorithm change. No builder-axis flip, target movement, collision exemption, or extra IK seed should be introduced merely because the fallback-model run failed.

## 5. Jimu shared perception gate

The single orange square tile RRTrack run is useful evidence but is not enough for the intended shared-perception architecture. The 30/30 accepted trajectory still showed millimetre-scale spread, did not exercise LOST recovery, and did not establish identity among identical pieces.

Before calling Jimu tagless-ready, perform a no-motion camera test with multiple visually identical magnetic pieces:

- at least 4 simultaneous pieces if physically available;
- stable instance IDs across a bounded sequence;
- one controlled temporary occlusion / reappearance;
- no identity swap accepted silently;
- record pose spread and confidence per instance;
- transform into the same calibrated base frame used by planning;
- do not use AprilTag as hidden identity ground truth for the claimed tagless result.

A separate AprilTag comparison may be used only as evaluation/reference evidence, not as the production input for the tagless claim.

## 6. PushT: software planning is far enough; do not tune authored fixtures further

The latest reported cuRobo2 evidence is sufficient to stop optimizing synthetic fixtures for now:

- 3 complete five-stage GPU chains passed;
- 2 normal tool-orientation cases failed at descend and remain failures;
- 1 blocker negative case was correctly rejected;
- 3/3 added-obstacle audits rejected;
- 30/30 execution-precondition injections rejected;
- 5 mm/s and 15 mm/s runs demonstrate speed entering trajectory timing.

Do not move contact/standoff points or shrink collision geometry simply to make the two failing authored orientations pass.

The next PushT blocker is **physical qualification**, not more synthetic MPC tuning.

Before real motion, one concrete physical tool configuration must be selected and measured. Codex must not guess these values. The profile must contain and pass qualification for:

```text
pusht.motion.tool_frame
pusht.motion.push_tcp_z_m
pusht.motion.tool_quaternion_wxyz
pusht.motion.pusher_contact_links
pusht.motion.tool_collision_geometry_verified
pusht.motion.object_centroid_z_m
pusht.motion.object_height_m
pusht.motion.static_collision_objects
pusht.observer.kind
plus the selected observer's calibrated transforms/source
```

The unresolved user/hardware decision is whether the final demo pushes with:

- the closed gripper itself, or
- a dedicated/printed pusher mounted to the gripper.

Until that is decided and measured, keep `hardware_profile_qualified=false` and do not manufacture a green profile.

## 7. RealMan / physical ladder remains blocked

No new code path should silently convert software success into hardware permission.

Before any object task motion:

1. actual SDK no-motion connection;
2. joint feedback/order/unit validation;
3. controller stop API validation;
4. gripper backend read/write semantics without task motion;
5. one reduced-speed free-space trajectory;
6. then isolated object actions.

`command_completed_unverified`, native `final=True`, surrogate pose success, and GPU planning success are not physical task success.

## 8. Next Codex pass

Start from the exact remote HEAD and keep the run small and decision-driven:

1. clean-HEAD compileall + full tests;
2. migrate exactly the two newly approved triangle assets by SHA into the audited overlay and rebuild the snapshot;
3. rerun Jimu standard/full-builder GPU controls and report whether the exact old geometry closes the 4 roof failures;
4. run focused current-table PickPlace diagnostics for `gluestick`, `hongshupian`, `tennis`, then implement only a justified minimal safe fix if evidence supports one;
5. rerun the actual workcell PickPlace requests affected by the fix;
6. do not spend another pass tuning PushT authored fixtures; instead prepare/validate the physical-profile checklist without filling unknown hardware values;
7. if the D435 and multiple Jimu pieces are available, run the multi-instance RRTrack no-motion identity test;
8. no robot motion in this pass unless the user explicitly starts the physical ladder after reviewing the no-motion evidence.

Write a single consolidated result file for this pass instead of many overlapping status MDs. Suggested path:

```text
docs/CODEX_THREE_SCENE_RELEASE_GATE_20260908_R2.md
```

Keep raw trajectories, joint arrays, hardware identifiers, and bulky local logs out of Git; commit summarized evidence and hashes only.
