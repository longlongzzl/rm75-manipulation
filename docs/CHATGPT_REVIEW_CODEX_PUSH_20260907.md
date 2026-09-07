# ChatGPT Review — Codex Three-Scene Push (2026-09-07)

Branch: `chatgpt/three-scene-software-closeout`

Reviewed Codex commit: `14ca79921d1b231b2f55ccc730f4fec86283e444`.

## Review conclusion

This Codex push is useful and should be retained. It restored the audited PickPlace/Jimu working snapshot, repaired the three-scene browser frontend, reached `73 passed` for `tests/three_scene` and `395 passed` for the full CPU suite, and correctly stopped before any real robot motion.

It is **not** a three-real-demo completion yet. The remaining blockers are concrete:

1. PickPlace native sim path still exits by timeout / segmentation fault and therefore has no valid migrated planning regression result.
2. Jimu native compatibility path can finish the four-wall program, but the old implementation contains a broad gripper/world collision-relaxation retry. That run cannot satisfy the strict acceptance gate.
3. PushT has CPU/browser and limited cuRobo2 initialization/FK coverage, but no qualified pusher/contact/table/tracking profile, so no complete short-push planning chain has been run.
4. The fixed migration root was too broad and copied unrelated LeRobot/PPO/policy trees. This paper explicitly excludes PPO. Reviewer commits now narrow future migration with `DENIED_PREFIXES`. The already-generated snapshot is intentionally kept manifest-consistent for the moment and must be regenerated locally before the next acceptance run.

## Mandatory next step A — regenerate a minimal snapshot

Do this in the **new `rm75-manipulation` repository only**. Do not modify or clean the old `/home/zhangzhao/Desktop/lerobot` checkout.

After pulling the latest reviewer head, remove the generated snapshot in the new repo and regenerate it from the same fixed source + audited overlay:

```bash
rm -rf rm75_app/_vendor/working_snapshot

PYTHONPATH=. python tools/migrate_working_sources.py \
  --source-repo /home/zhangzhao/Desktop/lerobot \
  --target-repo . \
  --source-ref 7aaff9da22486b7d25557b3795dd258f9b65f10d

PYTHONPATH=. python tools/apply_audited_worktree_overlay.py \
  --source-repo /home/zhangzhao/Desktop/lerobot \
  --target-repo . \
  --manifest configs/workcell/approved_worktree_overlay_20260907.json

PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only
```

The regenerated snapshot must contain **no** paths under:

```text
lerobot-sim2real/lerobot_sim2real/rl/
lerobot/common/policies/
lerobot/common/optim/
src/lerobot/common/policies/
src/lerobot/common/optim/
```

If regeneration exposes a missing import, add the smallest explicit dependency needed by the real PickPlace/Jimu import chain. Do not restore an entire RL/policy tree.

Then rerun:

```bash
PYTHONPATH=. python -m compileall -q rm75_app tools tests/three_scene
PYTHONPATH=. python -m pytest -q tests/three_scene
PYTHONPATH=. python -m pytest -q tests
```

## Mandatory next step B — PickPlace segfault diagnosis, no algorithm redesign

Do not treat the staged old `.so` files as a validated runtime. The observed sequence `JIT timeout -> copied extension .so -> exit 139` is compatible with an extension/cache/ABI mismatch and is not evidence that the PickPlace algorithm is broken.

Run one clean extension build using the same Python/Torch/CUDA environment as the native PickPlace process, with a fresh cache outside the repository. Prefer a one-time longer compile timeout rather than copying opaque old build products:

```bash
rm -rf /tmp/rm75_native_curobo_clean
mkdir -p /tmp/rm75_native_curobo_clean
export TORCH_EXTENSIONS_DIR=/tmp/rm75_native_curobo_clean
```

Then execute the same fixed-scene PickPlace command with `execute-real` absent and allow enough time for the first extension build. Record:

- exact Python, Torch, CUDA and cuRobo source path;
- extension build log;
- first native stack trace / signal if it still crashes;
- whether the crash occurs before planner creation, during planner warmup, or during the first actual planning call.

Do **not** reduce grasp candidates, disable collision, loosen tolerances, change `lazy_place`, or enable reverse/extra fallbacks to make this pass. Production defaults remain `lazy_place + primary_only`.

## Mandatory next step C — Jimu strict-contact gate

Keep the old working behavior for compatibility, but do not count a run that calls the broad world-collision relaxation branch as a strict success.

Instrument the final-contact retry so every use records:

```text
step/piece id
current scene fingerprint
permitted contact target/support
links whose collision state changes
world objects present at that moment
whether the relaxed branch was actually required
```

For the paper acceptance path, the allowed exception must be target-local: contact with the intended magnetic target/support may be permitted for the final capture corridor, while table, unrelated pieces, robot self-collision, and all other world obstacles remain active. If the current cuRobo1 interface cannot express this pair-local exception safely, return a typed `strict_contact_not_supported` result and keep the old broad-relaxation path compatibility-only. Do not silently label it strict.

Run four-wall first, then triangle-roof. A prerequisite/step failure blocks dependent steps. `command_success` and `verified_task_success` remain separate.

## Mandatory next step D — PushT qualification before motion

Do not invent pusher dimensions or transforms. Keep `integration_qualified=false` until the actual tool/workcell values are measured or confirmed.

Before any real movement, populate and validate at least:

```text
push_tcp_z_m
pusher/tool orientation or T_tcp_pusher
permitted contact links
pusher collision geometry qualification
workspace/table bounds
target T geometry dimensions
tracking source + T_base_camera
T_marker_object if a tag is used
Cartesian speed_mps limits
```

Then run a GPU no-motion complete-chain test that plans **pre-contact -> contact -> short push -> retract before execution** and audits every waypoint. Only after that passes should the real-motion ladder start.

## What not to do

- Do not mix PPO/RL code into this paper repository or claim it as part of PushT.
- Do not delete/reset/clean the old `lerobot-realman` worktree.
- Do not count CPU PushT surrogate as physics/real validation.
- Do not count native process exit 0 as object-level task success.
- Do not count Jimu broad collision relaxation as a strict planning success.
- Do not proceed to full real robot motion while PickPlace native, Jimu strict contact, or PushT qualification remains unresolved.

## Required next report

Write the next results to:

```text
docs/CODEX_THREE_SCENE_RESULTS_20260907_R2.md
```

It must state the exact tested commit, regenerated snapshot file count, proof that denied RL/policy prefixes are absent, CPU test totals, PickPlace native diagnosis/result, Jimu strict-contact result, PushT qualified/unqualified fields, and every unrun hardware item as `NOT_RUN`.
