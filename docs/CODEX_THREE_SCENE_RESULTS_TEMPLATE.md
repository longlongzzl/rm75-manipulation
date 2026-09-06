# Codex 三场景本地验证回传模板

## 0. 基本信息

- Commit tested:
- Branch:
- Python:
- OS:
- CUDA / driver:
- cuRobo2:
- ManiSkill:
- RealMan SDK:
- Robot IP used:

## 1. 旧仓库 dirty worktree 审计

- Old repo path:
- Old repo HEAD:
- Reproducible baseline ref: `7aaff9da22486b7d25557b3795dd258f9b65f10d`
- `git status --short` before:
- Dirty files touching six reviewed entrypoints/dependencies:
- Classification of each relevant dirty change: `irrelevant_local` / `debug_only` / `candidate_final_fix` / `unknown`
- Was any dirty file modified/overwritten by this validation? MUST be `NO` unless user explicitly approved.
- Recommendation: use fixed baseline / stop for user review

## 2. 迁移完整性

- Migration command:
- Snapshot manifest path:
- Source commit recorded:
- Six entrypoint blob checks:
- `--verify-only` result:
- Old repo status unchanged after migration:

## 3. 代码完整性与 CPU 测试

- `compileall`:
- `pytest tests/three_scene`:
- Full repository pytest:
- Failures / skipped / NOT_RUN:

## 4. 前端

### PickPlace
- Page loads:
- Preview:
- Status polling:
- Original input prompt bridge:
- Stop:

### Magnetic
- Page loads:
- Import `jimu_builder_scene_v1`:
- Round-trip preserves fields:
- Preview:
- Original input prompt bridge:
- Stop:

### PushT
- Page loads:
- Preview:
- Sim request:
- Live status:
- Stop:

## 5. PickPlace 回归

- Frozen scenes used:
- Candidate screening timing:
- Relation screen mode:
- Grasp fallback mode:
- Planning successes / total:
- Simulation/task successes / total:
- Difference vs old working version:
- Raw output paths:

## 6. Magnetic 回归

- Old known scenarios used:
- Four-wall result:
- Triangle-roof result:
- Builder JSON used:
- Role/dependency behavior preserved:
- Source retry behavior preserved:
- Partial/full-open + retreat preserved:
- Attached payload collision preserved:
- Planning successes / total:
- Simulation/task successes / total:
- Difference vs old working version:
- Raw output paths:

## 7. PushT GPU / cuRobo2

- Planner environment:
- Contact/tool profile qualified?:
- Full short push preplanned before descent?:
- TCP corridor audit:
- Collision audit:
- `speed_mps` affects trajectory timing?:
- Stale/replayed observation rejection:
- Re-observe if object moved during planning:
- GPU cases successes / total:
- Raw output paths:

## 8. Camera / tracking

- Tracking source:
- Calibration files:
- Fresh timestamp behavior:
- Lost/recovery behavior:
- One observe → push → fresh observe loop:

## 9. RealMan no-motion

- Connection/preflight:
- Joint feedback:
- Stop API:
- Gripper backend:
- API/signature mismatches:
- Did any motion occur? MUST state explicitly.

## 10. Physical motion ladder

For each item use `PASS`, `FAIL`, or `NOT_RUN`.

- Reduced-speed free-space trajectory:
- Isolated gripper:
- One PickPlace atom:
- Multi-object PickPlace:
- One Magnetic piece:
- 2/4/6+ Magnetic structure:
- One PushT short push:
- PushT push → fresh observation:
- PushT closed-loop multi-step:

## 11. Final summary

- PickPlace software state:
- Magnetic software state:
- PushT software state:
- Remaining blockers:
- Files changed by Codex:
- Commits made by Codex:
- Open questions for ChatGPT/user:

Do not replace missing evidence with assumptions. Any unexecuted item must remain `NOT_RUN`.
