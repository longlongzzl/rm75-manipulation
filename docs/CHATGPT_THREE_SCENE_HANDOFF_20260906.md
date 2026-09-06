# RM75 三场景软件收尾与 Codex 本地验收交接

日期：2026-09-07（由 2026-09-06 版本修订）  
分支：`chatgpt/three-scene-software-closeout`  
目标：PickPlace、磁吸积木、PushT 三条真实 RM75 demo 共用新架构、前端、真机边界和验收记录。

## 0. 重要状态

本分支包含三场景统一工作台、PickPlace/Jimu 工作版本迁移器与适配器、PushT 闭环控制与 RM75/cuRobo2 真机边界、停止/授权/input 确认桥和 CPU 回归测试。

**没有宣称 GPU、相机或真机已经在 ChatGPT 环境验证。** 本地未执行项必须写 `NOT_RUN`。

## 1. 旧工作版本来源：固定基线 + 审计过的 dirty overlay

旧仓库：`longlongzzl/lerobot-realman`。可复现基线：

```text
7aaff9da22486b7d25557b3795dd258f9b65f10d
```

六个已审阅入口 blob SHA 与该提交匹配。但 Codex 已确认本机旧目录存在会改变 PickPlace/Jimu 行为的后续 dirty 修改，因此最终迁移策略为：

```text
7aaff9d committed snapshot
  + read-only dependency-closure audit
  + explicit SHA256-addressed approved worktree overlay
  + rerun software/GPU/simulation validation
```

规则：

1. 禁止 `git reset --hard`、`git clean`、checkout 覆盖、stash、rebase 或修改旧目录；
2. `7aaff9d` 是可复现基线，不自动等于最终用户版本；
3. dirty tracked 文件若含最终完成版的重要修复，标记 `candidate_final_fix` 并做依赖闭包审计；
4. 只有明确批准的文件才能覆盖固定基线，每个文件必须记录 `path + SHA256 + reason`；
5. 约 1.3 万个 untracked 文件绝不能整体复制；只有依赖闭包明确需要的单个源码/资产才能逐文件纳入；
6. overlay 后必须重新 compile、依赖完整性、GPU/仿真回归；overlay 本身不代表通过。

### 1.1 优先审计文件

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

以及实际 import/动态加载的 `place_rules.py`、`object_specs.py`、assembly planner、magnetic snap、AprilTag、RealMan 和资产文件。

潜在未跟踪目录 `jimu_builder_frontend/`、`jimu_tasks/`、plate assets、`Demo_Triangle/` 不是自动批准项；只有依赖闭包证明需要时才逐文件纳入。

### 1.2 只读审计

```bash
OLD=/home/zhangzhao/Desktop/lerobot
BASE=7aaff9da22486b7d25557b3795dd258f9b65f10d
mkdir -p runtime_data/three_scene

git -C "$OLD" rev-parse HEAD
git -C "$OLD" status --short
git -C "$OLD" diff --stat "$BASE"
git -C "$OLD" diff --name-status "$BASE"
git -C "$OLD" ls-files --others --exclude-standard

git -C "$OLD" status --porcelain=v1 -z > runtime_data/three_scene/old_worktree_status.bin
sha256sum runtime_data/three_scene/old_worktree_status.bin
```

保存重点 diff：

```bash
git -C "$OLD" diff "$BASE" -- \
  pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py \
  pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place_sam6d.py \
  pick_jiaobang/rm75_jiaobang_pick_real_with_foundationpose.py \
  pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo.py \
  pick_jiaobang/curobo_rm75_planner.py \
  Beta_demo-codex-v0.9/curobo_rm75_planner.py \
  Beta_demo-codex-v0.9/jimu_sam6d_pose_provider.py \
  Beta_demo-codex-v0.9/rm75_jimu_four_wall_direct_sam6d.py \
  Beta_demo-codex-v0.9/rm75_jimu_four_wall_portable.py \
  Beta_demo-codex-v0.9/rm75_jimu_triangle_roof_apriltag_portable.py \
  lerobot/common/robots/realman_lerobot/realman_arm.py \
  FoundationPose/run_demo_scale.py \
  > runtime_data/three_scene/priority_dirty.patch
```

按实际 imports、动态加载路径和真实启动命令追依赖闭包，生成：

```text
runtime_data/three_scene/approved_worktree_overlay.json
```

格式参照 `configs/workcell/audited_worktree_overlay.example.json`。原因不清楚的文件不要加入。

## 2. 拉取并先做软件完整性检查

```bash
git fetch origin
git checkout chatgpt/three-scene-software-closeout
git pull --ff-only origin chatgpt/three-scene-software-closeout
git rev-parse HEAD

PYTHONPATH=. python -m compileall -q rm75_app tools tests/three_scene
PYTHONPATH=. python -m pytest -q tests/three_scene
```

若仍有语法、缺文件、前端 404 等软件问题，先修软件并记录，不进入 GPU/真机。

## 3. 迁移旧工作快照

### 3.1 固定提交基线

```bash
OLD=/home/zhangzhao/Desktop/lerobot
BASE=7aaff9da22486b7d25557b3795dd258f9b65f10d

PYTHONPATH=. python tools/migrate_working_sources.py \
  --source-repo "$OLD" \
  --target-repo . \
  --source-ref "$BASE"

PYTHONPATH=. python tools/migrate_working_sources.py \
  --target-repo . \
  --verify-only
```

必须从 Git object 读取固定提交、验证关键 blob、不复制日志/权重/字体/失败渲染、写 `rm75_app/_vendor/working_snapshot/MIGRATION_MANIFEST.json`，且旧仓库 status 不变。

### 3.2 审计确认的最终版 dirty 修复

```bash
PYTHONPATH=. python tools/apply_audited_worktree_overlay.py \
  --source-repo "$OLD" \
  --target-repo . \
  --manifest runtime_data/three_scene/approved_worktree_overlay.json

PYTHONPATH=. python tools/migrate_working_sources.py \
  --target-repo . \
  --verify-only

PYTHONPATH=. python -m compileall -q rm75_app/_vendor/working_snapshot
```

source HEAD、worktree status SHA256 和每个文件 SHA256 都必须与审计 manifest 一致。禁止多层 overlay；若批准集合改变，只重建**新仓库** vendored snapshot，绝不改旧仓库。

完成后做无运动的旧 PickPlace/Jimu import、`--help` 或 preview smoke，验证动态依赖闭包。

## 4. 三场景前端：实际入口

本分支真正的入口是 `rm75_app.workcell`，不是 `rm75_app.web.three_scene_frontend`。

```bash
PYTHONPATH=. python -m rm75_app.workcell serve \
  --profile /ABS/PATH/TO/machine.json \
  --port 7861
```

打开：

```text
http://127.0.0.1:7861/workcell/
```

同一页面内必须检查三个 tab：PickPlace、磁吸积木、PushT。还要检查 CSS/JS、状态轮询、preview、原运行时 `input()` 确认、Stop、单任务互斥、real 授权以及浏览器不能提交任意 shell argv。

本阶段不要加 `--allow-real`。只有 no-motion 与软件/GPU 验证后，且用户明确要进入实机阶梯时才启用 real 权限。

## 5. PickPlace 回归：实际 CLI

目标：迁移，不重写已完成算法。

```bash
PYTHONPATH=. python -m rm75_app.workcell run \
  --profile /ABS/PATH/TO/machine.json \
  --request examples/workcell/pickplace_preview.json
```

再用原固定 scene 做迁移前/后对比：SAM3/SAM6D、抓取候选、`lazy_place + primary_only`、候选筛选速度、完整抓取/放置规划。禁止减少候选、关闭碰撞或放宽成功条件。

真机在 GPU/模拟回归通过后从单原子开始。

## 6. 磁吸积木回归

必须保留 `jimu_builder_scene_v1`、floor/wall/roof、AprilTag anchor、同类料槽重试、grasp-release binding、partial/full open、retreat、attached payload collision、magnetic snap/capture、原依赖顺序和返回值语义。

前端需验证已有 builder JSON 导入、未知字段保留、编辑/预览、round-trip、preview/sim/real 请求。

最小 preview CLI：

```bash
PYTHONPATH=. python -m rm75_app.workcell run \
  --profile /ABS/PATH/TO/machine.json \
  --request examples/workcell/magnetic_preview.json
```

动态回归必须使用旧版已知 four-wall / triangle-roof 场景，不得用简化两块积木替代复杂回归。

## 7. PushT 本地验证

闭环定义：

```text
fresh real observation
 -> finite-horizon candidate pushes
 -> choose one short push
 -> plan complete approach/contact/push/retract chain
 -> audit
 -> re-observe before movement
 -> execute RM75
 -> fresh observation
 -> replan
```

### 7.1 CPU

```bash
PYTHONPATH=. python -m rm75_app.workcell run \
  --profile /ABS/PATH/TO/machine.json \
  --request examples/workcell/pusht_preview.json

PYTHONPATH=. python -m rm75_app.workcell run \
  --profile /ABS/PATH/TO/machine.json \
  --request examples/workcell/pusht_sim.json
```

CPU surrogate 只能叫回归，不是 ManiSkill/真实物理证据。

### 7.2 GPU / cuRobo2

验证：完整短推在下压前全部规划；故意接触 T 时仅资格确认 pusher contact links 放行；其他 links/桌面/障碍物/自碰撞保持启用；TCP corridor/姿态逐点审计；`speed_mps` 进入时间参数化；规划期间目标移动触发 re-observe/replan；stale/lost/non-monotonic timestamp 失败；执行异常不盲重试。

### 7.3 相机与真机

优先复用当前 PickPlace 的 RRTrack 链。AprilTag 只作 bring-up 时必须有明确 `T_base_camera` 与 `T_marker_object`。

真机顺序：no-motion SDK/preflight → reduced-speed free-space → 单短 push → push+fresh observation → 多轮闭环。人工在场、物理急停可用。

## 8. 结果回传

以 `docs/CODEX_THREE_SCENE_RESULTS_TEMPLATE.md` 为模板，新建：

```text
docs/CODEX_THREE_SCENE_RESULTS_20260907.md
```

记录实际 commit、环境、dirty 依赖审计、固定 snapshot manifest、overlay manifest（若有）、CPU、前端、PickPlace、Magnetic、PushT GPU、相机、no-motion、实机动作和所有 `NOT_RUN`。失败必须进入分母。

## 9. 禁止事项

- 不覆盖/清理旧 `lerobot-realman` dirty worktree；
- 不把 `7aaff9d` 自动说成最终用户版；
- 不整体复制 dirty/untracked worktree；
- 不在没有 SHA256 + reason + 依赖审计时 overlay；
- 不重写已完成 PickPlace/Jimu 算法追求架构整洁；
- 不关闭碰撞、自碰撞、freshness 或真机资格检查；
- 不把进程退出码 0 当作实物任务成功；
- 不把 CPU PushT surrogate 叫 ManiSkill/真实世界模型验证；
- 不自动执行完整真机任务。

完成软件完整性、固定基线 + 审计 overlay、CPU 和无运动 smoke 后先回传，再决定真机动作。
