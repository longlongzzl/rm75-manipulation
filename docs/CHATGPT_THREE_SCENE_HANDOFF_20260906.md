# RM75 三场景软件收尾与 Codex 本地验收交接

日期：2026-09-07（由 2026-09-06 版本修订）  
分支：`chatgpt/three-scene-software-closeout`  
目标：PickPlace、磁吸积木、PushT 三条真实 RM75 demo 共用新架构、前端、真机边界和验收记录。

## 0. 重要状态

本分支包含：

- 三场景统一前端与工作单服务；
- PickPlace / Jimu 旧工作版本的迁移器和适配器；
- PushT 前端、观测、闭环控制、cuRobo2 短程推送与 RealMan 真机执行边界；
- 统一停止、真机授权、输入确认桥和结果日志；
- CPU 回归测试与示例配置。

**这里没有宣称 GPU、相机或真机已经在 ChatGPT 环境验证。** 所有本地未执行项必须写 `NOT_RUN`。

## 1. 旧工作版本来源：固定基线 + 审计过的 dirty overlay

已核对旧仓库：`longlongzzl/lerobot-realman`。六个已审阅入口在提交：

```text
7aaff9da22486b7d25557b3795dd258f9b65f10d
```

其 blob SHA 与预期全部匹配。

Codex 随后确认用户本地旧目录存在大量未提交修改，其中一些会改变 PickPlace/Jimu 行为。因此最终策略不是“只用固定提交”，也不是“整个 dirty worktree 全部复制”，而是：

```text
7aaff9d committed snapshot
  +
read-only dependency-closure audit
  +
explicit content-addressed worktree overlay
```

规则：

1. **禁止** `git reset --hard`、`git clean`、checkout 覆盖、stash、rebase 或任何会改变旧目录的操作；
2. `7aaff9d` 是可复现的**基线**；
3. dirty tracked 文件若包含最终完成版的重要修复，应作为 `candidate_final_fix` 审计，而不是被丢弃；
4. 只有依赖闭包证明实际运行需要、并且写入 manifest 的文件才能覆盖固定基线；
5. 每个 overlay 文件必须记录 `path + SHA256 + reason`；
6. 约 1.3 万个 untracked 文件绝不能整体复制；仅当运行依赖闭包明确需要某个未跟踪源码/资产时，才可逐个纳入 manifest；
7. overlay 之后必须重新做 compile、依赖完整性、GPU/仿真回归；overlay 本身不代表验证通过。

### 1.1 优先审计的行为相关文件

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

以及上述入口实际 import/动态加载的 `place_rules.py`、`object_specs.py`、assembly planner、magnetic snap、AprilTag、RealMan 和资产文件。

潜在未跟踪目录如 `jimu_builder_frontend/`、`jimu_tasks/`、plate assets、`Demo_Triangle/` **不是自动批准项**；只有依赖闭包证明需要时才逐文件纳入。

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

保存重点入口 diff，但不要修改旧仓库：

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

根据 import、动态加载路径和真实启动命令追依赖闭包，然后生成：

```text
runtime_data/three_scene/approved_worktree_overlay.json
```

格式参照 `configs/workcell/audited_worktree_overlay.example.json`。**不要在原因不清楚时把文件加入 manifest。**

## 2. 拉取并先做软件完整性检查

```bash
git fetch origin
git checkout chatgpt/three-scene-software-closeout
git pull --ff-only origin chatgpt/three-scene-software-closeout
git rev-parse HEAD

PYTHONPATH=. python -m compileall -q rm75_app tools tests/three_scene
PYTHONPATH=. python -m pytest -q tests/three_scene
```

若仍有语法、缺文件、前端 404 等软件完整性问题，先修软件并记录，不要进入 GPU/真机。

## 3. 迁移旧工作快照

### 3.1 先安装固定提交基线

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

迁移器必须：

- 从 Git object 读取固定提交，不读取 dirty 文件字节；
- 验证六个入口 blob SHA；
- 不复制日志、模型权重、字体、失败渲染等无关资产；
- 写 `rm75_app/_vendor/working_snapshot/MIGRATION_MANIFEST.json`；
- 保持旧仓库 `git status` 不变。

### 3.2 若审计确认存在最终版 dirty 修复，再应用显式 overlay

```bash
PYTHONPATH=. python tools/apply_audited_worktree_overlay.py \
  --source-repo "$OLD" \
  --target-repo . \
  --manifest runtime_data/three_scene/approved_worktree_overlay.json

PYTHONPATH=. python tools/migrate_working_sources.py \
  --target-repo . \
  --verify-only
```

要求：

- source HEAD 和 worktree status 哈希必须仍等于审计时；
- 每个文件 SHA256 必须匹配 manifest；
- 禁止多层 overlay；若需要修改批准集合，应删掉**新仓库中的 vendored snapshot**并重新从固定基线构建，绝不改旧仓库；
- overlay 之后 `runtime_gpu_verified=false`、`dependency_completeness_verified=false`，直到本地重新验证。

随后：

```bash
PYTHONPATH=. python -m compileall -q rm75_app/_vendor/working_snapshot
```

并执行无运动的旧 PickPlace/Jimu import/`--help`/preview smoke，证明动态依赖闭包没有缺失。

## 4. 三场景前端

统一入口：

```bash
PYTHONPATH=. python -m rm75_app.web.three_scene_frontend \
  --host 127.0.0.1 --port 7860 \
  --machine-profile /ABS/PATH/TO/machine.json
```

必须检查：

- `/workcell/pickplace`
- `/workcell/magnetic`
- `/workcell/pusht`
- 三个页面均不是 404，CSS/JS 正常；
- 状态轮询；
- preview 请求；
- 原运行时 `input()` 确认显示与回复；
- Stop；
- 同一时间只允许一个任务占用机器人；
- `real` 模式必须经过本地明确授权；
- 浏览器不能提交任意 shell argv。

## 5. PickPlace 回归

目标：**迁移，不重写已经完成的算法。**

```bash
PYTHONPATH=. python tools/run_workcell_task.py \
  --profile /ABS/PATH/TO/machine.json \
  --request examples/workcell/pickplace.preview.json
```

再用原固定 scene 做迁移前/迁移后对比：

- SAM3/SAM6D 对象结果；
- 抓取候选；
- `lazy_place + primary_only`；
- 已有候选筛选速度；
- 完整抓取/放置规划；
- 不允许减少候选、关闭碰撞或放宽成功条件换取通过。

真机在 GPU/模拟回归通过后再做，从单原子开始。

## 6. 磁吸积木回归

磁吸不是普通 PickPlace。必须保留旧版本中的：

- `jimu_builder_scene_v1` / builder 语义；
- floor / wall / roof；
- AprilTag anchor；
- 同类料槽和失败重试；
- grasp-release binding；
- partial open / full open / retreat；
- attached payload collision；
- magnetic snap / capture；
- 原依赖顺序和返回值语义。

前端至少完成：导入已有 builder JSON、保留未知字段、编辑/预览、round-trip、preview/sim/real 请求。

先跑旧版已知 four-wall / triangle-roof 场景，再比较迁移后结果。不要用简化两块积木例子替代复杂回归。

## 7. PushT 本地验证

物理闭环：

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
PYTHONPATH=. python tools/run_workcell_task.py \
  --profile /ABS/PATH/TO/machine.json \
  --request examples/workcell/pusht.preview.json

PYTHONPATH=. python tools/run_workcell_task.py \
  --profile /ABS/PATH/TO/machine.json \
  --request examples/workcell/pusht.sim.json
```

CPU surrogate 只能叫回归，不是物理仿真证据。

### 7.2 GPU / cuRobo2

必须验证：

- 完整短推在下压前全部规划；
- 故意接触 T 时只允许资格确认的 pusher contact links；
- 其他 robot links、桌面、障碍物、自碰撞保持启用；
- 接触段 TCP corridor 和姿态逐点审计；
- `speed_mps` 进入时间参数化；
- 规划期间物体移动触发 re-observe/replan；
- stale / lost / timestamp 不递增立即失败；
- 执行异常不盲目重试。

### 7.3 相机与真机

优先使用当前 PickPlace 同一 RRTrack 链。AprilTag 只作为 bring-up 时必须有明确 `T_base_camera` 和 `T_marker_object`。

真机顺序：no-motion SDK/preflight → reduced-speed free-space → 单短 push → push+fresh observation → 多轮闭环。人工在场、物理急停可用。

## 8. 结果回传

以 `docs/CODEX_THREE_SCENE_RESULTS_TEMPLATE.md` 为模板，保存新的：

```text
docs/CODEX_THREE_SCENE_RESULTS_20260907.md
```

必须包含：实际测试 commit、环境、dirty 依赖审计、固定 snapshot manifest、overlay manifest（若有）、CPU、前端、PickPlace、Magnetic、PushT GPU、相机、no-motion、实机动作、所有 `NOT_RUN`。失败必须进入分母。

## 9. 禁止事项

- 不覆盖/清理旧 `lerobot-realman` dirty worktree；
- 不把 `7aaff9d` 自动说成最终用户版本；
- 不整体复制 dirty/untracked worktree；
- 不在没有 SHA256 + reason + 依赖审计时做 overlay；
- 不重写已完成 PickPlace/Jimu 算法追求架构整洁；
- 不关闭碰撞、自碰撞、freshness 或真机资格检查；
- 不把命令退出码 0 当作实物任务成功；
- 不把 CPU PushT surrogate 叫 ManiSkill/真实世界模型验证；
- 不自动执行完整真机任务。

完成软件完整性、固定基线 + 审计 overlay 迁移、CPU 和无运动 smoke 后先回传，再决定真机动作。
