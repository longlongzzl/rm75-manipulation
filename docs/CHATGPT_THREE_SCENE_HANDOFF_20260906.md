# RM75 三场景软件收尾与 Codex 本地验收交接

日期：2026-09-06  
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

## 1. 旧工作版本的来源与 dirty worktree 处理

已核对旧仓库：`longlongzzl/lerobot-realman`。六个已审阅入口在提交：

```text
7aaff9da22486b7d25557b3795dd258f9b65f10d
```

其 blob SHA 与预期全部匹配。

但是用户本地旧目录存在大量未提交修改。因此执行策略是：

1. **禁止** `git reset --hard`、`git clean`、checkout 覆盖、stash pop 或任何会改变旧目录的操作；
2. `7aaff9d` 是**可复现基线**，不是自动宣称“用户最终完成版”；
3. 在正式迁入前，对 dirty worktree 做**只读审计**；
4. 若 dirty diff 只含日志、路径、本机参数、临时调试或与三场景无关文件，可继续按 `7aaff9d` 迁移；
5. 若 dirty diff 修改了下列已完成算法或其直接依赖，则不要擅自丢弃，也不要自动混入：记录差异，分类为 `candidate_final_fix`，先在回传 MD 中列出，等待 ChatGPT/用户确认后再纳入。

重点审计：

```text
pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py
pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place_sam6d.py
pick_jiaobang/rm75_jiaobang_pick_real_with_foundationpose.py
Beta_demo-codex-v0.9/rm75_jimu_four_wall_direct_sam6d.py
Beta_demo-codex-v0.9/rm75_jimu_four_wall_portable.py
Beta_demo-codex-v0.9/rm75_jimu_triangle_roof_apriltag_portable.py
```

以及这些入口直接 import 的 `place_rules.py`、`object_specs.py`、`curobo_rm75_planner.py`、assembly planner / magnetic snap / AprilTag 相关文件。

### 1.1 只读审计命令

在旧仓库根目录：

```bash
git rev-parse HEAD
git status --short
git diff --stat 7aaff9da22486b7d25557b3795dd258f9b65f10d
git diff --name-status 7aaff9da22486b7d25557b3795dd258f9b65f10d
git ls-files --others --exclude-standard
```

对上述重点文件逐个保存 diff：

```bash
mkdir -p /tmp/rm75_dirty_audit
git diff 7aaff9da22486b7d25557b3795dd258f9b65f10d -- \
  pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py \
  pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place_sam6d.py \
  pick_jiaobang/rm75_jiaobang_pick_real_with_foundationpose.py \
  Beta_demo-codex-v0.9/rm75_jimu_four_wall_direct_sam6d.py \
  Beta_demo-codex-v0.9/rm75_jimu_four_wall_portable.py \
  Beta_demo-codex-v0.9/rm75_jimu_triangle_roof_apriltag_portable.py \
  > /tmp/rm75_dirty_audit/core_entrypoints.patch
```

**不要修改旧仓库。**

## 2. 拉取本分支并先做完整性检查

```bash
git fetch origin
git checkout chatgpt/three-scene-software-closeout
git pull --ff-only origin chatgpt/three-scene-software-closeout
git rev-parse HEAD
```

然后：

```bash
PYTHONPATH=. python -m compileall -q rm75_app tools tests/three_scene
PYTHONPATH=. python -m pytest -q tests/three_scene
```

如果这里仍出现语法截断，停止并回传具体文件和行号，不要自己猜补。

## 3. 迁移旧工作快照

先确保第 1 节 dirty 审计已经完成。

如果 dirty 审计没有发现必须纳入的最终算法修复，则以固定提交迁入：

```bash
PYTHONPATH=. python tools/migrate_working_snapshot.py \
  --source-repo /home/zhangzhao/Desktop/lerobot \
  --source-ref 7aaff9da22486b7d25557b3795dd258f9b65f10d
```

若本机旧仓库路径不同，只改 `--source-repo`，不要改 source ref。

迁移必须满足：

- 从 Git object 读取固定提交，不读取 dirty worktree 文件内容；
- 不复制模型权重、日志、字体、失败渲染等非必要资产；
- 生成 `rm75_app/_vendor/working_snapshot/MIGRATION_MANIFEST.json`；
- 校验预期入口 blob SHA；
- 旧仓库工作区前后 `git status --short` 完全相同。

随后：

```bash
PYTHONPATH=. python tools/migrate_working_snapshot.py --verify-only
```

## 4. 三场景前端

统一入口：

```bash
PYTHONPATH=. python -m rm75_app.web.three_scene_frontend \
  --host 127.0.0.1 --port 7860 \
  --machine-profile /ABS/PATH/TO/machine.json
```

必须检查三页：

- `/workcell/pickplace`
- `/workcell/magnetic`
- `/workcell/pusht`

以及：

- 页面状态轮询；
- preview 请求；
- 原运行时 `input()` 确认显示与回复；
- Stop 按钮；
- 同一时间只允许一个任务占用机器人；
- `real` 模式必须经过本地明确授权；
- 浏览器不能提交任意 shell argv。

## 5. PickPlace 回归

目标：**迁移，不重写已经完成的算法。**

先做：

```bash
PYTHONPATH=. python tools/run_workcell_task.py \
  --profile /ABS/PATH/TO/machine.json \
  --request examples/workcell/pickplace.preview.json
```

再用原固定 scene 做迁移前/迁移后对比：

- SAM3/SAM6D 对象结果；
- 抓取候选；
- `lazy_place + primary_only`；
- 已有候选筛选速度指标；
- 完整抓取/放置规划；
- 不允许通过减少候选、关闭碰撞或放宽成功条件换取通过。

真机测试在 GPU/模拟回归通过后再做，且从单原子开始。

## 6. 磁吸积木回归

磁吸不是普通 PickPlace。必须保留旧版本中的：

- `jimu_builder_scene_v1` / 现有 builder 语义；
- floor / wall / roof 角色；
- AprilTag anchor；
- 同类料槽和失败重试；
- grasp-release binding；
- partial open / full open / retreat；
- attached payload collision；
- magnetic snap / capture 相关逻辑；
- 原依赖顺序和返回值语义。

前端至少完成：

- 导入已有 builder JSON；
- 保留未知字段；
- 编辑 / 预览；
- 导出后 round-trip 不破坏原数据；
- preview / sim / real 请求。

先跑旧版已知 four-wall / triangle-roof 场景，再比较迁移后结果。不要用一个简化的两块积木例子替代原复杂场景回归。

## 7. PushT 本地验证

PushT 物理定义：

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

- 完整短推在下压前全部规划完成；
- 故意接触 T 时只允许经过本地资格确认的 pusher contact links；
- 其他机器人 link、桌面、障碍物、自碰撞保持启用；
- 接触段 TCP corridor 和姿态误差逐点审计；
- `speed_mps` 实际进入时间参数化；
- 规划期间物体移动会触发 re-observe/replan；
- stale / lost / timestamp 不递增立即失败；
- 执行异常后不盲目重试同一个 push。

### 7.3 相机

优先使用与当前 PickPlace 一致的实时跟踪链。若暂时用 AprilTag bring-up，必须使用明确的 `T_base_camera` 和 `T_marker_object`，禁止假设 tag 中心就是 T 中心。

### 7.4 真机

先：

1. no-motion SDK/preflight；
2. reduced-speed free-space trajectory；
3. 单个短 push；
4. push → fresh observation；
5. 多轮闭环。

全程人工在场、物理急停可用。

## 8. 结果回传

填写：

```text
docs/CODEX_THREE_SCENE_RESULTS_TEMPLATE.md
```

另存为：

```text
docs/CODEX_THREE_SCENE_RESULTS_20260906.md
```

必须包含：

- 实际测试 commit；
- Python/CUDA/cuRobo/ManiSkill/RealMan SDK 环境；
- dirty worktree 审计结论；
- snapshot 迁移 SHA/manifest；
- CPU 测试；
- 三页前端；
- PickPlace；
- Magnetic；
- PushT GPU；
- 相机；
- no-motion 真机；
- 实机动作测试；
- 每个未运行项明确 `NOT_RUN`；
- 失败必须进入分母，不能只列成功案例。

## 9. 禁止事项

- 不覆盖/清理旧 `lerobot-realman` dirty worktree；
- 不把 `7aaff9d` 自动说成最终用户版本；
- 不在没有审计时把 dirty 修改混进迁移快照；
- 不重写已完成 PickPlace/Jimu 算法来追求“架构更漂亮”；
- 不关闭碰撞、自碰撞、freshness 或真机资格检查；
- 不把命令执行成功当作实物任务成功；
- 不把 CPU PushT surrogate 叫 ManiSkill/真实世界模型验证；
- 不自动执行完整真机任务。

完成 CPU + 旧版本迁移回归 + no-motion 检查后先回传，再决定是否启用真机动作。
