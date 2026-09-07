# Three-scene R2 — 2026-09-07

按 `CHATGPT_REVIEW_CODEX_PUSH_20260907.md` 的 A → B → C → D 执行。
结论：精简迁移和 CPU 验证通过；三个真实 demo **仍未完成验收**。没有机械臂或夹爪运动。

## 0. 基本信息

- Reviewer head：`160e02c`（本轮拉取成功）。分支：`chatgpt/three-scene-software-closeout`。
- 最终 CPU tested commit：`4a208b367d336c2d0a359397f585ea16b8f8f755`。
- Native 实验在 `160e02c` + 本轮 contact audit 工作树上运行；实际 runner/contact hook 原样提交到 `4a208b3`，逐文件 SHA256 在下方 JSON。不是把前一轮日志当成新结果。
- Native Python：`/home/zhangzhao/anaconda3/envs/foundationpose310/bin/python`，3.10.20；Torch 2.7.1+cu128；NumPy 2.2.6；MPLib 0.1.1；SAPIEN 3.0.2；ManiSkill 3.0.0b22。
- GPU：RTX 5060 Ti；driver 580.173.02；Torch CUDA 12.8；nvcc 12.8.93。
- cuRobo1：`/home/zhangzhao/PycharmProjects/curobo/src/curobo`，HEAD `d64c4b005459db10c5dd867d8b30a87d5bda9bdb`。这是外部运行依赖，不是迁移源基线。
- CPU suite：本机 Anaconda Python 3.12.7/Linux。PushT constructor check 使用 `curobo2/bin/python`；本轮没有重新运行 cuRobo2 GPU 初始化或 FK。
- Robot IP used：NONE。RealMan SDK 在线版本：NOT_RUN。
- 机器证据：[three_scene_closeout_20260907_r2_summary.json](../benchmarks/unified_scenarios/three_scene_closeout_20260907_r2_summary.json)，包含原始日志、编译记录、contact JSONL、失败和未运行项。

## 1. 旧仓库 dirty worktree 审计

- 旧目录 `/home/zhangzhao/Desktop/lerobot`；HEAD `36798efbd12814841607951c9af470b309b34fd3`。
- 固定基线 `7aaff9da22486b7d25557b3795dd258f9b65f10d`；沿用已批准的 12 文件 overlay，未扩大批准集合。
- 逐文件分类及理由沿用 `configs/workcell/approved_worktree_overlay_20260907.json` 和上一轮报告；本轮没有把其他 dirty/untracked 重新判为可纳入。
- 原 status SHA256 和本轮迁移/实验后只读检查一致：`7a7759758ac9b3ed67e54374cdcb10f9837bb3c5507c79bae41348deacb60658`。
- 旧目录修改/覆盖/reset/clean/stash：**NO**。

## 2. A — 精简迁移完整性

只把新仓库的 generated snapshot 移到 `/tmp/rm75_snapshot_pre_r2_160e02c` 作为可恢复备份，随后重新生成；没有删除旧仓库内容。Git 中移除的 174 个旧快照文件也可从 `14ca799` 恢复。

```bash
GIT_OPTIONAL_LOCKS=0 PYTHONPATH=. python tools/migrate_working_sources.py \
  --source-repo /home/zhangzhao/Desktop/lerobot --target-repo . \
  --source-ref 7aaff9da22486b7d25557b3795dd258f9b65f10d
GIT_OPTIONAL_LOCKS=0 PYTHONPATH=. python tools/apply_audited_worktree_overlay.py \
  --source-repo /home/zhangzhao/Desktop/lerobot --target-repo . \
  --manifest configs/workcell/approved_worktree_overlay_20260907.json
PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only
```

- 最终 **807 个 manifest 输入文件**（原 981）；固定输入 803 + 4 个明确批准的新资产；12 项 overlay 中另 8 项是替换。
- Manifest：`rm75_app/_vendor/working_snapshot/MIGRATION_MANIFEST.json`；六入口固定 blob 检查通过，overlay 后 installed SHA256 验证通过，运行后再次 `--verify-only` 通过。
- 实际递归检查目录/文件，而非仅检查选择器：以下前缀匹配数分别为 **0 / 0 / 0 / 0 / 0**：
  `lerobot-sim2real/lerobot_sim2real/rl/`、`lerobot/common/policies/`、`lerobot/common/optim/`、`src/lerobot/common/policies/`、`src/lerobot/common/optim/`。
- 原 PickPlace/Jimu parser/import smoke 均通过，无新增缺失 import；没有恢复任何 RL/policy 树。
- 没有改动 vendored 算法或伪造 manifest GPU/dependency 通过标记。Native 日志仍引用外部 `.maniskill` 机器人资产，以及旧目录 mesh 路径；本轮不宣称完全脱离原机器的依赖闭包。

## 3. 代码完整性与 CPU 测试

命令均为 `PYTHONPATH=. python ...`：

| 阶段 | compileall `-q rm75_app tools tests/three_scene` | `pytest -q tests/three_scene` | `pytest -q tests` |
| --- | --- | --- | --- |
| A 重建后 | PASS（包含 vendor） | 73 passed / 3.24s | 395 passed / 22.04s |
| audit 初版 | PASS | 79 passed / 3.73s | 401 passed / 25.85s |
| 最终 `4a208b3` | PASS | **80 passed / 3.87s** | **402 passed / 22.54s** |

每轮 0 failed / 0 skipped。原 vendor invalid-escape SyntaxWarning 不是新增语法错误，未为消警修改旧源码。
新增 7 测覆盖：关闭前拒绝、桌面遗漏前拒绝、兼容关闭/恢复审计、几何指纹变化、原异常 fallback 不吞严格拒绝、上下文恢复、worker 保留 typed failure 并非零退出。

最终日志：`/tmp/three_scene_r2_{compile,tests,full}_committed.log`；原日志内容收录 JSON。

## 4. 前端

- 本轮复跑完整三场景 CPU suite，包含三 tab HTTP/preview、JS syntax、授权、Stop/input bridge 等已有测试。
- 本轮真实浏览器重新检查、截图、21 件 builder round-trip：**NOT_RUN**。上一轮已有结果见 `CODEX_THREE_SCENE_RESULTS_20260907.md`，不重复冒充本轮执行。
- 原生 prompt → 页面 → 原生完整执行确认：**NOT_RUN**；未开启 `--allow-real`。
- 新修复：worker 原先会把 native `command_success=false` 覆写成 `command_completed_unverified`；现在返回 `status=failed`、`failure_code=strict_contact_not_supported`、非零退出，不进入后验成功确认。

## 5. B — PickPlace 干净扩展构建与崩溃定位

本轮只做 **1 次全新缓存 native frozen trial**，无旧 `.so` 复制。缓存 `/tmp/rm75_native_curobo_clean` 开始时不存在，创建后使用；第一次构建给 900s 上限。

在新 vendored 根目录执行：

```bash
TORCH_EXTENSIONS_DIR=/tmp/rm75_native_curobo_clean \
PYTHONFAULTHANDLER=1 PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 \
timeout 900 /home/zhangzhao/anaconda3/envs/foundationpose310/bin/python -X faulthandler \
  pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py \
  --object-name gluestick --skip-foundationpose \
  --fixed-scene-pose-file pick_jiaobang/test_scenes/gluestick_desk_regression.json \
  --render-mode none --auto-execute \
  --curobo-torch-extensions-dir /tmp/rm75_native_curobo_clean
```

`execute-real` 缺省 false；`auto-execute` 只跳过此原生模拟流程的确认。未实例化真实执行器。

- 五个扩展完成干净 JIT：kinematics_fused、geom、lbfgs_step、line_search、tensor_step。实际观察到 nvcc/ninja 编译；`build.ninja`、`.ninja_log`、产物 SHA256 收录 JSON，二进制未提交。
- 日志：`[prefetch] warmed shared cuRobo planner ... in 172935.2ms`。
- 随后 **exit 139 / SIGSEGV**：`mplib/planner.py:65` 调用 C++ `ArticulatedModel(...)`，上层为迁移 `rm75_jiaobang_pick_move_v10_perpendicular_to_object.py:275` → `create_demo`。
- 崩溃阶段：**cuRobo shared planner 已创建/预热完成；MPLib 场景 planner 构造中；首次场景实际抓放规划之前**。不是本轮 JIT timeout，也不能再仅归因于复制旧扩展。
- 根因仍 **NOT_ESTABLISHED**：MPLib 二进制/模型/环境边界待定位；此次 Python faulthandler 栈不能证明具体 ABI 或 mesh 根因。未取得 C++ gdb 栈。
- 原场景 planning 完成 **0/1**；对象级成功未观测（不计通过）。候选筛选计时、完整抓放链：NOT_RUN，因初始化崩溃。
- 未改 candidates、collision、tolerance、lazy_place/primary_only 生产设置，未增加 reverse/额外 fallback。原算法未重设计。
- 原日志 `/tmp/three_scene_r2_pick_clean.log` 完整收录 JSON。

## 6. C — Magnetic strict-contact 与兼容审计

新增 `rm75_app/workcell/contact_audit.py` 是新适配器边界，不修改 hash-verified native snapshot。工作台 Magnetic 默认安装严格模式；原入口保持兼容行为。审计 runner 只提供已知场景固定 argv，不接收任意 shell 参数。

```bash
PYTHONFAULTHANDLER=1 PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 \
timeout 240 /home/zhangzhao/anaconda3/envs/foundationpose310/bin/python \
  tools/run_native_contact_audit.py --scene four-wall \
  --extensions /tmp/rm75_native_curobo_clean \
  --output runtime_data/three_scene/r2_four_wall_strict
```

- **严格 trial：exit 42，`strict_contact_not_supported`，command_success=false，verified_task_success=null。**
- 第一块 `right_wall`，step `winner_chain_jimu_parallel_grasp_place_paired_relation_ik` 请求关闭 `gripper_Left_Support_Link`、`gripper_Right_Support_Link` 世界碰撞，在 mutation 之前拒绝；changed_links=[]。
- 当时完整 world objects/pose/dims、已有 disabled object 状态和 scene fingerprint 收录 JSON；fingerprint `838ec31493c4a61f108580b8d3e9c6cd2395af3844e5f18dea17910193ef22ee`。
- 这比最终接近重试更早；因此该 trial 的“最终重试是否必需”是 **null / 未到达**，不伪造 true。
- 静态检查还确认最终接近会调用 `include_table=False`。严格 hook 在该阶段 world refresh 之前拒绝；单测覆盖。但此次原生严格 trial 在更早的配对 IK 已停止，不声称实际到达桌面拦截点。
- 当前 cuRobo1 adapter 的 toggle 是 link 对整个世界，不是 link↔目标/支撑 pair。没有实现或宣称安全的 pair-local 例外；此边界只是失败出口，不是对全部旧规划阶段的安全认证。

另作 **1 次兼容证据 trial**：上述 runner 加 `--compatibility-audit`，output 为 `runtime_data/three_scene/r2_four_wall_compatibility`，timeout 180s。退出 0；旧程序 `final success=True`，仅为 command-level 兼容结果，不是 strict 或物体成功。

- 120 条审计记录；四块 right/back/left/front wall 各出现 1 次 final `gripper_world_relaxed`，共 **4 次**。原 constrained attempt 返回 None 后才进入这些 retry。
- 每次记录 piece/step、当前场景 SHA256 指纹、物体几何和姿态、请求/实际修改 links、启用/恢复、原失败前提。关闭后与恢复后的 world 状态也记录。
- `permitted_contact_target` 使用运行时当前对象；目标支撑没有确认到可资格化的 pair，所以 `permitted_contact_support=null`，不猜测支撑授权。
- 原日志位置 255 / 684 / 1156 / 1627 均明确“retrying with gripper world collision relaxed”。这 4 次不能计作严格接触成功。
- 严格 case：**0/1 通过**；兼容 case：command **1/1**，strict acceptance **0/1（未资格化）**。两次尝试均进入记录；实物成功均未知。
- **triangle-roof：NOT_RUN**，four-wall 严格前置失败，后续依赖步骤被阻塞，没有跳过墙去跑屋顶。
- 原 builder、角色、料槽重试、抓取/释放绑定、partial/full-open、retreat、attached payload、magnetic capture 算法不变；本轮没有认证其物理效果。

原日志 `/tmp/three_scene_r2_four_wall_{strict,compatibility}.log`，两次 result 和全部 contact JSONL 均收录 JSON。

## 7. D — PushT 资格字段与 GPU

只读核查 `runtime_data/three_scene/machine.json` 和提供的 configs；未发现可确认的实际工具/工作台资格配置。**integration_qualified=false 保持不变。**

| 必需项 | 当前值／资格结论 |
| --- | --- |
| push_tcp_z_m | null，未确认 |
| 工具姿态 / T_tcp_pusher | tool_quaternion_wxyz=null；无已确认 T_tcp_pusher |
| permitted contact links | []，未确认 |
| pusher collision geometry | tool_collision_geometry_verified=false |
| workspace / table | workspace=[0.15,0.65,-0.3,0.3] 仅 CPU 示例；static_collision_objects=[]，无合格桌面 |
| T geometry dimensions | bar 0.10×0.03、stem 0.03×0.07 仅 CPU 示例；object_height/centroid_z=null，非实测资格 |
| tracking source / T_base_camera | realsense_apriltag 仅配置种类；T_base_camera=null；现场实时来源未验证 |
| T_marker_object / tag | T_marker_object=null，marker_size_m=null；marker_id=0 不是资格证明 |
| Cartesian speed_mps | 0.015 m/s 仅示例上限，未做工具/现场速度确认 |

用 curobo2 Python 对真实 `CuroboPushExecutor` 构造器传入现有 profile，backend/arm/observer 均 None：得到 `ValueError: push_tcp_z_m must be a number`。没有创建 planner、相机或机械臂连接。

- GPU pre-contact → contact → short push → retract 完整预规划：**NOT_RUN**，资格前置缺失。
- 逐 waypoint TCP/姿态/碰撞审计、速度时间参数化的本轮 GPU 实证：**NOT_RUN**；CPU mocks/已有单测不替代该证据。
- stale/replayed/moved-target CPU 回归包含在全量 suite；本轮 live/GPU 验证 **NOT_RUN**。
- GPU complete-chain 成功数 **0 executed**，不是 0/0 PASS；不得用 CPU surrogate 或前轮 FK 冒充。

## 8. Camera / tracking

现场采集、fresh timestamp、lost/recovery、observe → push → fresh observe：全部 **NOT_RUN**。
已有 PickPlace camera extrinsic 没有自动挪作 PushT 工具/标签资格；没有修改现场标定文件。

## 9. RealMan no-motion

SDK 连接/preflight、joint feedback、Stop API、gripper backend、在线 API/signature 检查：全部 **NOT_RUN**。
Robot IP 仍占位，不连接、不调用运动接口。机械臂运动 **NO**；夹爪运动 **NO**。

## 10. Physical motion ladder

以下逐项均 **NOT_RUN**：reduced-speed free-space；isolated gripper；one PickPlace atom；multi-object PickPlace；one Magnetic piece；2/4/6+ structure；one PushT short push；push → fresh observation；PushT closed-loop multi-step。

## 11. 总结与后续边界

- A 完成：807 文件快照、禁止前缀零匹配、import/compile/全部 CPU 通过。
- B 完成诊断实验但未修复崩溃：干净构建后依旧 MPLib ArticulatedModel SIGSEGV；需后续隔离 MPLib/模型/运行环境，不能靠减少候选换绿。
- C 完成审计与 typed strict rejection：兼容路径保留，严格 four-wall 未通过，triangle-roof 依赖阻塞。需确认安全的 pair-local 能力及桌面/支撑资格后才重开验收。
- D 完成现有字段核查，但实际资格数据缺失，**不能完成资格填充或完整 GPU 推链**。需要用户/现场提供实测或确认的工具、几何、工作台、相机/标签变换与速度边界。
- 代码提交 `4a208b3`；本报告及机器 JSON 随后单独提交并 push。除了新仓库生成快照排除项，改动仅 contact audit/runner、worker 失败传递及 7 项单测；没有改旧仓库或重设计 PickPlace/Jimu 算法。
- 本轮停在 no-motion 边界，不发起机械臂运动。上述失败不能标成三场景真实 demo 完成。
