# Codex 三场景本地验证回传 — 2026-09-07

按结果模板填写。**NEEDS_REVIEW：软件与本机迁移完成，严格 GPU/物理任务验收未完成。没有连接机器人、没有机械臂或夹爪运动。**

首轮 71 项 CPU 测试通过，但实际浏览器发现 JS 截断；修复后最终 three_scene 73 passed，全量 395 passed。原工作版本已实际迁入仓库，不是 sibling 链接。四墙原程序报告 4/4 cycle 成功，但使用了基线自带的碰撞放宽；不能以此宣布严格碰撞门通过。屋顶程序发现同类分支后停止。PickPlace native 运行仍有环境级失败。PushT CPU 与 GPU 初始化/FK 已跑，完整短推因未标定 profile 未运行。

机器结果：[three_scene_closeout_20260907_summary.json](../benchmarks/unified_scenarios/three_scene_closeout_20260907_summary.json)。其中保留每次原运行退出码、原始 cycle 标记、完整 profile rows、smoke 报告、browser 报告、CLI 结果及原始日志 SHA256。不要把 native 的 success、CPU surrogate success 或进程退出 0 当作真实物体成功。

## 0. 基本信息

- Commit tested: 拉取后的精确父提交 `7b55c4b8ade0a7213abc69adbeb94095dbbd3361`；最终测试包含本回传提交内的适配器/前端/测试修复及安装后的 snapshot。
- Branch: `chatgpt/three-scene-software-closeout`；开始时工作区干净。未合并 main。
- Python: CPU `/home/zhangzhao/anaconda3/bin/python`，3.12.7；原运行环境 `/home/zhangzhao/anaconda3/envs/foundationpose310/bin/python`，3.10；cuRobo2 `/home/zhangzhao/anaconda3/envs/curobo2/bin/python`，3.11.15。
- OS: Linux x86_64，内核 6.8.0-137-generic。
- CUDA / driver: RTX 5060 Ti，driver 580.173.02；cuRobo2 torch 2.11.0+cu128 / CUDA 12.8。原 cuRobo 从 `/home/zhangzhao/PycharmProjects/curobo/src` 加载，不能与 cuRobo2 当成同一后端。
- ManiSkill: 原运行使用 foundationpose310 的原依赖，并加载 migrated portable ManiSkill task；没有宣称单独完成新的全物理安全门。
- RealMan SDK: 本轮未加载/连接硬件 SDK 做实机资格验证，**NOT_RUN**。
- Robot IP used: **NONE**。
- 本机 profile: `runtime_data/three_scene/machine.json`，由生成器创建，hardware_reviewed=false，三项 integration_qualified=false。未伪填 IP、接触 link 资格、推头姿态或外参。

## 1. 旧仓库 dirty worktree 审计

- Old repo path: `/home/zhangzhao/Desktop/lerobot`，已核对 origin 为 `longlongzzl/lerobot-realman`。
- Old repo HEAD: `36798efbd12814841607951c9af470b309b34fd3`。
- Reproducible baseline ref: `7aaff9da22486b7d25557b3795dd258f9b65f10d`。
- `git status --porcelain=v1 -z` before/after: **字节完全相同**，SHA256 `7a7759758ac9b3ed67e54374cdcb10f9837bb3c5507c79bae41348deacb60658`。
- 本轮只读审计续用上一轮逐文件分类，并核对当前 hash、静态 import、动态 task 路径、实际旧启动说明。原状态文件位于 `runtime_data/three_scene/old_worktree_status.bin` / `old_worktree_status_after.bin`；重点 diff 为同目录 `priority_dirty.patch`。
- Was any dirty file modified/overwritten? **NO**。没有 reset/clean/checkout/stash，没有执行旧目录程序，也没有写旧 CUDA cache。
- Recommendation: 已按用户本轮授权完成 fixed + audited overlay；**不把 overlay 视为最终算法正确性证明**。未整体复制 1.3 万项 untracked。

### 本轮批准集合与闭包理由

逐文件原 SHA256、reason、source HEAD/status hash 已存 [approved_worktree_overlay_20260907.json](../configs/workcell/approved_worktree_overlay_20260907.json)，与实际应用的 runtime manifest 相同。

| 集合 | 数量 | 依赖依据 / 保留行为 |
| --- | --- | --- |
| PickPlace direct、FoundationPose base、curobo wrapper、PickPlace planner | 4 tracked dirty | SAM6D → direct → targeted/base/wrapper/planner；保留当前 lift/transport、clearance/return 预验证、q_sent 返回及相应配置/cache key |
| Jimu portable、triangle-roof、pose provider | 3 tracked dirty | triangle → portable/provider → PickPlace 链；保留 builder frame、半板对称/候选、同类料槽重试、partial release/retreat 和 AprilTag 角点约定 |
| RealManArm 旧共享封装 | 1 tracked dirty | base executor 的 real_robot 配置依赖；只恢复源码，不连接硬件 |
| full/half plate GLB 及各自 COACD PLY | 4 单独 untracked 资产 | portable 的 DEFAULT_SIM_ASSET_FILE / DEFAULT_HALF_SIM_ASSET_FILE 显式引用；固定提交缺少这些几何文件；逐文件 hash 纳入，不再生成简化碰撞模型 |

未纳入：Jimu 目录同名 planner 的 dirty 版本（正确原 import 链实际选择 PickPlace planner）；`FoundationPose/run_demo_scale.py`（本次 SAM6D/固定输入路径不调用它，原外部模型安装仍独立）；大批 builder frontend、Demo_Triangle、任务/诊断目录。未改变的入口及 place_rules/object_specs/assembly/magnetic 依赖由固定基线保留。

旧 `run_demo_triangle_github_safe_full_roof.sh` 实际走另一 Demo_Triangle 流程，并带减少候选/IK seeds 的参数；本轮**没有拿它替代指定 portable 回归，也没有继承其缩减参数**。前端使用的 21 件 builder JSON 仅作为单个只读测试输入，不整体纳入其目录。

## 2. 迁移完整性

- Migration command: 下列命令均实际执行。

```bash
PYTHONPATH=. python tools/migrate_working_sources.py --source-repo /home/zhangzhao/Desktop/lerobot --target-repo . --source-ref 7aaff9da22486b7d25557b3795dd258f9b65f10d
PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only
GIT_OPTIONAL_LOCKS=0 PYTHONPATH=. python tools/apply_audited_worktree_overlay.py --source-repo /home/zhangzhao/Desktop/lerobot --target-repo . --manifest runtime_data/three_scene/approved_worktree_overlay.json
PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only
PYTHONPATH=. python -m compileall -q rm75_app/_vendor/working_snapshot
```

- Snapshot manifest path: [MIGRATION_MANIFEST.json](../rm75_app/_vendor/working_snapshot/MIGRATION_MANIFEST.json)。固定基线 **977 files**；overlay **12 files**（8 替换 + 4 新增）；最终 **981 files**，约 182 MB 原输入，逐 manifest 条目纳入 Git，不包含运行日志/JIT cache/失败渲染。
- Source commit recorded: `7aaff9d…`，overlay 另记录旧 HEAD、状态 hash、各文件内容 hash，不伪造 Git blob。
- Six entrypoint blob checks: 固定基线 **6/6 匹配**。
- `--verify-only`: 基线、overlay 后和动态运行后均 **PASS**。Vendor compile **PASS**，仅保留两个原文件 invalid escape SyntaxWarning，未修改原源码消警告。
- Old repo status unchanged after migration: **YES**，动态运行后复查也相同。
- 只做规定的已知旧根路径 relocation；没有改原候选、碰撞集合、容差、成功条件。snapshot 仍是绝对安装路径绑定，移动仓库需重新迁移；不能靠改 manifest 骗过校验。
- `dependency_completeness_verified` / `runtime_gpu_verified` 仍为 **false**，因为动态闭包/严格回归并未全部通过。

### 无运动原入口 smoke

```bash
python tools/create_workcell_profile.py --output runtime_data/three_scene/machine.json
PYTHONPATH=. PYTHONDONTWRITEBYTECODE=1 python tools/check_working_entrypoints.py --task pickplace --profile runtime_data/three_scene/machine.json --request examples/workcell/pickplace_preview.json --output runtime_data/three_scene/pickplace_native_contract.json
PYTHONPATH=. PYTHONDONTWRITEBYTECODE=1 python tools/check_working_entrypoints.py --task magnetic --profile runtime_data/three_scene/machine.json --request examples/workcell/magnetic_preview.json --output runtime_data/three_scene/magnetic_native_contract.json
```

PickPlace 首次 PASS。Magnetic 首次 FAIL：适配器把 pick_jiaobang 提前放入 sys.path 后，portable 不再自行 prepend，Jimu 同名 planner 抢先被导入，缺 `DEFAULT_CUDA_ARCH_LIST`。仅修改 `workcell/legacy.py` 的入口路径装配，恢复原 script 启动顺序；新增同名依赖回归。重跑 Magnetic **PASS**，两项 missing_required_flags 均为空。没有调用原 main、planner、相机或 robot executor；因此这是 import/parser/argv smoke，不是物理任务通过。

### 原环境扩展与外参路径

原环境首次 JIT 启动阻塞，PickPlace 和 four-wall 分别 timeout 180 s。为复用原已安装环境，新增 `tools/stage_native_runtime_extensions.py`，仅只读复制明确列出的五个 cuRobo `.so` 到 `/tmp/rm75_native_extensions_20260907`，逐文件 SHA256 相同；没有复制 build/lock/整个 cache，也没有写旧缓存。该目录是本机运行依赖，**不属于源码 overlay，也不上传动态库**；具体 hash 在机器摘要。

four-wall 随后暴露外参缺失。新仓库已有 `assets/calibration/camera_extrinsic_opencv.npy`，与旧 `rm75_pick_place_app/assets/calibration/…` SHA256 同为 `f9f2cc15238fccf58fac5949e9d8f6b33885ab2fcaa1f2edbd96e00da58e0bad`；仅用原 CLI 显式指向该真实文件，没有构造外参。

## 3. 代码完整性与 CPU 测试

| 实际检查 | 结果 |
| --- | --- |
| 拉取后 compileall | PASS |
| 拉取后完整 tests/three_scene | 71 passed，3.49 s |
| 安装 snapshot 后中间轮 | 72 passed / 1 failed，14.02 s |
| 最终 tests/three_scene | **73 passed / 0 failed / 0 skipped，3.85 s** |
| 最终全仓 tests | **395 passed / 0 failed / 0 skipped，22.53 s** |
| 最终 compileall / Node JS syntax | PASS |

中间失败必须保留：missing-source 测试 fixture 把整个实际 rm75_app 链接进去，安装 vendor 后不再真的“缺 source”，意外进入 native worker 并超时。修复 fixture：只链接应用代码，明确排除 `_vendor`，保留原“缺 source 必须失败”的断言，不延长超时或跳过测试。

实际命令：`PYTHONPATH=. python -m compileall -q rm75_app tools tests/three_scene`；`PYTHONPATH=. python -m pytest -q tests/three_scene`；`PYTHONPATH=. python -m pytest -q tests`。原始日志 `/tmp/three_scene_20260907_tests.log`、`tests_final.log`、`tests_final_v2.log`、`full_final.log`、`compile_final.log`（均带相同 `three_scene_20260907_` 前缀）。

## 4. 前端

启动：`PYTHONPATH=. python -m rm75_app.workcell serve --profile runtime_data/three_scene/machine.json --port 7861`，**未加 allow-real**。本实现是 `/workcell/` 同页三个 tab，不虚构 `/workcell/pickplace` 等独立路由。

Browser 技能发现没有可连接实例；按故障流程确认列表为空后，用仓库既有浏览器测试脚本在隔离 `/tmp` Playwright 环境和已有 Chromium 运行。本轮没有改变 CSP 或开放 unsafe-eval。

真实浏览器发现两次失败：原测试 `wait_for_function` 字符串求值被 CSP 拒绝；改 locator assertion 后暴露 **app.js 第 82 行截断**。按已存在服务契约补齐页面初始化、物体下拉框、活动任务恢复、nonce 确认按钮绑定；新增 Node 语法测试。CPU Python green 不能代替该检查。

最终命令：

```bash
/tmp/rm75_three_scene_browser_20260907/bin/python tools/check_workcell_browser.py --url http://127.0.0.1:7861 --chromium /home/zhangzhao/.cache/ms-playwright/chromium-1228/chrome-linux64/chrome --design /home/zhangzhao/Desktop/lerobot/Beta_demo-codex-v0.9/jimu_tasks/tag1_standard_three_layer/builder_scene.json --output runtime_data/three_scene/browser_full_builder
```

### PickPlace
- Page loads / Preview / Status polling: **PASS**，结果为 command_completed_unverified，不显示实物成功。
- Original input prompt bridge: 按钮绑定补齐；原真实引擎等待点到浏览器端到端 **NOT_RUN**（本轮没有人工确认 native 运行）；CPU nonce/真实测试子进程桥接测试通过，不混为实机证据。
- Stop: 原 native 实机停止 **NOT_RUN**；共享 UI Stop 在 PushT surrogate 实际验证通过。

### Magnetic
- Page loads / Preview: **PASS**。
- Import `jimu_builder_scene_v1`: **PASS**，旧设计共 **21 件、12 活动件**。
- Round-trip preserves fields: **PASS**，导出 JSON 与导入完全相同，包括额外未知嵌套字段；未拿两块示例替代此项。
- 原输入桥 / native Stop: 同上，完整原引擎 GUI 集成 **NOT_RUN**。

### PushT
- Page loads / CPU Sim request / Live status / Stop: **PASS**。
- 真机按钮在未授权服务上禁用。CSRF/跨源拒绝/请求白名单/服务互斥/授权由完整 CPU suite 回归；实际机械臂资源冲突和急停 **NOT_RUN**。

最终浏览器报告含 **5 项检查、0 page errors**；截图与 report 在 `runtime_data/three_scene/browser_full_builder/`，机器摘要保留报告。完整设计的目标编辑后物理可达性不在此浏览器结果内。

## 5. PickPlace 回归

- Frozen scenes used: 已提交 `pick_jiaobang/test_scenes/gluestick_desk_regression.json`（world-pose 格式），使用原 **direct** 入口明确 `--skip-foundationpose --fixed-scene-pose-file`，不冒充已重跑 SAM3/SAM6D 相机链。SAM6D 入口另做 import/parser smoke。
- Candidate screening timing / 迁移前后 GPU 成功率比较: **NOT_AVAILABLE / NOT_RUN**，两次完整启动没有形成可用的物体规划结果，不能引用历史 14/16 代替。
- Relation screen / fallback: 未调整候选、seeds、碰撞或策略来换通过；没有拿新 cuRobo2 实验链替代原入口。
- Planning/task successes / total: **0/2 启动尝试完成**；首次 JIT 180 s 超时（exit 124），恢复隔离扩展缓存后 native segmentation fault（exit 139）。崩溃根因尚未确定，不归咎于某个算法或认为已修好。
- Simulation/physical task success verified: **0**，没有实物结果观测。
- Difference vs old working version: 源码/hash 比对已记录；完整运行前后等价 **NOT_RUN**。旧目录不执行、不写入。

两次命令在 vendor 根执行（第二次追加 `--curobo-torch-extensions-dir /tmp/rm75_native_extensions_20260907`）：

```bash
PYTHONDONTWRITEBYTECODE=1 timeout 180 /home/zhangzhao/anaconda3/envs/foundationpose310/bin/python pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py --object-name gluestick --skip-foundationpose --fixed-scene-pose-file pick_jiaobang/test_scenes/gluestick_desk_regression.json --render-mode none --auto-execute
```

`auto-execute` 仅跳过原仿真确认；原 parser 的 `execute-real` 是 store_true，缺省 false，原 main 只在该显式开关下创建硬件 executor。首次命令曾被审核拦截，补充源码守卫证据后才获准运行；没有绕过安全审查或加入 real 参数。

Raw: `/tmp/three_scene_20260907_pick_native_sim.log`、`pick_native_sim_cached.log`（后者同前缀）。

## 6. Magnetic 回归

| 运行 | 原结果 | 严格验收 |
| --- | --- | --- |
| four-wall 首次 | JIT timeout，exit 124 | FAIL |
| four-wall 恢复扩展后 | 缺相机外参，exit 1 | FAIL |
| four-wall 恢复已核对外参路径 | 原 cycle 1–4 success=True，final success=True，exit 0 | **不能 PASS：触发原有 gripper/world relaxed 重试** |
| triangle-roof 固定场景 | 计划 12 cycles，记录前 6 个 cycle=True；发现同类放宽后人工停止，exit 143 | **未完成，不把 6 个标记当全任务通过** |

- Old known scenarios: 原 portable 第一层 four-wall；原 triangle profile 两层加 roof 的 **12-cycle** 程序，固定 assembly anchors 输入，非实时感知。
- Builder JSON: 前端使用旧 21 件设计；上述动态 native 回归使用原内置结构/profile，不把这两项混为同一场景的物理验证。
- Role/dependency、source retry、partial/full-open/retreat、attached payload: 迁移保留原实现，四墙日志可见各 cycle/partial release/prefetch；独立严格物理判定 **NOT_RUN**，未用缓存目标同步证明磁吸稳定。
- Planning/native successes / total: 4 次程序尝试中仅 1 次原程序完整报告成功；**严格安全通过 0/4**。内部候选失败保留在 profile rows，不只列成功候选。
- Simulation/physical task successes: **未通过独立观测验证**；原 dry-run/cache/visual 同步不等于真实接触物理成功。

在 vendor 根执行的四墙成功返回命令：

```bash
PYTHONDONTWRITEBYTECODE=1 timeout 120 /home/zhangzhao/anaconda3/envs/foundationpose310/bin/python Beta_demo-codex-v0.9/rm75_jimu_four_wall_portable.py --render-mode none --jimu-build-layers first --auto-execute --curobo-torch-extensions-dir /tmp/rm75_native_extensions_20260907 --camera-extrinsic-opencv-path /home/zhangzhao/Desktop/rm75-manipulation/assets/calibration/camera_extrinsic_opencv.npy
```

triangle 命令：

```bash
PYTHONDONTWRITEBYTECODE=1 timeout 180 /home/zhangzhao/anaconda3/envs/foundationpose310/bin/python Beta_demo-codex-v0.9/rm75_jimu_triangle_roof_apriltag_portable.py --render-mode none --jimu-build-layers two --jimu-second-layer-triangle-profile --no-jimu-demo-triangle-apriltag --sam6d-fixed-scene-result-file Beta_demo-codex-v0.9/jimu_portable_repro/scenes/jimu_assembly_anchors_default_sam6d.json --auto-execute --curobo-torch-extensions-dir /tmp/rm75_native_extensions_20260907 --camera-extrinsic-opencv-path /home/zhangzhao/Desktop/rm75-manipulation/assets/calibration/camera_extrinsic_opencv.npy
```

**关键审阅点**：`_apply_deferred_two_step_final_approach` 在受约束接近失败后调用 `_set_world_collision_for_links(..., enabled=False)` 再重试。固定 `7aaff9d` 原文件 **第 3263 行已含同一 relaxed 提示**，不是本轮或 overlay 新加的补丁；当前迁入文件约第 3453 行。保留工作版本与“不以放宽碰撞通过”的要求在这里产生验收冲突。本轮没有把开关改松，也没有把该原行为当成安全通过；识别后停止屋顶进程。下一步需要明确严格模式应怎样约束原接触例外，不能未经审阅改算法或降低门槛。

Raw: `/tmp/three_scene_20260907_four_wall_sim.log`、`four_wall_sim_cached.log`、`four_wall_calibrated_path.log`、`triangle_sim.log`（后三者同前缀）。GPU/profile rows 在机器摘要，原 `.jsonl` 留 vendor 的 ignored `planning_profile_logs/`。

## 7. PushT GPU / cuRobo2

- CPU preview / surrogate loop: **PASS**，通过实际 `workcell run` CLI 与浏览器分别执行；新观测来自显式 surrogate，不是 ManiSkill 或相机。
- Planner environment: **PASS（有限 smoke）**。真实 cuRobo2 backend 初始化，GPU FK 对模拟零关节输入返回有限 TCP pose，关节顺序 joint_1…joint_7。只调用 backend/FK，未构造 RealManArm。
- Contact/tool profile qualified?: **NO**。示例 push_tcp_z_m / tool quaternion / measured contact links 等缺失。
- Unqualified profile rejection: **PASS**，实际构造 Push executor 抛出 `ValueError: push_tcp_z_m must be a number`；没有填假坐标、没有把 tool_collision_geometry_verified 改为 true。
- Full short push preplanned / TCP corridor / collision audit / speed_mps GPU timing: **NOT_RUN**，不能把初始化/FK 当全推链通过。
- Stale/replayed observation / planning-time drift: CPU suite 覆盖；真实 tracker+GPU 集成 **NOT_RUN**。
- GPU cases successes / total: 初始化/FK smoke 1 次成功；完整短推 **NOT_RUN（0 个合格输入）**，不写为 1/1 完整链。
- Raw: `/tmp/three_scene_20260907_push_gpu.log`，完整内容在机器摘要。

CLI 实际执行三个 preview 和一个 PushT sim：

```bash
PYTHONPATH=. python -m rm75_app.workcell run --profile runtime_data/three_scene/machine.json --request examples/workcell/pickplace_preview.json
PYTHONPATH=. python -m rm75_app.workcell run --profile runtime_data/three_scene/machine.json --request examples/workcell/magnetic_preview.json
PYTHONPATH=. python -m rm75_app.workcell run --profile runtime_data/three_scene/machine.json --request examples/workcell/pusht_preview.json
PYTHONPATH=. python -m rm75_app.workcell run --profile runtime_data/three_scene/machine.json --request examples/workcell/pusht_sim.json
```

GPU smoke 使用 `Curobo2Backend(Curobo2BackendConfig())` → `_ensure_planner()` → `tool_pose_for_configuration(JointConfiguration(names, np.zeros(7)), 'gripper_tcp')` → 用未修改 machine profile 尝试构造 `CuroboPushExecutor` 并记录上述拒绝，finally 释放 backend；没有调用 arm.execute 或 execute_push。

## 8. Camera / tracking

- Tracking source: **NOT_RUN**（本轮不采集真实相机）。
- Calibration files: 只核对既有相机外参文件 hash并用于旧固定场景解释；PushT `T_marker_object`/推头几何 **NOT_RUN**，没有假设 tag 中心等于 T 中心。
- Fresh timestamp / lost recovery / observe → push → fresh observe: 真实链路全部 **NOT_RUN**。

## 9. RealMan no-motion

- Connection/preflight / Joint feedback / Stop API / Gripper backend / API-signature 实测: 全部 **NOT_RUN**。没有现场操作员/设备资格输入，未连接 IP。
- Did any motion occur? **NO**。原仿真命令无 execute-real，服务无 allow-real；没有任何实际机械臂或夹爪运动。

## 10. Physical motion ladder

Reduced-speed free-space、isolated gripper、one PickPlace atom、multi-object PickPlace、one Magnetic piece、2/4/6+ 实物结构、one PushT short push、push → fresh observation、closed-loop multi-step：**全部 NOT_RUN**，符合本轮禁止实际运动的要求。

## 11. Final summary

- PickPlace software state: fixed + audited snapshot 已落盘并纳入 Git，原 SAM6D import/parser/preview PASS；完整 native 仍失败，不能宣布恢复验收完成。
- Magnetic software state: 导入/适配/前端/原四墙程序可运行；严格碰撞与独立物理验收未通过，屋顶部分运行后停止。
- PushT software state: CPU/browser PASS，cuRobo2 backend/FK PASS；完整推链等待真实标定 profile，未执行实机。
- Remaining blockers: PickPlace native 崩溃/JIT 环境；旧引擎固有碰撞放宽与本轮严格验收要求的冲突；PushT 合格推头/contact/table/tracker 配置；真实 no-motion SDK 现场条件。
- Files changed by Codex: manifest 限定的 vendor 输入与 overlay 审计清单；`workcell/legacy.py` 导入顺序；前端 `app.js` 截断修复；浏览器测试与 Node/import/隔离 fixture 回归；运行库 staging 工具；本 MD/机器摘要/.gitignore。没有修改旧仓库，没有修改原算法源码（除既定路径 relocation/逐文件 overlay）。
- Commits made by Codex: 本结果所在提交，可用 `git log -1 --format=%H -- docs/CODEX_THREE_SCENE_RESULTS_20260907.md` 获取。
- Open questions for ChatGPT/user: 请先审阅原接触放宽分支的严格验收策略，并提供/确认 PushT 实际标定 profile。不能因旧程序显示 success 就开启真机。原 PickPlace 崩溃需单独环境诊断，不以重新设计算法替代。

本轮停止在软件/无实际运动证据回传；不将以上部分完成夸大为三条真机 demo 全部收尾。
