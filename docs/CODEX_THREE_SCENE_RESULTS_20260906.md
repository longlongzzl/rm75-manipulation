# Codex 三场景本地验证回传 — 2026-09-06

按 `CODEX_THREE_SCENE_RESULTS_TEMPLATE.md` 填写。结论：**BLOCKED_PENDING_SOURCE_REVIEW**。第 1 节只读审计发现 `candidate_final_fix`；第 2 节 compileall 通过，但 CPU 测试存在真实失败。未执行第 3 节迁移，未自行纳入或丢弃 dirty 修改，未继续 GPU、相机或真机阶段。

## 0. 基本信息

- Commit tested: `006a64ee5339ce5e200e3a134ef02209b55259b3`。从 `6820409bf7149e6b003a05419a5fc73d8075d5be` fast-forward 拉取，开始时新仓库工作区干净。
- Branch: `chatgpt/three-scene-software-closeout`。
- Python: CPU 测试 `/home/zhangzhao/anaconda3/bin/python`，3.12.7。
- OS: Linux 6.8.0-137-generic x86_64，glibc 2.39。
- CUDA / driver: RTX 5060 Ti / 580.173.02；`curobo2` 环境 torch 2.11.0+cu128，build CUDA 12.8。本轮只查询环境，没有 GPU 规划测试。
- cuRobo2: Python 3.11.15；源码 `/home/zhangzhao/PycharmProjects/curobo2/curobo/curobo/__init__.py`，distribution `nvidia-curobo=0.0.0`；版本号不能替代算法验收。
- ManiSkill: `realman` 环境 Python 3.11.14，ManiSkill 3.0.0b22 / SAPIEN 3.0.2；只读 package metadata，未运行物理回放。
- RealMan SDK: `realman` 环境存在 `Robotic_Arm` 包；`rm-robot-interface` distribution metadata 未找到，SDK 版本/接口实测为 `NOT_RUN`，未以包存在冒称可用。
- 环境边界：`curobo2` 环境未发现 `mani_skill`、`sapien`、`Robotic_Arm`、`cv2`、`pyrealsense2`。PushT GPU+相机+SDK 的组合环境尚未闭合；本轮没有安装或猜配。
- Robot IP used: **NONE**，没有连接机械臂。

## 1. 旧仓库 dirty worktree 审计

- Old repo path: `/home/zhangzhao/Desktop/lerobot`。
- Origin（只读核实）: `https://github.com/longlongzzl/lerobot-realman.git`。
- Old repo HEAD: `36798efbd12814841607951c9af470b309b34fd3`。
- Reproducible baseline ref: `7aaff9da22486b7d25557b3795dd258f9b65f10d`，该 Git object 存在。
- `git status --short` before: **141 行**，完整输出在机器审计的 `old_status_before`；未跟踪逐文件清单共 **12979 项**，没有把它们全部当日志或最终源码。
- 固定基线到当前工作区的 tracked diff：48 files，10318 insertions / 888 deletions。该总数包含基线到 HEAD 的已提交差异，**不全是未提交改动**。下表重点行为改动已分别与 HEAD 核对，确实存在于 dirty worktree。
- Was any dirty file modified/overwritten by this validation? **NO**。没有 reset、clean、checkout 旧目录、stash、导入/执行旧运行模块或在旧目录写 pycache。
- Before/after `git status --short`: **完全相同**（`cmp` exit 0）。这是状态比对，不夸大为对所有 12979 个未跟踪文件做了前后内容哈希。
- Recommendation: **stop for user review**。不满足“无关键最终修复候选再迁移”的前提。`candidate_final_fix` 表示可能属于最终行为修复，**不是已证明正确、已授权纳入或已经通过测试**。

机器审计：[three_scene_source_audit_20260906.json](../benchmarks/unified_scenarios/three_scene_source_audit_20260906.json)。包含完整 status、逐文件 untracked 清单、相关文件 baseline blob / 工作区 SHA256 / diff SHA256 / hunk 索引、是否为 HEAD dirty、CPU 结果和原始日志校验和。

### 1.1 六个重点入口逐项分类

以下 `P/` = `pick_jiaobang/`，`J/` = `Beta_demo-codex-v0.9/`。`+/-` 为固定基线到工作区的行数；四个变化入口的差异也都存在于 HEAD dirty。

| 文件 | +/- | 分类 | 只读看到的行为变化与风险 |
| --- | --- | --- | --- |
| P/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py | +1536/-80 | `candidate_final_fix`，夹杂 `debug_only` | 延后最终接近、释放前 clearance/return 预验证、带物 transport/carry waypoint、可复用 lift、返回起点缓存、执行返回关节以及下一周期 prefetch 状态。属于运行/规划逻辑，不是路径迁移 |
| P/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place_sam6d.py | 0/0 | unchanged | 入口本身相同，但直接 import 上一 dirty 文件，不能据自身 SHA 相同认定链路相同 |
| P/rm75_jiaobang_pick_real_with_foundationpose.py | +49/-5 | `candidate_final_fix` | reset 目标优先级/7 维有限值检查；夹爪同步后恢复 arm q；轨迹函数返回 `q_sent` 而不是一律返回计划末点，涉及实际动作语义 |
| J/rm75_jimu_four_wall_direct_sam6d.py | 0/0 | unchanged | 本身相同；引用 PickPlace direct/SAM6D 及规则依赖，整体链路仍受 dirty 修改影响 |
| J/rm75_jimu_four_wall_portable.py | +866/-67 | `candidate_final_fix`，夹杂 `debug_only` | builder/table-locked 位姿、平板/半板对称轴、最终接触候选与排序、按物体类型筛选料槽重试、partial release 重复次数/频率、夹爪同步、释放后退让和 transport 默认参数；另有录制元数据 |
| J/rm75_jimu_triangle_roof_apriltag_portable.py | +625/-26 | `candidate_final_fix` | builder 坐标系/角色偏移、triangle tray slots、half-square 对侧抓取和局部 IK batch、物体规格/料槽布局、AprilTag 参数传递与默认行为 |

### 1.2 关键依赖和混合变更分类

| 文件/变更组 | 分类 | 依据 / 处置 |
| --- | --- | --- |
| P/curobo_rm75_planner.py、J/curobo_rm75_planner.py | `candidate_final_fix` | 各 +8/-0：新增 `trajopt_tsteps`、`interpolation_steps` 并透传 MotionGen。默认 None 不等于可忽略，因为上层可显式启用 |
| P/rm75_jiaobang_pick_place_targeted_curobo.py | `candidate_final_fix` | +24/-0：新增对应 CLI、planner cache key 与构造参数；是 direct 的直接依赖 |
| J/jimu_sam6d_pose_provider.py | `candidate_final_fix`，夹杂 `debug_only` | +601/-54：frontend texture/PnP 镜像约定、检测变体、多帧采样/角点融合、PnP 候选与重投影筛选；另有料槽投影 overlay。定位算法/标定阈值不能当纯日志丢弃 |
| lerobot-sim2real/lerobot_sim2real/config/real_robot.py | `irrelevant_local`（本机参数） | 相机分辨率从 None 改为 640×480、30 FPS。虽然非算法，仍影响相机配置契约；分类不授权丢弃 |
| lerobot/common/robots/realman_lerobot/realman_arm.py | `candidate_final_fix` | 共享硬件依赖包含本地夹爪目标累加、直接 release、UDP feedback、reset/控制 socket 等行为改动；不能按无关 RL 修改整体排除 |
| FoundationPose/learning/training/predict_pose_refine.py、predict_score.py | `irrelevant_local`（路径） | 权重/config root 改到本机 PycharmProjects 安装。没有自动收入快照；之后仍需显式环境配置 |
| FoundationPose/run_demo_scale.py | `candidate_final_fix`（兼容性） | 旧 trimesh remove API 改为 `update_faces(nondegenerate_faces/unique_faces)`，涉及运行兼容性，不是纯路径 |
| P/place_rules.py、P/object_specs.py、J/sam6d_to_assembly_state.py、J/magnetic_snap.py | unchanged | 分别核对固定基线/HEAD diff；这些文件本身没有 tracked 修改，不代表上述调用链没有变 |
| direct 中 collision diagnostics、Jimu trajectory 录制/元数据、provider 投影 overlay | `debug_only` 子块 | 只将可辨认的记录/诊断子块如此分类；所在文件还有行为变更，不能整文件当 debug 排除 |
| 未跟踪 J/jimu_builder_frontend/、jimu_tasks/、轨迹回放脚本、half/full plate 资产、Demo_Triangle/ 及旧启动脚本 | `unknown` | 可能包含最终 UI/任务/资产，但没有固定提交来源与完整行为审阅；仅清单留证，不收入迁移 |
| rm75_pick_place_app/ 实验新链、其他 RL/评估脚本和其余未跟踪数据 | `unknown`（未逐块终审） | 并非本轮六入口等价迁移的已确认直接依赖。不能把大量 diff 统一判作 irrelevant；已保留清单，未动这些文件 |

依赖检查采用静态 import/路径阅读，没有运行旧入口：triangle → portable/provider；portable 显式把旧根目录 `pick_jiaobang` 加到 sys.path → direct/SAM6D、`object_specs`、`place_rules`；direct → curobo wrapper/targeted → FoundationPose executor。动态 import 和 profile 参数仍需后续运行验证，不宣称所有环境依赖已经闭合。

特别需要审阅的子块：

- direct 新增 `free-grasp/free-post-place/free-all` 与 unconstrained MotionGen **诊断选项**，默认仍 constrained；其行为不能作为恢复算法时自动启用的捷径。
- four-wall 新增受约束 clearance 与返回路径预验证等默认项，同时 low-hover 默认由 0.02 m 改为 0.01 m，partial release repeats/hz 改变；这些不是无害格式变更。
- half-square relation slots/top-pairs/batch 及对称性/排序改变，需要分块确认来源和效果；本轮没有减少候选、改碰撞或放宽成功标准来跑测试。
- 两处夹爪同步后重新设置模拟 arm q 的变更必须和真实跟踪证据区分，不能把这种状态同步当物理稳定证明。

### 1.3 原始审计位置和命令

本轮原始目录：`/tmp/rm75_dirty_audit_20260906_7hqq77/`。

```bash
GIT_OPTIONAL_LOCKS=0 git -C /home/zhangzhao/Desktop/lerobot rev-parse HEAD
GIT_OPTIONAL_LOCKS=0 git -C /home/zhangzhao/Desktop/lerobot status --short
GIT_OPTIONAL_LOCKS=0 git -C /home/zhangzhao/Desktop/lerobot diff --stat 7aaff9da22486b7d25557b3795dd258f9b65f10d
GIT_OPTIONAL_LOCKS=0 git -C /home/zhangzhao/Desktop/lerobot diff --name-status 7aaff9da22486b7d25557b3795dd258f9b65f10d
GIT_OPTIONAL_LOCKS=0 git -C /home/zhangzhao/Desktop/lerobot ls-files --others --exclude-standard
```

保存 `status_before.txt`、`status_after.txt`、`baseline_diff_stat.txt`、`baseline_name_status.txt`、`untracked.txt`；六个入口完整 diff 保存在 `core_entrypoints.patch`，依赖 diff 在 `dependencies.patch`，provider HEAD dirty diff 在 `jimu_pose_provider.patch`。原始 patch 留本机，不自动提交用户旧代码；Git 内机器审计保留哈希和逐文件 hunk 索引。`/tmp` 不保证长期存储。

## 2. 迁移完整性

- Migration command: **NOT_RUN**，发现关键行为候选，按第 1 节停止在迁移前。
- Snapshot manifest path: 本轮未生成，**NOT_RUN**。
- Source commit recorded: 未迁移；待决可复现基线仍为 `7aaff9da22486b7d25557b3795dd258f9b65f10d`，不宣称最终用户版。
- Six entrypoint blob checks: `migration.py::KEY_BLOBS` **6/6 匹配**，从旧 Git object 只读核对。注意其六项与新交接第 1 节六项不是完全同一集合：KEY_BLOBS 包含 P/curobo_rm75_planner.py，而第 1 节包含 J/rm75_jimu_four_wall_direct_sam6d.py；本轮审计覆盖两者并集，未偷换集合。
- `--verify-only` result: **NOT_RUN**。
- Old repo status unchanged after migration: 迁移 **NOT_RUN**；只读审计前后相同。
- 文档/实际交付不一致：本提交不存在 `tools/migrate_working_snapshot.py`，实际有 `tools/migrate_working_sources.py`。后者使用 `--source-repo/--target-repo`，没有文档中的 `--source-ref/--verify-only` CLI。本轮只报告，不猜补或替换命令执行。

## 3. 代码完整性与 CPU 测试

新仓库根目录实际执行，未修改任何测试或业务代码：

```bash
PYTHONPATH=. python -m compileall -q rm75_app tools tests/three_scene
PYTHONPATH=. python -m pytest -q tests/three_scene
PYTHONPATH=. python -m pytest -q tests
```

| 检查 | 结果 | exit | 日志（上述 /tmp 目录下） |
| --- | --- | --- | --- |
| compileall | PASS，无输出，之前五个截断文件的 Python 语法问题未再出现 | 0 | compileall.log |
| pytest tests/three_scene | **69 passed / 1 failed / 0 skipped，3.29 s** | 1 | pytest_three_scene.log |
| Full repository pytest | **391 passed / 1 failed / 0 skipped，25.18 s** | 1 | pytest_full.log |

两次 pytest 同一失败：`tests/three_scene/test_three_scene_service.py::test_wsgi_static_and_csrf_origin`。`rm75_app/workcell/server.py:31` 读取 `rm75_app/web/static/workcell/index.html` 时 FileNotFoundError；Git/磁盘只有该目录的 `app.js`、`style.css`，缺 index.html。失败保留在 70 和 392 的各自分母中，这两个 suite 是包含关系，不能相加当独立样本。

这不是语法截断，也不是依赖缺 pytest；没有补造页面、改断言或跳过失败。需要恢复完整前端资产。

## 4. 前端

### PickPlace
- Page loads: 浏览器 **NOT_RUN**；共享 WSGI `/workcell/` CPU 测试 **FAIL**（缺 index.html）。
- Preview: 页面实际操作 **NOT_RUN**。
- Status polling: 页面实际操作 **NOT_RUN**。
- Original input prompt bridge: 原引擎交互 **NOT_RUN**，CPU suite 不替代集成验收。
- Stop: 原引擎/硬件集成 **NOT_RUN**。

### Magnetic
- Page loads: **NOT_RUN**，共享入口缺页。
- Import `jimu_builder_scene_v1`: 实际浏览器 **NOT_RUN**。
- Round-trip preserves fields: 旧完整 builder 设计集成 **NOT_RUN**。
- Preview: 实际浏览器 **NOT_RUN**。
- Original input prompt bridge: 原引擎交互 **NOT_RUN**。
- Stop: 原引擎/硬件集成 **NOT_RUN**。

### PushT
- Page loads: **NOT_RUN**，共享入口缺页。
- Preview: 实际浏览器 **NOT_RUN**。
- Sim request: 实际前端/CLI **NOT_RUN**。
- Live status: **NOT_RUN**。
- Stop: 硬件集成 **NOT_RUN**。

附：文档的 `rm75_app.web.three_scene_frontend`、`tools/run_workcell_task.py` 不存在；实际存在 `rm75_app.workcell` CLI。文档示例 `pickplace.preview.json` / `pusht.preview.json` / `pusht.sim.json` 与实际 `_preview.json` / `_sim.json` 文件名不同。先记录交付契约差异，不自行绕过顺序改用其他入口。

## 5. PickPlace 回归

- Frozen scenes used: **NOT_RUN**，来源版本尚待确认。
- Candidate screening timing: **NOT_RUN**。
- Relation screen mode: 本轮运行 **NOT_RUN**，未修改配置。
- Grasp fallback mode: 本轮运行 **NOT_RUN**，未修改配置。
- Planning successes / total: **NOT_RUN（0 个执行案例）**，不写 0% 或通过。
- Simulation/task successes / total: **NOT_RUN（0 个执行案例）**。
- Difference vs old working version: 见第 1 节静态差异；迁移前后功能对照 **NOT_RUN**。
- Raw output paths: 本轮无规划/物理结果；仅第 1/3 节审计和 CPU 日志。

## 6. Magnetic 回归

- Old known scenarios used: **NOT_RUN**，未拿简化案例替代。
- Four-wall result: **NOT_RUN**。
- Triangle-roof result: **NOT_RUN**。
- Builder JSON used: **NOT_RUN**，未自动纳入未跟踪设计。
- Role/dependency behavior preserved: 动态验证 **NOT_RUN**。
- Source retry behavior preserved: 动态验证 **NOT_RUN**，dirty 中确有同类型筛选变更待确认。
- Partial/full-open + retreat preserved: 动态验证 **NOT_RUN**，dirty 中确有行为变化待确认。
- Attached payload collision preserved: 动态验证 **NOT_RUN**，本轮未改碰撞设置。
- Planning successes / total: **NOT_RUN（0 个执行案例）**。
- Simulation/task successes / total: **NOT_RUN（0 个执行案例）**。
- Difference vs old working version: 见第 1 节，不把静态分类当恢复成功。
- Raw output paths: 本轮无规划/物理结果。

## 7. PushT GPU / cuRobo2

- Planner environment: 只读环境查询见第 0 节；实际 GPU 测试 **NOT_RUN**。
- Contact/tool profile qualified?: **NOT_RUN**，未更改资格标志。
- Full short push preplanned before descent?: **NOT_RUN**。
- TCP corridor audit: **NOT_RUN**。
- Collision audit: **NOT_RUN**。
- `speed_mps` affects trajectory timing?: GPU 实测 **NOT_RUN**。
- Stale/replayed observation rejection: 实际 tracker/GPU 集成 **NOT_RUN**。
- Re-observe if object moved during planning: 实际 tracker/GPU 集成 **NOT_RUN**。
- GPU cases successes / total: **NOT_RUN（0 个执行案例）**。
- Raw output paths: 本轮无 GPU planning 结果；不复用上轮 PickPlace 14/16 作为 PushT 证据。

## 8. Camera / tracking

- Tracking source: **NOT_RUN**，未采集相机。
- Calibration files: **NOT_RUN**，未猜配 `T_base_camera` / `T_marker_object`。
- Fresh timestamp behavior: **NOT_RUN**。
- Lost/recovery behavior: **NOT_RUN**。
- One observe → push → fresh observe loop: **NOT_RUN**。

## 9. RealMan no-motion

- Connection/preflight: **NOT_RUN**。
- Joint feedback: **NOT_RUN**。
- Stop API: **NOT_RUN**。
- Gripper backend: **NOT_RUN**。
- API/signature mismatches: **NOT_RUN**；包存在不等于 SDK 链路通过。
- Did any motion occur? **NO**。没有连接机器人，也没有下发机械臂/夹爪命令。

## 10. Physical motion ladder

- Reduced-speed free-space trajectory: **NOT_RUN**。
- Isolated gripper: **NOT_RUN**。
- One PickPlace atom: **NOT_RUN**。
- Multi-object PickPlace: **NOT_RUN**。
- One Magnetic piece: **NOT_RUN**。
- 2/4/6+ Magnetic structure: **NOT_RUN**。
- One PushT short push: **NOT_RUN**。
- PushT push → fresh observation: **NOT_RUN**。
- PushT closed-loop multi-step: **NOT_RUN**。

## 11. Final summary

- PickPlace software state: 固定来源可读取，但 dirty 中存在关键候选修复；迁移暂停，未重设计算法。
- Magnetic software state: 定位/半板/释放/料槽链存在关键候选修复与未跟踪资产；迁移暂停。
- PushT software state: Python 完整性通过；本轮三场景 CPU suite 69/70，全量 391/392；不是前端/GPU/真机验收通过。
- Remaining blockers: 来源版本决策；缺失 index.html；交接命令/示例与文件布局不一致；后续环境/相机/硬件资格尚未验证。
- Files changed by Codex: 仅本回传 MD 和 `benchmarks/unified_scenarios/three_scene_source_audit_20260906.json`。未修改原交接、模板、业务代码、测试、迁移规则或旧仓库。
- Commits made by Codex: 本回传所在提交，可用 `git log -1 --format=%H -- docs/CODEX_THREE_SCENE_RESULTS_20260906.md` 获取；测试对象是第 0 节精确父提交，不是把文档提交冒称重新测过业务代码。
- Open questions for ChatGPT/user: 请按第 1 节确认哪些 dirty 子块属于最终工作版、哪些只保留作诊断，并给出可审计来源；同时补齐前端 index.html 和一致的执行命令。确认前不丢弃、不混入，也不启动完整真机任务。
