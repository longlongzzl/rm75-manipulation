# 三场景续报：Jimu 独立返回执行门 — 2026-09-08

**NEEDS_REVIEW；三条非真机链尚未全部完成。** 本轮修复 Jimu 原独立返回绕过新仓库执行审计的问题，完成 4 次 GPU/SIM 运行（包含一次验证工具失败）。默认四墙 4/4、默认屋顶 12/12，完整原任务仍 8/12；未降低碰撞、减少候选或放宽成功条件。

按 [回传模板](CODEX_THREE_SCENE_RESULTS_TEMPLATE.md) 填写。机器摘要：[independent return summary](../benchmarks/unified_scenarios/three_scene_jimu_independent_return_20260908_summary.json)。原始轨迹、逐点数据和日志只保留在本地 `runtime_data/three_scene/jimu_independent_return_20260908/`；完整任务 worker 为 `96f60daba2fd4d8da01576bee9d28dbd`。

## 0. 基本信息

- Tested base：`4a46c6f` + 本回传提交差异；branch `chatgpt/three-scene-software-closeout`。
- 最终运行代码 SHA256 见机器摘要，适用于修正后四墙、默认屋顶、完整任务。首轮四墙的 probe 尚用了错误的 joint_limits 字段路径，单独记录，不冒充最终版本；最终 CPU suite 还包含运行期间补充的纯源码调用链断言。
- Linux、CPU Python 3.12；GPU 为 foundationpose310 / Python 3.10 / torch 2.7.1+cu128 / 原 cuRobo1；RTX 5060 Ti 8151 MiB，driver 580.173.02。ManiSkill/SAPIEN/模型沿用现有依赖，未升级。
- GPU 串行，MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2；每次上限 900 s。四次均已终态，无 timeout/cancel 重启。
- RealMan SDK connection / Robot IP used：NONE；无机械臂、夹爪运动或相机采集。

## 1. 旧仓库 dirty worktree 审计

- 旧目录 `Desktop/lerobot`，HEAD 沿用已审计 `36798efbd12814841607951c9af470b309b34fd3`。
- 本轮前后 status SHA256 同为 `15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`。
- `tag1_standard_three_layer` 三个原任务文件及原 `JIMU_BUILDER_JSON_EXECUTION.md` 的 SHA256 与 [既有只读审计](CODEX_THREE_SCENE_RETURN_STATE_20260908.md) 一致，运行前后未变。
- 完整任务仍为读取旧目录的独立 SIM comparison，采用原文档明确记载的启动配置；未把这份配置宣称为唯一最终工作版本。
- 六入口及直接依赖沿用 fixed `7aaff9d` + 12 approved overlay 分类。未新增 overlay，未自动纳入未跟踪任务配置；旧文件修改/覆盖/reset/clean/stash：NO。

## 2. 迁移完整性

- 本轮无重新迁移；snapshot manifest 仍为 `rm75_app/_vendor/working_snapshot/MIGRATION_MANIFEST.json`。
- `PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only`：exit 0，**807 文件**（含六入口）验证通过。
- source commit 仍为 `7aaff9da22486b7d25557b3795dd258f9b65f10d`，dependency/runtime qualification 标志仍 false。
- 未编辑 snapshot 源码、旧仓库或外部 cuRobo/模型。新快照目录下 native 生成的 ignored 运行/失败渲染输出仅留本地，不声称没有运行产物。

## 3. 代码完整性与 CPU 测试

| 最终检查 | 实际结果 |
| --- | --- |
| compileall：rm75_app、tools、tests，含 snapshot | PASS |
| 完整 tests/three_scene | **400 passed**，1 warning，15.38 s |
| 完整 tests | **722 passed**，1 warning，31.96 s |
| snapshot verify / git diff --check | PASS |

无 failed/skipped；仅既有 trimesh Scene.dump 弃用 warning。本轮新增 18 项测试，覆盖独立返回无接触豁免、当前状态到首点的连接段、原返回预抬升的释放语义、未知标签 fail closed、三个原 SIM 出口、probe 非运动 sink/失败保留及脱敏摘要。

### 修复的真实调用关系

源码审计确认，原 `_plan_and_execute_return_to_cycle_start` 虽然使用关节空间规划，但其最终返回和预抬升**都调用 `execute_pose_path_stage`**。另有预验证路径直接调用同一 pose 入口。之前门禁只匹配 `post_place_clearance*`，独立返回标签会被跳过；不能仅因函数叫 joint-space return 就把缺口归到 joint executor。

新仓库按原执行标签分类，未修改原函数体或选择分支：

| 原标签 | 审计语义 |
| --- | --- |
| `post_place_clearance`、`return_to_cycle_start_prelift` | release_only：保留 finger ↔ just-released target 规则，其他 world/self 检查保持 |
| `post_place_clearance_return_to_cycle_start` | 原边界拆分；只对 clearance 段应用释放规则，return 段全检查 |
| `return_to_cycle_start`、`post_place_direct_return_to_cycle_start` | return_only：全路径 world/self 检查，**permitted_links=[]，无接触目标豁免** |

执行顺序为原 partial-open 设置 → 同步实际 SIM 夹爪模型 → 观测 → gate → 原执行。独立返回还加密检查 `demo.current_arm_qpos()` 到路径首点的连接段，不改写原路径。桌面、释放物体、世界/自碰撞开关、未挂载负载与模型同步资格缺失时拒绝，未知 release/return 标签拒绝。

单测执行从 hash-verified snapshot 提取的**原 `execute_pose_path_stage_jimu` 与原 base 函数体**，覆盖 rendered、motion-window、原执行器三个出口：都在原 partial-open 后同步再审计，负例不能调用出口。另有纯 AST 单测固定“joint-space return → pose executor”的实际调用关系。

没有为通用 `execute_joint_path_stage_jimu` 声称新增覆盖；它不是这里审计到的原独立返回出口。未认证任意外部调用、真机或未运行的其他原分支。

## 4. 前端

- 三场景 frontend/CLI/overlay/request、Stop/input bridge 随完整离线 suite 复跑；未修改前端。
- 实际浏览器点击、页面/preview/polling/Stop、builder 导入/round-trip：本轮 NOT_RUN。
- 完整任务实际经过 WorkcellService → native worker → 原 builder，不能据此声称已做本轮浏览器交互或真实控制器 Stop。

## 5. PickPlace 回归

本轮未改 PickPlace 运行逻辑，GPU NOT_RUN。沿用 [上一轮三次 GPU](CODEX_THREE_SCENE_PICKPLACE_LIFT_20260908.md)：原单胶棒 1/1；两次七物体均 native 4/7、final=False，刷子/网球释放退让未关闭。胶棒/薯片罐首个原抬升目标与现有底座模型重叠约 7.82/28.97 mm；未恢复 payload/base 免检。

候选筛选 10× warm 基准、实时定位到全链：本轮 NOT_RUN；不是由本轮 CPU suite 推定通过。

## 6. Magnetic / Jimu GPU 回归

默认场景使用 `tools/run_native_contact_audit.py --scene four-wall|triangle-roof --transport-world-checked-compatibility --probe-independent-return`；保留原 fixed anchors、所有候选/seed/原重试，扩展缓存沿用本地已构建 cuRobo 扩展。

完整任务使用 `tools/run_workcell_native_validation.py --task magnetic --task-dir <原 tag1_standard_three_layer> --jimu-start-command-doc <原文档> --timeout-s 900`，只读原 21 件 builder/manifest/fixed poses，不改生产启动默认值。

| GPU run | 原完整任务结果 | 时间 | 搬运审计采样 | 自然执行分段 gate |
| --- | --- | --- | --- | --- |
| four_wall_probe_v1 | **验证工具失败**，0 个完成 cycle，无 final，exit 42 | 36.010 s | 88 点；含已规划/预取的路径，不是已完成搬运 | 0 次；不计通过 |
| four_wall_probe_v2 | **4/4、final=True**，exit 0 | 58.275 s | 177 点 | 4 次通过：clearance 218 / return 619 点 |
| triangle_roof_probe_v1 | **12/12、final=True**，exit 0 | 276.320 s | 740 点 | 12 次通过：clearance 681 / return 2118 点 |
| original_bundle_v1 | **8/12、final=False**，20 次尝试，exit 42，worker failed | 227.042 s | 401 点 | 8 次通过：clearance 445 / return 1424 点 |

首轮验证错误为 `CudaRobotModelConfig` 没有直接 `joint_limits` 属性；修正到原实际 `kinematics.kinematics_config.joint_limits.position`，并同步修正测试 fixture。首轮在两个返回正例复放之后、准备负例时停止，部分结果保留；不是原规划器退化，也不从分母删除。

后面三次共 **24 次自然融合执行 gate**，clearance 1344 + return 4161 点；两段共享边界分别计数，不能称为 5505 个独立状态。完整任务第一块 right_wall 有 2 个 clearance 接触样本，经原 finger/released-target 交集检查通过，world_filter_calls=2；return 没有豁免。所有 24 次 gate passed/state_unchanged=true。四次运行 `loaded_mplib_modules=[]`，搬运 `world_exempt_links=[]`。

完整任务保留 8 true / 12 false 原 cycle 标记：四块屋顶各按原重试流程失败，未到对应 release/return 执行。1427 次 near-IK promotion 被原当前碰撞检查拒绝，未放行 arm/table/self 碰撞。默认屋顶 12/12 不替代完整任务验收。

### 独立返回 GPU 门禁复放，不是额外完整链

验证工具仅在首条原自然融合路径的释放模型同步之后，复用其**原 return 段**，向生产 gate 提供明确的 injected SIM start 和非运动 sink。实际任务的路径对象、关节值、world 与执行分支不改；probe 的 `actual_execute_calls=0`。

- 修正后四墙：两个独立返回标签各全检查 **158 点**；一条注入 GPU 碰撞配置的负例被拒绝，sink=0。
- 默认屋顶：对应两个标签各全检查 **234 点**；同类负例被拒绝，sink=0。
- 两个完整 probe 各 3/3：合计 **4 个正例 + 2 个负例**，状态均未变。正例只调用非运动 sink 一次，不调用真实/原模拟执行器。
- 首轮另有两个 158 点正例，但负例 NOT_RUN，probe 整体失败，单独列出。
- 负例配置位于原关节限制内，由原 GPU `check_start_state` 确认为 WORLD_COLLISION，再插入**测试副本**。这是测试故障注入，不是新增/减少任务规划候选，不拿它报告物体任务成功率。

这三次成功/部分成功自然任务均走原融合 pose 执行。**独立返回在 GPU 中是生产 gate 复放证据，不是原调度自然选中独立分支的完整链证明**；其三个原出口有上述精确原函数 CPU 测试。独立 prelift 标签、实际起点连接段负例本轮仅 CPU 覆盖，不冒充自然 GPU 观测。

原 partial/full-open、抓放绑定、角色/层依赖、source retry、magnetic snap/capture、目标/碰撞几何和成功阈值均不变；无受控 10× warm 性能测量，不把不同运行的时间或采样数量差异称为性能优化。

## 7. PushT GPU / cuRobo2

本轮 PushT GPU NOT_RUN，生产逻辑未改。既有 [六次 GPU 与门禁](CODEX_THREE_SCENE_RELEASE_CACHE_20260908.md) 中 3 次完整五段链通过；[工具几何诊断](CODEX_THREE_SCENE_PUSHT_ENVELOPE_20260908.md) 确认两个失败朝向分别在下降 10/16、11/16 提前接触。

真实工具选型、TCP 与接触模型仍待确认；没有把 5 mm 圆推头假设直接当作实际闭合夹爪，也未通过提前接触/改成功条件来通过。CPU 与保护逻辑随全量 suite 复跑。

## 8. Camera / tracking

新采集、RRTrack 推理、实时多实例与遮挡恢复、推后 fresh observation：本轮 NOT_RUN。旧固定 AprilTag anchors 仅作为旧场景回归输入，不代表切回 AprilTag；既有橙色单薄片证据不等于装配精度或全定位链资格。

## 9. RealMan no-motion

SDK connection/preflight、实际 joint feedback、控制器 Stop、真实 gripper backend：NOT_RUN。无机器人连接；probe 的非运动 sink 不等于真实控制器验收。

## 10. Physical motion ladder

真实自由空间、夹爪、PickPlace/Jimu/PushT 各级运动：全部 NOT_RUN。没有新增运动许可。

## 11. Final summary

- 实际修复：Jimu 独立 pose-return 标签不再绕过模型同步和无豁免执行审计；原返回预抬升与融合退让/返回区分处理。
- 新增 18 项单测，最终全量 **722 passed**、三场景 **400 passed**；807 文件 verify PASS。
- 四次 GPU 运行全部保留：一次工具错误、四墙 4/4、默认屋顶 12/12、完整任务仍 8/12。独立返回 GPU 复放的 4 正例/2 负例通过，非新自然完整链。
- PickPlace 七物体、Jimu 完整任务及定位精度、PushT 真实工具映射等仍未关闭，三条非真机链不宣称全部完成。
- 修改限于新适配层、验证工具、单测、脱敏摘要和回传；旧仓库/snapshot 源码/外部规划库不改。
- **仅本地提交，未 push。** 此前未推送父提交中的原始日志/轨迹、内部路径和硬件标识上传被环境审核拒绝，尚无新内容许可；不重试、不改写历史绕过，原始证据继续本地保留。
