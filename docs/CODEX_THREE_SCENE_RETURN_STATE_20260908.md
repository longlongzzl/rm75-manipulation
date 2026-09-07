# 三场景无运动后续：Jimu 返回终点与释放模型状态 — 2026-09-08

**NEEDS_REVIEW：三条非真机链路尚未全部关闭，Jimu 不是只差真机。** 本轮实际六次 Jimu GPU：两次完整尝试均为 8/12、final=False，另四次是限次诊断取消，不算全链尝试通过。保持 PickPlace/PushT 生产规划不变，重点定位原 Jimu 完整任务第六件返回失败，以及此前未覆盖的实际释放执行边界。未连接 SDK、未采集相机、无机械臂或夹爪运动。

按 [回传模板](CODEX_THREE_SCENE_RESULTS_TEMPLATE.md) 填写。机器摘要：[return state summary](../benchmarks/unified_scenarios/three_scene_jimu_return_20260908_summary.json)。此前 PickPlace/PushT 实际 GPU 结果仍见 [上一轮完整回传](CODEX_THREE_SCENE_RELEASE_CACHE_20260908.md)，不能把本轮 CPU 复跑写成新的 PushT GPU 运行。

## 0. 基本信息

- Tested base：`4b9ad7ccbc31865ed3582a773e56cf38943bdff3` + 本回传提交差异；最终 Python 文件 SHA256 见机器摘要。
- 最终运行模块 SHA 对应 `jimu_documented_start_sync_v4`；此前 v1/v2/v3 为分阶段历史实验，下文明示当时尚未增加的同步/诊断字段，不声称六次都使用最终版本。
- Branch：`chatgpt/three-scene-software-closeout`；Linux；RTX 5060 Ti 8151 MiB，driver 580.173.02。
- CPU Python 3.12；本轮 native GPU 为 foundationpose310 / Python 3.10 / torch 2.7.1+cu128 / 原 cuRobo1。PushT cuRobo2 环境未更改，本轮 GPU NOT_RUN。
- ManiSkill：沿用原 portable 环境和已迁移资产，未升级依赖或重建几何。SDK connection / Robot IP used：NONE。
- GPU 串行，MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2。worker 明确 `mode=sim`、服务 `allow_real=False`；只读固定输入，不调用实时定位。

## 1. 旧仓库 dirty worktree 审计

- 旧目录 `/home/zhangzhao/Desktop/lerobot`，HEAD `36798efbd12814841607951c9af470b309b34fd3`。固定源 `7aaff9da22486b7d25557b3795dd258f9b65f10d`。
- 本轮前后 status SHA256：`15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`。旧文件修改/覆盖 **NO**；未 reset/clean/stash，未整体复制 untracked。
- 原任务 manifest/builder/fixed scene 三文件仍按 [任务闭包审计](CODEX_JIMU_TASK_BUNDLE_AUDIT_20260908.md) 只读使用；未纳入 overlay/snapshot/Git，每次运行均复核三文件哈希。
- 本轮新读到两份旧 untracked 文档，分类为 **candidate_final_fix（启动配置证据），具体场景适用性 unknown**，不是新增批准的迁移文件：

| 原 `Beta_demo-codex-v0.9/` 下文件 | SHA256 | 依据与限制 |
| --- | --- | --- |
| `JIMU_BUILDER_JSON_EXECUTION.md` | `2be34886644608584353ce7d6f7190af516ff4e6c71425fa8bd6a761eb78f94c` | 主 builder SIM 命令明确记载 `--jimu-sim-start-joints-deg 45 0 0 -90 0 -90 60`；同文 fixed-scene 示例未带该 flag，不能据此宣称唯一旧工作配置 |
| `jimu_tasks/README.md` | `1d429ff2ee89ddda1ce4ddc6abb19fb25a80e33290a2685d9a5e284aed35b423` | 另一 tag2 启动示例也记载同角度；不等于 tag1 的已验证启动命令 |

新验证参数只解析文档中七个有限角度，不执行文档命令、不继承其中 real 标志；缺失/冲突/非数值/同行额外 flag 均拒绝。运行前后文档 SHA256 一致才可报告 completed。**仅独立 documented-start comparison，未自动修改原默认起始姿态或迁移 manifest。**

## 2. 迁移完整性

- 本轮无重新迁移；fixed `7aaff9d` + 原 12 项 approved overlay 不变，snapshot **807 files**。
- `PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only`：PASS，含六个入口完整 manifest 校验。
- 未编辑旧仓库、hash-verified snapshot、外部 cuRobo 或模型；`dependency_completeness_verified` / `runtime_gpu_verified` 不擅自改成 true。

## 3. 代码完整性与 CPU 测试

| 最终检查 | 实际结果 |
| --- | --- |
| compileall：rm75_app、tools、tests，含 snapshot | PASS |
| 完整 tests/three_scene | **330 passed**，1 warning，6.73 s |
| 完整 tests | **652 passed**，1 warning，24.88 s |
| git diff --check | PASS |

相对上一轮新增 29 项测试；无 skipped。既有 trimesh 弃用 warning 保留。覆盖失败返回的刷新后世界/起终点、结果和 solver 参数不变、诊断限次、状态污染拒绝、真实执行/prefetch 排除、释放开度同步、完整路径加密、缺失几何证据不冒充通过、原文档解析及摘要不导出 raw joints/geometry。

## 4. 前端

- 三场景 frontend/CLI/overlay/request 权限随完整 suite 复跑；未改前端代码。
- PickPlace/Magnetic/PushT 的实际浏览器 page load、preview、polling、input prompt、Stop：本轮 **NOT_RUN**，此前真实浏览器结果保留在前轮回传。
- 本轮 Jimu 实际经过 WorkcellService → native worker；限次诊断取消后 worker=cancelled，但不冒充浏览器 Stop 或真实控制器 Stop。

## 5. PickPlace 回归

- 本轮 GPU、相机定位、10× warm benchmark：**NOT_RUN / NOT_MEASURED**。原 `lazy_place + primary_only`、候选/seeds/阈值不变。
- 上一轮：单胶棒实际 workcell **1/1、final=True**；七物体 **4/7、9 次原 cycle 尝试、final=False**。整段反向退让复用保护已有单测与执行正例，仍不能宣称七物体闭环完成。
- 无新物体级成功观测，physical task_success=null。

## 6. Magnetic / Jimu 回归

### 6.1 默认返回目标为什么失败

新增诊断包裹原 `_plan_return_to_start_joint_curobo` 内的原 `_profile_plan_to_joint_state`，在原 world refresh 后、事务 restore 前记录失败的起点和目标。先调用原 solver，原结果对象、参数、重试和候选不变；诊断不清空场景、不移除连杆、不做碰撞消融。

`jimu_return_diagnostic_v1`，106.374 s，前五个 cycle true 后限次取消，未完成 12 件、没有 final。首个失败来自 `back_second_wall` 的 `pre_release_post_place_clearance_post_world_z_up_return_check`：

- 原状态 `TRAJOPT_FAIL`；**start valid，goal WORLD_COLLISION**。
- 唯一负间距对：`gripper_Left_Support_Link` ↔ `scene_obstacle_back_second_wall`，高层 OBB **−14.891247 mm**，native internal cache **−14.891211 mm**；raw GPU constraint 非零。
- 原返回目标近似 `[90,0,0,-90,0,-90,60]°`，落入未来第二层后墙。不是返回起点被误判。
- 查询时未释放、未附着，world/self 均启用、无 disabled links。原返回 refresh 此时还省略 table；**即使没有桌面，也已经与后墙碰撞**，不能通过进一步免检解决。
- 诊断完整，前后碰撞状态不变；限次取消只代表取证结束，不是任务成功。

### 6.2 原文档起始姿态比较

`jimu_documented_start_v1` 使用相同原三文件任务，仅增加旧文档明确记载的 `[45,0,0,-90,0,-90,60]°` 原 SIM flag。**8/12、final=False**，250.546 s，8 true + 12 false cycle 标记（第 9 件保留同类 source retries）。搬运 402 点；1427 次不合格 near-IK promotion 被拒。

第六件返回阻塞消失，前两层完成，屋顶仍未通过；没有断言整个失败都由起始姿态造成。此运行发生在下述实际释放模型同步修复之前，不能作为修复后全链结果。

### 6.3 释放边界的实质状态缺口

原 Jimu 为保留 partial-open，跳过公共 direct dry-run wrapper；现在直接观察其 `_jimu_execute_pose_path_stage_base`，位置在原 partial-open 设置之后。全路径按最大 .01 rad 加密，调用当前原 cuRobo world/self check。**这是诊断，不是新增执行放行门**，`diagnostic_only=true / execution_guard=false`。

| 本轮限次 GPU 运行 | 实际证据 | 结论 |
| --- | --- | --- |
| `jimu_release_observation_v1`，27.104 s | 首件融合 clearance+return：70 原点 / 213 加密点，全 valid；SIM gripper=.718900025，planner locks=.6，误差 **.118900025 rad** | 错误开度模型下的 valid 不能资格化 |
| `jimu_release_sync_v2`，27.322 s | 同步后误差 **0**；70 / 215 点，其中开头 2 点 WORLD_COLLISION | 不隐藏修复后暴露的接触；该版本未记录碰撞对 |
| `jimu_release_sync_detail_v3`，27.010 s | 同步后误差 **0**；70 / 213 点，开头 2 点完整几何证据 | 仅左右夹爪 ↔ 刚放下的 right_wall，见下文 |

同步调用已有 `pickplace_gripper_state.update_from_demo`：更新所有 native kinematics owner 的 locked-joint transforms，保持 tensor 地址、碰撞球拓扑/半径、disabled payload rows 和实际夹爪命令不变。只用于 released SIM；缺失 native binding 或 released 状态立即拒绝。下一原 planning refresh 仍按既有机制恢复 nominal locks。

v3 首点左右 `gripper_*_2_Link` 对目标的重叠为 **.549478 / .547435 mm**，第二点降到 **.226341 / .199955 mm**，后续 sampled states 全 valid；高层与 internal cache 证据一致，没有 arm/table/其他积木负间距。属于用户已批准的夹爪—释放目标接触范围，**但不据此新增整段返回豁免**。

三个限次运行 table 均存在，world/self 启用、payload detached、无 disabled links，仅 inactive `active_target_object` 被禁用；诊断后状态不变。`state_unchanged` 比较起点是在有意同步之后，不把这次模型更新写成“完全没有状态改变”。

### 6.4 修复后完整回归

`jimu_documented_start_sync_v4` 使用最终同步和详细观测模块，原任务与 documented start 不变：**8/12、final=False**，229.205 s，worker=failed，runner exit 42。共 20 次 cycle 标记：前八件 true，第九件四种屋顶源按原同类重试产生 12 个 false；不是丢掉失败后报告 8/8。

- 8 次实际释放观测，共 **1864 个加密样本**；每次开度模型误差 **0 rad**，table 存在、world/self 启用、状态恢复一致。首件 2 个负例仍为上述 finger ↔ just-released target，其余七条全 valid。全部仍为诊断，不算独立执行门通过。
- 搬运 **404 点**，无 world-exempt links；1427 次不合格 near-IK promotions 被拒。原失败候选的 all-link 几何记录包含 link_5/table 最深 −57.484653 mm、link_6/table −53.409695 mm；这些是候选证据，不能据此宣称所有屋顶姿态全局不可达。
- 两次完整比较的 `loaded_mplib_modules=[]`；四个取消运行无最终 module census，记 NOT_RECORDED，不补成 []。
- 原任务三文件、原启动文档前后 SHA256 均一致。与同步前 250.546 s 的一次比较不是受控性能基准，不宣称优化提速。

### 6.5 保留行为与未关闭项

- floor/wall/roof、21 件 builder（9 固定 + 12 可移动）、原 manifest 角色/料位/层高、同类料槽重试、候选/seeds、partial/full open、原缓存/返回语义均保留。
- loaded transport 始终全 world/table/robot 检查；payload 自身接触仅 finger-local；near-IK arm/table 碰撞不放行。
- 原默认 four-wall 4/4、默认 triangle 12/12 是前轮结果，本轮未将其重跑或等同完整 builder。
- 原始日志、joint paths、world 几何仅在本地 `runtime_data/three_scene/jimu_return_20260908/` 和对应 `runtime_data/workcell/jobs/<job_id>/`；本地各 run/result.json 记录 job_id，摘要按 run 名、原始 SHA256 与失败分母关联。
- **独立释放执行门仍未关闭**：当前只是正确模型上的观测；融合路径包含 clearance 和 return，尚未建立分段资格证据。不能把本观察或原 native cycle success 写成严格/实物通过。
- 原任务/启动文档仍待逐文件迁移审核；屋顶失败、RRTrack 多实例身份和装配精度仍缺闭环证据。

## 7. PushT GPU / cuRobo2

- 本轮生产代码未改，CPU 全量复跑；**新增 GPU 运行 NOT_RUN**。
- 前轮真实 GPU 六次保留：orthogonal 15 mm/s、同场景 5 mm/s、yaw-matched 三条完整链通过；baseline 下降 10/16 失败，rotated fixed-tool 下降 11/16 失败，预设邻物阻挡正确拒绝。正常输入 3/5（含慢速重复），不能只报正例。
- 前轮三条 GPU 完整链上的无关障碍审计 **3/3 拒绝**，执行前注入的 freshness/session/drift 等门禁 **30/30 拒绝且 execute=0**；不是相机实测。
- 本轮只读核对失败 IK 原日志：baseline 最后一批 4×32 rows 全 false，部分 self contacts 和位置残差保留；未放宽阈值、删候选或据此宣称全局不可达。
- 实际 pusher/场景/观察 profile 仍未资格化，integration_qualified=false、hardware_reviewed=false；不能用 authored fixture 参数替代实测。

## 8. Camera / tracking

新采集、RRTrack 新推理、live freshness/遮挡恢复、真实 observe → push → observe：本轮全部 **NOT_RUN**。此前橙色薄片 29/30 accepted 及坏深度拒绝→恢复保留，不是装配精度验证。此次旧任务是历史 AprilTag 固定输入，不是将当前 RRTrack 改回 AprilTag。

## 9. RealMan no-motion

SDK connection/preflight、joint feedback、真实 Stop API、gripper backend：**NOT_RUN**。GPU SIM 与 worker cancel 不等于硬件 no-motion 资格验证。

## 10. Physical motion ladder

自由空间、夹爪、PickPlace、Jimu、单短推、推后观测和多轮 PushT：全部 **NOT_RUN**。没有机械臂或夹爪运动。

## 11. Final summary

- 本轮确认两个不同问题：默认 Jimu 返回终点撞未来后墙；释放后 planner 开度未跟随实际 SIM partial-open。前者通过原文档配置做独立比较，后者修正模型状态同步，不新增延时、碰撞免检或候选。
- 三条非真机链路仍未全部完成；不能说 Jimu 只差真机，也不能说 PushT 有 GPU 正例即已具备真实工具资格。
- 本轮本地提交见本报告所属 commit；**未 push**。此前父提交中的原始轨迹/日志、内部路径及硬件标识上传被环境审核拒绝，尚无新增许可；不重试、不改写历史绕过。
