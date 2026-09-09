# a35328c 审阅：PickPlace IK、Jimu 状态事务、PushT 正式仿真

日期：2026-09-09
审阅提交：`a35328c8e851d41f0df151f0cb510c4fbb0c3c1d`
分支：`chatgpt/three-scene-software-closeout`

这是代码审阅和下一轮实施指导，不是已实施的算法修复。980 项全量测试通过来自 Codex 第 15 节回传；本轮 ChatGPT 未重新执行 CPU/GPU/相机/机械臂测试。不得把本文件提交当作新增运行证据。旧仓库及 vendor 源码不改，不启动硬件。

## 1. 结论与证据范围

主要证据为 `docs/CODEX_THREE_SCENE_RELEASE_GATE_20260908_R2.md` 第 13–15 节，以及 `benchmarks/unified_scenarios/jimu_lift_103mm_20260909_summary.json`。

- Tennis 当前卡在合格 hover/release IK 产生之前，不是已有合格 IK 后 MotionGen 找不到路径。有限采样失败不证明任务不可达。
- Jimu 在 103 mm 显式试验中已完成前八块墙，首先失败的是右屋顶拟议退让后的返程起点，随后又被接触状态策略中止。不能将这两层失败统称为四块屋顶 IK 无解。
- PushT 已有工具与动态 T 的 PhysX 接触结果，可以推进正式闭环物理仿真；当前环境没有 articulated RM75，也没有将物理后的 T 状态接回多轮控制器。

## 2. PickPlace：先查契约，再优化 IK

### 2.1 当前实际问题

运行后端仍是原 cuRobo1。抓取候选 16 个，整夹爪端面水平且朝下的放置约束保留后，每个抓取关系有 6 个原轴向角，共 96 条关系。回传为 pregrasp/grasp 各 16/16，hover/release 各 0/96。

`_fast_chain_evaluate_paired_relation_records` 实际调用顺序是：

```python
start_qs = q_grasps + q_grasps
goals = hover_poses + release_poses
```

这个分块顺序与结果拆分一致，没有证据表明误用了交错顺序。真正的限制是两个放置端点独立从抓取关节配置求解，没有利用已成功的相邻端点解。

`solve_batch_start_goal_ik` 每批 reset_seed，传入抓取 q 作为显式 seed 和 retract_config，内部 num_seeds=32、return_seeds=1。reset 不等于每次获得新的探索。retract_config 在启用 regularization 时是参考姿态，不是机械臂必须经过的执行起点。参考官方 cuRobo1 IKSolver 的 solve_batch / solve_single 文档，并以本机安装源码核对形状和默认值；不要把 cuRobo2 参数直接移植过来。

### 2.2 先完成两个不改目标的检验

**FK—IK 回环及工具帧。** 从一个当前世界内有效的原 q 出发，用实际 cuRobo EE FK 产生目标，再以同 q 作显式 seed。分别验证 single、batch、当前前台实际调用包装；随后验证同一个目标经 demo TCP 到 cuRobo EE 的转换是否一致。记录关节顺序、弧度、四元数 wxyz、base/world 和 gripper_tcp/EE 变换。

`_convert_demo_tcp_pose_to_curobo_ee_pose` 的矩阵乘法顺序本身合理，但 EE link 查询不到时会退回 demo TCP。增加明确的 link 存在性检查和刚性变换一致性证据；不能让缺 link 静默变成另一个 IK 任务。没有证据证明当前现场真的触发了该 fallback。

**球心—TCP 关系闭合。** `_sphere_tcp_object_distance_m` 取 abs(t_z)，忽略 t_x/t_y，并截断距离；`_sphere_release_tcp_pose_from_center` 用轴向距离构造目标，后续轴向旋转不改 TCP 位置。对每条实际关系核对：

```text
t = T_tcp_obj[:3, 3]
predicted_center = p_world_tcp + R_world_tcp @ t
predicted_center == desired_center
```

球体不要求最终朝向，不等于 TCP—球心平移可以任意改变。仅当原关系确实满足中心轴假设时该简化等价。先测残差，不凭代码可能性直接修改目标；本轮尚未证明它是当前 0/96 的原因。

### 2.3 建议分开的最小 IK 对照

保留原候选、目标、误差阈值、碰撞球/余量、自碰撞与负载，所有新增策略先仅用于 frozen SIM 的显式选项。

1. **基线**：保存原 96 条关系、目标、显式 seed、参考 q、世界/负载/夹爪状态指纹。失败至少区分位置误差、SO(3) 角误差、world/self collision、joint limits；不要只存 IK_FAIL。
2. **seed 与参考姿态解耦**：先保持每目标 32 个内部起点，对照原抓取参考与配置中的标准参考；已有同类放置或相邻目标解可作为额外显式起点替换部分随机起点。保持原抓取 seed，不把缓存解直接当成功。缓存必须包含机器人、工具、世界、负载、夹爪与目标身份，并在当前上下文重新验证。
3. **目标附近连续求解**：保留原独立批量路径作为基线；某个 hover 合格后，以它初始化同一关系的 release；反过来也可作为独立对照。若两端仍全失败，先解决其中一个端点，不能声称 continuation 已经能救回问题。中间同姿态位置只用于寻找种子，最终目标不动，中间序列不能未经路径验证进入执行器。
4. **多解分支**：仅在已能产生合格 IK 后，将返回解数从 1 对照为少量 K，例如 4，保留 goal×solution 维度和逐行误差，按原约束验证连通性。这个改动能改善端点分支选择，但不能单独解释或解决当前 raw success 全零。
5. **水平面内 yaw 补充**：前四项之后仍失败，再做独立候选覆盖实验。现有六个轴向角不是连续 yaw 空间；保留六个原角，只添加其间有界角度，不增加俯仰、横滚、目标平移或更宽容差。这属于候选策略变化，单独报告新增工作量和成功数。

`solve_ik` 另有可确认的诊断对应风险：实际 q 由 `_nearest_success_solution` 选择，debug 误差却始终取原结果第 0 行。修复为选中解对应的误差，或以该解重新 FK 核验。当前 Tennis 批量每目标只返回一行，不能把这个潜在 single-IK 错配宣布为本轮根因。

gluestick 的世界碰撞拒绝和 hongshupian 的带负载抬升碰撞另记；不要把所有 PickPlace 失败都用增加 IK seeds 处理。原 5 秒目标须分别记录筛选、修复求解和完整规划耗时，不把完整 native 用时冒充候选筛选。

## 3. Jimu：优先修阶段状态一致性，不再盲目加高度

### 3.1 最新第一失败和后续中止不同

103 mm 试验经过 front_wall 和前八块墙。右屋顶首先在以下预验证阶段失败：

```text
pre_release_post_place_clearance_post_plane_main_m_near_up_return_check
INVALID_START_STATE_WORLD_COLLISION
released=false
```

返程目标有效、起点无效。非零世界碰撞 link 为 gripper_base_link、gripper_Left_2_Link、gripper_Right_2_Link；具体 mesh 对尚未定位。之后原 final-approach 重试触发 preexisting_disabled_world_objects，记录包括 active_target_object、scene_obstacle_right_roof_triangle、virtual_table_plane。其余三块屋顶没有本次完整候选证据。

### 3.2 已有恢复机制不能忽略，但恢复范围不同

`pickplace_curobo_only.serialize_return_planning` 已在 GPU RLock 内使用 `preserve_planner_world`，恢复 world、所有 solver/cache owner、enabled 状态及部分签名。已有 tests/three_scene/test_pickplace_return_transaction.py。不能再简单诊断为完全没有 finally 或锁。

但 `transport_contact.install_transport_contact` 还维护独立闭包 `excluded_sources`。其 refresh 每次先 pop，再仅按本次 exclude_object_names 设置；`preserve_planner_world` 恢复物理世界时没有恢复这个闭包。toggle 又用它及 attached 状态判定源物体缓存是否为允许的 disabled 条目。

这是值得优先复现的跨模块状态一致性缺口，尚未由现场逐步 trace 证明是该次中止的唯一根因。另一个关键点：preserve_planner_world 会刻意禁用不属于恢复世界的缓存行。合法 stale cache 与本应存在却被关闭的桌面/邻物必须区别，不能只靠 disabled 名称集合下结论。

### 3.3 下一轮只围绕首个右屋顶构造一条证据链

在预验证前、返程临时世界内、恢复后、下一候选进入前记录：planner/thread/candidate/source、当前期望世界对象、实际 enabled/cache 行、attached 来源与球、gripper locks、excluded_sources、allowed_disabled_objects，以及真正的 unexpected_disabled 差集。摘要不上传原始关节或轨迹。

将原返程事务扩展为统一的阶段状态事务，或者给现有事务增加适配器状态参与者；不要再叠一套互不知情的全局字典。正常返回、普通求解失败和 BaseException 都应恢复已捕获状态，仍由严格检查确认正确。

验收最少覆盖：嵌套 return/prelift、失败后下一候选、后台/前台隔离、合法陈旧缓存、当前必需桌面被关闭、非源邻物被关闭。正确恢复不是全量 enable，也不是清空 disabled 标志；碰撞的原候选仍失败，状态完好才能继续原有重试，真实状态损坏仍应中止。

同时为首个返程起点做对象级碰撞定位，优先只读 mesh 距离/独立检查器，不在生产共享世界中做破坏性逐物体 ablation。区分检查的是真实已释放状态还是拟议释放后状态，核对拟议夹爪开合、目标物体位置、payload 是否应移除；不要把 released=false 自动解释为模型错误。

### 3.4 为什么不能只说原版也无解

新执行并非原版完全等价：source_adapter 替换了旧 MPLib/FCL 路径；payload_contact_config 收紧了负载与底座豁免；near-IK guard 新增当前碰撞复核；WorldOnlyContactUnsupported / StrictContactNotSupported 可中止原 except Exception 重试。vendor 字节未改不代表运行语义未改。

从用户旧成功工况恢复相同 builder、物体模型、起始 q、参数、工具、夹爪状态与实际函数绑定。仅在只读快照副本做 no-motion 对照，并用同一现行检查器审计旧返回路径。若旧路径也通过现行审计，则追踪新版本在哪个目标/seed/状态/候选处首次分叉；若不通过，明确指出具体模型/策略差异，不臆断旧成功是假成功。

保留 100 mm 生产默认；103 mm 仍只是已授权试验。先处理右屋顶状态与碰撞对象，再决定是否改退让路径。不要继续扫高度掩盖事务问题。

## 4. PushT：可以进入正式闭环仿真开发

用户已选择闭合夹爪，不再重复追问工具选型。第 15 节动态 T 位移约 9.2 mm 是 PhysX 结果；单次工具推移 12 mm、初始目标距 30 mm，本来不能据单次未到目标判断控制器失败。约 3 mm /6 度的预测偏差和 retreat 接触说明需要推后重观测，不能把近似预测当状态。

代码边界明确：`PushTContactEnv.SUPPORTED_ROBOTS=['none']`，工具是 build_kinematic，T 是动态 compound；evaluate() 为空。`PushTController.run` 有 choose_push/执行/再观测，但没有接此环境，且计时为 time.time、非 real 成功固定标 surrogate_pose。`CuroboPushExecutor.plan_push` 已能独立生成完整 PreparedPush；execute_push 则强制真实观测及闭合夹爪资格，不应伪造这些标志来运行模拟器。

### 4.1 第一阶段：动力学闭环

新增显式 PhysicsObserver / PhysicsExecutor，复用 controller 的有限步、停滞和确认逻辑，每轮 plan_push 根据上轮真实物理后的 T pose 和工具/关节状态重新规划，再 env.step 执行，不能反复读取最初保存轨迹。

Observer 读取动态刚体状态；physics 与墙钟分别记录，使用可注入 clock 或清晰的统一时间契约，不能因仿真运行较快导致 freshness 错判。成功标记为 physics_pose/tool_only_physics，而非 live_pose 或 surrogate_pose。到达后以物理步推进稳定观察窗口，检查线/角速度及姿态，不能只 sleep 墙钟。

先使用 simulator state 作为明确的真值观测隔离控制问题，再单独做 RGB-D/RRTrack 观测，不把物理 ground truth 冒充视觉定位。先跑冻结的平移、旋转、混合、邻障碍、不可行和取消案例，保留全部分母；实测误差再用于校准 surrogate，拟合与验证场景分开。

### 4.2 第二阶段：完整 RM75 articulated 仿真

在原机器人模型上加入关节位置驱动/PD 跟踪，七关节路径以目标发送，反馈取实际 q/qvel，闭合夹爪状态由仿真关节得到。episode 初始化可 set_qpos；执行过程中不能用 set_qpos/末端 teleport 假装伺服成功。不要同时叠加独立 kinematic 工具碰撞体与 articulated 工具造成双重接触。

保留桌面、非目标障碍、自碰撞及全部工具/负载几何。GPU 碰撞规划与物理接触指标分别报告，原机器人质量/惯量/驱动参数明确来源，不擅自复制 Panda 的控制器配置。物理材料目前未标定，只能称模型验证。

服务前端显式区分 surrogate、tool-only physics、full-arm physics；保留 preview 和 Stop。最终验收必须由 UI/service/worker 实际提交到对应物理后端，不只独立脚本通过。所有 simulation qualification 与 hardware qualification 分离，本轮仍禁止真机运动。

## 5. 下一轮回传要求

优先级：Tennis 契约与固定预算 IK 对照、Jimu 首个右屋顶状态事务并行推进；PushT 从接触回放转动力学闭环，随后整臂伺服。

只新增一份汇总回传，分别列出确定缺陷、实验支持的假设、仍未证明项。每项提供 tested commit/实际差异、输入 hash、成功失败分母、首次失败阶段、状态恢复断言与耗时。已有 980 项不替代新增场景测试；改完再跑全量。禁止通过减碰撞球/buffer、放宽 IK/姿态/接触误差、把替代对象或预取成功算目标完成来收尾。

审阅结束标记：A35328C-REVIEW-END
