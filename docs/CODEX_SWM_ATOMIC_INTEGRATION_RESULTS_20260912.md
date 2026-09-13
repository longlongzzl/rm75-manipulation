# SWM 原子接线与本地验证结果，2026-09-12

## 结论：部分实现，尚未完成三个任务的原生迁移

本次不只有运行新增测试：新增了原生阶段求解、原任务编译桥、共享定位采集、原生场景端口和反馈记录代码，并实际运行了 CPU PhysX 同动作多参数重放。但三个 workcell 任务尚未形成可安装、可验收的完整原生 runtime；不能将本报告作为三个原按钮已启用 SWM 的证明。

`register_runtime_factory` 注册表仍默认为空，worker 未安装未完成的工厂。`swm.enabled=true` 缺工厂时仍返回 `SWM_ATOMIC_ADAPTER_REQUIRED`，没有删检查或回退旧整任务冒充成功。pull/rotate 没有注册。

| 范围 | 已实现内核 | 原生已接线 | 真实模型已调用 | 实际仿真 | 硬件 |
| --- | --- | --- | --- | --- | --- |
| SWM 检查点事务 | 原有内核保留；新增阶段与端口测试 | 共享 tracker 的同帧批处理入口、cuRobo 场景更新和 SAPIEN actor 端口代码；部署上下文未安装 | 本次定位测试用明确替身，未调用 FP/RRTrack 权重 | 完整技能检查点目前只有 fixture 证据 | NOT_RUN，未连接 |
| PickPlace grasp/place | 独立技能、重新观测、陈旧状态拒绝继续有效 | 新阶段调用原 `PickPlaceCoordinator` 的 pose/linear 求解方法；不调用完整 `run()`；放置根据新实测 attachment 求解 | NOT_RUN | 前后六个检查点的 fixture 接线通过；原生笔/cuRobo/整臂抓放 NOT_RUN | NOT_RUN，未连接、未发夹爪或运动命令 |
| Jimu | 使用 grasp/place 序列；无结构验证器拒绝构造任务桥 | 新桥调用原 task builder，保留实例、依赖、支撑元数据和原世界转换；原 worker 的 generation proof 初始化尚未接入新工厂 | NOT_RUN | 原 `MagneticPickPlaceTaskBuilder` 的编译桥测试通过；实际 2～3 件装配 NOT_RUN | NOT_RUN |
| PushT 原子推段 | 仍有 before/pre_execute/after 及实测 transition 合同 | 新 `PushNativePhase` 调用原 `CuroboPushExecutor.plan_push`，转交原生分段轨迹；动态完整场景、持续控制器和 recorder 尚未连成任务 runtime | NOT_RUN | 原 T、原工具、整臂逐推段闭环 NOT_RUN | NOT_RUN |
| 参数辨识 | 原调度、残差、后验保留；新增反馈 recorder 和实际 actor 读回 | `physics_replay.py` 实际接通本机 ManiSkill/SAPIEN CPU API；任务后的自动 manager 接线未完成 | 不涉及 SAM3D/FP/LLM 调用 | 已运行：有信息推送、无信息动作、不同偏心推送，各 4 个参数候选；只含工具与合成立方体 | 无硬件数据 |
| 高层 LLM | 原 AST 编译器保留 | provider 到三个新 runtime 的完整初始化未接通 | SAM3D、FoundationPose、商业/本地 LLM 均 NOT_RUN | 两个真实模型生成任务的闭环 NOT_RUN | NOT_RUN |

## 本次代码变更

- `rm75_app/swm/native_scene.py`：`SharedRRTrackCapture` 复用已存在 tracker，每个检查点只采集一个新 `FrameObservation`，所有实例处理同一帧；检查实际 mesh SHA、原采集时间、帧序号及曝光对齐的机器人反馈。不创建第二个 tracker，不把缓存时间刷新成当前时间。
- 同文件的 `CuroboScenePort` 调用共享 backend 的世界更新和 attachment 方法；`SapienScenePort` 更新现有私有镜像 actor 并读回位姿，同时要求 attachment/material 管理器确认状态。未安装完整管理器或缺 actor 时不能声称部署完成。
- `rm75_app/swm/native_skills.py`：独立 grasp 只规划 approach/grasp/lift，关闭夹爪后结束于 lift；place 使用抓后实测 `T_tcp_object` 重建 release TCP，再规划 preplace/place/retreat。没有把一次旧 pick-and-place 改名为 grasp。共享执行器 wrapper 以实际关节读回检查动作完成后返回 `ExecutionReceipt`。
- 同文件的 `PushNativePhase` 复用原推段求解器，不调用整个 PushT session。必须继续完成完整动态场景、局部目标、当前场景审计和反馈供给后才能启用。
- `rm75_app/swm/native_tasks.py`：从已有 `ManipulationPlan` 产生 grasp/place 请求，每步使用最新 SWM 重建原 `TaskSceneState`，调用原 builder；保留世界到机器人基座的已有平移标定、碰撞代理和 Jimu 多支撑 metadata。新增显式 `native_asset_name` 身份绑定要求。放置目标不能被换成未经原编译器批准的矩阵；Jimu 无独立结构验证器直接拒绝。
- `rm75_app/swm/feedback.py`：有界实测 TCP recorder，收取动作结束后的反馈，直到覆盖最终曝光；不补末端静止样本、不接受命令来源、拒绝重复动作 ID、不同会话或旧反馈。
- `rm75_app/execution/realman_executor.py`：在原共享执行器中加入默认关闭的 `feedback_observer` 回调，发送轨迹期间读取实际关节值；没有新增驱动、连接、回零或授权逻辑。测试用有跟随误差的 FakeSession 证明回调记录的是读回值而非命令值。真实执行未运行。
- `rm75_app/swm/physics_replay.py`：逐物理步读取实际工具 actor 位姿。修复实际运行发现的曝光覆盖边界问题：仅在实际物理时间覆盖采样时刻后取样，避免浮点容差提前结束，产生不足的反馈时间范围。
- `tools/validate_swm_physics_replay.py`：可复现的离线 CPU 工具级物理辨识实验，使用明确的合成 CAD 立方体、桌面与障碍物。参考运行先施加命令，随后所有候选都只重放该参考运行的实际工具 actor 读回序列。
- 新增三个 `tests/swm/test_native_*.py` 文件，覆盖阶段、反馈、原任务编译和场景端口接线。

## 检查点先行验证

先运行原 SWM 基线，再运行新阶段检查点测试，最后才启动物理辨识子系统。

`test_native_grasp_place_use_fresh_full_checkpoints_and_replan_place` 验证了：

1. `before_grasp → pre_execute_grasp → after_grasp → before_place → pre_execute_place → after_place` 六次真实调用观测源，六个不同 snapshot ID。
2. 每个快照均包含 `a / b / table`，不是只更新目标物体。
3. grasp 执行只出现 `approach / grasp / lift`，抓取规划没有生成 place 轨迹。
4. 抓后改变 fixture 中的实测相对位姿 2 mm，place 求解使用新的 attachment；不是复用抓前求出的放置轨迹。
5. pull/rotate 能力仍为 false。

这里的“真实调用”指 Python 接线确实调用原阶段方法与观测源，**观测/IK/执行数据仍是 fixture，不是相机或整臂物理状态**。场景端口测试包含无操作 actor 的位姿读回失败案例，但没有运行完整双原生世界的 attachment 事务。

## 实际 CPU PhysX 证据

目录：`runtime_data/swm_atomic_20260912/`。每个候选保存 `request.json / result.json / worker.log`，实验保存 `report.json`。所有候选串行，`workers=1`；没有同时启动 16 个规划器。

| 实验目录 | 候选完成情况 | 归一化损失 / 结论 |
| --- | --- | --- |
| `physics_cpu_01` | 参考运行成功；4 个候选均失败 | 初次发现最终采样时间略超反馈末尾。错误为 `Invalid observation times for physical replay`，失败证据保留。 |
| `physics_cpu_02` | 4/4 有效 | 低摩擦 27.91359；已知摩擦/密度 800 为 `1.20793e-8`；高摩擦 0.68833；同摩擦/密度 1600 为 `1.26147e-8`。后验有区分度，但密度不可辨识。 |
| `physics_cpu_stationary` | 4/4 有效 | 损失均约 `9.3286e-10`，权重均约 0.25，`updated=false`；不会把无信息实验当成功辨识。 |
| `physics_cpu_holdout` | 4/4 有效 | 工具偏心 8 mm，是不同动作。低摩擦 20.82199；先前选中的已知摩擦候选 `6.4580e-6`；高摩擦 0.43400；同摩擦/不同密度 `6.3214e-6`。先前候选在新动作上误差仍低，密度仍不可辨识。 |

中心推送的所有候选共享实测动作 SHA256：

`e30b8059f2b9bce6390ae222cc4d5b9396162489277757e33702c31c7724db53`

偏心推送的所有候选共享另一条实测动作 SHA256：

`ab972e1693601159520a4e615a848af388caec24c98d825d74d3e5e9b34f63a8`

本实验的起始桌面/障碍物来自固定的合成环境构造；目标物体及工具输入来自该环境读回，稳定前缀后再构造参数实验。报告中的七关节占位字段不代表模拟了机械臂。这里没有相机采集、真实 T 网格、原夹爪碰撞球、整臂碰撞安全、磁吸、实际硬件动作或完整任务检查点验证。偏心实验也重新输出诊断后验，不能声称已实现冻结后验的生产控制器评估或三个任务中的自动信念更新。

## 测试与环境

所有本地测试/仿真均通过 `python3 tools/run_network_isolated.py -- ...` 启动，日志确认非 Unix socket 已在内核阻断。pytest 显式指定 `tests/` 或具体正式测试路径，设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。没有导入、执行或收集 vendor 旧 `test_*.py`。

| 命令范围 | 实际结果 |
| --- | --- |
| 初始 `tests/swm` | 100 passed |
| 新阶段检查点测试 | 修复 fixture 构造缺字段后 2 passed |
| 中间 SWM + 原执行器/协调器/定位桥回归 | 146 passed |
| 正式完整 `tests/` 回归 | 1505 passed, 1 failed, 1 warning；48.33 s |
| 最终 `tests/swm` + `test_realman_executor.py` + `test_pickplace_coordinator.py` + `test_rrtrack_scenario_bridge.py` + `test_magnetic_pickplace_adapter.py` | 151 passed；1.06 s，包含后来新增的任务编译桥测试 |

全量回归失败为 `tests/three_scene/test_jimu_scene_import.py::test_installed_files_match_manifest_hashes_and_old_repo_unchanged`：导入文件哈希检查先通过，但外部 `/home/zhangzhao/Desktop/lerobot` 的当前状态哈希与导入时记录不一致。本次没有改该仓库、没有更新或删除该检查、没有改导入验收基线。不能据此宣称全量回归全绿。

预期状态哈希：`92f5eb5557dd2c6c541473b527391d4d94d54a9e7973f29b5c5d4c384f20bca5`。

此次测试读到：`5d4e28c8e74cef42ccb7b123ac492bc44217896f4d6fb7f923306fd9820dff57`。

本机已确认 SAPIEN、ManiSkill、PyTorch、trimesh、SciPy 可用；CUDA 可用、设备数 1，系统 cuRobo 包、原 cuRobo 源码路径及 RM75 配置均存在。**没有以缺 GPU 为理由掩盖原生接线尚未完成。** 未创建实际 cuRobo 规划上下文或运行三个任务的完整原生技能仿真。

## 尚未完成、因此保持不可用的接线

1. worker 内的可信初始化工厂：将原 profile/generation proof/原场景资产、已存在共享定位服务、原求解器、执行器及两个真正可运行的世界端口组装成同一生命周期，并提供清理逻辑。当前仅有组件和编译桥，不是已安装部署。
2. PickPlace 笔示例：新阶段使用共享求解方法，但尚未等价迁入原已验收流程的全部候选筛选、回退、对称性/容器目标校验和场景配置；未做整臂物理 grasp/place 验收，不启用该技能。
3. Jimu：需把原 proof 生成入口实际接到新编译桥，完成料槽/底板/已放置件的完整采集，安装独立实测结构检查，跑实际 2～3 件设计。不能用每步命令成功证明结构成功。
4. PushT：需要把原静态 `_scene` 编译与完整 SWM 动态障碍物同步起来，把原持续控制器逐段目标接到 runtime，并把共享执行器反馈经同一标定时钟持续接入 recorder，覆盖 `after_push` 曝光后，再调用 `AdaptivePhysicsManager`。当前工具级物理实验不能代替该接线。
5. 原生审计、镜像 attachment 与物理信念应用：组件仍要求可信审核/管理器，尚未提供三个部署上下文的完整实现和实际双端一致性运行证据。不用返回传入 snapshot ID 的空 sink 填补它们。
6. 模型推理与高层任务：没有本次真实 FP/SAM3D/LLM 推理证据，没有完成原 provider 生成两个任务后 compile→仿真→检查点→再规划的闭环。
7. 硬件：完全未运行。此任务及任何软件测试均不构成机械臂、夹爪、相机 live 的授权；后续真机动作仍需逐项单独许可。

验收状态：`PARTIAL_NATIVE_MIGRATION / WORKER_FACTORIES_NOT_INSTALLED / HARDWARE_NOT_RUN`。

## 2026-09-13 续报：正式 worker 闭合前私有预测否决

代码 `191e7e8`：复用原定位注册、原子执行器及 CPU 私有镜像，在原 approach/grasp 实测静止后重新采集完整 12 对象状态，再预测原绝对 mimic 闭合。正式 worker_34 在第 26 个私有物理子步检测到左指支撑链接触桌（合力零、穿透约 14.223 微米），在主仿真发送闭合命令前拒绝。私有步进不改变主状态/驱动，临时镜像与正式主环境、镜像、planner 释放均有 true 读回。

内核：新增 10 项通过；串行全量 1767 passed / 1 原有 warning / 42.56 s。首次全量与原生 worker 重叠启动造成 3 项工作站锁拒绝，原失败日志保留，未删除或绕过互斥。原生接线：上述新采集和正式否决已实跑，但仍无笔完整六检查点或 lift/place 成功。模型推理：未运行。实际仿真：是完整原场景的单参数闭合预测及正式执行前拒绝，不是已合格的接触重放、参数辨识或后验进入下一规划。硬件：未授权、未连接、未操作。

缺适配器检查、旧阶段审计和原实测触桌保护保留，未完成技能不冒充可用。下一步将闭合拒绝反馈给原联合候选选择，继续完成笔正式闭环。滚动权威材料为 CODEX_SWM_RELEASE_PLAN.md、benchmarks/release/swm_acceptance.json、CODEX_SWM_RELEASE_ACCEPTANCE.md；有界证据在 benchmarks/release/evidence/pen_closure/worker_32 至 worker_34。仍为 PARTIAL_DELIVERY，本轮仅本地提交，未推送。

## 2026-09-13 续报：原候选循环中的闭合否决

提交 `4562bd1` 将既有私有 CPU 预测接到原抓取候选循环，明确区分“预测碰撞，可换候选”与“模型/身份异常，必须中止”。候选端点与零初速只作为私有假设，记录 measured=false，不冒充实际 approach 后读回；原执行前独立新采集与全部保护保留。

内核：定向 21 passed，串行全量 1774 passed / 1 原有 warning / 42.55 s。原生接线：普通 worker_35 在规划期间实际否决唯一返回候选，未执行主动作。实际仿真：完整 12 对象，28 个私有子步后左指支撑链接触桌；主状态和目标未变，资源释放正常。模型推理：未运行。硬件：未授权、未连接、未操作。

只读 worker_36 证明原始 70 个候选经联合筛选仅剩 1，运动预算实际为 8；目前不能声称已经在真实场景找到替代候选。下一步对已否决 ID 接通原关系筛选的有界续搜，继续六检查点，不放宽碰撞或成功条件。证据见 benchmarks/release/evidence/pen_closure/worker_35/、worker_36/；滚动计划、机器验收表与最终报告同步。PARTIAL_DELIVERY，本轮未推送。

## 2026-09-13 续报：原关系筛选两轮续搜，当前回归未通过

WIP `05026e6`：原筛选器增加已尝试 ID 排除，保留全部原父子候选和配对；原子层跨轮共享总运动预算。普通 worker_37 实跑 round 1 返回唯一候选并被闭合预测拒绝，round 2 排除该 ID，预算 8→7，返回零候选。未执行主动作、未取得笔六检查点，资源释放正常。

内核：串行全量 1776 passed / 3 failed / 1 warning / 45.26 s，失败为本轮新测试夹具未触达预期分轮路径，尚未修正；不能报全绿。原生接线：两轮调用与排除/预算生效有正式事件，但无替代成功候选。模型推理未运行，实际仿真为私有候选预测和规划续搜，非多参数辨识。硬件未授权、未连接、未操作。

证据在 benchmarks/release/evidence/pen_closure/worker_37/。下一步修正测试构造并继续原关系不可行原因诊断，保留所有碰撞、适配器与成功检查。仍为 PARTIAL_DELIVERY，本轮仅本地提交，未 push。

## 2026-09-13 续报：修复测试，实测定位连续轴策略缺口

提交 `9d75dc4` 修复上轮三项测试夹具，并在正式日志保存每轮原关系诊断。内核：17 项定向通过，串行全量 1779 passed / 1 warning / 45.57 s；失败历史保留。

原生接线：worker_38 证明离散抓取端点可行 2、预抓取 36、完整抓取组合仅 1，连续轴解析启用但未尝试。源码核对确认原 coordinator 在失败后有连续轴解析及放置关系重建，而现原子路径尚未恢复这段纯规划策略。

实际仿真/规划：worker_39 只调用原解析器及原重建函数，70 输入产生 369 候选，来自 54 个原 source ID，每个对应 32 个重建放置候选；主状态/驱动不变，未调用旧整任务、未执行新增动作。该结果不是完整路径或笔六检查点通过。模型推理 FP/SAM3D/LLM 未运行，硬件未授权、未连接、未操作。

下一步共享原连续轴候选构建，并将结果交回现有原子规划、闭合预测和阶段审计，不能调用旧整任务执行尾部。证据见 benchmarks/release/evidence/pen_closure/worker_38/、worker_39/。PARTIAL_DELIVERY，本轮未 push。

## 2026-09-13 续报：连续轴候选构建正式原子接线

提交 `9888ae7` 将原连续轴解析/配对重建共享为纯规划入口，原子路径不调用旧整任务。离散与轴搜索共用原运动预算，新几何重新审核、相同已尝试几何跳过。

内核：39 项定向通过，串行全量 1786 passed / 1 warning / 45.85 s。原生接线：普通 worker_40 实际从 369 轴候选筛出 34 个关系，再按剩余预算选择 7 个；不再只是外部候选诊断。实际仿真：六个轴候选和一个原离散候选进入私有闭合预测，均触桌拒绝，主状态/驱动未变，资源释放正常；另一个轴候选未进入预测，不能算通过。未执行主原子动作，笔六检查点仍未完成。

FP/SAM3D/LLM 未运行；硬件未授权、未连接、未操作。下一步核对预测开爪初态与原控制语义，并在原预算下完善闭合感知候选选择，保留执行前新采集、碰撞和成功标准。证据在 benchmarks/release/evidence/pen_closure/worker_40/，仍为 PARTIAL_DELIVERY，本轮未 push。

## 2026-09-13 续报：私有开爪准备及原缓存 IK 核对

提交 `726325e` 将原开/闭爪 pure mimic 转换及原静止指标共享。候选先在私有完整世界真实开爪、保持到全 13 关节原静止标准，再预测闭合，不修改主控制器/状态，不放宽阈值。

内核：31 项定向通过，串行全量 1793 passed / 1 warning / 42.29 s。原生接线/实际仿真：普通 worker_41 七次候选均用 11 个控制周期完成私有开爪准备，随后均在闭合阶段触桌拒绝；主状态/驱动未变，资源释放正常，尚无笔六检查点成功。

只读 worker_42 本次取得 36/36 个原缓存 IK，最大 FK 位置误差约 4.08 mm、旋转误差约 0.01183 rad；未改预算/排序，在额外运动规划前明确停止。这些数据只支持后续有界预筛或排序，不能替代精确端点复核和实测更新。FP/SAM3D/LLM 未运行，硬件未授权、未连接、未操作。

证据在 benchmarks/release/evidence/pen_closure/worker_41/、worker_42/。PARTIAL_DELIVERY，本轮未 push。

## 2026-09-13 续报：缓存闭合排序正式运行，当前候选仍全触桌

提交 `487f993` 接入有界 CachedClosurePriority，保留原评分/来源排序及精确端点门，不让缓存预测授予执行权限。预测上限 64，完整运动预算仍为 8；缓存或模型异常中止。

内核：29 项定向通过，串行全量 1803 passed / 1 warning / 42.33 s。原生接线/实际仿真：普通 worker_43 完成 34 次缓存物理预测，均先完成开爪准备再于闭合阶段触桌拒绝；主状态/驱动未变，资源正常释放。仍无主原子动作或笔六检查点成功。

独立私有标记 FK 对照表明 pad 中点相对 TCP 随关节状态改变，但不等于接触面标定，未据此改 builder 默认 0.012777 m 偏移。下一步应核对原抓取偏移与实际夹爪接触几何，不以增预算或降标准处理全体触桌。FP/SAM3D/LLM 未运行，硬件未授权、未连接、未操作。

证据在 benchmarks/release/evidence/pen_closure/worker_43/、gripper_fk_01/。PARTIAL_DELIVERY，本轮未 push。

## M1 / 实体支撑碰撞网格与标记点区分

本轮 progress：在 487f993 生产代码及 worker_43 保存的 34 组私有预测基础上，新增 gripper_geometry_01 原生碰撞网格 FK 诊断。显式 PYTHONPATH 的网络隔离运行 exit 0；首次缺 PYTHONPATH 的导入失败保留，未连接设备。两侧实际 ConvexMesh 的 vertices、scale、shape local pose 和 native link FK 组合得到 136 条形状/状态记录，私有 q 读回核对通过，资源 closed=true，无主世界、无物理步进。

开爪态碰撞网格最大 TCP 局部 Z 为 0.005021438228 m，触桌末态最大为 0.025202232189 m；全部候选的最低 base Z 分别为 0.001218812432 m、-0.000440565498 m。它们是跨候选极值，不能当作同一动作的接触标定或安全深度。确认 bi ObjectSpec 显式 grasp_z_offset=0.012777 m，因此仅修改 builder 默认参数不会改变笔候选。实体网格、pad 标记点和规划锁定关节的碰撞球中点必须分别处理。

证据：benchmarks/release/evidence/pen_closure/gripper_geometry_01/，含可复现命令、原生结果和输入/URDF SHA256。下一实际动作：把原候选抓取方向、目标笔几何与闭爪支撑网格的扫掠范围对齐，核对原有抓取细化策略，再在原预算内选择可审核几何；不按跨候选最大值盲调默认深度。

分层状态：内核回归仍引用 487f993 的 1803 passed，非本轮重新运行；原生接线无新生产改动；模型推理未运行；本轮实际原生运行仅私有碰撞网格 FK，不是新增物理重放或正式 worker 成功；硬件 NOT_AUTHORIZED_NOT_RUN。笔六检查点仍未完成，保持 PARTIAL_DELIVERY，未推送。

## M1 / 完整物体静止接入原阶段预算，+4 mm 假设仍未闭环

本轮 progress，生产提交 `53a912b`。正式原生执行器现通过 trusted context 绑定 NativePrimaryCapture.read_settle_state：在原关节静止条件满足时，读取全部注册物体的原生速度；只有机器人和物体共同连续 3 次静止才完成阶段，仍最多原 200 控制步。物体阈值保持线速度范数 <=0.001 m/s、角速度范数 <=0.005 rad/s。只明确有限的物体运动可继续等待，非有限/缺失/身份异常传播，不清零速度、不写位姿、不增加等待预算。此运动读回不是 SWM 检查点，闭爪前独立新采集仍保留。失败包含具体物体、位姿及速度。

先在未改等待逻辑的 worker_44 运行单一规划深度假设：进程内 bi ObjectSpec 从 0.012777 改为 0.016777 m，沿负 approach 方向，不是通用 world-Z 平移。复用正式 worker/main、原 builder、原场景和全部审计；不改生产默认配置。该次候选缓存预测与精确端点预测均未否决，原阶段审计通过，实际主 approach/grasp 执行到闭爪前，但独立采集拒绝物体未静止。没有正式 grasp 成功。

53a912b 的 worker_45 用同一假设复跑：approach 在 14 控制步后全部静止；grasp 满 200 步后机器人最大速度 7.27e-5 rad/s、端点误差 4.73e-5 rad，但 bi 线速度约 [-0.0004234,-0.0012440,0.0000184] m/s，角速度约 [0.0808439,-0.0380244,-0.0025953] rad/s。因此未发闭爪/抬升，worker exit 1。两次主环境、镜像和 planner 清理读回均 true。这个证据不能证明 +4 mm 是正确默认抓取深度，也尚未定位导致笔运动的具体接触对。

内核：定向 **34 passed / 1 failed / 0.13 s**；原生进程退出后串行完整 tests 为 **1808 passed / 1 failed / 1 existing warning / 78.94 s**。唯一失败为新增 test_settle_readback_does_not_turn_nonfinite_feedback_into_wait 预期 ObservationUnavailable，而原底层 _array 对 NaN 更早抛出 SceneInvalid；安全拒绝仍发生，本轮没有修改或跳过此失败断言。下一步先修正该测试的异常类型，再记录抓取到位期间目标接触与运动来源，保留等待预算与全部保护。

分层：原生接线本轮新增完整物体静止等待；实际仿真为上述单一深度假设，不是默认配置已通过；FP/SAM3D/LLM 未运行；硬件 NOT_AUTHORIZED_NOT_RUN。笔六检查点、其余 G0-G10 门槛未完成，保持 PARTIAL_DELIVERY。证据见 benchmarks/release/evidence/pen_closure/worker_44 和 worker_45；本轮本地提交，未推送。

## M1 / 回归恢复全绿，抓取轨迹与初始保持对照区分

本轮 progress，提交 `3425529` 仅修正上一轮新增 NaN 测试的异常期望为原 SceneInvalid；不修改安全拒绝。定向 **35 passed / 0.11 s**，原生作业退出后完整隔离 tests 为 **1809 passed / 1 existing warning / 42.44 s**，53a912b 的失败历史保留。

原深度 +4 mm 进程内假设继续只用于诊断，未写入生产默认配置。worker_46 在关节满足端点条件后记录 192 个物体静止读回，笔只有桌面非零合力、未读到机器人合力。worker_47 扩展到原每控制步：244 个样本覆盖 approach 20、settle_approach 14、grasp 10、settle_grasp 200，仍只有笔/桌面非零合力，笔从 approach 第一个控制步已运动。两次均在 grasp 静止预算处停止，未闭爪/抬升；不能用控制边界零合力声称从未发生接触。

worker_48 做原初始姿态保持对照，不执行已规划轨迹：在同一执行前边界，保持实测原初始七关节与原开爪命令，真实主物理步进 200 控制周期。笔在第 1-8 步运动，第 9-200 步全部满足原静止条件，未读到机器人非零合力。对照明确在结束时抛 diagnostic-complete，中止原子任务，不冒充 worker 成功。无 q/qdot/对象 setter，无场景或物理参数变更，三次资源清理读回全部 true。

该对照否定“原笔在原桌面 200 步内无法静置”的简单解释，但尚不能定位执行抓取轨迹后持续运动的机制。下一实际动作是物理子步中的接触点/separation/唤醒读回，区分瞬时接触、接触裕量影响与其他运动来源；继续保留静止门槛、原总预算和独立新采集，不盲改阻尼、默认深度或碰撞条件。

证据及复现命令：benchmarks/release/evidence/pen_closure/worker_46/README.md，另有 worker_47、worker_48 有界结果与输入/源码摘要。分层状态：内核本轮全绿；原生接线沿用完整物体静止等待；实际仿真为诊断/保持对照而非笔六检查点完成；模型 FP/SAM3D/LLM 未运行；硬件 NOT_AUTHORIZED_NOT_RUN。G0-G10 尚未交齐，保持 PARTIAL_DELIVERY，本地提交，未推送。

## M1 / 子步确认正间距零冲量接触候选伴随唤醒

本轮 progress，生产仍为 `3425529`，未修改生产控制器或默认参数。worker_49 在原主 CPU PhysX scene.step 前后只读采样，244 原控制步对应 **1220 物理子步**、1709 总记录。目标/机器人接触候选出现在 1006 子步，共 21839 个点；全部机器人点 separation 为正，范围 **0.032645259053-0.039997987449 m**，全部 impulse 为零，无目标/机器人穿透点。该数据不是全世界碰撞审核，只覆盖目标相关接触。

第一次睡眠到活动发生在 grasp 的 control 43/substep 3，与左 Support Link/笔约 39.94 mm 正间距、零冲量接触候选同时出现；control 44/substep 1 再次出现睡眠到活动。该证据支持接触裕量关联唤醒，而不是夹爪实体撞击，但不能单凭时间重合认定具体写入操作导致持续唤醒。原 grasp 静止等待仍满 200 步失败，未闭爪/抬升，资源清理读回 true。

worker_50 尝试对照“只有请求与原生读回完全相等时省略驱动 setter，保留 set_action/缓存更新”；尚未得到物理结果，因为诊断错误地断言控制器只有 arm/gripper，在初始化就失败。文本已确认原配置还有 gripper_passive/PassiveControllerConfig。没有省略任何驱动写入，没有运行该假设的动作子步；不能算否定或验证假设。本轮保留失败脚本和身份检查，下一步修正为明确保留被动组、只对正确的两个 PD 控制器进行原生同值对照，再判断是否值得接回正式执行器/预测器。不可缩小接触裕量、放宽静止条件或改 solver 来掩盖问题。

证据：benchmarks/release/evidence/pen_closure/worker_49/substep_summary.json、worker_49/README.md 及 worker_50 失败记录，含输入/原始日志/源码 SHA256。内核仍引用上轮 1809 passed / 1 warning / 42.44 s，非本轮重跑；原生接线未改；实际仿真为子步诊断而非成功原子闭环；模型 FP/SAM3D/LLM 未运行；硬件 NOT_AUTHORIZED_NOT_RUN。所有 G0-G10 尚未交齐，保持 PARTIAL_DELIVERY，未推送。

## M1 / 同值驱动三组对照，尚无同时静止解

本轮 progress，修正 worker_50 的诊断清单，明确保留 arm/gripper/gripper_passive，PassiveController 类型检查保留，原被动组不包装。生产仍为 `3425529`，未改变正式默认深度、控制器、接触裕量或物理参数；本轮不重复运行内核测试，沿用 1809 passed / 1 warning / 42.44 s。

worker_51 同时对 arm/gripper 省略请求与原生读回完全相等的 setter，保留原 set_action/缓存和不同值写入：arm 215 次省略/29 次原写入，gripper 244 次省略/0 次写入。笔在 1220 子步中 1119 次休眠，末态速度零；但 grasp 的 200 步末关节最大速度 0.001589460997 rad/s（joint_6），joint_4=-0.001049020677 rad/s，仍不满足原机器人静止标准。没有把休眠的笔当作整个阶段通过。

worker_52 仅省略 arm 同值写入、worker_53 仅省略 gripper 同值写入：两者机器人末速度均 7.27e-5 rad/s，但笔仍运动，结果与原行为相同。因此需要两组共同处理才改变目标休眠行为，但现证据不支持直接上线双组省略，亦不支持放宽机器人速度阈值。

worker_54 的附加机器人休眠诊断失败：脚本查询了 PhysxArticulation 聚合对象没有暴露的休眠属性，没有取得有效新反馈样本。后续只做网络隔离的类型接口查询，无仿真或设备动作；已确认 PhysxArticulationLinkComponent 有 sleeping，而聚合对象只有 root/get_root 等。下一步应从可信原生 root link 读回休眠状态、绑定身份，并与完整原始 q/qdot 及实际位移对照；不可据接口缺失把速度置零或假定机器人静止。本轮保留该失败，不作物理结论。

四次原生作业均 exit 1，资源清理读回全部 true，无闭爪/抬升成功、无笔六检查点成功。证据和命令见 benchmarks/release/evidence/pen_closure/worker_51/README.md 及 worker_51-54 有界记录。内核沿用上轮全绿；原生生产接线未变；实际仿真为上述对照；FP/SAM3D/LLM 未运行；硬件 NOT_AUTHORIZED_NOT_RUN。保持 PARTIAL_DELIVERY，不标记完成，未推送。
