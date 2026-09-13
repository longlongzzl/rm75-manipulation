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

## M1 / 全链接静止与 qdot 缓存区分，新增接线待修复原生身份映射

本轮 progress。旧生产 3425529 上 worker_55 取得 214 个完整关节读回、192 个根链接休眠样本，最后 20 个位置向量完全相同，而 qdot 缓存仍保留 joint_6=0.001589460997 rad/s。worker_56 进一步绑定全部 20 原生链接；192 个休眠样本的所有链接空间线/角速度均精确为零。原始 qdot 未改写，原静止保护仍拒绝。PhysX 官方休眠语义参考 https://nvidia-omniverse.github.io/PhysX/physx/5.3.0/_api_build/class_px_articulation_reduced_coordinate.html ，文档支持链接空间速度为零，不替代本机缓存差异的实测证据。

WIP 生产提交 `2f48069` 新增 NativeArticulationVelocity：仅完整链接身份、前后全部休眠、空间速度精确零、关节位置不变且原始缓存有限/状态一致时，推导有效零速度；原缓存和证明保留在 primary/SWM/阶段记录中，不写 q/qdot，不改变任何阈值。清醒状态保留原速度，证据不一致拒绝。定向 **47 passed / 0.12 s**，原生退出后串行完整 tests **1817 passed / 1 existing warning / 45.31 s**。

但实际 worker_57 在场景注册时失败 Native velocity joint identity mismatch，尚未进入原子动作。新增 helper 直接使用带命名空间的原生 joint.name，与原 canonical wrapper 名称不匹配；没有复用已有 native_robot_mirror._native_drive_joints 的句柄双射。新测试使用同名夹具，未覆盖这个真实边界。主环境/planner 关闭 true，robot mirror 尚未创建（null），不能填成三项均 true。因此当前接线不能宣称原生可用，测试全绿不代表交付。

下一步修复 native_velocity 的可信 wrapper->native handle 映射，复用 robot.joints_map 和原 _native_drive_joints，保留完整 13 关节身份检查，禁止后缀猜测或删除检查；补命名空间/乱序/外来句柄负例，再重跑同一原生对照。生产默认深度及同值写入策略尚未改变；笔六检查点、其余 G0-G10 未完成。模型未运行；硬件 NOT_AUTHORIZED_NOT_RUN；PARTIAL_DELIVERY，未推送。证据及精确命令见 benchmarks/release/evidence/pen_closure/worker_55/README.md、worker_56 和 worker_57。

## M1 / 原生关节映射修复，实际闭爪后的保持触桌被保护拦截

本轮 progress，生产提交 `120cd1a`。NativeArticulationVelocity 复用原 _native_drive_joints 的 canonical-wrapper/native-handle 双射，按句柄映射底层数组，不猜名称后缀。补命名空间、native 顺序反转和缺失/外来/重复句柄测试。定向 **51 passed / 0.12 s**；原生作业退出后串行完整 tests **1821 passed / 1 existing warning / 45.91 s**。

worker_58 仍使用明确的 +4 mm 深度和双组同值写入省略诊断配置，未设为生产默认。实际通过初始化、独立检查点和原阶段审核，approach 14 步、grasp 轨迹端点 13 步后，机器人有效速度与完整物体状态共同静止；原始 qdot 与全链接休眠证据保留。这里是阶段到位，不是 grasp 原子成功。

随后闭爪前独立新采集与私有预测运行；私有预测只覆盖 20 控制步/100 物理子步，未触发否决但明确不合格模型证明。主世界实际执行 20 个闭爪周期，之后第 **79** 个闭爪保持周期，原保护检出左 Support Link/桌面 Z 向力 **0.537089586258 N**，立即拒绝后续动作，未抬升/放置。所有创建资源清理 true，worker exit 1。没有成功 after-grasp 检查点或六检查点闭环。

下一步补齐私有闭爪预测的原闭合后静止窗口，并对齐实测动作/驱动写入语义；保留原 200 步预算、接触标准、精确端点复核和真实新采集，不能把前 20 步无否决当作完整动作预测成功。证据与准确命令见 benchmarks/release/evidence/pen_closure/worker_58/README.md。内核全绿；原生映射已实跑通过但闭爪尚失败；FP/SAM3D/LLM 未运行；硬件 NOT_AUTHORIZED_NOT_RUN；PARTIAL_DELIVERY，未推送。

## M1 / 控制周期写入共享，闭爪后预测窗口接通

本轮 progress，生产 `eb7f3f0`。新增 NativeDriveCommands，明确原 arm/gripper/gripper_passive 分组；主控制器保留原 set_action/缓存更新，仅请求与实际原生精度目标完全相等时省略整组位置 setter。私有预测共享整组规则，不再在控制周期重写被动位置目标、全部速度目标或时间步；完整镜像初始化仍保留原状态应用。源策略在受管理上下文退出时恢复，阈值、物理参数和原执行预算不变。

私有预测在原 20 控制步闭爪之后增加最多原 200 步 closed_hold，逐物理子步保持禁止接触检查，连续 3 次要求全 13 关节速度、臂端点误差与全 12 物体速度满足原条件；有效睡眠速度及原始缓存分开记录，不把 idle 当作持物或模型成功。定向 **39 passed / 0.09 s**，完整隔离 tests **1831 passed / 1 existing warning / 48.21 s**。

worker_59 只保留 +4 mm 深度假设，不再额外包驱动诊断。实际 72 次私有预测（缓存预筛和精确端点合计）全拒绝：63 次 gripper_close 禁止接触，9 次 closed_hold 满 200 步仍不满足条件。九次臂端点误差 **0.061010956764-0.075673103333 rad**，均超过原 0.02；其中五次机器人/物体已静止但臂偏离，四次机器人与笔仍运动。这不是复现 worker_58 的同一个 0.537 N 晚期触桌事件，不宣称模型一致性；只能证明补全窗口后不再把前 20 步无否决视为足够。

worker_60 另跑不修改深度、不使用诊断脚本的正式默认入口：45 次私有预测全部在 gripper_close 触发接触拒绝。两次均无主 approach/grasp/lift，源控制器写计数为零，因此当前原生运行验证了策略绑定与私有写入，并未新验证源 setter 执行分支。全部私有世界关闭、主状态/驱动未变、最终资源清理 true；worker exit 1，仍无可行笔轨迹。

下一实际动作是核对原接触感知闭爪策略和实际夹持几何，解决闭合后的臂偏离/持物不稳定，而非增大预算或放宽速度/端点/碰撞条件。命令和证据见 benchmarks/release/evidence/pen_closure/worker_59/README.md 及 worker_60。内核全绿；原生接线新增共享控制与完整预测；实际仿真仍未完成六检查点；FP/SAM3D/LLM 未运行；硬件 NOT_AUTHORIZED_NOT_RUN。G0-G10 尚未交齐，PARTIAL_DELIVERY，未推送。

## 2026-09-13 worker_61: private feedback closure evidence

Recovered the terminal result without restarting the simulation. Evidence:
`benchmarks/release/evidence/pen_closure/worker_61/{probe.py,summary.json,feedback_hypothesis.json,result.json}`.
The isolated private hypothesis uses depth 0.016777 m instead of original 0.012777 m and adaptive jaw targets, with unchanged physical parameters and existing collision/idle thresholds. After 84 adaptive controls (20 close plus 64 hold), three stable samples satisfy robot and all 12 object idle checks; endpoint error is 0.0009313821792602539 rad. Final pre-step finger forces are 0.6742882116617259 and 0.6092549420255825 N. This is not a holding-angle test or a lift result.

The worker intentionally reports failed at the diagnostic barrier. Prediction reports no forbidden contact, not skill qualification. Primary drive writes are zero; primary-unchanged and resource-release events were emitted. The nominal closed targets in the base prediction report are NOT the adaptive action: the separate feedback trace records that action.

Kernel: no production change or new regression run in this continuation. Native wiring: formal grasp/place remains incomplete. Actual simulation: private adaptive closure only. Model inference: NOT_RUN. Hardware: NOT_AUTHORIZED_NOT_RUN. Overall: PARTIAL_DELIVERY; no gate promoted.

Next actual implementation: share a bounded feedback closure policy between private prediction and the original primary executor, retain independent holding evidence, and refresh measured jaw/attachment geometry before re-auditing the remaining lift. Do not reuse fixed-close predictions or stale lift geometry. Missing-adapter guards remain intact.

## 2026-09-13: shared feedback closure wiring, not native qualification

Added `native_feedback_closure.py` and connected its bounded policy to both the private closure predictor and `NativePrimaryExecutor`. Both use measured original finger forces, the original semantic command transform, and at most 20 close plus 200 hold controls. Private evidence now includes each adaptive command and measured pre-step forces; the old full-close target is explicitly nominal, not the executed action. Collision checks and object/robot idle limits remain unchanged. A force band does not certify holding.

Primary execution now rejects subsequent trajectories after feedback closure until measured jaw/attachment lift re-audit is installed. This is an explicit incomplete adapter, not a successful grasp or a legacy fallback. Original independent holding checks remain necessary. No production native worker was run with this change yet.

Tests (network isolated, explicit tests scope): new targeted suite 7 passed in 0.05 s; full tests 1837 passed / 1 failed / 1 existing warning in 43.09 s. Failure: `tests/swm/test_native_primary_execution.py::test_original_target_finger_contact_is_not_holding_verification` supplies only left-finger force and expects completion after 23 controls; the new bilateral feedback condition instead exhausts the bounded settle budget. Keep this failure visible. Next change should separate allowed unilateral contact from completed bilateral closure in that test contract, then implement measured-state lift re-audit before claiming the skill available.

Kernel regression: NOT_GREEN. Native wiring: feedback strategy connected, post-close audit adapter incomplete. Actual simulation: no new run; worker_61 remains diagnostic evidence for the earlier implementation only. Model inference: NOT_RUN. Hardware: NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY; G0-G10 completion is unproven.

## 2026-09-13 worker_62: formal original-depth primary closure

Production `0d0accc` ran through the ordinary worker CLI, using original pen depth 0.012777 m and the unchanged frozen table input, not the worker_61 diagnostic override. Primary simulation executed approach, grasp and feedback closure, then correctly rejected the remaining lift with `Measured jaw and attachment lift re-audit is not installed`. Command/task success is NOT established. Evidence: `benchmarks/release/evidence/pen_closure/worker_62/` contains the result, bounded per-command/stage/checkpoint/resource summary and launch command.

The independent actual-preclose private prediction completed 20 close plus 58 hold controls, three stable samples, endpoint error 0.0025734901428222656 rad, all robot/object idle, and no forbidden contact detected. These are PRIVATE prediction metrics, not primary endpoint metrics or model-agreement proof. Its final pre-step finger force norms were 1.7707003173761935 / 1.773088389949838 N. The source remained unchanged during private stepping. Primary drive release reported arm 29 writes / 99 exact-equal skips and gripper 60 writes / 68 skips. Native resource-release flags were true.

Test responsibility corrected without deleting the unilateral-contact negative: allowed single-finger contact now must exhaust the bounded feedback settle budget rather than imply bilateral closure. A separate bilateral fixture completes closure but proves the unaudited lift remains blocked. Targeted 25 passed (0.08 s); full official tests 1839 passed / 1 existing warning (42.64 s), terminal exit 0. These fixtures do not certify physical holding.

Kernel: regression green. Native wiring: primary feedback closure actually exercised; measured postclosure lift adapter remains missing. Actual simulation: partial primary execution, NOT completed grasp/place or six-checkpoint closure. Model inference: NOT_RUN. Hardware: NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY. Next actual implementation is fresh complete postclosure capture, original independent holding validation, and the existing native auditor applied to remaining lift with measured jaw and attachment state; do not merely clear the guard.

## 2026-09-13 worker_63 / 3ba7f6f: measured postclosure checkpoint and lift audit

The formal native context now installs `MeasuredLiftAudit` in the original primary executor. Before remaining lift, it calls the existing full checkpoint synchronizer at `after_close_before_lift`, requires independent holding of bi, original seven-arm/six-jaw identities and idle feedback, derives measured attachment, and applies the existing native stage auditor to the remaining trajectory. Only its start is replaced with measured q within the original 0.02 rad endpoint allowance; endpoint/timing are retained. Missing adapter, incomplete observation or failed audit still prevents execution.

Actual original-depth worker_63 reached four distinct snapshot IDs: initialization, before_grasp, pre_execute_grasp and after_close_before_lift. Primary stage settling: approach 14 controls (error 0.00002384185791015625 rad), grasp 13 (0.00017702579498291016 rad), closed hold 51 (0.00287020206451416 rad). The new measured holding/jaw checks passed far enough to invoke native collision auditing, which rejected `Native escape introduced a new collision/link`. No lift ran; no grasp/place success or six-checkpoint completion. Resources released normally. Evidence: `benchmarks/release/evidence/pen_closure/worker_63/`.

Kernel regression: targeted 30 passed / 1 failed (0.10 s); full 1844 passed / 1 failed / 1 existing warning (46.49 s), terminal exit 1. The new failing assertion directly compares canonical jaw tuple pairs against the input dictionary; do not call this green. Native wiring: postclosure capture/audit actually exercised, remaining path rejected. Actual simulation: partial primary execution only. Model inference: NOT_RUN. Hardware: NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY.

Next actual work: correct the representation-only assertion, persist exact rejected collision pairs (the new adapter currently emits success evidence only), and use the original coordinator to replan the remaining lift from measured jaw/attachment state rather than forcing the old candidate through. Preserve escape/collision limits and incomplete-skill checks. The new checkpoint is not a substitute for after_grasp or after_place.

## 2026-09-13 worker_64 / 3902fb2: exact measured-jaw collision rejection

The existing native stage auditor now retains at most 64 collision rows, total count, sampled-path size, original escape allowlist and truncation status. The measured-lift adapter emits this evidence on failure and rethrows the original exception; no success fallback or relaxed collision policy was added. The canonical tuple/dictionary assertion is corrected. Targeted 33 passed (0.42 s); full formal network-isolated tests 1846 passed / 1 existing warning (45.36 s), terminal exit 0.

Formal original-depth worker_64 reproduced postclosure rejection before lift. Its native GPU sampled-path audit reports 54 contacts across 45 samples, none truncated: right Support_Link/table at samples 0-2 (maximum 0.00038554519414901733 m); attached payload/table at samples 0-5 (maximum 0.005577594041824341 m); left/right Support_Link self-collision at all 45 samples (approximately 0.0016985 m). Original escape links remain attached_object/left_pad/right_pad. These are planning collision-model diagnostics, NOT newly measured physical mesh penetration or hardware contact. Complete bounded rows and measured six-jaw positions are in `benchmarks/release/evidence/pen_closure/worker_64/summary.json`.

This changes the next action: a different arm lift path alone cannot be presumed to resolve self-overlap between jaws held at fixed measured positions. Compare actual native collision meshes and planning sphere coverage/identity at that same measured configuration before choosing a geometry correction or grasp adjustment. Do not ignore Support_Link pairs, broaden the escape allowlist or shrink spheres merely to pass. Preserve independent full postclosure capture, original holding predicate and sampled native audits.

Kernel: full regression green at 3902fb2. Native wiring: postclosure observation/audit exercised, lift rejected. Actual simulation: partial primary execution, no completed grasp/place. Model inference: NOT_RUN. Hardware: NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY; all unfinished task/skill adapters remain unavailable. The report does not promote a green software suite into native task success.

## 2026-09-13 gripper_geometry_02: conservative support-sphere rejection isolated

At worker_64's exact measured postclosure snapshot, an owned private native articulation was initialized for FK only: zero physics steps, no primary world, no device access, terminal exit 0. Each original support link has one actual native convex collision mesh with 25 vertices. A separating-axis projection proves at least 0.02502849625110304 m between the two convex meshes (a lower bound, not an exact nearest-distance claim). Native FK agrees with original URDF six-jaw transforms to below 4.4e-7 maximum matrix error. The private robot closed normally.

Original YAML spheres transformed by those native link poses have 0.011301566541410148 m raw pair clearance. Global sphere inflation consumes 0.005 m per pair; original self-collision padding consumes another 0.008 m, yielding 0.001698433458589852 m overlap, closely matching worker_64's independent GPU diagnostic. Right support native mesh minimum base Z is 0.0031289186202737435 m; the coarse sphere minimum is 0.002114213968366382 m and becomes -0.000385786031633618 m after the original 0.0025 m global buffer. This explains the recorded table rejection without claiming the native mesh penetrates the table. Native contact offset remains 0.01 m and was not modified.

Evidence: `benchmarks/release/evidence/pen_closure/gripper_geometry_02/{probe.py,summary.json,command.txt}`. This is native FK/configuration-geometry evidence, not another successful task, physical replay, GPU readback or hardware test. No production files changed; previous full regression remains 1846 passed at 3902fb2, not rerun for this diagnostic.

Next actual implementation: investigate tighter conservative support-link sphere coverage while retaining original global/self buffers, every collision pair, and full native convex-volume coverage. A fit must prove coverage of the collision volume, not just sampled vertices or visual surfaces; no shrinking radii or deleting spheres merely to pass this pose. If coverage/capacity cannot be established, keep the current rejection. Lift replanning alone cannot fix the fixed-jaw conservative overlap. Overall PARTIAL_DELIVERY; kernel baseline green, native task incomplete, model inference NOT_RUN, hardware NOT_AUTHORIZED_NOT_RUN.

## 2026-09-13 gripper_cover_01: bounded convex-volume cover (13567cea1e098495b2bef52d77b9b898d06e0787)

Implemented a geometry helper based on convex-hull centroid/face tetrahedra and longest-edge bisection. Each successful leaf sphere contains all tetrahedron vertices, therefore the complete tetrahedron by convexity; output float32 centers/radii are checked with outward rounding. Partition volume consistency and maximum hull-halfspace excess are reported. This is not merely surface point sampling, and halfspace excess is not mislabeled as Euclidean Hausdorff distance. Successful helper output still explicitly says not deployable until native capacity and consumer binding are validated.

Targeted software tests: 6 passed in 0.12 s under required network isolation. Actual native support collision meshes were then fitted in an owned private articulation with zero physics steps and no primary world. Both exceeded the frozen 8192-partition/sphere budget: worst remaining halfspace excess 0.0019401842279060336 m versus required 0.0005 m. Diagnostic process exit 0 means evidence collection completed, NOT a successful fit. The helper failed closed and no sphere configuration, padding, self-collision pair, or task guard was changed. Private robot closed normally.

Evidence: `benchmarks/release/evidence/pen_closure/gripper_cover_01/`. The full suite was not rerun for this unbound helper; last full result remains 1846 passed at 3902fb2. Next actual work is an improved bounded sphere reuse/coverage method; one independent centroid ball per tetrahedron is too inefficient for this frozen tightness. Do not simply enlarge the budget or relax tightness to label the fit successful. Then validate native consumer capacity and full geometric coverage before any production replacement.

Kernel: new helper targeted tests pass; native wiring unchanged. Actual simulation: no new task run, geometry fit failed capacity. Model inference NOT_RUN. Hardware NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY; formal grasp/place and full G0-G10 remain incomplete.

## 2026-09-13 gripper_cover_02 / 5fbe221: sphere reuse remains capacity-rejected

The unbound conservative-cover helper now permits one float32 ball to contain multiple complete tetrahedra, while each ball stays within the same expanded hull halfspaces. Both candidate sphere count and proof partition count remain bounded by 8192, with unchanged 0.0005 m tightness. A cube regression proves one sphere can cover all 12 root tetrahedra; partition-volume and all-vertex containment checks remain mandatory. Seven targeted tests passed in 0.12 s. This is not production collision configuration or an installed skill.

Actual native left/right support meshes were fitted again, using zero physics steps and an owned private articulation. Both fits failed the proof-partition capacity: left 7668 candidate spheres / 8192 cells, right 7665 / 8192. Neither has complete coverage evidence; both remain deployable=false. Private robot closed, diagnostic terminal exit 0. The same process was awaited to completion, not restarted. Evidence: `benchmarks/release/evidence/pen_closure/gripper_cover_02/`.

The two bounded fitting results reject further blind subdivision tuning as the next immediate step. Evaluate exact convex-geometry narrow-phase certificates under the original global/self clearances, retaining coarse sphere detections as broad phase. Such a certificate must cover the same native volume, current six-jaw state and pair-specific original margins; unknown/missing evidence must retain rejection. This is a proposed next implementation, NOT permission to disable pairs, waive table collisions or weaken safety margins. Native planner/executor integration and later full path validation remain required.

No production geometry or safety parameters changed, and no full suite was rerun for this unbound algorithm. Previous full result remains 1846 passed at 3902fb2. Native grasp/place remains incomplete. Model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN; overall PARTIAL_DELIVERY.

## 2026-09-13 gripper_clearance_01: advisory separation certificate (db7432b43eea57273458e3f8c6e38e4c340bb81b)

Added an advisory full-convex-volume separating-axis certificate with explicit original clearance terms, outward numeric projection guards, vertex identities/digests and bounded input/axis counts. No sufficient axis means an inconclusive certificate, not proven penetration. Every result explicitly denies execution authorization and policy override. Seven targeted tests passed in 0.10 s; no full regression rerun for this unbound helper.

At worker_64's exact independently observed jaw configuration, actual native support collision convexes have conservative separation lower bound 0.025028496251031818 m. Original pair margins total 0.013000000000000001 m (2 x 0.0025 global plus 2 x 0.004 self), leaving at least 0.012028496251031816 m. All native shape pairs in these two support links were included. This is an advisory certificate for that measured configuration, not all paths/jaw configurations, not a table-clearance certificate and not grasp/place success. Private native FK used zero physics steps and no primary world; resources closed; terminal exit 0.

Evidence: `benchmarks/release/evidence/pen_closure/gripper_clearance_01/`. Changing formal behavior from unconditional coarse-sphere rejection to a certified convex narrow phase is an execution-rule change. Separate user approval is requested before production installation, even with unchanged clearances. Until then, existing native audits continue to reject exactly as before; no collision pair is ignored and all missing-adapter checks remain. This does not block independent software/feedback work elsewhere in the full goal.

Kernel: targeted advisory-helper tests pass; previous full result remains 1846 passed at 3902fb2. Native wiring and task simulation: unchanged/incomplete. Model inference NOT_RUN. Hardware NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY.

## 2026-09-13 S5 feedback query timing / 9c75694

No separate collision-policy approval has arrived. Formal coarse collision rejection and hardware prohibitions are unchanged. Independent S5 work now records host query start/completion/duration around the shared RealMan executor's existing actual joint-feedback read, without adding another query. Samples explicitly label captured_at as host query completion, not device acquisition time; device_sample_time_known=false and hardware_frequency_qualified=false. Invalid actual values, joint identities and backward/nonfinite query clocks reject before recorder delivery.

The executor retains only the most recent synchronous observer timing, including callback success/failure and query-plus-callback duration. No unbounded timing collection was added. The callback error is preserved under the tested valid clock path. This is useful software timing instrumentation, not measured SDK/device latency, clock calibration, hardware frequency qualification, TCP FK integration or completed parameter feedback closure.

Targeted 11 passed (0.46 s). Full formal tests: 1866 passed / 1 existing warning in 66.23 s, terminal exit 0 at source 9c75694. All ran through tools/run_network_isolated.py with explicit tests/ scope and PYTEST_DISABLE_PLUGIN_AUTOLOAD=1; new timing tests use a pure fake session, no SDK imports, no device commands or connection. No native task/model run was added this turn.

Next independent S5 work: bind measured joint feedback to original calibrated TCP FK and explicit clock provenance, then actual action recording through post-observation coverage and parameter-version consumption. Kernel regression green; native task wiring remains incomplete; actual grasp/place still blocked at measured lift collision audit; model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY, not SOFTWARE_DELIVERY_READY.

## 2026-09-13 worker_65 / cbf372b: actual action-bound native TCP feedback

The shared primitive executor now binds its generated actual_action_id before first execution and releases the feedback scope on success/error. The original primary sink emits measured TCP/joint feedback before the atomic action and at control/settle boundaries. It reads the original native TCP link and robot base transform at measured q, checks unchanged joint state across the read interval, records complete six-jaw feedback, and labels the TCP coordinate frame as base_link. No commanded trajectory point is substituted for measurement. No hardware clock or physical simulation time is fabricated.

Formal original-input worker_65 produced 130 native feedback rows under one actual_action_id, spanning before_atomic, approach, grasp, gripper_close and their settle stages. It then failed the unchanged native lift collision audit; no lift or completed grasp/place. First/last native TCP and measured joints, host read intervals, sequence numbers and actual action identity are retained in `benchmarks/release/evidence/pen_closure/worker_65/summary.json`. Resource release flags were true. These records are not yet physical-time-axis-qualified or post-exposure-covering; no identification or posterior update is claimed.

Targeted tests 25 passed (0.21 s). Full official isolated regression 1868 passed / 1 existing warning (43.36 s), terminal exit 0 at cbf372b. Native runtime action/TCP emission was actually exercised, unlike fixture-only FK wiring. Remaining S5 work includes simulation physical-time mapping, post-observation coverage and recorder/transition/posterior/next-planner binding. The failed action must not be fitted as a successful primitive.

Kernel regression green. Native wiring: partial, measured action feedback installed. Actual simulation: partial primary action only; model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Collision-policy approval remains pending and formal rejection is unchanged. Overall PARTIAL_DELIVERY; G0-G10 completion is not established.

## 2026-09-13 worker_66 / 5490333: native physical clock shared by feedback and checkpoint

Bound a new clock epoch to the owned ManiSkill environment's actual elapsed_steps after trusted initialization. The installed original environment executes _sim_steps_per_control scene steps and only then increments elapsed_steps. Clock reads freeze the scene/physics identity, native timestep and control/simulation frequencies; regressed counters or changed periods reject. Native moving TCP feedback and idle robot observations now carry this clock, separately from host query times.

Formal worker_66 produced 130 feedback rows under one action and epoch. The physical clock advances from 0 to 128 control steps / 640 PhysX substeps, using native dt 0.009999999776482582 s, ending at 6.3999998569488525 s. The independent after_close_before_lift snapshot contains the same epoch, counter and physical time. The worker still fails the unchanged native collision audit before lift; no successful primitive or parameter fit. Resource-release flags were true. Compact evidence: `benchmarks/release/evidence/pen_closure/worker_66/`.

Tests: targeted 26 passed / 1 failed (0.10 s); full 1874 passed / 1 failed / 1 existing warning (46.39 s), exit 1. NaN is rejected by the existing _array helper as SceneInvalid, while the new test expects ObservationUnavailable. Also identified: _array casts booleans to float before the new dtype check, so boolean-counter rejection is currently ineffective. Fix both explicitly before claiming the new clock input contract complete. The native run used the actual integer environment counter, not these malformed fixtures.

Next work: preserve counter type through validation, reconcile nonfinite error semantics, then use the shared physical epoch for post-observation recorder coverage and transition/replay. A failed action is not eligible for a successful-primitive fit. Kernel regression NOT_GREEN; native physical time wiring exercised but remaining recorder/posterior consumption incomplete. Model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Collision-policy approval remains pending. Overall PARTIAL_DELIVERY.

## 2026-09-13 physical counter input fix / fde3a92

NativeSimulationClock now preserves the source counter dtype rather than routing it through the float-converting _array helper. Only finite integral native arrays in the exact supported range are accepted. Boolean, float (including integral-valued float), fractional, string/object, NaN and out-of-range counters reject as ObservationUnavailable; a failed read does not advance the last accepted counter. This closes worker_66's reported NaN exception mismatch and ineffective boolean check without changing native timestep, epoch, collision rules or device permissions.

Targeted 14 passed (0.02 s). Full formal tests 1882 passed / 1 existing warning (42.48 s), terminal exit 0 at fde3a92; both invoked through network isolation with explicit tests/ collection. No new native task or model run this turn. Prior worker_66 physical-clock evidence retains its original source attribution, not retroactively relabeled as a native run of this fix.

Next actual work remains the shared physical epoch's post-observation recorder coverage and transition/replay wiring. Formal pen grasp/place is still incomplete at the unchanged lift collision audit; separate collision-policy approval remains pending. Kernel regression green, native/task/parameter-loop gates incomplete, model inference NOT_RUN, hardware NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY.

## 2026-09-13 worker_67 / 9d84b8f: complete native object clocks and replay time projection

NativePrimaryCapture now checks that the owned physical clock stayed unchanged through the complete object batch and attaches that same clock to every object observation. Formal original-scene worker_67 confirms all 12 objects and robot retain epoch 7781991974fc4a90beb449b3ccbe2ab5 at control counter 128 / 640 physical substeps in after_close_before_lift. It still fails the unchanged native lift collision audit. Evidence: `benchmarks/release/evidence/pen_closure/worker_67/`. No successful grasp/place, push transition, replay or posterior update occurred.

The existing MeasuredFeedbackRecorder now preserves per-sample simulation clocks when present. The existing bind_measured_transition path invokes a strict physical-time projection for native-clock recordings, retaining original host snapshots and receipt times. Both host exposures must be covered; objects/robot/tool must share a consistent physical epoch and fixed timestep. Identical repeated reads at one physical instant may be deduplicated; differing poses, missing clocks, regressed/mixed epochs and insufficient coverage reject. This code wiring does not mean the worker's post-exposure recorder lifecycle or original PushT factory is installed.

Targeted 15 passed / 1 failed (0.49 s). Full 1887 passed / 1 failed / 1 existing warning (45.71 s), exit 1. The new same-time-motion negative fixture reused one pose object three times; deepcopy preserves aliasing, so its mutation changed all three poses. Fix independent fixture poses and add a full recorder-to-transition integration check before declaring the new projection validated. No safety check was removed to make tests pass.

Kernel NOT_GREEN; native full-batch clock evidence exercised; remaining post-observation recording, transition/replay and next-planner posterior consumption incomplete. Model inference NOT_RUN. Hardware NOT_AUTHORIZED_NOT_RUN. Separate collision-policy approval remains pending; formal rejection unchanged. Overall PARTIAL_DELIVERY.

## 2026-09-13 recorder-to-transition contract / 5932037

Fixed independent pose allocation in the same-physical-time/different-pose negative fixture. Full contract inspection also found that validate_transition accepted only the historical simulator_ground_truth physics source, not the current native_primary_PhysX_readback source. The latter is now accepted only with consistent native object/robot physical-clock projection evidence and duration; existing domain, immutable snapshot digest, actual action, observation acceptance, support and initial-pose checks remain. Historical replay provenance remains supported.

A complete MeasuredFeedbackRecorder begin/finish/transition fixture now proves that missing after-exposure coverage rejects without discarding the active recording. After the final measured sample arrives, the original binder emits physical action times [0, 0.5, 1.0], object times [0, 1.0], retains the original host-time snapshot, and releases the handle/window. Removing physical projection evidence from that native-shaped transition rejects. These are explicitly physics-shaped SOFTWARE FIXTURES, not a new native push, four-parameter replay, fit or posterior consumption result.

Targeted 20 passed (0.22 s). Full formal isolated tests 1889 passed / 1 existing warning (45.84 s), terminal exit 0 at 5932037. No new native worker/model run this turn. Kernel regression green; native recorder post-exposure lifecycle still must be installed in an actual successful PushT worker and connected through replay/posterior to the next planner. Pen grasp/place remains incomplete at unchanged lift collision rejection. Collision-policy approval pending; hardware NOT_AUTHORIZED_NOT_RUN; model inference NOT_RUN; overall PARTIAL_DELIVERY.

## 2026-09-13 PushT original-session native feedback boundary

The original PhysicsSession now samples the articulated native TCP link, seven arm joints and six jaw joints after actual control steps, alongside the existing object observation stream. Reads reject nonfinite values, changed joint state and changed native physical clock. A generated action identity is retained through subsequent post-action observations; action-start feedback is emitted separately. The kinematic-tool branch returns no measured tool feedback, rather than relabeling commanded FK as measurement. Existing planner, executor, contact checks and missing-adapter gates are unchanged.

Kernel: targeted 4 passed (0.03 s); full network-isolated tests/ regression 1893 passed / 1 existing warning (45.75 s), exit 0. These tests use software fixtures and do not prove an actual PushT run. Native wiring: original control-step producer installed, not yet a registered SWM PushT runtime. Actual simulation for this change NOT_RUN; model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Prior native pen and four-parameter replay evidence retain their original code attribution.

Next: connect complete independent scene checkpoints and an owned recorder lifecycle to the original bounded push stages, then replay the same measured action and consume its posterior in the next original planner call. The old retreat collision exemption is not a complete SWM stage audit. Pen remains incomplete at the unchanged measured lift collision rejection; no collision policy was relaxed. Overall PARTIAL_DELIVERY, not SOFTWARE_DELIVERY_READY.

## 2026-09-13 PushT native worker_01 at 25ab4a5

Ran the existing frozen translation case through WorkcellService and its real simulation worker, under tools/run_network_isolated.py, full_arm_physics CPU PhysX, internal gripper collision checking enabled, original default gravity compensation none, 180 s timeout. Job d0079fb3a9ee450ba3b0278d1eecc390 terminated after 82.9866 wall seconds. Harness exit 0 is NOT task success: worker status failed with Physics forbidden early-target or static-obstacle contact.

The original planner produced one complete chain. Actual physics advanced 1734 control steps (legacy accumulated time 57.80000000001903 s) and wrote 1734 observation rows. The inspected first row contains original native TCP, seven measured arm joints, six jaw joints and native epoch 0a03ef54fc6b475f860d3479cb91c926, 30 Hz control / 240 Hz simulation / eight substeps, actual dt 0.004166666883975267 s. This establishes that the new reader runs in actual PhysX, not merely a fixture. No full action-window validity or successful transition is claimed.

Failure is specific: gripper_Right_Support_Link contacted simulation_table during retreat at legacy time 57.79583333335236 s, impulse 0.021869080141186714 Ns. There were no early-target or unapproved target-link contacts; one static-obstacle contact triggered the existing rejection. Legacy retreat_collision_checks is false, although internal gripper checking is enabled. Peak measured joint tracking error was 0.023855859623156173 rad and TCP position discrepancy 0.011488937733728844 m. These are observations, not acceptable tolerances. No safety gate was relaxed and this failed primitive is not eligible for fitting.

Committed bounded evidence, frozen machine/request and source-hashed result: benchmarks/release/evidence/pusht_feedback/worker_01/. Full per-step trace and video remain under runtime_data/workcell/jobs/d0079fb3a9ee450ba3b0278d1eecc390/physics/. Next action changes from merely wiring a producer to auditing/replanning the original retreat stage under complete geometry and measured state before SWM factory admission. Also retain independent scene checkpoint, recorder, replay and posterior-to-next-planner requirements. Pen remains incomplete at its separate unchanged lift collision rejection.

Kernel: no code changes this turn; prior 25ab4a5 regression remains 1893 passed / 1 warning. Native wiring: producer exercised, SWM runtime incomplete. Actual simulation: FAILED_RETREAT_TABLE_CONTACT. Model inference NOT_RUN. Hardware NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY; no G0-G10 or SOFTWARE_DELIVERY_READY claim.

## 2026-09-13 PushT retreat rejection gate and native worker_02

The original physics planner now subclasses the existing CuroboPushExecutor only to apply its native exhaustive collision audit to all timed retreat samples after the legacy generator has restored collision checks. No target contact permission is granted. This occurs inside original candidate selection, before publishing a complete chain. This additional gate uses the measured pre-push scene, NOT a verified post-push scene; legacy retreat_collision_checks remains false rather than falsely certifying the generator or complete SWM stage audit.

Same frozen full-arm translation case, internal gripper checking enabled, network isolated, 180 s cap: job 8c4eac4b9e004f14a56633cdc65f69a1 reached initialization and planning, then cooperatively cancelled at timeout. Final worker result is cancelled; the harness snapshot was still running before close and is not authoritative final status. Physics summary contains 61 initialization control steps / 2.033333333333329 legacy seconds, zero completed plans and zero target/static contacts. No push, successful retreat, transition or fit. Bounded source-hashed harness result, final worker result and physics summary are committed under benchmarks/release/evidence/pusht_feedback/worker_02/.

Kernel: 3 targeted fixture tests passed (0.01 s); full isolated tests/ 1896 passed / 1 existing warning (47.40 s), exit 0. Native wiring: additional rejection gate installed; actual simulation: planning timeout, not success. Model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Next work must persist bounded candidate rejection diagnostics during planning and replan/audit retreat against post-push measured geometry, not remove the gate to regain an executable legacy chain. SWM factory, complete independent checkpoints and replay/posterior consumption remain incomplete. Overall PARTIAL_DELIVERY; pen collision rules and missing-adapter guards unchanged.

## 2026-09-13 PushT worker_03: stage-scene error identified

Added immediate bounded last_retreat_rejection.json persistence in the original planning child. Native audit failures retain scope, timed sample count, endpoint joints and source observation before propagating unchanged. Candidate search cancellation no longer loses its last retreat rejection. This does not grant execution permission or change collision filtering.

Actual network-isolated frozen full-arm translation run, internal gripper checking enabled, job 17db870355e14697a8351004cc14a69a: cooperative cancellation at the 60 s planning timeout. Captured native rejection audits 266 timed retreat samples against the pre-push target. At sample 0, gripper_Right_Support_Link penetrates pusht_target_0 by 0.018787501379847527 m and right_pad by 0.009477505460381508 m in the planner's collision model. This is a planning-state observation, NOT physical penetration in an executed action.

Known defect in the preceding gate: pre-push target geometry is not the retreat-stage target geometry. A strict rejection against that stale target can block otherwise feasible candidates, and does not qualify the post-push stage. The next repair must use stage-correct predicted geometry for joint feasibility and an independently measured post-push scene before executing retreat; target collisions must not simply be ignored. The original worker_01 real retreat/table contact remains a separate unresolved physical safety failure. No successful push, replay, fit or posterior-to-next-plan result.

Evidence: benchmarks/release/evidence/pusht_feedback/worker_03/. Kernel: 3 targeted passed (0.01 s), full isolated tests/ 1896 passed / 1 existing warning (47.40 s), exit 0; this does not test away the known native stage-scene defect. Native wiring partial, actual simulation cancelled, model inference NOT_RUN, hardware NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY; missing-adapter and collision checks preserved.

## 2026-09-13 PushT worker_04: predicted stage-scene repair

Repaired stale pre-push geometry in the physics planner's added retreat gate. Reuses original predict, friction_scales and surface_contact_xyz binding; audits each timed retreat in predicted post-push private scenes and restores the observed planning scene in finally. Predictions are explicitly prediction, never measured observations. No target deletion, primary setter or collision exemption was added.

Actual isolated full-arm translation, internal gripper checking enabled, 60 s cap: job d78a0f24ec96482bbfd6126b56058aa3 cancelled during planning. Native rejection scope is now predicted_post_push_ensemble, 266 timed samples, right support against pusht_target_0, first reported penetration 0.009140042588114738 m. This is predicted collision, not physical penetration or successful retreat. Evidence: benchmarks/release/evidence/pusht_feedback/worker_04/.

Kernel: 5 targeted passed (0.02 s), full isolated tests/ 1898 passed / 1 existing warning (43.83 s). Native wiring partial; actual simulation cancelled; model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Stage contact state/retreat generation, independent post-push measurement, SWM runtime and replay/posterior consumption remain incomplete. Earlier actual retreat/table contact is unresolved. Overall PARTIAL_DELIVERY; missing adapters and pen collision checks preserved.

## 2026-09-13 PushT original stage execution boundaries

Original TimedProgram now exposes copied, time-clamped stage programs preserving original samples, timings and order. PhysicsSession drives these stages separately and collects a new physical observation before and after each stage, emitting action-bound object and native tool feedback. Waiting at a boundary cannot automatically advance into the successor. The native environment preserves the current stage's contact classification during holds rather than classifying all holds as post_settle. This changes the original continuous execution into observable stage boundaries; it is not a new planner or whole-task SWM wrapper. Events explicitly set swm_scene_audit_verified=false.

Known execution-order gap identified after implementation: the new first before-stage observation advances physics after the old initial drift check. Actual start must be checked again after capture before commanding the stage. No native execution was launched with this change. Full measured scene audit and stage replanning remain uninstalled, and this implementation is not qualified for SWM factory admission. Fix this gap before an actual run; green regression does not supersede it.

Kernel: 3 targeted software tests passed (0.02 s); full isolated tests/ 1901 passed / 1 existing warning (42.41 s), exit 0. Actual simulation NOT_RUN_THIS_CHANGE; model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Prior native retreat rejection evidence retains its previous code attribution. Next: enforce post-capture measured start and stage state, then install independent post-push audit/replanning and recorder completion. Overall PARTIAL_DELIVERY; no primitive, parameter loop or task completion claim.

## 2026-09-13 PushT post-capture start guard repair

Fixed the execution-order gap reported with 4de2c40. Immediately before installing each clamped stage, after its before-stage physical observation, PhysicsSession reads actual joints again and requires seven finite actual/expected values and the original 1e-5 rad start agreement. Drift/nonfinite/wrong-shape feedback rejects without changing the active stage. The report stores the last stage, measured gap, unchanged limit and post-capture scope. This does not rebase commanded points or loosen collision rules. It also does not replace full stage scene auditing: a drifted native stage still requires measured-state replanning before execution can continue.

Kernel: 9 targeted fixture tests passed (0.03 s), full network-isolated tests/ 1907 passed / 1 existing warning (66.46 s), exit 0. Native wiring ordering repaired; actual simulation NOT_RUN_THIS_CHANGE; model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Previous native candidate rejection remains unresolved and is not retroactively relabeled as evidence for this change. Next work remains actual measured-state stage planning, independent scene audit and successful original task execution, then recorder/replay/posterior consumption. Overall PARTIAL_DELIVERY; no completed SWM runtime, atomic task or SOFTWARE_DELIVERY_READY claim.

## 2026-09-13 measured single-stage planner protocol

Installed a planning-only stage operation in the original PushT GPU child and connected PhysicsSession's post-capture boundary to it. The stage operation uses original native FK/planner, Cartesian IK, time_parameterize and collision/TCP audits; retreat reuses original backoff/lift generation then re-audits without inheriting its collision exemption. Worker response checks bind stage, source observation and measured start, followed by the original post-capture start guard. Timed stage IPC inputs now require finite bounded arrays and strict times. Existing original chain feasibility search and missing SWM adapters remain.

Kernel: 11 targeted fixtures passed (0.04 s); full isolated tests/ 1909 passed / 1 existing warning (42.71 s). Actual planning-only GPU protocol diagnostic reused worker_04's recorded native first observation/joints, with an approach endpoint equal to that recorded q. No live primary world, device or trajectory execution. Process exited 0 but returned stage failure: AttributeError: 'AuditedPhysicsPushExecutor' object has no attribute 'names'. This exposes original execution-context initialization skipped by the new stage entry, not a valid plan. Evidence and exact frozen protocol request: benchmarks/release/evidence/pusht_feedback/stage_planner_01/.

Known issue to fix next: initialize canonical original planner joint identity before using original FK/auditor on the new entry, and exercise native stage planning again. Do not replace this with guessed joints or a successful fixture response. Actual simulation NOT_RUN_THIS_CHANGE; native stage protocol NOT_USABLE; model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Full scene/SWM verification, successful push, replay and posterior-to-next-plan remain incomplete. Overall PARTIAL_DELIVERY.

## 2026-09-13 original stage joint identity fix and actual GPU planning

Repaired the names initialization gap from stage_planner_01. The measured-stage entry reads joint_names from the original native planner, requires exact ordered joint_1 through joint_7, and initializes the original executor FK/auditor identity. Actual start q now explicitly rejects nonfinite values. No guessed or reordered joint identity is accepted.

Repeated the original recorded-feedback zero-displacement approach request in the actual GPU child: stage_validated=true, no error. Also ran a nonzero diagnostic target, changing goal joint_1 by 0.02 rad while retaining the original recorded start and observation. The original Cartesian-pose planner returned an equivalent redundant-arm solution, 49 timed samples / 1.6000000000000016 s, native planning position error 2.6061726998705126e-7 m and orientation error 1.3638354801059904e-7 rad. Existing TCP and collision audits completed. This proves a nonzero measured-start approach can pass the native stage protocol, not that every stage or the task executes. Evidence: benchmarks/release/evidence/pusht_feedback/stage_planner_02/ and stage_planner_03/.

Kernel: 3 targeted passed (0.03 s); full isolated tests/ 1910 passed / 1 existing warning (42.38 s), exit 0. Both GPU diagnostics were planning-only under network isolation using recorded physical data; no primary world or device was driven. Actual simulation NOT_RUN_THIS_CHANGE, model inference NOT_RUN, hardware NOT_AUTHORIZED_NOT_RUN. Native retreat feasibility, independent complete SWM checkpoints and successful feedback/replay/posterior loop remain incomplete. Overall PARTIAL_DELIVERY.

## 2026-09-13 actual original-session measured approach execution

Added a bounded diagnostic entry tools/run_pusht_measured_stage_probe.py using the original PhysicsSession, scene, drives, EventLog and StopToken. It obtains a new observation and native joint/TCP feedback, requests original stage replanning for a small approach target, rechecks actual start, executes native physics and obtains an independent after observation. No whole-task wrapper or SWM factory is installed by this probe; task_success and swm_scene_audit_verified remain false.

Actual CPU PhysX run completed under network isolation: 53 planned samples / 1.7333333333333354 s; observations sequence 1 then 2 in session 22d27e9b6bf441ceb1c5dc976f0986a9. Native clock epoch c6cf8eefe1e5420b878b073c2a6d061f advances from control counter 61 to 145, physical time 2.0333334393799305 to 4.83333358541131 s. Action identity 7bb6ea7f2b6e45f895c93d2d90e4727f binds native feedback. Final joint error 0.009797009639441967 rad is within the original 0.02 rad endpoint gate; it is not within the old 1e-5 rad next-start gate, illustrating why measured-state replanning is necessary. Resources_closed=true; process exit 0. T remained unchanged, as expected for this noncontact approach. No parameter fit is appropriate.

Bounded evidence: benchmarks/release/evidence/pusht_feedback/measured_stage_01/; full per-control traces and video remain runtime_data/swm_release_pusht_measured_stage_01/. Kernel: full isolated tests/ 1910 passed / 1 existing warning (44.38 s), exit 0. Native wiring and actual simulation exercised for this one diagnostic approach only; not complete PushT, SWM capture/mirror proof or posterior consumption. Model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Next: exercise successive measured-state stages and resolve actual retreat/contact feasibility without dropping collision checks, then admit the proper SWM runtime and measured transition lifecycle. Pen and other original G0-G10 gates remain incomplete; overall PARTIAL_DELIVERY.

## 2026-09-13 strict contact escape and two actual full push cycles

Reviewed the original worker_01 measured push endpoint and original permitted contact links. Its physical TCP z was 0.012445226311683655 m versus nominal 0.02 m. Planning-only retreat from that recorded q/object/jaw state generated the original backoff/lift but blanket target rejection stopped at 0.0008766688406467438 m initial support/target contact. The original profile explicitly permits that support link. Installed strict contact escape using the same native collision diagnostics and original allowed link set: only initially present target contact may persist, depth cannot increase (1e-7 m numerical comparison allowance only), cleared contact cannot recur, and final contact must be empty. Any static obstacle or self contact rejects. No collision margin or table exemption changed.

Repeating the recorded-state GPU retreat with this rule still rejected left support / simulation_table penetration 0.001117965206503868 m. Thus legal target contact no longer hides the independent table failure. The recorded diagnostic observation sequence 1723 is the native control-sample index, not a newly acquired live worker exposure; this is explicitly frozen planning evidence. See retreat_measured_01/ and retreat_measured_02/ under benchmarks/release/evidence/pusht_feedback/.

Then ran the original frozen full-arm translation worker with its existing original-agent gravity-compensation option, internal gripper collision checks enabled, unchanged collision/success gates and network isolation. Job 889567891ce2443bae81af10c117e73a advanced 3819 control steps and had no recorded static contacts. Two full approach/descend/contact/push/retreat cycles actually completed: independent before/after observation sequences 2..11 and 13..22; each stage was replanned from measured state. Third cycle completed approach (24/25) and entered descend (26) before the 180 s cooperative timeout. Final worker status cancelled, NOT task success. The last saved candidate rejection (contact deepening) is a candidate-search diagnostic, not the terminal cancellation cause.

The original controller fitted empirical response features twice, at observations 12 and 23. These are its existing ResponseEstimator gains, NOT SWM density/friction PhysX replay or parameter posterior qualification. Last task observation at step 2 was [0.3639264404773712,-0.1844215840101242,-0.19367603957653046], error 0.0263544264688112. Evidence: benchmarks/release/evidence/pusht_feedback/worker_05/, including source-hashed harness result, terminal result, physics summary and actual stage events.

Kernel: 16 targeted passed (0.05 s); full isolated tests/ 1918 passed / 1 existing warning (42.82 s). Native stage/reobservation/replanning exercised for two actual complete push cycles, but SWM complete-scene acknowledgement remains false and full task is not achieved. Model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Next: integrate these genuine boundaries with complete SWM snapshots and recorder/replay/posterior, without substituting the original empirical response fit for the requested physics identification. Pen and remaining G0-G10 gates are still incomplete; overall PARTIAL_DELIVERY.

## 2026-09-13 original compound T support in existing PhysX replay

Found that the existing SWM replay accepted only a single convex mesh, cuboid or sphere, whereas original PushT constructs one rigid actor from two boxes. Added compound_cuboids to the existing replay: a bounded, hashed metric descriptor preserves each part transform/dimensions and applies candidate density at each native box construction. Invalid dimensions, transforms or changed descriptors reject; planning-proxy rejection remains. No convex hull substitution or new physics engine. Native result now reads target mass and collision-shape count directly.

Extended the existing validate_swm_physics_replay.py with an explicit optional original PushT profile geometry input. The actual isolated CPU experiment uses that profile's original rectangles and height, but retains the existing tool-only reference protocol and its single spherical pusher. It first measures the reference tool action, then replays exactly that measured action in four private hypotheses. This is NOT worker_05's full-arm action, not original articulated gripper geometry, and not the required live recorder-to-next-planner integration.

All four native replays valid. Target has two native collision shapes; density 800 kg/m3 produces mass 0.1631999909877777 kg, density 1600 produces 0.3263999819755554 kg. Losses: low friction 63.8351704924458; known parameters 0.003545988297785913; high friction 5.088953323966347; doubled density 0.008748067648051945. density_informative=false and unique_parameters_identified=false are preserved. Measured action digest e30b8059f2b9bce6390ae222cc4d5b9396162489277757e33702c31c7724db53. Evidence: benchmarks/release/evidence/pusht_feedback/compound_replay_01/; full original requests/results remain runtime_data/swm_release_compound_replay_01/. Earlier four-parameter baseline was not overwritten.

Kernel: 6 targeted passed (0.03 s); full isolated tests/ 1924 passed / 1 existing warning (43.32 s). Native replay geometry extended and actually stepped; complete SWM PushT runtime/scene registration, original tool geometry, worker action recorder, posterior consumption and task success remain incomplete. Model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY, not SOFTWARE_DELIVERY_READY.

## 2026-09-13 actual PushT before/after SWM scene commits

Installed PushTSceneCapture in the original full-arm PhysicsSession. It reuses native_body/body_state to register every original rigid actor from actual box geometry and local transforms, exports hashed physical compound descriptors and meshes, and constructs the existing SceneWorldModel. This frozen scene contains dynamic_T and simulation_table. Capture requires all 13 measured joint velocities within .001 rad/s, original target stability, no target/tool contact, unchanged identity base/world calibration, consistent physical clock and unchanged q during the batch. Robot q, six measured jaw joints, native object poses/velocities and clock enter prepare_checkpoint/commit_checkpoint. No primary pose setter or invented mirror acknowledgement. The original execution path invokes it before and after a full bounded push.

Actual original-session approach probe completed with two different SWM snapshots (revisions 1/2) and resources_closed=true. More importantly, formal original full-arm translation worker_06, existing original-agent gravity compensation and internal gripper checks, completed all five stages of one push and committed before_push revision 1 then after_push revision 2. Native capture sequences are 1/2, same epoch e8a941e7619c4f78bca7b76731638573, counters 61/1995, physical times 2.0333334393799305/66.50000346824527 s. Target moved from [0.3499999940395355,-0.18000002205371857,0.010000000707805157] to [0.36324542760849,-0.18457777798175812,0.010000001639127731] with independently read rotation. Both snapshots include the table, original seven joints and six jaw measurements.

Snapshot IDs: b06857056e6889bb13166e79fe1536d82ee832bc8b7df669ba7165da24d155ee and 40adac0424b7d350c8d7e3b792aa971948689bfa9c51799143b15dc54abf6d89. A second before_push revision 3 was committed before the 120 s cooperative timeout; final task status cancelled, not success. Evidence including native geometry, manifest and immutable snapshots: benchmarks/release/evidence/pusht_feedback/worker_06/. Copied manifests intentionally retain original runtime asset paths and digests; copies are evidence, not relocated live manifests.

Kernel: full isolated tests/ 1924 passed / 1 existing warning (43.95 s). Native observed-world SWM updates actually exercised, but transactional mirror acknowledgement remains false; runtime admission, worker action recorder/transition, original tool replay and next-planner posterior remain incomplete. Separate scene_capture_01 evidence covers the approach probe only. Model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY, with pen and remaining G0-G10 requirements unchanged.

## 2026-09-13 worker_07: actual action recorder to SWM transition

Connected the existing MeasuredFeedbackRecorder and ExecutionReceipt to the original PhysicsSession. Native samples are collected before/after each SWM capture and every physical control step; initial idle settle samples retain native_stage and map to the recorder's post_settle phase. begin_action follows the before snapshot; finish_command occurs after the owned idle/no-contact check but before the after exposure. The recorder remains active through the after exposure, then the original transition binder validates identities, host coverage and native physical-time projection. The request's local object target is the original push prediction using the original bound surface contact, not a claim that the local goal was reached. Exceptions cancel handles and close clears the recorder; failed/incomplete actions do not produce transitions.

Actual full-arm worker_07 (original-agent gravity compensation, original internal collision checks, network isolated) completed one push and emitted transition d6d1c7ce856efc236e7de88f742a564891bfe97b7b62dd681c143dbe447917a0, action e4c4292527034280bea6752260b21243. Initial/final snapshot IDs: 3db46ea41d381e3d15d190d1124c0bfb1fba0e4303e0c5c54dc1929d78547d56 and 3cbb1ec79080d7c441c7d04f69c7110f5f530319d0848283790788352764dd53. 1939 host samples became 1935 physical samples after four identical stationary reads were deduplicated; both tool/object interval endpoints are 0 and 64.46667002886534 s. Native epoch 845bea42a6c945c797a4d306cdb4ba5c, physical start/end 2.0333334393799305/66.50000346824527. Original host snapshots were not retimestamped. Actual action digest 6b421056b381e8268ab6a6f6cfb224699236de8b46c054835cd4d58736ad4866. Full transition and bounded worker evidence: benchmarks/release/evidence/pusht_feedback/worker_07/.

Worker later cancelled during the next push at its 120 s cap; not task success. No PhysX parameter fit, original gripper geometry replay, mirror acknowledgement or posterior consumption is claimed. Those remain next integration work after regression repair.

Kernel NOT_GREEN: full isolated tests/ 1922 passed / 2 failed / 1 existing warning (43.49 s). Both failures are tests/three_scene/test_pusht_physics_loop.py assumptions about the former execute_push function body and calling it with a SimpleNamespace without the extracted _execute_push method. The original drift check remains in the implementation; adapt tests to cover both the cleanup wrapper and unchanged internal gate, do not delete the checks. Actual PhysicsSession completed transition binding despite these fixture failures. Model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN. Overall PARTIAL_DELIVERY.

## 2026-09-13 recorder wrapper regression repair

Repaired the two failures reported with d562c5d without changing production behavior or removing safety assertions. The AST check now examines the extracted _execute_push body and still requires the original 1e-5 rad gate and measured drift report. The behavioral drift test uses a PhysicsSession instance with controlled dependencies, exercising its actual public cleanup wrapper instead of a SimpleNamespace missing the internal method. Added wrapper tests for RuntimeError and BaseException propagation with recording cancellation, and successful return without spurious cancellation.

Targeted 20 passed (0.11 s); full network-isolated tests/ 1927 passed / 1 existing warning (42.75 s), exit 0. Kernel regression green. No new native run this turn; worker_07's actual 1935-sample transition retains its d562c5d source attribution. Actual worker-action PhysX replay, original tool geometry, mirror acknowledgement and posterior consumption remain incomplete. Model inference NOT_RUN; hardware NOT_AUTHORIZED_NOT_RUN; overall PARTIAL_DELIVERY.
