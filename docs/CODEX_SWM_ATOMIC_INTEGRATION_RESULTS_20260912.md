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
