# SWM 软件交付滚动计划

目标：完整通过 `CODEX_SWM_RELEASE_GOAL_827A15E.md` 的 G0–G10。当前为 `PARTIAL_DELIVERY`，硬件未获许可。

基线：`7e3fee4196e7337e18217a0d72518789113eda77`，开始时工作树干净。

## 当前 M0 → M1

1. 修正导入资产发布验证与外部旧仓库状态的责任边界，增加临时源仓库负例及源闭包字节保护。
2. 完善 `swm/integration.py` 的受管理运行上下文，确保初始化异常、失败、取消、正常退出都清理资源。
3. 实装笔的正式 worker 上下文与阶段状态审计，串行运行实际原生 planner/主仿真，取得六个独立检查点证据。

## 后续顺序

- M2：动态参照与 Jimu 原 proof/两底板/原结构模板。
- M3：原 T 连续推送，反馈录制→实测辨识→下一规划消费新版本。
- M4：现有许可内实际离线 FP/SAM3D 与真实 Agent 程序/恢复。
- M5：受约束 pull/rotate、统一控制台与耐久、最终完整发布回归。

验收记录唯一入口：`benchmarks/release/swm_acceptance.json`。所有门槛保持原分母与阈值；运行前冻结具体输入/种子及摘要，不用事后成绩挑选输入。

执行约束：测试/仿真强制网络隔离，正式 pytest 范围限 `tests/`，GPU/PhysX 重任务串行；不连接或操作实体设备。不存在需要用户输入才能继续 M0/M1 软件实现的外部阻塞。

## 最新证据

G0 可移植验证与 S1 管理上下文已提交 `9c38a5449fbe02b4eb35481cb99f40749ded1d74`。定向 125 passed；全量 1518 passed、1 个既有弃用警告。源闭包字节摘要见验收表。下一实际动作：复用 frozen-world 初始化，创建笔的主物理世界、私有镜像、阶段 auditor 与正式 worker 工厂；禁止以当前上下文替身测试宣布 G1 通过。

## M1 阶段推进：349890e

已将抓取候选的后续放置可达性筛查接回共享阶段求解器；预筛路径不执行，place 继续使用后测 attachment 重算。NativeStage 摘要现绑定独立夹爪/attachment、释放几何和接触策略。新增 cuRobo 顺序阶段 auditor，逐段插值查询现有碰撞模型，审核状态切换端点，并在成功/失败后恢复观察对应的规划状态。

软件回归：SWM 120 passed；正式全量 1523 passed、1 个既有警告（45.32 s）。这些不是实际 GPU/主仿真成绩。未注册任何不完整 worker 工厂，缺适配器检查保留。

下一步仍是笔的正式 worker，而不是扩展 fixture 数量：先明确原生碰撞诊断对已附着目标的 world-obstacle 排除（当前诊断会临时启用场景对象，auditor 可能保守误拒绝），接独立 jaw 观测和原代理几何恢复；再复用原 frozen-world create_demo 创建主仿真及私有镜像，取得六次新采集和原生阶段审计证据。夹爪开合中间几何、候选恢复与联合筛查的原生结果仍待验证，不据此宣称 S2 全部完成。

## M1 初始化资源所有权：82be044

已复用原 create_demo 的初始化边界，并在 gym.make 返回后、reset/demo 构造之前注册清理；完整 actor 注册表缺失、初始化异常、取消、意外 live runtime、抓前持续冻结或额外世界创建均拒绝。仅软件替身测试：初始化生命周期加 managed runtime 共 18 passed（0.07 s）；未运行本轮全量。

原生启动尚未执行：源码确认原 resolve_planning_artifact_paths 会在 sim URDF 同目录生成 planning URDF/SRDF，当前新入口仍继承该行为。已询问是否修正为本次运行目录，未收到明确确认。不得运行 tools/validate_swm_native_bootstrap.py 来绕过这项待修复风险；该脚本仍为未提交草稿。无活跃仿真进程，不将待确认解释为真实运行中的等待。下一原生步骤仍为先消除此写入风险，再运行冻结输入初始化、真实读回并接正式 worker。

## S5 独立推进：7f51ceb

在原生启动输出路径确认待回期间，已接通另一明确断点：SharedPrimitiveExecutor 在 push 命令前调用 recorder.begin_action，完成反馈核验后将同一录制句柄放入 ExecutionReceipt；runtime 在后测曝光后调用原 transition observer。严格缺覆盖检查保留，命令异常/后测失败/未安装 observer 都释放对应动作窗口；完成窗口裁剪、无动作时有界滚动缓存，活动窗口满额仍拒绝丢弃起点。

定向 25 passed；正式全量 1540 passed、1 个既有警告（71.32 s）。均为软件回归，不是实际推送或 PhysX 成绩。尚未接实际关节反馈 FK/时钟映射、查询延迟、原 T 两动作辨识及 N+1 的下一规划消费；历史 action-id 集合长期有界策略仍需收口。M1 主线仍优先，初始化输出路径风险未绕过，未启动原生验证草稿。

## 当前 M1 原生初始化证据：d0c8708

此前待确认的输出路径风险已按继续修复要求消除：初始化只写新建私有目录，目录存在即拒绝，外部 SRDF 只读复制；旧资产路径不再作为生成物输出。验证入口已提交，不再是未提交草稿。

已串行运行原 current_table 冻结桌面（SHA256 e5dd3154bdb6b509791861478170ffc5f0b6e2c8f08ea89b9b7a5111ee2fd908），未运行旧 episode：

- bootstrap_01：Python 3.12 完成原场景初始化、两次读回及 env.close 返回，但进程退出 139，清理验收失败，保留失败记录。
- bootstrap_02：改用原机器配置指定的 foundationpose310 Python 3.10，同一冻结输入，读回 9 个对象、7 个机械臂关节及 6 个夹爪关节的 q/qdot，两次采集区间独立，env.close 返回且进程退出 0。仅初始化通过，不是 SWM 原子闭环通过。

本轮 SWM 139 passed（0.93 s）；未重跑全量，上一条全量 1540 passed 保留原提交归属。下一步固定原 profile 的 Python 环境，接正式 worker 的私有规划镜像、独立 jaw/holding 观测及原 builder；补齐 virtual table/walls 等完整障碍身份，再运行笔 grasp/place 六个新检查点。不能把此次 2 次无动作读回当作 6 个技能检查点，也不能因进程正常退出宣称所有原生资源泄漏检查完成。

## 当前 M1 同进程环境接线证据：6a4ccbe

主仿真独立成功的 Python 3.10 环境无法初始化共享 cuRobo2：实际探测报缺少 cuda.core，退出 1，主环境正常关闭。未移除后端版本检查，也未安装新包。

已验证现有同 ABI 组合：`envs/curobo2/bin/python`（3.11.15、torch 2.11.0+cu128、cuda-core 0.7.0），在 sys.path 末尾追加现有 `envs/realman/lib/python3.11/site-packages` 提供 ManiSkill/SAPIEN。原 9 对象主仿真与共享 Curobo2Backend 在同一进程初始化成功，实际返回 joint_1..joint_7，主环境 close 返回且退出 0。未复制、修改或安装环境。准确启动命令与证据摘要在验收表。

下一正式接线应沿用这个已测组合，而非继续假定 Python 3.10 单环境足够：将依赖来源固定到可信 worker 初始化；补真实全场景规划镜像、TCP/夹爪/持物证据以及 virtual infrastructure，再运行 grasp/place 六检查点。此次规划器仅空场景初始化，无路径/动作；不能记为碰撞场景已同步或原子 worker 通过。本轮没有新增软件测试数量或重跑全量。

## 当前 M1 真实场景同步：f7da832

新增只读 primary→原 FixedSceneAtomTaskBuilder 碰撞编译及 cuRobo GPU 张量确认。对象使用共享完整尺寸代理；桌面读取 SAPIEN 碰撞盒；虚拟墙保留原尺寸与 actor 位姿；资产哈希/尺度、GPU 名称/数量/启用位/尺寸/逆位姿不一致均拒绝。SWM 145 passed（0.93 s），其中新增 6 项为 GPU 存储协议 fixture，并非实际 GPU 同步成功。

已在已测 Python 3.11 组合中运行 `swm_release_pen_scene_sync_01`，退出 1：主仿真和规划器初始化成功，但共享 builder 的平移基座前置检查拒绝真实场景。原环境源码显示 `_initialize_episode` 设置基座 `p=[-0.3,0,0]`、yaw=90°；当前只减 offset 的约定不足。没有将场景送入 GPU，也没有放宽检查或执行动作。

下一首要动作：在原 AtomTaskBuilderConfig/FixedSceneAtomTaskBuilder 增加显式 SE(3) 基座合同，让 CompiledNativeTask 的物体/目标转换与碰撞编译共用该外参；记录原生读回的完整矩阵并重跑同一冻结场景。之后才继续独立 jaw/holding、完整 SWM 镜像与正式 worker 六检查点。该缺口是可继续实现的软件工作，不是外部许可阻塞。

## 当前 M1 SE(3) 与真实 12 障碍同步：079e654

已在原共用 frames、AtomTaskBuilderConfig/FixedSceneAtomTaskBuilder 和 CompiledNativeTask 中贯通完整 T_world_base；目标 world→base、后测对象 base→world 与碰撞代理使用同一刚体变换。保留平移兼容，拒绝两套外参同时指定、缩放、反射及非刚体矩阵，配置冻结复制避免外部矩阵修改。

重跑同一输入 `swm_release_pen_scene_sync_02`：退出 0，完整原基座矩阵已记录，9 个对象加原 SAPIEN 桌面和两面虚拟墙共 12 个障碍成功进入共享 cuRobo GPU；名称、数量、启用位、尺寸和逆位姿读回一致。原资产/仿真资源字节及尺度一致性检查通过，未缩小共享代理。保留上次 sync_01 拒绝记录。

定向 10 passed；正式全量 1556 passed、1 个既有警告（41.68 s）。下一步不再停在基座/空场景初始化：补原生 TCP、独立 jaw/holding、全场景 SWM manifest 与 private mirror，接可信 worker 工厂并执行笔 grasp/place 六检查点。当前 GPU acknowledgement 的 robot_state_qualified、attachment_qualified 均为 false，工厂仍不完整，不能据 12 障碍同步解锁执行。


## M1 / 实测机器人与夹爪几何接线：3709e06

复用共享夹爪碰撞控制器，读入六个独立关节而非由 holding 推断开合；IK 缓存身份绑定该六关节配置摘要。原生主世界读取 TCP、八个夹爪链节及九对象双指接触力，沿用原 ManiSkill 持物谓词（0.5 N、95 度），不从命令推断持物。

实际串行运行 `runtime_data/swm_release_pen_robot_sync_01`，网络隔离、180 秒上限、退出 0。主仿真 TCP 与共享 cuRobo FK 比较、八链节与原 URDF 六关节 FK 比较以及 GPU 碰撞球中心/半径读回均通过既定检查。完整原始 observation、误差 acknowledgement、命令和输入/结果/事件/日志 SHA256 已记录在机器验收表；未放宽碰撞或几何阈值。

内核：正式全量 **1560 passed / 1 warning / 41.87 s**。原生接线：初始静止机器人/夹爪几何对齐已实测，持物 attachment 与正式 worker 仍未完成。模型推理：本轮未运行。实际仿真：初始化、场景同步和原生状态查询已运行，无抓放动作。硬件：未授权、未连接、未运行。

下一步将此独立观测接入同一 SWM 检查点及私有镜像，再安装完整可信 worker 并执行笔 grasp/place 六次新采集。初始空手接触查询不证明抓取成功；本次不解锁未完成技能、不删除缺适配器检查，G1/G2/S4 仍不标整体通过。代码已本地提交，尚未推送。


## M1 / 私有机器人镜像接线与实际失败：d1232a3

已有 CuroboScenePort 在非 fixture 域要求独立 13 关节 q/qdot，调用真实 GPU 障碍及机器人几何读回；不再从持物状态推断夹爪开合。原生 held attachment 的存储读回尚未完成，明确拒绝，不用 Python 缓存冒充。SapienScenePort 要求其自有机器人端口、同世界 actor 身份和实测对象速度，检查私有机器人/自由对象读回。

新增 SapienRobotStatePort 自行创建 CPU PhysX 场景，不接收主机器人或 executor；但实际 `swm_release_robot_mirror_01` 失败：原 URDF 含视觉组件，纯 PhysX 场景缺 render system，初始化报错且清理导致进程退出 **134**。本次使用上一轮保存的主仿真反馈，绝非新采集，更不是技能检查点；没有得到私有镜像一致性确认。错误、输入及脚本/结果/日志哈希保留在机器验收表，不删除失败。

内核：全量 **1570 passed / 1 warning / 44.99 s**。原生接线：状态端口已接，但私有构造未通过，完整工厂仍未安装。模型推理：未运行。实际仿真：私有初始化尝试失败，无动作。硬件：未授权、未连接。保持 PARTIAL_DELIVERY，未完成技能不可用。

下一步修正私有构造器的原 URDF 视觉依赖与异常清理后重跑，再接完整观测/镜像和笔的正式 worker。该新代码缺陷与上一轮报告命令的引号修正均已告知用户，按编辑约束待确认；它们不是硬件授权请求。其他独立软件工作可继续。代码本地提交，未推送。


## M1 / 私有镜像构造修复及新采集草稿：067accf

按继续修复要求处理此前已说明的问题：为原 URDF 视觉组件保留被动 RenderSystem，物理仍为 CPU PhysX，无相机/控制器/executor；关闭时显式清理私有场景。报告 `native_robot_sync_01.command` 改用 shlex shell 引号，旧错误记录保留，argv 和原成绩未改变。

实际 `swm_release_robot_mirror_02` 退出 0，私有端口关闭成功。13 关节 q/qdot 及 10 个链节（含 TCP）均从原生 articulation 读回；最大位置误差 1.54141578e-07 m、角度误差 0.000132411645 rad。输入仍是上一轮保存的实测状态，不是新采集；没有证明完整对象/attachment 镜像或正式 worker。

相关定向 15 passed / 0.42 s。新增 NativePrimaryCapture 草稿读取绑定 actor 的姿态/速度、独立机器人反馈，检查序号、完整注册集、采集中变化及 SE(3) 速度换算，但尚未提交/开放：其 5 项新测试全部因 Pose 替身错误失败（0.10 s）。本轮未重跑全量，历史 1570 passed 不能归属于当前含草稿的工作区。

实际完整采集探测 `primary_capture_native_01` 退出 1：原初始化改变 cwd，驱动使用相对输出路径，最终写入 capture_result.json 失败；该错误还可能遮蔽更早的采集异常。无批次成功证据，不计任何技能检查点。主环境关闭，错误和输入/脚本/结果/日志哈希已保留。

下一步先修正本轮测试 Pose 替身和采集驱动的绝对路径，再运行完整 12 对象新批次；之后接 metric manifest、完整私有 actor/attachment 镜像和正式笔 worker。内核有上述新测试失败；原生只有私有机器人对齐通过；模型未运行；实际采集探测失败；硬件未授权、未连接。全部未完成技能保持不可用，状态仍 PARTIAL_DELIVERY。修复代码和报告本地提交，未推送。


## M1 / 完整原生新采集通过：53772ab

已修正上一轮 Pose 测试替身和采集驱动相对输出路径。实际 capture_02 随后暴露此前被写入错误遮住的字段不匹配：原生反馈使用 primary_sequence，而草稿读取 sequence；该失败退出 1 并保留。现按原生合同接线，不生成伪序号。

实际 capture_03 退出 0，主环境关闭，产生两批各 12 对象的新观测：原九对象、桌面、两面虚拟墙，原生序号 4/5，采集耗时约 17.74/17.40 ms，机器人与对象读取跨度约 12.24/11.85 ms。姿态、原生速度/静态约束来源、独立机器人关节/接触反馈及完整序号均有记录；采集中变动、未静止及重复样本仍拒绝。没有执行抓放或连接实体设备。

定向 5 passed / 0.08 s；正式全量 **1575 passed / 1 warning / 45.25 s**。此前五项测试失败已修复，历史失败条目保留，不改写成通过。

内核：本轮全量通过。原生接线：完整新批次和此前私有机器人镜像分别已实测，但基础设施目前仍用描述符摘要绑定，metric 文件资产清单、完整 actor/attachment 双镜像事务及正式 worker 尚未完成。模型推理：未运行。实际仿真：两批无动作采集成功，不算技能检查点。硬件：未授权、未连接。

下一步将真实新采集接入正式 metric manifest 和完整双镜像确认，然后推进笔 worker 的 grasp/place 六个真实技能检查点；禁止重复空采集凑数量。保持 PARTIAL_DELIVERY，缺适配器检查和未完成技能禁用状态不变。代码/报告本地提交，未推送。
