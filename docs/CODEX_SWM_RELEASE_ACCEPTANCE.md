# SWM 软件交付验收

状态：`PARTIAL_DELIVERY / HARDWARE_NOT_RUN`。

完整要求固定为 `CODEX_SWM_RELEASE_GOAL_827A15E.md` 的 G0–G10，不能以此前组件测试或工具级 PhysX 结果代替三个任务的 worker 验收。精确运行与证据索引维护在 `benchmarks/release/swm_acceptance.json`。

当前正在执行 M0/M1。尚未形成可宣布 `SOFTWARE_DELIVERY_READY` 的启动配置和完整证据；本报告将随真实运行更新。硬件许可保持独立。

## M0 / S1 本地回归

提交 `9c38a5449fbe02b4eb35481cb99f40749ded1d74`：定向 125 passed；正式 `tests/` 全量 1518 passed、0 failed、1 warning（42.96 s）。命令：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- python3 -m pytest tests/ -q
```

保留导入资产原始哈希与重定位验证，发布测试不再读取外部旧仓库。实际 native worker 生命周期仍待验收，G0/G1/G10 尚未整体通过。所有实体设备均未连接。

## M1 / 阶段状态与联合可行性增量

代码提交 `349890e`。抓取现在筛查后续放置可达性，但执行边界仍只到 lift；place 使用新观测重新求解。阶段摘要包含夹爪、附着、接触与释放几何；cuRobo auditor 使用既有模型查询，不创建执行器。新增负例覆盖开放夹爪端点碰撞、状态链不连续、摘要变更和失败后恢复。

- 内核/软件回归：SWM **120 passed**；全量 **1523 passed / 1 warning / 45.32 s**。测试运行期间提交了同一未再修改的代码字节。
- 原生接线：阶段求解器已修改，auditor 尚未安装进正式 worker；独立 jaw 观测、已附着目标的原生碰撞排除策略及夹爪开合过程审核仍待接通/验证。
- 模型推理：本轮未运行。
- 实际仿真：本轮未运行主世界/原生 planner，不将测试模型计作 PhysX 或 cuRobo 运行。
- 硬件：未授权、未连接、未运行。

G1 和全部最终验收要求保持未通过；本次不是 SOFTWARE_DELIVERY_READY。

## M1 / 初始化生命周期增量

提交 `82be044`：原 create_demo 的环境即时归属及关闭合同；18 项软件测试通过（0.07 s）。命令：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- python3 -m pytest tests/swm/test_native_bootstrap_lifetime.py tests/swm/test_runtime_context.py -q
```

原生接线仍未安装，模型推理/实际仿真/硬件本轮均未运行。原构造器的 URDF/SRDF 输出位置会落在资产目录，新入口尚未隔离此写入；因此不能运行或据此宣布初始化合格。路径修正确认待回。上一条 1523 passed 是上一提交的全量结果，不能归属于本次新增代码。

## S5 / 录制回执真实组件接线

提交 `7f51ceb`：录制在共享执行器发送 push 前建立，完成回执绑定同一 handle；后测到达后原实测转换器收到该回执，不再因 observations=None 跳过。未伪造 TCP 样本。runtime 的成功、命令失败、后测失败路径均释放窗口；保留严格双曝光覆盖判据。

- 内核/软件：定向 25 passed；正式全量 **1540 passed / 1 warning / 71.32 s**，测试期间提交相同代码字节。
- 原生接线：回执生命周期已实现，但生产 FK/时钟/后测持续采样及正式工厂仍未安装完整。
- 模型推理：本轮未运行真实模型。
- 实际仿真：本轮未运行，测试传感器和执行 sink 为显式 fixture。
- 硬件：未授权、未连接、未运行。

原 T 连续主物理执行和 N→N+1→下一规划的证据缺失，G6 未通过。原生初始化 URDF/SRDF 输出路径问题仍待修正确认。

## M1 / 原冻结桌面实际初始化

提交 `d0c8708` 修正私有初始化产物目录，已消除前述待确认写入风险。SWM 回归 **139 passed / 0.93 s**。

实际运行 `tools/validate_swm_native_bootstrap.py`，全部通过原网络隔离及 180 s 上限启动，未执行旧 episode：

- `runtime_data/swm_release_pen_bootstrap_01`：系统 Python 3.12，原生对象和关节读回成功，但退出 139，判定失败。JSON 的 initialized_and_read/primary_closed 不能覆盖进程失败。
- `runtime_data/swm_release_pen_bootstrap_02`：原 profile 指定 Python 3.10，原 9 对象桌面，13 个活动关节 q/qdot，两次独立原生读回；主环境 close 返回、进程退出 0。仅初始化证据成立。

内核：本轮定向回归通过；原生接线：主世界初始化可用但工厂仍未完整安装；模型推理：未运行；实际仿真：已运行初始化与状态读取，未运行原子任务；硬件：未授权、未连接。独立 holding、virtual infrastructure 完整观测、私有镜像、实际 cuRobo 阶段审计和六检查点仍缺证据，G0/G1 不标整体通过。两个运行摘要、时间区间和原始文件哈希见唯一机器验收表。

## M1 / 主仿真与共享规划器同进程探测

提交 `6a4ccbe`，新增验证工具的 `--probe-planner`。两次同输入运行均经网络隔离、180 秒上限、串行执行：

- `swm_release_pen_planner_probe_01`：foundationpose310 Python 3.10 原主仿真成功，cuRobo2 因缺少 `cuda.core` 失败，退出 1；失败保留。
- `swm_release_pen_planner_probe_02`：现有 curobo2 Python 3.11，加同 ABI realman site-packages 作为末尾补充路径，主仿真和共享 cuRobo2 初始化成功、7 关节身份一致、退出 0。

只证明现有依赖可以同进程共存，未安装新包。规划器初始化使用空场景，明确 `planner_scene_qualified=false`、`planner_executed=false`、`motion_executed=false`。未把空场景当作可执行碰撞场景；原 9 对象仍未完整接进规划器。模型推理与硬件本轮均未运行，正式 worker 和六检查点仍待完成。最新全量测试仍归属 `7f51ceb`，本轮没有另做全量回归。

## M1 / 完整碰撞场景接线与实际拒绝

提交 `f7da832`，新增 primary 碰撞编译和 GPU 张量确认，SWM 回归 **145 passed / 0.93 s**。六个新负例验证缓存名称不能掩盖原生存储中的禁用、位姿、尺寸或数量错误，仍属软件 fixture。

实际运行 `runtime_data/swm_release_pen_scene_sync_01`：退出 1；原主仿真和 cuRobo2 初始化成功，随后因非纯平移基座被拒绝。原环境源码包含 90° yaw 基座设置，下一步需原 builder 显式 SE(3) 支持。此轮没有实际 GPU 全场景 acknowledgement，也没有原子动作；不能把新适配器或测试标记为完整场景已同步。

模型推理与硬件未运行，主环境正常关闭。G1/G2/S4 仍未整体通过；上一条全量测试仍保留原提交归属。本轮失败输入、错误及原始日志哈希已记录在机器验收表。

## M1 / SE(3) 修复后的真实全障碍同步

提交 `079e654`。定向 10 passed；全量 **1556 passed / 1 warning / 41.68 s**，测试跨提交期间代码字节未改变。

`runtime_data/swm_release_pen_scene_sync_02` 使用与拒绝案例相同冻结输入和已测同 ABI 环境，退出 0。完整实测 T_world_base 进入原 builder 与 SWM 任务转换。共享 cuRobo GPU 中读回 **12 个障碍**：9 个原对象、真实桌面碰撞盒、两面原策略虚拟墙；身份、尺寸、逆位姿、启用状态一致。没有改成小场景或缩小碰撞代理。

内核：全量回归通过；原生接线：真实障碍几何同步已取得证据，机器人/夹爪/attachment 和正式 worker 尚未完成；模型推理：未运行；实际仿真：原主世界初始化、完整几何同步及读回已运行，未执行原子动作；硬件：未授权、未连接。G1/G2/S4 不标整体通过，也不将无动作读回计作六个技能检查点。


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
