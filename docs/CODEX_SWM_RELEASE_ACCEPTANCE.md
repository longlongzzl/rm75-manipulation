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
