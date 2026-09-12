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
