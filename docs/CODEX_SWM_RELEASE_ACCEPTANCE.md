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
