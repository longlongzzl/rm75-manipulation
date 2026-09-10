# RM75 三场景迁移增量

先读 `docs/CHATGPT_THREE_SCENE_HANDOFF_20260906.md`。

包含实际代码：PickPlace/Jimu 原工作版本适配与固定提交迁移器、三页前端与统一任务服务、PushT 实时感知/短程预测/cuRobo2/RM75执行链、测试与本地验收脚本。

**本 ZIP 是增量代码，不包含旧仓库完整源码与模型资产。** 安装器从用户本地的 `lerobot-realman` Git 固定提交复制它们到新仓库的 `rm75_app/_vendor/working_snapshot/`。没有迁移时，原版 sim/real 会明确失败；不会伪装执行。

```bash
python tools/install_three_scene.py \
  --target /实际的rm75-manipulation \
  --legacy-source /实际的lerobot-realman
```

默认不连接机械臂。原版运行、GPU、相机和实机须按交接文档逐项验收；不要把 CPU surrogate 或 mock SDK 结果当成真实机器人结果。


本地离线回归使用以下命令，系统网络阻断会继承到所有子进程；保护安装失败会终止执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- python3 -m pytest tests -q
```

旧快照中有导入即控制机械臂和夹爪的 `test_*.py`，不可收集或导入。根目录 pytest 配置只允许收集 `tests/`。仿真也通过上述隔离入口启动，并保持 `mode=sim`、`allow_real=False`。隔离依赖 Linux 的 libseccomp，仅允许 Unix 本地进程通信，不提供串口或 USB 隔离。真机动作必须单独审批。


PushT 当前支持 360 组接触点/方向/推距候选、批量前瞻评分和每批最多 4 条 GPU 接近轨迹；默认推距候选覆盖 6–50 mm，临近目标时缩短，使用实际观测更新接触响应。撤退为后退 10 mm、上抬 50 mm，按用户要求仅在撤退规划中忽略碰撞。2026-09-10 的同场景隔离全臂仿真已用 30 mm + 6 mm 两次夹爪行程通过原成功阈值；配置、校准来源和证据见 [多候选推送记录](docs/CODEX_PUSHT_BATCH_SEARCH_20260910.md)。
