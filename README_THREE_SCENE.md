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
