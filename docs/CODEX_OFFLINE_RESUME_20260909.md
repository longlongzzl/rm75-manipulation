# 2026-09-09 续跑：禁止真机操作、IK 图册及 PushT 闭合几何

## 执行约束与修复

用户确认上一会话曾操控实体夹爪，并明确要求不得再次发生。上一会话在导出图册后使用了未限定目录的 `python3 -m pytest -q`，收集到了旧快照中导入即连接、控制硬件的脚本。不能用任务自身的 `mode=sim` 日志证明整个测试过程没有硬件操作。

本次加入以下保护后才恢复测试及仿真：

- `pytest.ini` 默认只收集 `tests/`；根 `conftest.py` 还拒绝显式指定其外的路径，并在递归收集时忽略其他目录。
- `tools/run_network_isolated.py` 使用 Linux seccomp 拒绝非 Unix 套接字及 io_uring 创建，清理继承的网络套接字。过滤器跨 fork/exec 保留，覆盖原生 SDK 和不同 Python 环境中的 worker。安装或自检失败直接终止命令。
- 回归验证原生 libc IPv4/IPv6 TCP/UDP 套接字被拒绝、exec 子进程继承限制、本地 Unix IPC 可用；用临时哨兵文件验证默认、根目录及显式旧脚本收集都不会导入旧模块。没有用真实设备地址探测保护效果。
- `AGENTS.md` 和 `README_THREE_SCENE.md` 记录必须使用的隔离入口和用户约束。隔离只针对网络，不代表串口、USB 或文件系统沙箱；本次没有访问真实设备。

完整回归命令：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- python3 -m pytest tests -q
```

结果：**1044 passed，1 warning，65.89 s**。warning 为已有 trimesh 弃用提示。日志：`runtime_data/three_scene/ik_gallery_pusht_20260909/offline_tests_resumed.log`。`git diff --check` 通过。未提交或推送代码。

## PickPlace 图册

入口：[完整 IK 图册](../runtime_data/three_scene/ik_gallery_pusht_20260909/gallery_final/index.html)。采集任务 `fca1a6daf7c24c058e5555da449510f5`；33 批、768 张，覆盖实际查询到的全部原生候选行，包括失败行和预取行。不是穷举整个 IK 解空间。

已逐一核验原图字节与采集证据一致、768 个图像文件互异、图像可解码、全部 HTML 链接存在。所有批次都包含机器人、规划器、物体注册表恢复及求解返回行未变的证据。视觉检查了胶棒的四阶段对照图。

| 物体 | 候选图数 | IK 通过行数 |
| --- | ---: | ---: |
| shuazi | 36 | 25 |
| bi | 120 | 57 |
| lvmukuai | 64 | 29 |
| carriot | 64 | 30 |
| gluestick | 156 | 43 |
| hongshupian | 264 | 73 |
| tennis | 64 | 58 |

通过行数混合多个阶段、前台、预取及重试，只用于图册索引，不能当任务成功率。同一抓取关系的预抓取、抓取、preplace、place 对照已单独展示；缺失的对应行明确留空。

该原生任务完成 5 个物体，胶棒和红薯片失败，完整清桌未通过。胶棒前台两次 place/release 都为 0/13。红薯片默认放置关系失败，补充关系中有端点 IK 通过行，但完整带载路径仍失败；不能用端点图代替完整路径验收。

## PushT 隔离续跑及新发现

本次复跑保持用户所选全闭合 `.91 rad`、原始完整 URDF、原碰撞球和阈值、原 ManiSkill 理想臂重力补偿，并使用仿真关节反馈更新碰撞几何。没有增加碰撞豁免。

```bash
python3 tools/run_network_isolated.py -- python3 tools/run_pusht_physics_validation.py \
  --backend full_arm_physics --case translation \
  --gravity-compensation original-agent \
  --closed-gripper-joint-position .91 --gripper-feedback-geometry \
  --output runtime_data/three_scene/ik_gallery_pusht_20260909/pusht_resumed_isolated \
  --timeout-s 180
```

任务 `ca7f2a783ec644d8be47aa5bce61556e`，耗时 40.787 s。第一段 approach 规划被拒绝，T 接触计数为 0，任务未通过。旧版 GPU 诊断将约 0.001434 的平方距离差误标成了线性米值；此前称为 1.434 mm 不正确，对应的链接仍为左右 `Support_Link`。已在[后续报告](CODEX_FAILED_OBJECT_IK_PUSHT_090_20260909.md)更正单位及适配器。物理接触日志也记录这两个支撑件接触。

进一步直接检查原始封闭 STL：每个支撑件有 6520 顶点、13064 面；每方向均匀选取 512 个原始顶点，计算到另一网格的带符号三角面距离。两者均 watertight；正的内部顶点样本是网格相交证据。结果如下，距离是样本深度，不冒充全网格最大穿透深度：

| 姿态 | 左侧样本进入右网格 | 右侧样本进入左网格 | 最大样本深度 |
| --- | ---: | ---: | ---: |
| 旧参考 .6 rad | 0/512 | 0/512 | 未观测到内部点 |
| 上限 .91 rad | 386/512 | 386/512 | 0.3702 mm |
| 实际仿真反馈角度 | 116/512 | 108/512 | 0.1151 mm |

静态几何证据和源网格哈希保存在 `runtime_data/three_scene/ik_gallery_pusht_20260909/closed_support_mesh_audit.json`。采样未发现相交不能证明碰撞自由；这里在 .91 和反馈姿态均找到相交点，足以证实原网格存在内部冲突。

因此，在这一固定闭合姿态和当前几何约束下，改变机械臂 IK 不会改变两个支撑件的相对位置。继续增加 IK 搜索不能消除该内部相交。下一步应检查夹爪机械闭合位置、连杆约束和资产几何的一致性；不能直接把关节上限当成无穿透的闭合位置，也不能删球或豁免冲突来声称 PushT 成功。本次保留完整失败证据，未将旧 .6 rad 模型的结果用作全闭合资格。
