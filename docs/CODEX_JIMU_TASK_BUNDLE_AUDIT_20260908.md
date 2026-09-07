# 原 Jimu 任务三文件闭包只读审计 — 2026-09-08

分类结论：以下三项为 `candidate_final_fix`（运行配置/固定数据，不是算法源码）。**未纳入 approved overlay 或 snapshot，也未复制进 Git。** 只读原文件进行本地 SIM 重放；迁移仍待明确审核，不擅自决定纳入/丢弃。

原目录：`/home/zhangzhao/Desktop/lerobot/Beta_demo-codex-v0.9/jimu_tasks/tag1_standard_three_layer/`。
旧 HEAD `36798efbd12814841607951c9af470b309b34fd3`；`git ls-files --stage -- <该目录>` 无输出，三项均为未跟踪文件。旧仓库不 reset/clean/stash/覆盖，不整体复制 untracked。

| 文件 | SHA256 | 依赖/理由 |
| --- | --- | --- |
| manifest.json | `811be89115ec9982ac425f799dedc0d08ab55ada996c42d0e524a54550a3ab2d` | 原 `--jimu-task-dir` 入口读取 `jimu_task_manifest_v1`；指定下列两文件及旧料位/层高/构建顺序 |
| builder_scene.json | `6478bafdf016817e7d118269da9199e6d4fe8c06570cbedfdfb03c1127ca6884` | manifest.builder_scene_json 的直接引用；21 件 = 9 固定 + 12 可移动，已用于前轮完整 builder 回归 |
| fixed_scene_pose_results.json | `c49b1f1663cdd33865dd89bd732be0a6df5e7c9af00b19e5fd1b04531f451e06` | manifest.sam6d_fixed_scene_result_file 的直接引用；原基座/料盘两项历史固定定位，不是本轮新相机观测 |

## 为什么 builder 单文件不能代表原完整工作任务

已审阅的原 triangle entry 提供 `_apply_jimu_task_manifest_defaults`，优先保留显式 CLI，其次读 manifest；传 `--jimu-builder-scene-json` **不会自动加载同目录 manifest**。此前工作台仅传 builder 和通用 synthetic anchors，未传 `--jimu-task-dir`。

原 manifest 中存在、单文件路径遗漏的配置：

- 每层按 right → back → left → front 构建；墙、二层墙、屋顶三组。
- 三角片实际槽位 `[4, 5, 6, 12]`；12 个任务角色和 2 个方片备用槽按明确的 14 槽顺序分配。不是简单将 12 个任务角色连续放入槽 0–11。
- 原累计层高参数 `[0.005, 0.004, 0.004] m`，最后接触低悬停高度 `.01 m`。这是原文件中的配置，不是 Codex 为通过实验新增的几何补偿。
- 固定定位及原 tag frame 参数；固定数据中的历史相机路径只是 provenance，不被复制或当作本轮 fresh observation。

实际日志比较：通用默认 triangle 场景的首个 front_roof_triangle 源位置 y=-.018906 m，而 builder-only 回归 y=-.168781 m，相差约 149.875 mm。builder-only 日志也明确使用 fallback tray role order。**差异已确认；不能仅凭这项比较断言它解释了全部屋顶失败**，因为两个场景还存在 base/support/target 等其他差异。

## 只读重放边界

`tools/run_workcell_native_validation.py --task magnetic --task-dir <原目录>`：

1. 解析 manifest 并验证三文件闭包；依赖必须是同目录内存在的文件，拒绝缺失、路径逃逸及其他 schema。
2. 使用原 builder、原固定定位和原 native `--jimu-task-dir`，不摘出个别参数调到通过；禁止同时覆盖 `--design/--fixed-sam6d/--fixed-world`。
3. 在新仓库实际 WorkcellService → native worker 中 SIM；关闭 live AprilTag 分支，不连接机械臂，相机不采集。
4. 保留 full-world transport、自碰撞、payload、near-IK 碰撞保护、原候选/seed/阈值。非搬运接触仍为已审计兼容策略，非真实资格。
5. 运行前后三文件 SHA256 均记录，若内容变化则不能报告 completed。旧 status 哈希另行核对。

本轮旧 fixed 文件来自历史 AprilTag 定位。这是恢复旧工作的离线基准，**不是把当前 RRTrack 方案切回 AprilTag，也不证明 tagless 装配精度**。
