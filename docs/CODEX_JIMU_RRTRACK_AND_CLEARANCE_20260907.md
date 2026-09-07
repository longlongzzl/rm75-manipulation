# Jimu RRTrack 实测与 PickPlace 退让回传 — 2026-09-07

## 0. 基本信息

用户放置一块橙色正方形薄片，要求验证 RRTrack 能否替代 AprilTag。随后机器重启，本轮从持久化 RGB-D 恢复。

**单块无 AprilTag 的感知计算链已跑通，不等于无 AprilTag 的磁吸装配精度合格。三链总目标仍未完成。没有机械臂或夹爪真实运动。**

- Tested base: `3d2aba1bf375e516e27b353dd0c1387e2327da79` + 本回传提交的修复。
- Branch: `chatgpt/three-scene-software-closeout`。
- CPU Python 3.12；感知/native Python `foundationpose310` 3.10，SAM3 使用 `sam3` 环境。
- RTX 5060 Ti 8 GiB；机器 RAM 15 GiB。重启后 GPU 作业串行，每组 `MemoryMax=9G / MemorySwapMax=512M`，有超时；不降低模板/候选/成功阈值。
- 机器人 SDK / Robot IP：未使用。相机：D435 `342522073637`。
- [实际感知结果 JSON](../benchmarks/unified_scenarios/jimu_rrtrack_live_20260907_summary.json)：含全部 30 帧轨迹、失败轨迹、提示词分母、原始文件 SHA256。现场照片留在本地，未上传 Git。

## 1. 旧仓库 dirty worktree 审计

- 旧 `/home/zhangzhao/Desktop/lerobot` 只读；没有 reset/clean/覆盖/新增 overlay。
- 重启后本轮前后 `git status --porcelain=v1 -z` 哈希均为 `15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`。
- 此值与此前报告的 `7a7759…` 不同，不能宣称跨重启 dirty 状态完全一致；本轮没有据此重新选择或纳入旧 dirty 文件。
- Cutie 的现有代码/权重仅只读复用，子进程 `PYTHONDONTWRITEBYTECODE=1`；未整体迁移 untracked。

## 2. 迁移完整性

- 固定 `7aaff9da22486b7d25557b3795dd258f9b65f10d` + 已批准的 12 个 overlay 文件不变。
- `PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only`：807 个文件 PASS。
- 六入口/其直接依赖沿用此前审计分类；本轮新修复仅在本仓库适配器和工具中，未修改 vendored snapshot 或外部 FoundationPose/SAM3/cuRobo 源码。

## 3. 代码完整性与 CPU 测试

- `compileall -q rm75_app tools tests/three_scene`：PASS。
- `pytest -q tests/three_scene`：**179 passed / 6.36 s**。
- `pytest -q tests`：**501 passed / 23.36 s**。
- 0 failed、0 skipped；1 个 trimesh `Scene.dump` 弃用警告。中间 170/492、172/494、495 全仓结果保留于本地，不替代最终总数。
- 新测试覆盖自定义 CAD 校验、初始化 argv、SAM3 原生分辨率、真实薄片无 UV 材质、直径标量类型，以及 PickPlace 退让逐点审计/诊断状态恢复/夹爪模型同步。

## 4. 前端

PickPlace、Magnetic、PushT 三个页面的实际浏览器加载、预览、状态轮询、输入桥和 Stop 本轮均 **NOT_RUN**。离线前端/契约测试随全量通过，不能替代浏览器证据。

## 5. PickPlace 回归

死机前已找到退让误碰撞的原因：规划模型把六个夹爪关节锁在 0.6 rad，而释放后的仿真实测约 0.011–0.0125 rad。四个夹指/垫片球与已释放 gluestick 的计算重叠约 1.45–4.44 mm。另发现原诊断逐障碍消融会错误重新启用起初禁用的目标缓存。

- 诊断现在逐次恢复原 enabled 状态，即使异常也恢复；不允许诊断改变后续规划世界。
- 释放后按仿真实测六关节分别同步所有 cuRobo kinematics owner，包括已缓存 IK；下一回合恢复原 nominal locks。
- 不改变真实碰撞球半径、拓扑、候选、容差；附着物已禁用的负半径哨兵逐 owner 原样保留。
- 真实硬件没有已验证夹爪几何输入时 fail closed，不能把仿真状态冒充实测硬件。
- 新退让执行前门禁检查返回路径每一个现有 waypoint 的 world/self；不设置夹爪世界免检。仍非连续物理执行/扫掠安全证明。
- 先前 audit v1/v2 仍退让失败；sync v1/v2 因负半径哨兵不同被严格拒绝；sync v3 已复用原 final-contact 的反向 5 点路径。v3 未包含最后添加的逐点门禁，不能拿来证明该门禁通过。
- sync v4：native 1/1 和 final success，但新增门禁挂在 joint-stage，原退让实际走 pose-stage，审计列表为空；**不算逐点门禁通过**。
- 已将门禁安装在原程序 motion-window 包装之外，覆盖 pose/joint 两条入口；碰撞时在转发执行之前失败。固定场景 runner 没有审计记录则明确失败。新增四个两入口正反例和幂等测试。
- 最终 `goal_pickplace_clearance_sync_v5`：exit 0，原程序 1/1、`final success = True`；实际记录 `samples=5 / all_valid=true / world_exempt_links=[]`。`loaded_mplib_modules=[]`。诊断前后目标缓存禁用状态一致，退让原路径复用成功。
- 原 final_target 策略跳过回到周期起点，未强制改变该行为；`native_return=null / verified_task_success=null`，不能把 dry-run/预览当作独立物理成功。
- [7 次退让诊断/修复结果与最后两次日志索引](../benchmarks/unified_scenarios/pickplace_clearance_20260907_summary.json)。重启前 `/tmp` 日志丢失如实标记，持久化 result/clearance 文件仍保留。
- 固定场景为原 `gluestick_desk_regression.json`。其他 PickPlace frozen/multi-object 本轮 **NOT_RUN**。候选筛选、relation screen、grasp fallback 未调参；无 MPLib fallback。

## 6. Magnetic 回归

- 之前已报告 four-wall 4/4、完整 triangle-roof 12/12 native 结果；本轮没有重跑结构规划，不将该记录当成感知验证。
- 本轮物体仅 **1 块正方形薄片**，使用旧审计模型 `red_jimu_plate_74x6p5x74.glb`，实际 extents 74 × 6.5 × 74 mm，scale=1。不是 105 mm 的积木块配置。
- 原 12 块顺序、角色依赖、重试、开爪退让及 payload/world 碰撞策略均未修改。
- 12 块真实同类实例辨识、已拼结构、三角片、料盘/基座定位均 **NOT_RUN**。

## 7. PushT GPU / cuRobo2

- 本轮复跑 `tools/check_pusht_chain.py`，exit 2 / `qualification_incomplete`；完整 GPU short-push **NOT_RUN，0 个完整 GPU cases**。
- 仍缺推头高度/姿态、物体高度/质心高度、接触 links、静态碰撞物、工具几何资格、相机/标记变换及标记尺寸；运行输入 observation/joints/goal 也未提供。
- 当前该 machine profile 是旧标记 observer，Jimu 的 RRTrack 成功不自动补齐 PushT 的工具几何或新 observer 接线资格。
- `hardware_reviewed=false`、`integration_qualified=false` 保留。五阶段预规划、碰撞/corridor/speed/freshness CPU 测试通过；未虚构真机配置来运行 GPU 案例。

## 8. Camera / tracking

### 实验和失败分母

1. 重启前 init v1：CLI 错误拒绝显式自定义 CAD 名称，未进入采集。
2. 重启前 init v2：RGB-D/参数/mesh 已落盘，无位姿输出；临时日志随重启丢失，没有可确认的进程退出码。不能将死机归因于某个未证实的模型/驱动错误。
3. 重启后 init v3：旧帧重放，`orange square magnetic tile.` 未检出掩膜，exit 1。
4. 5 个固定描述对照，原阈值 0.35 / 分辨率 1008 / 原 mask 参数：`orange square magnetic tile.`、`magnetic tile.` 失败；`orange square plastic panel.` 0.972656、`square plastic panel.` 0.929688、`orange object.` 0.714844 成功。全帧文字检测，没有人工 bbox 或 AprilTag。
5. init v4：选取明确的塑料薄片描述，42/42 原视角模板完成，SAM6D score **0.870117**，投影 bbox IoU **0.898678**，中心误差 **0.6474 px**。原深度有效比例 **91.20%**；`depth_repair.applied=false`、`pem_refine.applied=false`。这些都是图像/模型一致性指标，不是实测 6D 误差。
6. track v1：FoundationPose 构造失败，薄片无 UV 的 PBR 单色被 trimesh 转成 `(4,)` 而非 `(N,4)` 顶点颜色。补齐同一原始颜色至所有顶点，几何不变。
7. track v2：重复 float/double 后端错误；另触发 SAM3 640 分辨率的 RoPE 断言。保留已输出的失败帧后主动停止该专属进程，不反复加载同样失败的恢复模型。
8. dtype probe：同一帧、同一直径 `0.10485347053880018`，NumPy float64 时失败，Python float 时细化成功。仅修类型；SAM3 恢复默认对齐模型原生 1008，不改质量阈值或候选数量。
9. track v3：初始化以外 **30/30 accepted**，13 `tracked` + 17 `snap_corrected`，exit 0。DINO/SAM3 恢复保留开启，42 视角/84 项恢复库实际构建。但 v3 没有触发 LOST，不能宣布遮挡/全局恢复验证完成。

### 质量边界

- 30 帧来自重启后真实 D435 采集，约 0.2 s 间隔；相机采集完成后离线逐帧重放。采集时间/设备 frame number 在 `capture.jsonl`，不是在线机器人观测时效资格。
- 30 帧全部保留：固定参考薄片 mask 内有效深度前两帧仅 21、872 像素，后续约 2823–2871 / 3137。低深度帧也得到 accepted；不可把 accepted 当成独立深度/物理质量保证。
- 全 30 帧位置标准差约 **[0.421, 0.906, 3.721] mm**；各轴范围 **[2.209, 6.910, 18.070] mm**。
- 相对初始化最大位移 **14.516 mm**，薄片法线相对初始化差约 **4.50–11.45°**。它们不是有真值支持的绝对误差，但说明“检测到/持续 accepted”不足以证明磁吸装配精度。
- RRTrack 实际使用 SAM3/SAM6D 初始化、Cutie、FoundationPose 局部细化、DINO/在线恢复库及 SAM3 恢复；不是完全不使用 FoundationPose。
- 相机坐标下输出，没有应用未知外参，也没有用 AprilTag 作为输入或真值。已有 `assets/calibration/camera_extrinsic_opencv.npy` 未重标定，本轮未证明 base-frame 抓放定位误差。
- 正方形朝向有几何对称性；未验证磁极/正反面/12 个同类实例身份。不能直接删除正式装配的 AprilTag 配置。

本地目录：`runtime_data/three_scene/jimu_rrtrack_live_20260907/`。日志、30 帧 RGB/米制深度、初始化结果、完整轨迹和参数均保留；相机-only 采集工具为 `tools/capture_rrtrack_rgbd.py`，输出新目录且帧数有界。

## 9. RealMan no-motion

SDK connection/preflight、joint feedback、Stop API、gripper backend 实机检查均 **NOT_RUN**。没有连接/驱动机械臂或夹爪；仅使用相机。

## 10. Physical motion ladder

Reduced-speed free-space、isolated gripper、one/multi-object PickPlace、one/2/4/6+ Magnetic、one PushT push、push→fresh observe、closed-loop multi-step 全部 **NOT_RUN**。

## 11. Final summary

- Jimu：**单块 tagless 计算链已跑通，装配级准确度未合格/未证实**。下一步需独立位置/平面法线依据、深度坏帧门禁、遮挡再获取及多实例测试；不能用 30/30 accepted 直接替代这些证据。
- PickPlace：修复实测夹爪几何不同步与诊断状态泄漏；最终全路径审计见第 5 节。MPLib 不应重新进入工作链。
- PushT：仍缺实测 motion/observer profile 和完整 GPU 链输入；未声称已完成。
- 本次修改：自定义 CAD CLI/参数传递、原生模型兼容、纯相机采集工具、PickPlace 类型/状态/逐点审计及其单测、回传文档/JSON。没有改三场景成功条件或放宽碰撞。
- 本文件与 JSON 所在提交即本轮回传；总目标保持 active。
