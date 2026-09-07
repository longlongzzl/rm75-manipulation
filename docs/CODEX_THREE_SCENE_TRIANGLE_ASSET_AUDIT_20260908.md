# 三场景续报：Jimu 原三角资产依赖漏迁审计 — 2026-09-08

**NEEDS_REVIEW：确认原入口的模型选择与旧工作目录不等价。尚未迁移候选资产，也未宣称三条链完成。**

机器摘要：[只读资产与帧审计](../benchmarks/unified_scenarios/jimu_triangle_asset_audit_20260908_summary.json)。本轮没有机械臂/夹爪运动、SDK、相机、GPU 任务或远端上传。

## 0. 基本信息

- Tested base：`ae56031b2f15f5f8abf4f2b2d94a880c6b670392` + 本回传差异；branch `chatgpt/three-scene-software-closeout`。
- 上轮为实际进展：完成五次 Jimu GPU、只读目标诊断与本地提交。本轮从 clean worktree 开始，未重启已终止 GPU 作业。
- CPU Python 3.12 / NumPy / trimesh。使用已验证 snapshot 中原 object_specs 尺度函数与选定原几何函数的 AST；**没有 import 原机器人入口或运行原 native task**。
- 新工具 `tools/audit_jimu_triangle_assets.py` 只读源资产、原 builder/manifest；没有安装/升级依赖、修改 planner 或外部模型。

## 1. 旧 dirty worktree 与逐文件候选审计

旧目录只读，没有 reset/clean/stash/覆盖或整体复制 untracked。运行前后 status SHA256 均为 `15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`；原任务三文件哈希未变。

旧 `Beta_demo-codex-v0.9/rm75_jimu_triangle_roof_apriltag_portable.py` 与已审阅 snapshot 的该入口 SHA256 相同：`c6c29a9adf6815e52426f81e496705d43e525d2628a26165dd3ecff8bb1c578b`。

原入口直接定义 `DEMO_TRIANGLE_MESH`。当以下 `.glb` 存在时，`_triangle_geometry_mesh_and_scale`、`_demo_triangle_spec` 选它并使用 scale=1；不存在才回退 `pick_jiaobang/meshs/red_triangle.glb`，按 real_longest_axis=0.12 m 缩放。**相同入口代码并不保证相同模型分支。**

| 候选文件 | SHA256 | 现状与理由 |
| --- | --- | --- |
| `Demo_Triangle/red_triangle_74x135x6p5.glb` | `fac68dee16a4f8d5b17d7de0f4007ac2445472c70d338c46dbb98908e38f4175` | 旧目录存在、untracked、1,493,004 bytes；已审阅 triangle entry 直接优先依赖，当前快照缺失 |
| `Demo_Triangle/red_triangle_74x135x6p5.glb.coacd.ply` | `f6d655ec58ef7db661241935c6d7f9ee3d0ac35c6807e0d2d1b57f9772582a12` | 旧目录存在、untracked、8,146 bytes；原 SIM coacd 碰撞资产，当前快照缺失 |

两项均重新分类为 **candidate_final_fix（原模型/碰撞资产依赖）**，不再只是整个 Demo_Triangle 目录的未知清单项；**仍未进入现有 12 项 approved overlay，未自行批准或复制**。已向用户请求仅这两个文件的逐文件纳入批准。

这不是切换到 `Demo_Triangle/run_four_wall.py` 或其另一套算法/缩减候选 profile。此前排除该目录的其他代码和启动参数仍成立；这里只审计指定旧 portable 入口自己使用的两个资产。

## 2. 迁移完整性与资格区别

当前 `verify_snapshot` PASS，807 文件，固定 `7aaff9d` + 原 12 项 overlay 不变。该验证只证明 manifest 中现有文件未被改动，**不证明 optional exists() 分支的运行依赖闭包完整**。

本轮没有重新迁移、追加 overlay、改变源文件或标记 dependency/runtime 资格通过。批准后应合并成单一显式 overlay 清单并按原流程重建**新仓库**快照，不能给旧仓库打补丁或在已有快照上形成未记录的第二层覆盖。

原 `.coacd.ply` 的 source MD5 与这份 `.glb` 一致（`83ddd8be05e9be9461cec12a2a54cee0`）；其实际包围盒约为 75.956 × 8.975 × 136.850 mm。保留原碰撞资产，不缩小到视觉 mesh 大小。源摘要匹配不等于已验证 SAPIEN 的全部参数/缓存复用，也不等于物理几何资格；本轮未调用分解器。

## 3. 编译与完整 CPU 测试

- compileall：`rm75_app tools tests`，包含 snapshot，PASS。
- 新增资产审计单测：**19 passed**，0.84 s。
- 完整 `tests/three_scene`：**475 passed**，1 warning，16.19 s。
- 完整 `tests`：**797 passed**，1 warning，32.51 s。
- 无 failed/skipped；唯一 warning 为既有 trimesh Scene.dump 弃用提示。

覆盖原 optional 模型选择、原 real_longest_axis 缩放、正负 Z 尖端、标准 target tip-up 与 builder 轴的区别、显式 parent transform 闭合、输入不变、非法父件/接触边/非竖直帧/非有限值拒绝、非单位 scene graph 拒绝、同名不同尺度不算等价、coacd 来源匹配/不匹配且不重建。

完整 suite 通过只证明这些离线断言与既有回归通过，不代替恢复资产后的 GPU 验证。

## 4. 前端

本轮前端未改；离线 frontend/CLI/Stop/互斥/授权等覆盖随完整 suite 复跑。真实浏览器导入/编辑/round-trip/Stop：NOT_RUN。没有用 CPU 资产审计冒充浏览器实操。

## 5. PickPlace

本轮新 GPU、native SIM、benchmark、相机：NOT_RUN。保留 [PickPlace / PushT 回传](CODEX_THREE_SCENE_PICKPLACE_PUSHT_20260908.md)：五固定场景 2/5，额外诊断后 2/6，七物体 4/7 且退让警告。其抬升、退让缺口不因 Jimu 资产发现自动关闭。

## 6. Jimu：模型与目标帧的实测差异

### 6.1 两个原分支的实际模型

| 项目 | 旧目录优先资产 | 当前快照回退资产 |
| --- | --- | --- |
| 原 scale 规则 | 1.0 | 0.12 / 原 mesh 最长边，约 0.1198997763 |
| 原 helper 测得 extents | 74.000 × 6.500 × 135.000 mm | 64.930 × 8.721 × 120.000 mm |
| 原 helper `tip_needs_local_y_flip` | false | true |
| 原 mesh 的尖端方向 | 本地 +Z | 本地 −Z |
| 标准目标执行原 tip-up rotation 后 | 朝上 | 朝上 |
| 原 builder 直接使用设计轴后 | 四个屋顶均朝上 | 四个屋顶均朝下 |

两个实际 mesh 的 scene node transform 均为单位矩阵；不是忽略 GLB scene graph 变换造成的方向推断。尺寸是加载实际文件并执行原缩放公式的测量，不是按文件名猜测。

### 6.2 原 builder 不是父子矩阵序列化损坏

四个屋顶显式 `parentRelativeTransform` 与原 `center/u/n/v` 重建的父子矩阵完全闭合（最大绝对差 0）；保持原 top → bottom 依赖和局部轴，未改未知字段。

原父子中心间距 104.500 mm、父墙高度 74 mm、屋顶层增量 4 mm：

- 135 mm 原模型：名义包围盒边缘间隙 0 + 4 = **4 mm**。
- 120 mm 回退模型：名义包围盒边缘间隙 7.5 + 4 = **11.5 mm**。

后者与上轮完整任务实际 `[jimu-builder][contact] ... edge_gap_mm=11.500` 一致。这里是原目标的名义边缘计算，不是磁吸捕获、真实释放或三角物理接触资格。

### 6.3 对上一轮结论的影响

[屋顶 IK 报告](CODEX_THREE_SCENE_JIMU_ROOF_IK_20260908.md) 的实际数据保留：标准场景 12/12、原完整任务 8/12、已观测前台 688/688 release 底座/墙体冲突。这些都是**当前回退模型**下的证据，不能认证旧工作目录的模型等价性，也不能把标准 target 自带的 tip-up 校正当成 builder 已获得同样校正。

本轮已证实“资产遗漏改变原分支、尺寸和 builder 尖端方向”；**尚未用恢复原资产后的 GPU 反事实对照证明它解释了全部屋顶 IK 失败**。不通过直接翻转 builder、修改抓法、移动墙体或放宽底座碰撞来代替正确的原依赖恢复。

批准迁移后仍须按原要求完成 snapshot verify/compile、原无运动 smoke、four-wall、标准 triangle-roof、原完整任务；保留前台/预取诊断、失败重试分母、搬运全世界/自碰撞/负载与释放/返回执行门。

## 7. PushT

本轮新 GPU：NOT_RUN；不改工具尺寸、TCP、contact links 或实际 profile。既有六次 GPU 仍是三条完整链通过、两条下降失败、一个障碍正确拒绝；3/3 追加障碍与 30/30 注入门禁拒绝保留。闭合夹爪还是独立推头及实际接触/TCP 配置尚待确认。

## 8. Camera / tracking

新 RRTrack、fresh observation、tagless 多实例装配精度与 PushT 推后重观测：NOT_RUN。Jimu 单片 RRTrack 历史结果不作为完整模型/装配资格。

## 9. RealMan no-motion

SDK/preflight、真实关节反馈、控制器 Stop、夹爪后端：NOT_RUN。未使用机器人地址。

## 10. Physical motion ladder

机械臂、夹爪、自由空间、单/多物体 PickPlace、Jimu 装配、PushT 单推/闭环：全部 NOT_RUN。

## 11. 总结与待批准项

这是一个具体的旧版本恢复缺口，不是“增加 seeds”或“夹爪底座免检”问题的授权。旧目录、现有 overlay、snapshot 和生产规划均未改；三条链目标仍未完成。

下一步需要用户明确批准表中两个资产按 SHA256 纳入单一 overlay，再按原顺序复跑。没有批准前不迁移或切换模型。

本轮仅做本地提交，未 push；此前父提交上传审核限制仍在，不重试、不重写历史绕过。原始轨迹、关节、完整运行日志继续留本地。
