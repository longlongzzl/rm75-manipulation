# 三场景无运动后续回传 — 2026-09-07

结论：**PushT 首次完成实际 cuRobo2 GPU 五阶段全链与快慢速度、障碍物负例验证。PickPlace 和无 Tag Jimu 尚未全部验收，不能说三条链都只差真机。没有连接机器人，没有机械臂/夹爪运动。**

[机器摘要](../benchmarks/unified_scenarios/three_scene_nomotion_followup_20260907_summary.json) 保留本轮 9 次 PushT 链路运行、1 次 GPU 诊断、6 次 PickPlace、1 次 30 帧 Jimu 回放的分母、聚合指标和原始证据 SHA256。前轮失败不覆盖，见 [上轮回传](CODEX_PICKPLACE_PUSHT_LOCAL_20260907.md)。原生完整日志、关节轨迹、RGB-D 只保存在本机。

## 0. 基本信息

- Tested base：`1be63d0a2790cbc359c91e3e8a38df42fb56ad2d` + 本回传提交内适配器/测试差异；机器摘要记录最后测试源码 SHA256。
- Branch：`chatgpt/three-scene-software-closeout`。没有 merge/rebase/reset。
- CPU Python 3.12；原 PickPlace / RRTrack：`foundationpose310` Python 3.10、torch 2.7.1+cu128、原 cuRobo1；PushT：`curobo2` Python 3.11、torch 2.11.0+cu128、cuRobo2。没有引入 MPLib 规划器。
- Linux、RTX 5060 Ti 8 GiB、driver 580.173.02。GPU 全部串行，单作业 MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2、600–900 秒 timeout。
- ManiSkill 沿用已迁移原环境；CPU PushT surrogate 不称为物理仿真通过。RealMan SDK / Robot IP：**NONE / NOT_RUN**。

## 1. 旧仓库 dirty worktree 审计

- 只读复用旧仓库及已有 Cutie 依赖，没有 reset/clean/覆盖/新增 overlay/整体复制 untracked。
- 旧 HEAD：`36798efbd12814841607951c9af470b309b34fd3`。本次核对的 status SHA256：`15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`，与本轮开始时继承的只读审计记录一致。
- 六入口/直接依赖的原分类和 12 个已批准 overlay 沿用，不擅自重新分类纳入当前 dirty 文件。仍不声称与重启前历史 status 哈希相同。

## 2. 迁移完整性

- 保留 fixed `7aaff9da22486b7d25557b3795dd258f9b65f10d` + 12 approved overlay，snapshot **807 files**。
- `tools/migrate_working_sources.py --target-repo . --verify-only`：PASS；compileall 包含 vendor。没有修改 snapshot 或外部模型/cuRobo 源码。
- manifest 的 `dependency_completeness_verified=false`、`runtime_gpu_verified=false` 不擅自改成 true。局部测试成功不代表所有入口完成资格化。

## 3. 代码与 CPU 测试

| 检查 | 本轮最终结果 |
| --- | --- |
| compileall：rm75_app、tools、tests，包含 vendor | PASS |
| 完整 tests/three_scene | **222 passed**，1 warning，6.44 s |
| 完整 tests | **544 passed**，1 warning，23.54 s |
| git diff --check | PASS |

与上一提交相比新增 33 项单测：Cartesian IK 5、释放接触边界 17、Jimu 深度/12 同类身份 3、GPU 结果/实测动力学门禁 8。没有 skipped；唯一 warning 是已有 trimesh Scene.dump 弃用提示。GPU 实验之后的最后检查另补了目标 cuboid 位姿有限性/四元数资格门及 2 项负例，随后复跑以上全部 CPU 测试；没有把这两项负例说成新增 GPU 运行。

## 4. 前端

本轮没有改前端。三场景前端/CLI/权限/Stop 的离线测试随完整 suite 通过。上轮真实浏览器三 tab、preview、PushT CPU loop、polling/Stop 的结果保留于前轮报告；**本轮浏览器、builder 实际导入导出、原 native input 桥、native worker Stop 均 NOT_RUN**，不冒充复跑。

## 5. PickPlace GPU / 原程序回归

继续原 `lazy_place + primary_only`，候选、seed、碰撞球、成功阈值未减少或放宽。全部使用原 fixed scenes，所有运行 `loaded_mplib_modules=[]`。

| 本轮运行 | 原生完整返回 | 释放后审计 |
| --- | --- | --- |
| current_table_all / release_v1 | FAIL，完成 2/7 cycle 后停止 | 刷子路径接触深度增加，拒绝 |
| current_table_all / release_v2 | FAIL，完成 5/7 cycle 后停止 | 刷子仍失败；后续原 transport 诊断遇到 shared-sphere mask，fail closed |
| legacy_gluestick / final | PASS，1/1 | 5 点全部通过 |
| gluestick_jitter_00 / final | PASS，1/1 | 4 点全部通过 |
| gluestick_yaw_00 / final | FAIL，0/1 | 没有完整 grasp/place pair，未到释放 |
| gluestick_swap_00 / final | PASS，1/1 | 14 点全部通过 |

本轮 **3/6 次 native 完整返回，3/6 次全部要求的 clearance 通过**；七物体完整严格回归 0/2。v2 的 cycle=True 不能掩盖原程序跳过失败 clearance 的 warning。

### 释放后的局部接触验证

新增的是 SIM-only 适配边界，不重写旧候选生成/规划：只有已明确 released 的当前物体、已知原 cuboid 可以成为许可目标。允许 finger/pad 与刚释放目标的已有接触，其他 links↔该目标用原球/盒 SDF 检查；所有 links↔其他世界物体及全部自碰撞用原 GPU 检查。路径加密到每关节 ≤.01 rad；目标 collider 的临时搜索状态在正常/异常路径都恢复，未屏蔽共享机器人球。

接触深度不许超过释放起点最大深度 + 1 µm。刷子原路径的首个加密点从 **4.258744 mm 增到 4.343470 mm**，增加约 **0.084727 mm**，仍被拒绝，没有提高门槛让它通过。v1 把路径拒绝作为全局终止；v2 仅将几何拒绝返回原候选循环的 `None`，其他原候选仍可尝试，未知状态/原生异常继续 fail closed。真正执行入口独立复查，不接受未经审计的路径。

剩余两类问题明确分开：刷子原释放/退让几何，及历史 PickPlace transport 的 whole-link masks。后者未在本轮放宽或冒充严格完整搬运验收；v2 在 masked diagnostic 阶段停止，不把它记成通过。yaw 场景失败也保留，不打开额外 legacy fallback。

`verified_task_success=null`：这些是原 native GPU 规划/预览，不是独立物理物体成功观测。

## 6. Magnetic / Jimu 规划

本轮未改 four-wall / triangle-roof 原结构规划、开爪/退让、payload 或依赖顺序；前轮 4/4 和 12/12 原场景结果继续保留，**本轮结构 GPU 复跑 NOT_RUN**。新增 12 个同类条目的身份单测：失败实例也占原索引，选择失败实例必须报错，不能过滤后把下一块误当成它。该单测不等于已经验证 12 块实物识别。

## 7. PushT GPU / cuRobo2

### 问题与修复

原 soft Cartesian line 不是严格直线。实际 GPU 诊断中原下降 17 个点的 coarse IK **17/17** 可行；native 线性代价 .5 / 1 / 2 仍分别偏离 **10.049857 / 9.959924 / 7.860889 mm**，超过原 3 mm corridor。继续试 4 时原插值缓存容量不足报错；该失败保留，不靠扩大缓存或改容差冒充通过。

保持原五阶段目标、10 个 push 候选、3 friction scales、原 IK seeds/返回种子数和物体/桌面几何。approach 仍用 MotionGen；四段直线沿原世界方向按 ≤5 mm 取点，用上一关节状态播种原 GPU IK，保留所有成功 IK variant；逐候选关节边按 ≤.02 rad 加密，检查 TCP corridor/姿态/碰撞。斜向线不吸附到坐标轴。任一中间点无可行边则不返回半条路径。

接触搜索阶段仅暂时移出 T 的两块目标盒；恢复后对完整时间参数化路径逐点检查，只允许已指定 finger/pad links↔T，桌面/无关物体/其他 links/自碰撞不豁免。未来目标的 3 friction × 2 进度碰撞审计保留。完整 approach → descend → contact → push → retreat 全部规划/审计结束后，执行器才有机会申请新观测和动作。

### 全部本轮 GPU 链路运行

| run | 完整链 | 说明 |
| --- | --- | --- |
| chain_v1 | PASS | 首次 orthogonal_tool 全 5 段 |
| positive_blocker_v2 | PASS | 完整链后注入无关障碍，实际 GPU audit 拒绝 |
| rotated_orthogonal_v1 | FAIL | T 旋转 .3 rad、工具固定，descend 11/16 无可行 IK |
| orthogonal_neighbor_blocked_v1 | 拒绝 | 正例场景只增加障碍，approach IK 被挡住 |
| baseline_v1 | FAIL | 原另一工具朝向的下降无完整可行 IK 链 |
| slow_v1 | PASS | 同 orthogonal_tool，5 mm/s |
| yaw_matched_v1 | PASS | T 和工具一起旋转 .3 rad，斜向完整链及障碍 audit 通过 |
| fast_metrics_v1 | PASS | 15 mm/s，全点 FK 速度/关节速度/加速度 + 障碍 audit |
| slow_metrics_v1 | PASS | 5 mm/s，全点动力学 + 障碍 audit |

共 **6/9 次完整链通过**（包含重复与速度对照，不是 6 个独立场景）；正常输入 6/8，1 个预设阻挡用例被拒绝。已完成路径上的无关障碍注入 **4/4 正确拒绝**，不是只拿 IK_FAIL 充当碰撞门负例。另有 1 次上述诊断失败。前轮 12 次 GPU 失败仍保留，不从累计记录删除。

固定工具旋转 T 失败与工具随 T 旋转的新仿真正例分开记录，未修改失败输入或自动转动用户工具来刷成功率。

### 实测速度与几何

| 同一 orthogonal_tool 路径 | 15 mm/s 设置 | 5 mm/s 设置 |
| --- | --- | --- |
| 规划计算耗时（非运动耗时） | 14.532 s | 20.363 s |
| 时间参数化总时长 | 76.800 s | 201.267 s |
| 最终审计点数 | 2309 | 6043 |
| 最大实测 TCP 速度 | 14.0357 mm/s | 4.68783 mm/s |
| 最大关节速度 | .128698 rad/s | .063884 rad/s |
| 最大关节加速度 | .494959 rad/s² | .449213 rad/s² |

上限为各自 TCP speed、.25 rad/s、.5 rad/s²，均未放宽。正交下降最大 corridor 偏差 **.015071 mm**，推段 **.007780 mm**；斜向正例最大约 **.016980 mm**，均远低于原 3 mm。这里是数值规划模型误差，**不是机器人物理定位精度**。

验证入口新增 `validation_success`：即使计划完成，只要后续诊断报错、缺阶段/速度证据、速度/加速度越限或请求的负例未拒绝，都不能返回成功。早期只有路径的记录不补造速度数据；斜向记录已有速度数据，可用相同门离线核对。

### 真实场景资格

全部为**真实 GPU + 明确人工定义的 low_table 仿真输入**，NoMotionArm.execute 会抛错，未连接 SDK，不能把仿真夹爪/桌面当成实际推头测量。真实 profile 没有被填造：复跑 `check_pusht_chain.py` 为 exit 2 / qualification_incomplete，仍缺 10 个 motion/observer 字段及 observation/joints/goal，integration_qualified=false。

## 8. Camera / RRTrack

使用已有单片橙色积木的真实 RGB-D 录像，**本轮没有新采集、没有 AprilTag**。SAM3/SAM6D 初始化、Cutie、FoundationPose、DINO/在线库及 SAM3 恢复配置保留，未降低原分辨率/模板量/质量门槛。

- 将原 `min_valid_depth_px=24` 门从 snap 扩到初始化、普通 tracking 与恢复候选。坏帧不得调用 pose refine 或更新位姿/记忆；恢复候选过滤后仍保持来源与概率对应关系。
- 初始化 mask：3137 像素，2861 有效深度。录像第 1 帧实际预测 mask：2997 像素、**17 有效深度**，正确 LOST / insufficient_depth / accepted=false / memory_updates=[]。
- 第 2 帧：候选 829 有效深度，通过已有恢复器，`recovered / source=online`，随后正常跟踪。
- 除初始化外 **29/30 accepted**：16 tracked + 12 snap_corrected + 1 recovered；1 个坏深度帧被拒绝。不能以追求 30/30 为由取消该门。
- 29 个 accepted 帧位置标准差约 **[.432, .431, 3.072] mm**，各轴范围 **[2.306, 2.479, 17.987] mm**；相对初始化最大变化 **14.595 mm**，薄片法线变化 **5.254–11.066°**。无真值，不称为绝对误差或已达到磁吸装配精度。
- 已验证实际坏深度失跟→下一帧恢复，不等同于真实遮挡、移出视野后异位再获取或 12 实例身份验收。后面三项与装配级标定/准确度仍 **NOT_RUN / NOT_MEASURED**。
- live observe → push → fresh observe：**NOT_RUN**。离线回放时间戳不是机器人在线 freshness 资格证明；stale/replay/drift 拒绝由本轮完整离线测试覆盖。

## 9. RealMan no-motion

真实 SDK connection/preflight、关节反馈、Stop API、夹爪 backend：**NOT_RUN**。GPU 起始关节使用仿真模型默认值，未冒充真机反馈。本轮没有任何机械臂或夹爪运动。

## 10. Physical motion ladder

低速自由空间、单独夹爪、单/多物体 PickPlace、单件/2/4/6+ Jimu、一次 PushT、推后新观测、多步真实闭环：全部 **NOT_RUN**。

## 11. 收尾与剩余项

- PushT：完整无运动 GPU、严格直线与快慢速度/无关碰撞负例已完成。两个原工具/目标组合仍不能规划，保留失败；实际推头、桌面、外参和观测输入需要测量/确认，不能凭空资格化。
- PickPlace：3 个单物体固定场景通过；yaw 和七物体仍未闭环，刷子接触增长及历史 transport mask 不作通过处理。没有重设计旧抓放算法。
- Jimu：原复杂结构规划结果保留；单片 tagless 坏深度/恢复与同类编号软件问题修好，装配准确度、多实物、真实遮挡仍不能宣称完成。
- 本轮改动仅新仓库适配器、验证/聚合工具、单测和回传。原始图像/轨迹/完整日志不新增到 Git。
- **本轮只提交本地，未 push。** 此前推送因父提交含原始轨迹/日志及内部路径/硬件标识，被环境审核拒绝；未得到对这些数据上传至指定远端的补充许可，没有重试或绕过。远端不能视为已更新，总目标仍未完成。
