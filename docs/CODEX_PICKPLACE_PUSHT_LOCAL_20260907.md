# PickPlace / PushT 本地后续回传 — 2026-09-07

结论：**NEEDS_REVIEW，不是三条真机 demo 完成。** 本轮实际执行本机 GPU 和前端测试，没有机械臂/夹爪运动，没有连接机器人。Jimu 不只是差真机：12 件原规划已有前轮结果，但无 AprilTag 感知的装配精度、多实例和遮挡恢复仍缺验收。

机器摘要：[pickplace_pusht_local_20260907_summary.json](../benchmarks/unified_scenarios/pickplace_pusht_local_20260907_summary.json)。包含本轮 **7 次 PickPlace、12 次 PushT GPU** 的完整运行分母、逐阶段结果、原日志/JSON SHA256 和最终代码 SHA256。原生完整日志、轨迹和截图只留本地 `runtime_data/three_scene/pickplace_pusht_local_20260907/`，本轮不新提交这些原始数据。

## 0. 基本信息

- Tested base: `9f8c9f9` + 本回传提交中的适配器、验证入口和单测差异；机器摘要记录完整父 SHA 与代码内容哈希。
- Branch: `chatgpt/three-scene-software-closeout`。
- Python: CPU 3.12；PickPlace `foundationpose310` Python 3.10 / torch 2.7.1+cu128；PushT `curobo2` Python 3.11 / torch 2.11.0+cu128。
- Linux，RTX 5060 Ti 8 GiB，driver 580.173.02。原 PickPlace 使用原 cuRobo1，PushT 使用 cuRobo2，不能合并为同一后端结果。
- ManiSkill：复用原环境及已迁移 task，本轮未升级外部依赖。RealMan SDK、robot IP：**NONE / NOT_RUN**。
- GPU 作业全部串行，单次进程 MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2，超时 600–900 s。使用持久化日志及已重建的原生 cuRobo 扩展缓存，未复制旧不明 ABI 的二进制。

## 1. 旧仓库审计

- 旧仓库只读，未 reset/clean/覆盖/重新应用 overlay。旧 HEAD 沿用 `36798efbd12814841607951c9af470b309b34fd3`。
- 本轮开始/结束的 `git status --porcelain=v1 -z` SHA256 均为 `15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`。
- 此哈希不同于重启前历史回传的 `7a7759…`；**不声称跨重启旧仓库状态相同**，不擅自重新迁移当前 dirty diff。
- 新工作均在新仓库适配层；未修改已审阅 vendor 文件或外部 cuRobo/FoundationPose/SAM3 安装。

## 2. 迁移完整性

- 继续 fixed `7aaff9da22486b7d25557b3795dd258f9b65f10d` + 12 个已审阅 overlay 文件。
- Snapshot **807 files**；`tools/migrate_working_sources.py --target-repo . --verify-only` **PASS**，compileall 包含 snapshot。
- 本轮没有重新迁移；六入口及 overlay 的完整性由现有 manifest 校验，不伪造新的来源提交。
- `dependency_completeness_verified=false`、`runtime_gpu_verified=false` 保持未资格化。

## 3. 代码与 CPU 测试

| 实际检查 | 结果 |
| --- | --- |
| compileall：rm75_app、tools、tests，含 vendor | PASS |
| 完整 tests/three_scene | 189 passed，1 warning，5.99 s |
| 完整 tests | 511 passed，1 warning，23.58 s |
| git diff --check | PASS |

唯一 warning 是已有 trimesh Scene.dump 弃用提示，没有跳过失败测试。本轮新增 10 项单测：固定场景/原生结果识别 4 项，返回预规划事务锁及世界恢复正常/异常路径 4 项，PushT 直线接口与时间重采样后的 corridor 审计 2 项。

## 4. 前端

按 Browser 技能连接后，浏览器实例列表为空；故障检查确认不可用。随后在隔离临时 Python 环境运行仓库既有 `tools/check_workcell_browser.py`，使用本地已安装 Chromium 和真实 HTTP，不开放真机开关。

- PickPlace 页面/preview/status polling：PASS。
- Jimu 页面/preview：PASS；本轮没有重做完整 builder 文件导入/导出及原程序交互提示桥接，**NOT_RUN**，不拿历史测试替代。
- PushT 页面、CPU surrogate loop、状态轮询、Stop：PASS；browser page errors=[]。
- 真机按钮禁用；没有请求硬件动作。测试服务已关闭。
- PickPlace/Jimu 原 native worker 的 prompt bridge/Stop：本轮 **NOT_RUN**。

## 5. PickPlace GPU / 原程序回归

### 共享状态修复

1. 七物体 v1：cycle 1 完成，下一物体释放后更新夹爪模型时，观察到后台返回预规划的临时 link mask，fail-closed 停止。
2. v2：给原两个 return-planning 函数加现有 GPU RLock 的事务级保护，仍发现后台场景更新把 `virtual_table_plane` 留在 disabled 状态，严格退让门拦截。
3. v3：事务结束以原生 update_world API 恢复前台 WorldConfig、MotionGen/IK/缓存 IK、障碍物 enable 状态和场景签名；异常路径同样恢复。没有锁住 worker 创建/消费 join，避免 join/锁死锁；原 mask try/finally 未取消。

未关闭预取、未更改原候选、seed、碰撞半径、成功阈值。夹爪同步遇到 attachment/mask 仍拒绝；新增异常信息明确列出状态。

### 固定结果，原生成功与严格退让分开

| 场景 | 原生结果 | 释放后严格退让 |
| --- | --- | --- |
| 原固定桌面 7 物体，v3 | 7/7 cycles，final=True | **未全通过**：6 段共 41 点通过；shuazi 退让失败 |
| legacy_gluestick | 1/1，exit 0 | 5/5 点通过 |
| gluestick_jitter_00 | 1/1，exit 0 | 4/4 点通过 |
| gluestick_yaw_00 | 0/1，exit 42 | 未到释放/退让 |
| gluestick_swap_00 | 1/1，exit 0 | 14/14 点通过 |

- 最新 5 个不同场景：原生完整返回 4/5；全部要求的退让均有审计且通过 3/5。包括 v1/v2 失败在内，本轮原程序成功返回 **4/7 次运行**。不能将 7/7 cycle 当作 7/7 严格完整链路通过。
- shuazi：反向 final-contact 终点与放置物碰撞；重新退让规划 `INVALID_START_STATE_WORLD_COLLISION`。原程序随后记录“without clearance”，通过旧 return prelift 补救继续，仍报 cycle=True。新入口单独记录 `clearance_failures` 和 `strict_clearance_success=false`，不掩盖它。
- yaw 场景：一个 grasp approach 的 line error 16.3 mm > 原 15 mm 上限被拒；最终 paired relation IK 未得到完整 grasp/place pair。未启用原禁用的 fallback、未放宽阈值。之后原目标切换还暴露 `bi` 不在单物体场景，不能把后者冒充首要失败原因。
- 所有运行 `loaded_mplib_modules=[]`。旧 FCL/MPLib 数据容器不再构造第二套规划器；未删除 hash 归档内的旧兼容源码。
- 原 native transport 仍有历史 whole-link masks、物体缩放等策略，本轮**未新增/放宽，也未宣称全链路 strict transport 已通过**。本轮严格审计仅覆盖已执行的释放后 clearance 路径，未形成连续碰撞或物理接触证明。
- `verified_task_success=null`：这些是 dry-run planning/preview，目标位姿同步不是独立物理成功观测。

## 6. Magnetic / Jimu

本轮不重跑 Jimu GPU、不改 12 件策略。前轮 four-wall 4/4、triangle-roof 12/12 和单片 RRTrack 结果见 [此前回传](CODEX_JIMU_RRTRACK_AND_CLEARANCE_20260907.md)。单片 30/30 accepted 仍有最大 14.516 mm 位移变化和 4.50–11.45° 法向差，不构成无 Tag 装配精度验收。12 个实物实例、遮挡恢复和真实磁吸装配：**NOT_RUN**。

## 7. PushT 实际 GPU 验证

### 场景资格边界

`tools/run_pusht_gpu_validation.py` 使用**真实 cuRobo2 GPU + 明确人工定义的仿真输入**，不是假 backend；NoMotionArm.execute 直接抛错。没有将这些仿真参数写入真实 machine profile，也没有将当前夹爪模型冒充已经测量的 5 mm 推头。

- 原 raised_table：TCP z=.15 m，hover=.08 m；baseline 和另一工具姿态均 approach IK 失败，诊断包含 `link_4 ↔ gripper_base_link` 自碰撞。
- 128 位姿 coarse IK 空世界定位实验：**14/128** 成功，仍启用自碰撞。这只是可达性诊断，不是完整桌面/任务通过。
- 另建 low_table 仿真：TCP z=.02 m、table top=-.01 m、T 高度=.04 m、hover=.08 m。保留 raised_table 的失败分母，不用新场景覆盖原结果。
- 原 Config 的 10 个接触候选、3 friction scales、IK/trajopt seeds、碰撞模型保持；corridor=3 mm、orientation=.05 rad、push=.012 m、speed=.015 m/s。

### GPU 暴露的问题与代码修复

- 原下降只调用带软 TCP 偏好的自由空间规划，实际返回弯曲路径。世界轴下降/接触/推/抬升现接到已有 `plan_linear_candidates`；任意斜向 push 不吸附到坐标轴，仍保留原搜索与严格审计。
- 本机线性代价强度 .5、1.0 两次均未达 3 mm 门槛；最终使用 1.0（原生接口默认值），没有放宽验收。该接口修改**不等于完整短推已经修好**。
- 新增原采样和时间重采样后全部点的 corridor/姿态审计，以及末端位置/姿态验收。接触阶段仅允许指定 finger/pad links ↔ T 两个 target boxes；桌面、邻居、其余 links、自碰撞不豁免。
- 最终错误记录整段最大偏差；早期 v3/v4 错误只记录第一个越界点，不能把 3.47/3.34 mm 当作整段最大值。

### 最终 GPU 四场景矩阵

| 输入 | 已通过的阶段 | 阻断位置 |
| --- | --- | --- |
| low_table baseline | approach，1324 timed points | descend，linear planner 未返回成功路径 |
| low_table orthogonal_tool | approach，1507 points | descend，最大 corridor 偏差 **9.95992 mm > 3 mm**；最大姿态差 .02349734 rad |
| low_table rotated T | approach，1259 points | descend，linear planner 失败 |
| low_table blocking_neighbor | 无 | approach IK 失败 |

完整 GPU 短推 **0/4**（正常输入 0/3；阻挡负例被拒但未到完整 collision-audit，**不宣称负例验收通过**）。本轮 12 次 GPU 程序尝试全部保留，0 次完整链通过。`gpu_low_orthogonal_v2` 因临时 patch 可执行路径失效实际重复了修复前代码，单独计为重复实验，未冒充新版本结果。

当前仍未到 contact/push/retreat 的 GPU 全链审计，也未运行 GPU 快慢速度对照；17 项 motion bridge CPU 单测覆盖速度参数化、stale/replayed/drift 拦截、全链先规划后执行、碰撞异常和世界恢复，**不能代替 GPU 缺失阶段**。

## 8. 相机 / 跟踪

本轮没有重新采集相机，没有 live observe → push → fresh observe。真实 PushT profile 仍缺 10 项：TCP 高度、T 质心高度/厚度、工具姿态、contact links、静态障碍物、工具碰撞几何确认、两项外参、tag 尺寸；另外缺本次 observation / joints / goal。`check_pusht_chain.py` 正确返回 qualification_incomplete、exit 2，integration_qualified=false。

## 9. RealMan no-motion

Hardware connection/preflight、真实关节反馈、SDK Stop、真实夹爪 backend：全部 **NOT_RUN**。GPU 测试关节起点来自 cuRobo 默认仿真状态，不冒充真实反馈。没有机械臂或夹爪运动。

## 10. Physical ladder

低速自由空间轨迹、独立夹爪、单次/多物体 PickPlace、单件/多件 Jimu、一次 PushT、推后新观测、闭环多步 PushT：全部 **NOT_RUN**，仍需独立授权及前置资格门通过。

## 11. 剩余问题与交付

1. PickPlace：共享预规划状态竞态已修复；继续处理 shuazi 释放后严格退让和 yaw 场景无完整 grasp/place 配对；单独审阅旧 transport 接触策略，不以兼容成功冒充全严格通过。
2. PushT：真实 GPU 已跑，但仍需要解决有碰撞约束的下降 IK/直线精度，随后才有 contact/push/retreat 与 GPU 速度对照；真实工具/场景/跟踪 profile 不能编造。
3. Jimu：保留 12 件路线，补装配级无 Tag 感知证据，不能直接跳真机。

本回传与代码只提交本地。上一轮推送 `9f8c9f9` 已被环境审核拦截（含内部路径/硬件标识/轨迹和原生日志，远端未验证）；用户此轮授权的是本地工作，未回答此前对这些原始数据的推送许可。本轮没有绕过或重试该 push，远端不声称已更新。
