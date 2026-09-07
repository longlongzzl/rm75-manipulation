# 三场景无运动工作台验证续报 — 2026-09-08

本轮从 9 月 7 日执行至 8 日。**尚未全部完成，也不是 Jimu 只差真机。** PushT 的完整 GPU 链证据已在前轮取得；本轮继续补齐真实工作台入口、完整 builder 前端和 PickPlace 搬运审计，保留所有失败。没有连接机器人 SDK，没有机械臂/夹爪运动。

机器摘要：[native closeout followup](../benchmarks/unified_scenarios/three_scene_native_closeout_20260908_summary.json)。前轮：[PushT 完整 GPU / RRTrack 回放](CODEX_THREE_SCENE_NOMOTION_20260907.md)。本机原始数据根目录为 `runtime_data/three_scene/closeout_followup_20260907/`；原始日志、轨迹和图像未新增到 Git。

## 0. 基本信息

- Tested base：`2869454b55ed2639766f4f4e4ac7cc8b49ac0e66` + 本回传所在提交的适配器/工具/测试差异；摘要记录最终源码 SHA256。
- Branch：`chatgpt/three-scene-software-closeout`。
- CPU：Python 3.12；原生 GPU：foundationpose310、Python 3.10、torch 2.7.1+cu128、原 cuRobo1。cuRobo2 用于前轮 PushT，本轮未混用两个后端。
- OS/driver：Linux、RTX 5060 Ti 8 GiB、580.173.02。GPU 串行；MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2；单运行 600–900 秒上限。
- ManiSkill：沿用已迁移 native/portable 环境。RealMan SDK / Robot IP：NONE / NOT_RUN。

## 1. 旧仓库 dirty worktree 审计

- 旧仓库 `/home/zhangzhao/Desktop/lerobot`，HEAD `36798efbd12814841607951c9af470b309b34fd3`。
- 继续只读；没有 reset、clean、覆盖、写旧 CUDA cache 或整体复制 untracked。**旧文件修改：NO**。
- 本轮前后 `status --porcelain=v1 -z` SHA256 保持 `15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`。不冒称与重启前历史状态哈希一致。
- 六个入口及其依赖沿用已批准分类/12 个 overlay，不新增 dirty diff 纳入决策。只读使用旧 `jimu_tasks/tag1_standard_three_layer/builder_scene.json`。

## 2. 迁移完整性

- fixed `7aaff9da22486b7d25557b3795dd258f9b65f10d` + audited worktree overlay 不变；snapshot **807 files**。
- `tools/migrate_working_sources.py --target-repo . --verify-only`：PASS。snapshot 文件与外部 cuRobo/模型代码均未修改；原 manifest 不擅自改为全链已资格化。
- 发现 `object_specs.py` 的字面量 `~/Desktop/lerobot/pick_jiaobang/meshs/` 未被原绝对路径迁移覆盖。24 个被引用资产与 snapshot 副本逐个 SHA256 相同；新进程内加载适配只重定向这一明确子目录，不改网格、缩放或碰撞模型，不需要读取旧路径运行。
- `FoundationPose/assets/red_jimu_cube.glb`、tingzi 两个其他资产的旧引用不在这 24 项内，不把该局部修复声称为所有历史应用完全脱离旧目录。Jimu portable 使用自己的已迁移板片资产。

## 3. 代码完整性与 CPU 测试

| 检查 | 最终结果 |
| --- | --- |
| compileall：rm75_app、tools、tests，包含 snapshot | PASS |
| 完整 tests/three_scene | 256 passed，1 warning |
| 完整 tests | 578 passed，1 warning |
| git diff --check | PASS |

比上一提交增加 34 项测试：前端取景、可信 SIM 策略/real 隔离、搬运世界恢复、只读诊断、真实 cuRobo 绑定、MPLib 导入拒绝、资产路径、原生结果判定。无 skipped；warning 是既有 trimesh Scene.dump 弃用提示。

## 4. 前端

遵循 browser skill 先检查应用内浏览器；实际无可用浏览器，按其故障回退规则使用本机 Playwright/Chromium。没有伪装浏览器已连接。

- PickPlace：页面、preview、真实 HTTP 状态查询 PASS。原生输入桥有单测；本轮没有实际 native 提示出现，不能把它记成现场交互通过。
- Jimu：导入原 **21 件（9 固定 + 12 可移动）**完整设计、未知字段/姿态轴 round-trip PASS。首次截图发现屋顶超出画布；只修显示投影自动取景，不改设计坐标。复测 80 个投影顶点均在画布内；编辑一件 x +1 mm 后导出，其他 20 件/元数据不变，随后恢复原设计再 preview。
- PushT：页面、preview、CPU surrogate loop、轮询和 Stop PASS；这里不是 GPU 或物理推动。
- 两次浏览器运行均保留。最终 `browser_full_builder_v2/report.json` **7 项 PASS、page errors=[]**；实际截图已人工查看。

## 5. PickPlace 回归

原 fixed scenes、原 `lazy_place + primary_only`、原候选/seed/阈值保留；只关闭不兼容局部 world-only 过滤的 CUDA graph 加速，不减少计算候选。

| 运行 | 原生结果 | 审计/失败 |
| --- | --- | --- |
| legacy_gluestick / transport v1 | 1/1、final=True | 61 搬运点、6 payload spheres；5 退让点 PASS |
| legacy_gluestick / transport v2（最终代码） | 1/1、final=True | 同样 61 搬运点 + 5 退让点 PASS；MPLib loaded=[] |
| current_table_all / transport v1 | 0/7 | 笔的已附着抬升仍排除了笔筒，策略拒绝 |
| current_table_all / transport v2 | 0/7 | 证实是 insert_vertical 固定排除放置容器，不是单障碍诊断回退造成 |
| current_table_all / transport v3 | 3 件后中止 | 胶棒规划失败后，打印诊断遇到已脱附 payload 再抛异常 |
| current_table_all / transport v4 | 5/7、final=False | 完整保留失败重试；胶棒/薯片罐仍失败，刷子退让 warning 仍在 |

v4 共 9 次 cycle 尝试（5 成功、4 失败），不是 9 个不同物体。笔、绿木块、刷子、胡萝卜、网球进入原成功计数；刷子没有合格的释放后退让，故严格完整任务仍失败。胶棒、薯片罐各在原后续流程中再次尝试，没有删候选跳过失败来报成功。

v4 **6 次 transport 审计 / 373 点**；每次 `world_exempt_links=[]`，负载 1/5/6 spheres 保留，桌面/其他物体/自碰撞继续检查。4 条合格 clearance 路径为 6/3/16/10 点。刷子已有接触深度增加仍被原 target-local 门拒绝，未扩大容差。

修复范围：已附着抬升/搬运时将误排除的笔筒恢复入世界，仅移出由 attached payload 表示的当前源物体；原单障碍免检建议不被执行。只捕获两个 **void 打印诊断**的 typed unsupported 状态，记录 `collision_state_qualified=false` 后返回原失败重试；真实规划/执行错误仍失败关闭。v4 没有 transport policy rejection，`loaded_mplib_modules=[]`，不代表所有对象规划可行。

本轮共 6 次 PickPlace GPU 运行，2 次完整严格通过（同一胶棒场景重复），4 次七物体未通过。最终资产路径版本 `pickplace_transport_gluestick_v2` 通过不重写先前失败记录。原实际 SAM6D 工作台 PickPlace 全链本轮 **NOT_RUN**：现有 PickPlace frozen 是 T_world_obj 格式，未找到同一场景可直接使用的 SAM6D T_cam_obj 结果。没有把别的红积木检测结果或猜测外参转换当成此证据。

## 6. Magnetic 回归

本轮通过 **WorkcellService → 实际 native worker → 原 triangle entry** 跑完整 builder；使用原 synthetic anchors，不是假装 RRTrack 相机实时输入。原 role/parent 顺序、同类源重试、partial/full-open + retreat、候选和 geometry 不重设计。

| 运行 | 结果 |
| --- | --- |
| workcell_jimu_full_v1 | 前 8 件成功，第 9 件失败后的诊断清空世界/移除负载，被 transport 保护拒绝 |
| workcell_jimu_full_v2 | 8/12，屋顶第 9 件原重试全部失败，final=False；旧 worker 却因进程返回 0 记 completed，已识别为误报 |
| workcell_jimu_full_v3 | **8/12、final=False、worker=failed，460.996 s；MPLib loaded=[]** |

- v3 为本轮完整 worker 的有效最新证据：**10 次有路径的 transport 审计 / 466 点**，无 world-link 豁免、保留 attached payload；不是物理动作验证。
- 原第 9 件存在完整世界 start/goal collision 与候选 IK/路径失败；没有移除屋顶邻居、托盘、桌面或降低成功条件来通过。50 次只读碰撞诊断保留状态，未进行空世界/移除负载/逐 link 免检消融。
- v1 未记录最终模块列表；v2 实测仍导入了 MPLib，是旧 Jimu “禁用 MPLib”初始化自身重新 import 的残留，不是调用 MPLib 规划的证据，但不满足 cuRobo-only 依赖要求。v3 已移除该导入及旧 always-clear 碰撞占位 planner，所有兼容查询绑定真实 cuRobo；新增导入保护会对任何 MPLib 请求直接失败。
- 原工作台将正常退出当作成功的问题已修复：按原 cycle/final/clearance 标记判定，保留重试失败，预取 episode 不算件数。物理 `task_success` 仍为 null。
- Four-wall 单独场景本轮 NOT_RUN，前轮 4/4 保留；本轮完整设计前 4 件成功不冒充独立四墙复跑。前轮简化入口 12/12 不等于本轮 21 件 builder 通过。
- **GPU 后 Stop PASS**：使用最终资产路径/碰撞绑定版本，首次 34 点搬运审计、6 payload spheres 后请求取消，最终 worker=cancelled，总运行 20.707 s。约 1.955 ms 是服务取消请求耗时，**不是实机刹停延迟**。没有开爪、退让或运动指令。

## 7. PushT GPU / cuRobo2

本轮代码未改变 PushT 规划；GPU 不重复计数。前轮真实 cuRobo2 完整五段链 **6/9 次运行通过**（含重复/快慢速度），已完成路径的无关障碍注入 **4/4 正确拒绝**。15/5 mm/s 的时间参数化、全点 FK 速度/关节速度/加速度及原 3 mm corridor 已验证，详情见前轮回传。

本轮全量 tests 和浏览器 CPU loop/Stop 复跑通过。固定工具两种原姿态组合仍有失败，真实 workcell 推头/桌面/profile 仍未测量资格化；不能用人工 low-table 仿真正例填成真实测量。

## 8. Camera / tracking

本轮相机、RRTrack、AprilTag、fresh observe → push → fresh observe **NOT_RUN**，没有采集新 RGB-D。前轮单橙色薄片真实录像的 29/30 accepted、坏深度拒绝/恢复记录保留，不声称装配精度、12 个实物身份或真实遮挡已完成。

## 9. RealMan no-motion

SDK connection/preflight、真实关节反馈、控制器 Stop API、夹爪 backend **NOT_RUN**。本轮验证的是仿真 worker 取消，不是控制器刹停。**没有任何机械臂或夹爪运动。**

## 10. Physical motion ladder

低速自由空间、单夹爪、单/多物体 PickPlace、单件/2/4/6+ Jimu、PushT 短推、推后新观测、多步真实闭环：全部 **NOT_RUN**。

## 11. Final summary

- PickPlace：搬运免检遗漏和次生诊断中止已处理；胶棒正例可通过，七物体仍不通过。下一步需处理原胶棒/薯片罐候选可行性与刷子释放/退让几何，不得以免检或容差放宽消除失败。
- Jimu：完整前端、真实 worker、去 MPLib 与 GPU 后 Stop 已有证据；完整 builder 屋顶仍失败，无 Tag 装配精度/多实物资格仍不足，不是只差真机。
- PushT：前轮无运动完整 GPU/速度/碰撞负例完成；本轮前端和全量测试通过。真实几何/观测资格仍缺。
- 改动仅新仓库适配器、前端显示、验证/聚合工具、单测和报告；hash-verified snapshot 与旧仓库不变。
- **未 push。** 先前推送因父提交含原始轨迹/日志及内部路径/硬件标识，被环境审核拒绝。本轮没有得到明确的数据上传补充授权，未重试/改写历史/绕过。远端不能视为更新；本地提交不是 push。
