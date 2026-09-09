# A35328C 回传：IK 契约、原生清桌顺序、Jimu 事务与 PushT 物理闭环

日期：2026-09-09。按完整 `CHATGPT_REVIEW_A35328C_IK_SIM_JIMU_20260909.md` 执行，已读到 `A35328C-REVIEW-END`。

## 1. 结论与边界

- **Jimu：103 mm 显式试验完成 12/12 原主循环，12/12 释放／返程独立审计通过。** 没有漏掉上抬回撤；先前碰撞候选仍拒绝，修正状态事务后原重试找到后续可行候选。100 mm 默认没改。
- **PickPlace：两个原生同场景顺序均为 5/7，全部成功顺序仍是 0/2。** Tennis 放到原规则 `slot_4` 时成功；不能据此宣称独立首槽的 0/96 IK 已修复。gluestick、hongshupian 仍失败。
- **PushT：已接通真实 service → worker → cuRobo2 重规划 → ManiSkill/PhysX 动态 T → 再观测，以及完整 RM75 articulated PD 分支。** 不再只是读取一次保存轨迹；但正式全闭合工具验收未通过，不能写“三条完成”。
- 没有启动机械臂、SDK、真夹爪、相机或 RRTrack。没有降低碰撞检查、球数量、候选、成功条件；没有修改旧仓库或 vendor 源码。物理材料未标定，所有 hardware qualification 仍 false。

本轮先从 `a35328c` fast-forward pull 到 `15f1fbf`；实验基于该提交加本轮工作树差异。最终代码逐文件 SHA、所有实际运行（含失败）的结果 SHA、输入 SHA 见 [有界 JSON](../benchmarks/unified_scenarios/review_a35328c_ik_sim_jimu_20260909_summary.json)。早期实验没有逐文件冻结适配器源码，不能把最终文件哈希冒充所有早期运行的源码哈希；后续原生顺序试验记录了启动时适配器哈希，物理 runner 记录了其主要实现文件哈希。

## 2. 已确认缺陷与实际修改

1. **阶段状态恢复不完整。** 原 GPU RLock/world/cache 恢复保留，新增 planner-local 事务参与者，将 transport 的 `excluded_sources` 和最近成功 refresh 的上下文一起捕获、恢复；包括嵌套、普通失败、BaseException、审计输出异常。没有新增独立全局状态字典。
2. **合法缓存误判。** 当最新成功 refresh 明确 `include_active_object=False`，且当前世界对象集合一致时，允许辨认“当前源物体、实际不在世界中的陈旧缓存行”。不全量 enable；当前必需桌面、邻物、仍在世界中的源物体、错误源身份仍立即拒绝。
3. **IK 诊断行错配风险。** Tennis 显式 SIM review 适配器使 single-IK 的 debug 与实际选中成功行一致，并拒绝缺失实际 EE link 的静默 TCP fallback。此补丁与新增参考／continuation／yaw 试验均未悄悄推广为默认候选策略；当前 Tennis batch 每目标一行，不能把 single debug 缺陷称作首槽根因。
4. **PushT 周期关节解没有统一表示。** 新 stage-state IK 入口复用已有“最近合法 2π 等价角”工具，仅适用于关节限位跨度允许整周的关节。本机改变的是 joint_7；保留全部成功行，仍逐边验证 FK、直线、世界和自碰撞。它消除部分无意义整周绕行，但没有单独修好第三轮回撤。
5. **物理时间／执行链缺口。** 新物理 observer/executor 读取实际 T 和 q/qvel，每轮重新调用完整五阶段规划，按时间执行 PD target；不调用真机 `execute_push`，不把预测当观测。成功确认推进物理步，检查原 6 mm/0.10 rad、3 次/.3 s 窗口，并额外检查速度稳定。
6. **仿真取消丢失视频／结果。** 首次 tool-only Stop 的组 SIGTERM 杀掉了编码器和 worker，保留为失败证据。仅对物理 SIM 改用 STOP 协作退出＋5 s 强制回收 watchdog；工具与整臂取消复测都正常封口。真机 Stop 分支未改。
7. **完整机械臂对 T 的接触范围。** 最终安全复查补齐全臂每个 link 对 T 的配置允许范围／阶段检查，不能只监测工具和静态桌面。最后 .91 复跑已包含该检查，仍在 approach 拒绝；之前 .6 试验没有这项完整 PhysX 连杆指标，不事后扩大其资格。tool-only 是单一刚体聚合工具，亦不冒充全臂接触资格。

## 3. Tennis：先固定目标对照，再用原顺序验证

固定输入 `assets/test_scenes/current_table.json`，SHA256 `e5dd3154bdb6b509791861478170ffc5f0b6e2c8f08ea89b9b7a5111ee2fd908`。9 个世界对象全部保留，原 cuRobo1，原 32 seeds，原 16 抓取关系及六个轴向角。原端面完全水平约束保留。

### 3.1 契约证据

- 当前有效原 q 的 single / batch / 前台包装 FK→IK 回环 **3/3** 通过。最大位置误差约 `1.47e-7 m`，SO(3) 误差约 `3.74e-7 rad`。
- 实际 EE=`gripper_tcp`，TCP→EE 刚性变换为单位阵；base/world 转换矩阵误差最大 `6.19e-8`。七关节弧度、四元数 wxyz 的顺序已记录。本次没触发缺 link fallback。
- 全关系球心闭合残差最大约 **0.009513 mm**。未发现足以解释首槽失败的中心轴简化误差，因此没有移动目标。
- 原显式 grasp seed 与 retract reference 的含义按本机 cuRobo1 源码及 [官方 IKSolver 文档](https://curobo.org/_api/curobo.wrap.reacher.ik_solver.html) 核对；参考 q 不是执行必须经过的 waypoint。

### 3.2 独立首槽试验（全部失败，不能证明不可达）

| 试验 | 最后批次关系 / 端点 | 合格端点 | 最后批次求解秒 | worker 总秒 |
|---|---:|---:|---:|---:|
| 原 grasp 参考 | 96 / 192 | 0 | 1.004 | 35.282 |
| 标准参考，保留 grasp seed、32 起点 | 96 / 192 | 0 | 1.160 | 43.008 |
| 附近同姿态种子搜索＋continuation | 96 / 192 | 0 | 1.627 | 37.203 |
| 保留六角，另加六个水平 yaw 中点 | 192 / 384 | 0 | 1.374 | 36.587 |

每次还有前置 16 关系 / 32 端点批次，均失败，完整分母在 JSON。Continuation 两批分别搜索 8、48 个 helper goals，每个仍 32 seeds，合格 helper 都是 0；所以实际上没有成功端点供连续推进，不能声称 continuation 已救回问题。没在 raw success 全零时增加返回 K 来冒充解决。新增 yaw 为 ±22.5°、±67.5°、±135°，没有新增 pitch/roll/目标平移。所有状态不变断言通过。

失败原始行保存位置误差、完整 FK SO(3)、world/self、关节限位和世界／负载／夹爪指纹；例如基线首 hover 约 7.74 mm、0.251 rad，当前配置 collision-valid。有限预算不能排除其他 IK 分支。

上述 batch 秒数不含候选建造、修复 helper 全部开销、MotionGen 和执行。不能把它们称作“完整任务 <5 秒”；也不能把 worker 总时间称作筛选耗时。首槽未进入合格放置端点后的完整 MotionGen 链。

### 3.3 两个原生同场景顺序

复用原 `--cycle-object-names --cycle-order-targets --repeat-count`，没有重写规划器；新增验收要求同一 demo/scene 身份、所有请求物体真正前台完成、带载世界覆盖及逐条 clearance。已放置物体仍留在原场景缓存和碰撞世界；不是独立 reset 后拼接成功数。

| 原请求顺序 | 实际成功物体 | 结果 | 总秒 |
|---|---|---|---:|
| shuazi → gluestick → bi → lvmukuai → carriot → hongshupian → tennis | shuazi / bi / lvmukuai / carriot / tennis | 5/7；失败源后续原重试仍失败 | 252.772 |
| shuazi → bi → lvmukuai → carriot → gluestick → hongshupian → tennis | 同上 | 5/7；失败源后续原重试仍失败 | 253.377 |

原程序会暂缓失败源再尝试剩余源，实际前台顺序和全部重试单独保存在 JSON。后台预取不计完成。两次各有 5 条独立 clearance 审计通过，但整个七物体任务均 FAIL。

第二次 Tennis 的首批 hover/release **32/32 端点通过**，原四图交集 10/16 关系，求解约 .555 s；使用原允许 `slot_4`。这比首槽更靠近机器人，且世界／占用状态已改变，属于新上下文，不是固定首槽的 seed 对照。原源码中打印的 `place_plan.tcp_verticality` 并不等同最终实际 TCP：新增视频姿态记录显示 hover→release 开始下向偏差约 `8.81e-6 rad`、最终 release `3.03e-6 rad`。

**还不能声称整个下降段持续水平：** 同一原运动窗口中记录到最大约 **0.778 rad** 的瞬时下向偏差。端点约束正确不等于整条执行曲线保持水平；此事实保留，列为下一轮路径审阅项，不用原 native True 掩盖它。

剩余阻塞：

- gluestick：第二个顺序实际 grasp 已有 **4/13** 合格，区别于此前 0/13；但完整带载抬升／搬运链仍失败。不是只要先搬刷子就全部解决。
- hongshupian：原带载 TCP-up lift 的名义目标，payload/base_link 两球对几何重叠约 **28.975 mm / 8.171 mm**，这些 pair 没有被忽略；原失败 IK 行独立检查约 28.962 mm，诊断 state unchanged。它不是简单的 seed 数问题，不能靠改顺序或豁免底座来记成功。

## 4. Jimu：回撤已做，状态修复使原重试能继续

同一原 builder、fixed scene、启动文档、已批准三角 GLB；100 mm 默认不动，只用用户已批准的 `--jimu-lift-plus-3mm`（103 mm）。没有新增退让算法／继续扫高度。

| 试验 | 主循环 | 释放审计 | 事务 adapter 恢复断言 | 总秒 |
|---|---:|---:|---:|---:|
| 仅补元数据恢复和 trace | 8/12 | 8/8 | 16/16 | 310.935 |
| 再正确区分 absent source cache | **12/12** | **12/12** | **43/43** | **513.173** |

第一轮 trace 中，真正必须存在的世界对象被禁用集合为空；被列为 disabled 的 active target、right roof、virtual table 缓存均已不在当时当前世界。最新 refresh 明确不包含活动源。仅恢复 `excluded_sources` 不是唯一修复，陈旧缓存分类也必须正确。

首个 right roof near_up 候选确实有 clearance/up 规划，返程起点仍碰**已放置的右屋顶自身**，不是料盘里的其他积木。只读原 GLB 三角面最近距离检查：gripper_base_link、Left_2、Right_2 球与右屋顶表面分别重叠 **4.719、1.594、1.220 mm**。这是拟议释放后预验证，原日志 `released=false`，不能自动判成实际释放状态错误。负表面距离减半径证明相交；正值不作为全场景无碰撞证明。

最终 23 条 transport 审计含预取，**不是 23 个任务**；12 个实际前台循环才是完成分母。没有新增搬运／返程 world exemption，12 条 release audit 的 `return_world_exempt_links=[]`、`self_collision_input_modified=false`、`state_unchanged=true`。roof IK 覆盖四个 roof 的三种阶段，12 个 batch 全是 prefetch；不冒充 foreground IK 覆盖。

原输入回读哈希完全一致：builder `6478bafdf016817e7d118269da9199e6d4fe8c06570cbedfdfb03c1127ca6884`；fixed `c49b1f1663cdd33865dd89bd732be0a6df5e7c9af00b19e5fd1b04531f451e06`；manifest `811be89115ec9982ac425f799dedc0d08ab55ada996c42d0e524a54550a3ab2d`。此处 PASS 是原仿真命令和路径审计，不是真实磁吸连接或相机跟踪通过。

## 5. PushT：物理闭环与完整机械臂的实际范围

### 5.1 实现与模型

新增 service/worker 显式 `surrogate / tool_only_physics / full_arm_physics`。物理环境用现有 foundationpose310，持续 GPU planner 子进程用现有 curobo2 环境；未安装或改动外部环境。每轮用实际 q 与动态 T pose 新建计划并核对 observation 身份，时间戳采用 `1 + 已推进物理秒`，GPU 规划墙钟另外记录。

完整机械臂加载原 URDF，只有 episode 初始化 set_qpos；执行只发关节 drive target，读取实际 q/qvel。没有叠加 kinematic 工具；没有复制旧 agent 的 gripper/link6/link7 全组自碰撞免检。原 38 球仍用于 GPU 工具模型，整臂 PhysX 用原 URDF mesh。所有 target、桌面、非目标几何保留。

URDF SHA256 `e2f7d65c81b9f292f5bb8bf0a0f93df861e9a331e61276424ce53b5bfb899fea`。驱动来自固定快照 `Beta_demo-codex-v0.9/jimu_portable_repro/maniskill_env/mani_skill/agents/robots/realman/realman_with_gripper.py`：臂 stiffness 1000、damping 100、force limit 20、friction .1；夹爪 1000/100/5/1；四连杆 anchors 原值保留。其他旧副本夹爪 force limit 有 10 的版本，本轮没有混称同一配置。T 材料 friction .3、density 1000 未标定。

### 5.2 六案例冻结矩阵：沿用 0.6 rad 模型（并非机械全闭合极限）

| case | tool-only 物理 | full-arm 物理（未补偿基线） |
|---|---|---|
| translation | 执行两推；第三次新规划 retreat 拒绝 | 执行两推；第三推 contact 碰桌面停止 |
| rotation | **到达，1 推，3 次稳定确认** | 第一推 contact 碰桌面停止 |
| mixed | 执行两推；第三次 retreat 拒绝 | 第一推 contact 碰桌面停止 |
| neighbor | 执行两推；第三次 retreat 拒绝 | 执行两推；第三推 contact 碰桌面停止 |
| infeasible | 下降几何映射正确拒绝，无推移 | 同样正确拒绝，无推移 |
| cancel | 首次强杀无完整结尾；修复后复测正常取消、视频封口 | 正常取消、视频封口，无推移 |

正常目标到达：tool-only **1/4**，full-arm **0/4**；修复后两分支各 **2/2** 拒绝／取消行为符合预期，不能把它们算到达成功。所有初次失败和修复复测均保留在 JSON，视频存在不等于通过。

纯旋转最终 T=`[.354623, -.178708, .148521]`，原目标 `[.35,-.18,.15]`，位置误差约 4.80 mm、yaw 误差约 .00148 rad，稳定观察 3 次。其他正常案例未到达即失败，没有用预测位置替代实际状态。

未补偿整臂的第一拒绝接触对是 **gripper_Right_Support_Link / simulation_table**，发生在 contact，不是已允许的 T 接触。translation 最大实际 TCP 跟踪偏差 **11.878 mm**，最大关节偏差 **.023749 rad**。同时原固定 spacer/base mesh 有 PhysX 自接触报告，未隐去，不能据此宣布模型自碰撞资格已完成。

### 5.3 独立修复对照和新发现

- 周期关节修复后，tool translation 仍在第三次 retreat 拒绝，最近分支直线偏差 **6.550 mm > 原 3 mm**，不是放宽后通过。
- 将 GPU IK 位置收敛容差从默认 5 mm **收紧到 1 mm** 的独立对照，仍停在相同 6.550 mm 路径门；精度参数的表面不一致不是已证明的唯一根因。
- 原 ManiSkill `BaseAgent.set_control_mode` 默认 `balance_passive_force=True`；其实现是仅对机械臂 links 关闭重力，模拟理想补偿。手动加载 articulation 时未自动取得这项行为。独立 `--gravity-compensation original-agent` 恢复同一原约定，**T 仍有重力**，没有修改碰撞、质量、惯量或 PD 常数。TCP 最大偏差降到 **1.344 mm**、关节 .007354 rad，四次实际推移无桌面接触；第五次新规划仍在 retreat 以 **6.293 mm >3 mm** 拒绝，最终 T x=.367516，未到 .38 目标。不能称完整成功或真实重力补偿控制器已验证。
- **闭合标签不能冒充全闭合。** 旧 config 的 `closed=.6 rad` 在原 URDF 中 pad 参考点距离仍约 44.706 mm；原关节上限 .91 rad 时约 10.823 mm。这是参考点距，不是标定内侧缝隙。用户已选全闭合，新增显式 .91 对照同时作用于 GPU 38 球、关节锁和物理 PD；并保留上述原理想补偿。它在第一个 **approach MotionGen** 被拒绝，0 推移，不能把 .6 的六案例结果用作 .91 的资格。全闭合时还记录到左右 Support mesh 接触，未新增 self/world 豁免。

最终加上全臂目标接触守卫后再次复跑 .91，结果同为 approach 拒绝、0 推移，61 个物理控制步，未观测到非许可 T 接触。至此共有 **20 次 PushT worker 尝试**，包含初始化 JSON 类型修复、取消修复、精度、补偿及两次全闭合对照；不是 20 个独立标准场景。硬件资格始终 false。SAPIEN 对原 STL 的 multiple-convex loading 有警告，也保留为模型验证限制，未据此删掉碰撞几何。

## 6. 测试、前端与视频

- 全量离线 `python3 -m pytest -q tests`：**1023 passed，1 warning，35.09 s**。完整 `tests/three_scene`：**701 passed，1 warning，20.49 s**。warning 为原 trimesh 弃用提示，非跳过测试。最后一次日志与哈希在 JSON。
- compileall、`node --check rm75_app/web/static/workcell/app.js`、`git diff --check` 通过。最终 `verify_snapshot` 校验 **809 文件**，fixed `7aaff9d` + audited overlay 的源码字节未变。
- 两个物理后端均通过实际 WSGI 提交到 preview worker；所有物理 case 都实际 service→worker 提交，不是独立求解脚本。前端新增明确的后端选择，保留 preview/Stop、关闭 real 资格。
- **浏览器 UI 点击验收未完成。** 按 Browser 技能连接、故障排查及一次可用浏览器枚举后返回空列表；没有用源码检查冒充点击验收。没有修改浏览器配置或用户账户。
- 曾误用无目录的 `pytest -q` 扫整个工作区，未完成且未计入分母；已停止该本次进程，改跑完整 `tests`。一次临时命令引用不存在测试文件、首次 phase trace 两个旧测试取“最后事件”失败，都保留在本地日志，修复后全部重跑。

可直接查看的本地视频（不录桌面；大视频不塞入 Git）：

- `runtime_data/three_scene/review_a35328c_20260909/videos/jimu_four_roofs_success.mp4`：116.6 s，四块屋顶原生成功窗口。
- `runtime_data/three_scene/review_a35328c_20260909/videos/pickplace_tennis_slot4_success.mp4`：58.5 s，第二个顺序的 Tennis。注意上文端点与中途姿态的区别。
- `runtime_data/three_scene/review_a35328c_20260909/videos/pusht_full_arm_first_failure.mp4`：原未补偿 0.6 模型第三轮失败末段，非全闭合资格视频。
- 两个 PickPlace 顺序及每个物理 worker 的完整视频在对应 `result.json` 所引用 job 的 `sim_video/` 或 `physics/`；最后两个源失败未执行的路径不会假装动画执行。

## 7. 最小复现与下一轮优先项

所有命令在项目根目录执行，串行 GPU，推荐原有 `MemoryMax=9G / MemorySwapMax=512M`，不使用真机参数。

```bash
# cuRobo1 / 原 ManiSkill Python
/home/zhangzhao/anaconda3/envs/foundationpose310/bin/python tools/run_workcell_native_validation.py \
  --task pickplace --object-name shuazi \
  --cycle-order shuazi bi lvmukuai carriot gluestick hongshupian tennis \
  --fixed-world assets/test_scenes/current_table.json --tennis-ik-review baseline \
  --audit-current-table-failures --record-sim-video \
  --extensions runtime_data/curobo_native_extensions --output <新目录> --timeout-s 900

# 实际 service/worker，GPU planner 与 PhysX 使用已有两个环境
python3 tools/run_pusht_physics_validation.py --backend full_arm_physics \
  --case translation --gravity-compensation original-agent \
  --closed-gripper-joint-position .91 --output <新目录>
```

下一轮仍不动真机：优先检查全闭合 .91 profile 的 approach 拒绝、固定连杆／左右 Support 实际接触及后续回撤分支；不要拿 .6 模型代替用户选择。PickPlace 保留已找到的原生 5 物体顺序和 slot_4 Tennis 证据，继续解决 gluestick 带载链、hongshupian payload/base 几何，以及 Tennis 下降窗口中途姿态。Jimu 不再盲目加高度；保留当前 12/12 结果，待独立现场定位／物理连接验收授权。浏览器可用后补真实 UI 提交和 Stop。

回传结束标记：CODEX-A35328C-RESULTS-END
