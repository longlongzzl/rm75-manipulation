# SWM 主架构：关键阶段同步、原子技能、实测驱动的物理辨识

基线：`44fe0434409cdac923afd9af271f8399378ed536`；协作分支 `chatgpt/three-scene-software-closeout`。

## 0. 本次需求成为总架构，不是再加三个各自独立的回调

统一方向为：**SAM3D 建模 → 注册 SWM → FoundationPose/RRTrack 在关键阶段同步 → LLM 分解并生成受限技能代码 → 仿真并行规划 → 原子技能执行 → 实测验证 → 更新位姿和物理参数分布 → 从新状态继续规划。**

“关键点同步”在此指抓取前、抓取后、放置前后等**执行关键阶段的检查点**，不是另起一套图像关键点检测器，也不是要求每个控制周期都运行视觉模型。物体的几何来源、真实位姿来源、仿真预测与物理参数置信度必须分开。

**本轮边界必须先读：**

- 已实现新的 SWM 核心、完整检查点事务、五类技能协议及通用执行循环、功能位姿工具、LLM 技能代码编译器、并行规划调度、物理参数采样/重放/匹配/重采样、检查点/轨迹关联、快照持久化和现有 worker 的显式入口。
- **尚未把三个原生大流程全部拆成并注册为 NativeAtomicBinding。** 当前 `register_runtime_factory` 注册表默认为空。因此不能说原有所有 PickPlace/Jimu/Pusht 按钮已经自动具备新同步机制。
- 启用 `swm.enabled=true` 但没有原生原子适配器时，返回 `SWM_ATOMIC_ADAPTER_REQUIRED`；不会静默回退到旧流程然后标记“SWM 验证成功”。不启用时保持旧控制台与已验证流程，不覆盖 vendor。
- 原子协议有 grasp/place/push/pull/rotate，不等于五种硬件动作均已就绪。pull/rotate 需要实际把手/作用轴/运动约束和对应求解器；未安装必须明确拒绝。
- 本轮没有相机、机械臂、夹爪、商业 LLM、SAM3D 权重、FoundationPose 推理或 PhysX 重放运行证据。原生适配与 GPU 实连仍须本机完成，不能把本轮核心测试代替它们。

这是一版可验证的软件内核与迁移入口，不是一键启用全部真机技能的发布。

## 1. 三个层次及“事实从哪里来”

### 1.1 SWM：物体与世界状态

`rm75_app/swm/scene.py` 的 `SceneWorldModel` 存储：

- 资产：asset_id、来源（sam3d/legacy/cad）、三角网格及碰撞模型的路径与 SHA256、米制尺度及其测量依据、物体系中的功能位姿。
- 实例：instance_id、人类可读名称、asset_id、fixed 属性。两块同名积木必须有不同 instance_id，不能只靠名字覆盖。
- 当前实测状态：`T_world_object`、采集时间、帧序号、传感器会话、校准版本、接受状态、位姿不确定度，以及机器人实测关节/TCP/持物状态。
- 物理信念：对象-支撑面的参数候选、匹配残差和权重、证据动作摘要、版本；不是把一个猜测密度直接当真实质量。
- scene revision / physics revision / snapshot_id。任务规划绑定快照，世界改变时旧计划失效。

**只允许实测检查点修改“当前实测位姿”。** LLM 的目标位置、仿真预测终点、规划器返回的末端位置不能写入该字段。

`SnapshotStore` 原子保存快照。重启可以保留几何与已记录参数信念，但实测位姿和持物状态必须重新采集；不会重启后接着执行一条旧运动指令。历史快照只作为证据。

### 1.2 技能层：统一执行与验证

每个原子技能包含 `plan → audit → execute → verify`。原生规划器和共享 RealMan executor 通过 `NativeAtomicBinding` 接入，不再创建第二套机器人驱动。

**一次完整 pick-and-place 不能简单换名字叫 grasp。** 抓取技能结束于提起并确认持物；放置技能结束于释放、安全退让和重新观测。Jimu 是受装配顺序与结构规则约束的一系列 grasp/place，不是单独绕过同步的代码分支。

### 1.3 上层：任务分解与技能程序

`SWMAgentPlanner` 接收任务、当前 SWM、可用技能/功能位姿、经过可信任务编译器产生的目标集合，要求大模型输出 decomposition + code。

```python
skills.grasp("block_01", functional_pose="side_grasp")
skills.place("block_01", target="slot_A")
skills.push("T_01", target="next_push_subgoal")
```

代码由 AST 解析为类型化 SkillRequest，**不使用 exec/eval**。允许白名单的技能调用和字面量 ID；拒绝 imports、任意属性、循环、文件/网络访问和直接 SDK 调用。上层“写代码”意味着组合工具，不意味着模型获得操作系统和机械臂底层控制权。

place 前必须已有同一物体的 grasp；手里有别的物体不能再抓或直接推；失败的步骤会阻止其依赖步骤执行。任意新高层目标仍须经原对象放置/装配编译器形成，不能让 LLM 从字符串直接更改碰撞配置或坐标系。

## 2. 视觉建模与关键阶段同步

### 2.1 SAM3D 负责几何，不负责凭空给出可信物理参数

`SAM3DAssetBuilder` 调用官方 inference(image, mask, seed, pointmap=...) 接口。输出通过安装版本的 mesh_exporter 显式取出三角网格；不能把 Gaussian 点云 PLY 当作可碰撞封闭网格。

米制尺度通过 RGB-D 点图或实际尺寸校验得到 `model_to_metric`，模型、碰撞代理、FoundationPose 物体系、功能位姿必须共同使用这个变换。只有严格刚体+正均匀尺度变换通过；网格与凸碰撞代理校验后才注册。碰撞代理体积不是实心材料的实际体积，空心容器拟合出来的是等效密度，不声称是真实材质密度。

已有资产可以 source=legacy 注册，不必为每次执行重新跑 SAM3D。模型内容/尺度改变，应重新绑定资产和定位，不能继承旧网格的位姿与参数拟合。

### 2.2 复用共享 RRTrack / FoundationPose

`RRTrackCheckpointSource` 消费现有 `RRTrackInstanceSample`，对齐 `instance_id / asset_name / mesh_sha256`，使用原采集时间和帧序号。输出丢失、恢复中或未被接受时，不能拿最后一次矩阵继续执行。

`FoundationPoseCheckpointEstimator` 只是对已经加载的 estimator 做检查点调用：短间隔可 track_one；首次、间隔较大或此前拒绝则用新掩码 register，并通过独立的渲染/深度一致性检查。它不另建实时 tracker，也不默认把“返回矩阵”当正确定位。

### 2.3 每项任务统一的检查点

| 任务 | 必需边界 | 实际验证内容 |
|---|---|---|
| PickPlace | before_grasp → after_grasp → before_place → after_place | 抓前对象/障碍物是否移动；抓后是否真的持有目标；放后实际落点及是否释放 |
| Jimu | 同上，对每块重复；最后 verify_structure | 原底板/已放置块位置、当前块身份、装配关系，不能只更新待抓件 |
| PushT | before_push → after_push，每个推段重复 | 新 T 位姿，实际推进/转角和目标偏差；不是拿预测位移更新状态 |
| Pull / Rotate | before / after 对应技能 | 把手或作用轴约束、实际物体/关节状态，必须有安装的约束感知原子适配器 |

规划可能很慢，所以还增加 **pre_execute_skill**：求解后再次观测，检查所有规划相关对象和机器人起点有没有改变。超过允许变化就丢弃旧计划重新求解；审计后状态变动或观测过期也拒绝执行。

第一版要求一个工作区域内注册对象的完整、时间一致的批次（包含底板、障碍物和已放置件）。多模型推理可共享同一次图像采集，不能把顺序读出的历史状态重新盖时间戳。固定件可由本次可观测的校准锚点推导，但必须保留真实采集来源；长期缓存固定件不能冒充新观测。遮挡/同类实例无法辨认时停止后续运动并请求重新观测，不能默认未变。

位姿写入同时通过 `TransactionalSceneMirror` 更新规划碰撞世界和仿真世界。拿在手中的物体按实测 `T_tcp_object` 更新 attachment，不另放一份自由刚体副本；释放后才重新成为独立世界物体。任何一端未确认同一 snapshot_id，SWM 无效，禁止继续。

## 3. 如何判定变动，如何重试

平移误差用米，方向误差用 SO(3) 旋转角，而不是直接相减欧拉角。

把三件事分开记录：

1. **场景漂移**：计划使用的实测状态与执行前新实测是否一致。变了则重同步/重规划，不发送旧动作。
2. **模型预测误差**：仿真预计终点与动作后实测是否一致。它不必然证明“有人挪动”，也可能是动力学、定位或接触误差。
3. **技能目标是否实现**：动作后实测是否满足目标及持物状态。发出指令成功不是该指标。

抓取后持物关系与预测不同，但确实拿稳时，应使用新的 attachment 规划放置，不要空耗地再次抓取。物体已达到任务目标，即使预测偏了，也不应为了追求仿真一致而再推一次。

放置后位置错误且夹爪已空，不允许直接重放同一 place；必须在上层生成重新抓取/调整的恢复程序。未知 SDK/碰撞/持物失败不能无限自动重试。确定无持物且动作已结束的 grasp/push 可以在明确预算内从新观测重试。原默认最多2次重规划，不承诺一定成功。

对球体等对称物体，应由原任务的对称性/功能约束适配器决定可观测且与任务相关的方向误差；**当前新内核默认完整 SO(3)**，未自动添加对称性豁免，不能因此假称球体方向验证已经可用。拉抽屉、转瓶盖等关节对象同样需要关节模型，不能用自由刚体重放替代铰链/滑轨约束。

## 4. PushT：参数辨识与动作规划是两个不同的并行过程

### 4.1 现状更正

基线 `pusht/response.py::ResponseEstimator` 拟合的是接触点对应的前进、侧滑和转角响应增益。`friction_scales` 是近似预测的不确定性系数，不是给 PhysX 写入若干组真实摩擦/密度。

因此本轮新增 `identification.py / physics_replay.py`，而不是把 response_fits 改名成 physical_parameters。旧算法保留作基线，不宣称现有部署已经具有真实 μ/ρ 识别能力。

### 4.2 辨识过去：同一实际动作，不同参数

参数候选为 θ=(静摩擦, 动摩擦, 密度)，给定实测起始 SWM s、真实执行反馈 u，以及动作后物体观测 y：

```text
相同的 s、相同的实测 TCP 时间序列 u
        ↓
[θ1 的独立物理重放, θ2 的独立物理重放, …, θK 的独立物理重放]
        ↓
在真实物体观测的时间点比较位置/旋转残差
        ↓
候选权重 / 最佳拟合候选 / 是否有区分度
```

**不能在每个 θ 下先重新优化另一条动作，再比较哪个终点最接近真机。** 那会把动作差异和物理参数差异混在一起。重放输入使用实际 TCP/关节反馈，不是仅使用给机器人发送的命令；工具跟随偏差不能被错误拟合为摩擦。

`bind_measured_transition` 在动作后视觉检查点已取得之后拼接证据，保证同一个实际动作 ID、模型、校准和传感器会话。工具反馈记录器必须覆盖最终视觉帧曝光时刻；记录不足时拒绝外推，不能猜一个“末端保持不动”。非实时视觉只有前后两帧时标为 endpoint_only；有更多独立接受帧时才标 sampled_trajectory，不虚构中间物体运动。

### 4.3 更新信念后，重新规划未来

匹配代价是按不确定度归一化的平移/旋转残差。保存全部候选分母，失败的物理重放也是结果。候选几乎不可区分时不收缩分布；有区分度时更新权重，保留探索质量，再围绕后验采样新候选。

当前采样默认16个假设，有界2～128，默认只开1个物理进程（可配置并发上限4）；尊重本机此前死机/内存限制，先串行校验后再启用并发。不要在有限显存上直接同时启动16份完整机械臂规划器。

**密度保留宽分布。** 慢速/准静态推送中，前后物体位姿可能对密度不敏感。代码会报告当前候选中最接近的密度，但不会宣称已识别唯一真值或自动让密度分布坍缩；可通过更多速度/加速段、不同方向和更丰富观测单独验证参数可辨识性。用户手动挪 T、支撑面变化、持物变化、定位失效的区间不拿来拟合。

新物理适配器使用 SAPIEN/ManiSkill 的独立 CPU PhysX 重放，设置材料与构建时密度，按照实测 TCP 驱动工具，目标物体只由动力学推进。**本轮该 native 适配器尚未在实际 PhysX 环境运行**；测试中的小数值模型只验证调度/打分，不代替原生物理。

当前重放实现的摩擦是共享有效 object-support-tool 系数，工具为实测轨迹约束的运动学刚体，**不是完整机械臂安全验证，也不是精确分离了每一对接触的摩擦**。真实动作依然要经过已有整臂规划、碰撞和执行保护。

### 4.4 规划未来：在新状态和参数分布下找动作

`ParallelHypothesisPlanner` 在每个候选世界内生成可行的原子动作，再对同一候选动作做跨参数验证；保留不确定性下仍可行的动作，以最坏情形代价排序。每个求解上下文私有、无真机执行器。

更新 θ 后不复用过去的那条轨迹。下一次仍从 **最新实测 SWM** 生成新动作，执行一段，重新定位，再更新。这才是“随机化 → 与真实对齐 → 再随机化”的闭环。

PushT 的最终目标属于复合任务；每个长推段属于 push 原子。原来的持续控制器应在每段边界调用新 runtime，并根据段后实测决定下一段局部目标，而不是把一个部分推进误算成已到最终目标，也不因一次原子步未到最终目标就结束所有推移。

## 5. 已有代码如何迁入，不复制第二套模型/驱动

| 新模块 | 实际内容 | 原生接线位置 |
|---|---|---|
| scene.py / storage.py | 模型实例、时间/置信度、事务同步、持久化 | 统一 SWM；原 `TaskSceneState` 通过 adapters 转换 |
| perception.py | SAM3D 资产入口、共享 RRTrack/FP 检查点 | 原 `runtime/rrtrack_pose_tracking.py` 与 `scenarios/rrtrack_bridge.py` |
| skills.py / adapters.py | 五种技能合同、运行/校验/恢复、原 solver/executor 回调 | 真正的抓取结束/放置结束边界，不包装整个旧 episode 冒充原子 |
| identification.py / measurements.py / physics_replay.py | 同动作多参数重放、稀疏观测对齐、参数分布更新 | 实际反馈记录器 + 原 PhysX 工具/模型 |
| planning.py | 私有上下文并行求解与交叉验证 | 现有 cuRobo/姿态候选工具；硬件执行仍串行 |
| program.py | LLM decomposition/code → 受限 AST → 原子序列 | 现有 LLM 完成接口；不让模型生成底层调用 |
| integration.py | 默认兼容、显式新路径、能力状态与拒绝静默回退 | 当前 workcell worker，原授权/隔离/租约之后 |

`SWMTools.get_object_pose` 返回对象位姿、来源时间和快照；`get_functional_poses` 将已注册物体系功能位姿变换到当前世界；`plan_to_functional_pose` 委托已有路径规划器。**当前不是从任意网格自动发明全部抓/推/拉功能位姿**，应把已有候选生成器接入并保存其来源，或注册经过验证的模型局部位姿。

`integration.contract_status(profile)` 提供只读架构/迁移状态，供后续控制台接入。它不采集相机、不执行动作；本轮没有修改已经验收的控制台，也没有新增 SWM 可视化面板。原控制台的 real 禁止条件、场景证明和停止逻辑都没有解除。

## 6. Codex 本地实施顺序（不是只跑测试就宣布完成）

### A. 先做统一检查点源与仿真镜像，禁止实机动作

从现有资产/场景注册3～5个实例，先用记录回放或仿真真值接入完整 checkpoint。`RRTrackCheckpointSource.capture_samples` 必须调用同一个已有定位服务，不能建立第二个 tracker。用本机只读/离线图像验证 capture time / identity / lost 拒绝；相机 live 运行需要用户单独允许，绝不连机器人代替数据源。

`TransactionalSceneMirror` 的两个 sink 要真正更新原 simulator/规划世界的对象、已放置件、障碍物与 attachment，并返回实际应用快照ID，不能无操作返回传入ID。同步处于闲置边界，不瞬移正在执行中的机器人。物理信念更新后下一次规划前同步最新版本。

### B. 从已成功的小例子提取原子边界

PickPlace 先“笔”：提取 grasp 和 place 的原始求解/执行阶段，NativeAtomicBinding 的 execute 必须返回共享执行器的实测 receipt。原抓取/放置/退让逻辑与碰撞配置不重写。先在仿真验证 before_grasp / after_grasp / before_place / after_place，再按已有顺序扩展。

Jimu 先已通过的2～3件设计：直接复用现有生成 proof、原 builder→world 变换和实际料槽身份，将已编译的世界目标交给 compose_task。每块的已放置位置使用检查点观测更新，不能写仿真目标冒充观测。最终任务验证交给独立结构关系检查，不因每个动作下发成功就说磁吸连接可靠。

PushT 将原每段长推作为 push 原子：before/after观测统一写 SWM；保留已有局部搜索、停止/等待和几何边界。对齐 actual_action_id 与经过执行器反馈的 TCP 数据。记录器延续到 after_push 帧采集完毕，再通过 bind_measured_transition 构造参数实验。

**只有这些 native bindings 真正完成后，才在 worker 子进程内安装 register_runtime_factory。父进程注册不会自动跨 subprocess 生效。** 在 task dispatcher 的可信初始化位置显式导入/注册已实现的原生工厂；不从浏览器传 Python 路径或可执行函数。之后再启用 swm.enabled，缺项仍拒绝。

### C. 先辨识可区分的合成物理场景，再碰真实世界

在本机实际 SAPIEN 版本上验证 physics_replay 构建与step接口、量纲、碰撞网格、工具球坐标系、初始速度/稳定状态和相同 u 是否确实相同。先固定一个已知摩擦、密度的仿真地面真值，给算法多组假设；检验匹配和分布更新，再用与拟合不同的动作检查预测改进。

至少加：全部候选近似等价、密度不可辨识、拒绝命令轨迹替代反馈、定位丢失、手动干预、支撑面移动、换网格、重用旧样本、取消和超时。不能只测一个数字距离最小就宣布物理辨识实现。

本机默认最多1个 native/GPU任务；新并行工具默认workers=1，并发能力在资源测试后再开启。此前控制机黑屏原因仍应单独排查，不能借新的参数搜索再次压满显存/内存。

### D. 最后再接高层LLM与控制台展示

复用已验证的 LLM provider，把 SWM 的对象与可用技能给上层，生成受限技能代码。先固定两个小任务做 compile→仿真→检查点→再规划闭环；与原直接任务请求并存，但UI明确标注哪个入口启用了SWM。

实际新控制台应显示：当前场景版本、每对象最后采集时刻、真实/物理/fixture来源、检查点失败原因、当前技能、技能目标误差、仿真预测误差、物理候选/权重与是否有区分度。本轮提供只读状态函数，未改控制台或实现完整面板。

## 7. 本轮测试与不可混用的证据

当前环境无法通过 git clone 获取完整仓库。源码树是当前已读取接口与先前交付的受限重建树；唯一改动的旧文件 worker 修改前 blob 为 `269a087f…`；console_api 保留原样，原隔离工具 blob `4d374ce…` 完整核对。没有导入 vendor。

本轮实际执行：

```bash
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- \
  python3 -m pytest tests/swm -q
python3 tools/run_network_isolated.py -- python3 -m compileall -q \
  rm75_app/swm rm75_app/workcell/worker.py
```

新增 **100 项通过、0失败、0跳过**；编译通过。测试包含真正在Python中执行的SWM状态机、技能顺序、陈旧场景拒绝、前后观测、参数重放调度/后验/重采样、AST程序、资产导出和存储恢复。观测/执行/LLM/SAM3D inference/物理响应使用明确替身；asset exporter测试使用本地trimesh几何，不是SAM3D推理。没有做新的真实API调用。

NOT_RUN：完整仓库回归、三个原生原子适配器、真实FP/SAM3D推理、真实PhysX参数重放、相机、真机和夹爪。**不要把100项测试或既有1377项回归当作这些项目的通过证据。**

回传一份 `docs/CODEX_SWM_ATOMIC_INTEGRATION_RESULTS_20260912.md`，按“已实现内核/原生已接线/真实模型已调用/实际仿真/硬件”五列记录。缺绑定、缺资产或观测失败必须写明，不能删除swm.enabled检查后改叫“成功”。

## 8. 方法与论文表述边界

SAM3D、FoundationPose、基于真实轨迹更新仿真参数分布并非本项目首次提出。SWM相关贡献应围绕实际验证过的统一对象级世界状态、技能边界闭环、模型不确定性驱动的并行求解和可执行任务接口来表述；不直接写“首次实现”“恢复唯一真实密度”或未经比较的性能提升。

官方参考：
- SAM3D Objects：https://github.com/facebookresearch/sam-3d-objects ，`notebook/inference.py` 的推理接口。
- FoundationPose：https://github.com/NVlabs/FoundationPose ，https://nvlabs.github.io/FoundationPose/ 。
- SimOpt：Chebotar et al., Closing the Sim-to-Real Loop: Adapting Simulation Randomization with Real World Experience，https://arxiv.org/abs/1810.05687 。
- DROPO：Tiboni et al., DROPO: Sim-to-Real Transfer with Offline Domain Randomization，https://arxiv.org/abs/2201.08434 。

结束标记：SWM-ATOMIC-ARCHITECTURE-20260912-END
