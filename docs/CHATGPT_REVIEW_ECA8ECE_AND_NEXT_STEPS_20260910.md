# eca8ece 代码审阅与三场景续接

日期：2026-09-10。
用户提交：`eca8ece3aa6aa86155cd34b2e4b5e59b471a1632`；父提交 `bc531b5cdd5df2ba60c6ba9e1fbbd3fee5146ac3`。
本次修复基线：`fae8a5ee522564cc9bdbf26195ccdd3ebf19e3dd`，位于用户提交之后。分支保持 `chatgpt/three-scene-software-closeout`，不回退、不覆盖用户改动。

## 1. 先纠正交付状态

上一条聊天说“没有推送新代码”不准确。远端确实已有 `fae8a5e`，包含三场景工作流代码、前端附加脚本、Jimu 生成器、随机 PushT 和交接说明。本轮直接比较 `bc531b5 → eca8ece`，并检查它与后续工作流的调用关系，不要求用户额外编写回传 MD。

旧文档 `CHATGPT_THREE_WORKFLOW_ITERATION_20260910.md` 保留为实现说明；本文件补充实际接口缺陷、测试边界、运行方式和优先级。旧文档中的152项记录本轮没有复跑，不当成本轮或完整仓库的测试证据。

## 2. 用户这次提交实际改变了什么

比较结果是一条提交、45个变更路径。不是仅仅多了诊断脚本。

### PickPlace

`workcell/failed_object_ik_search.py` 增加胶棒/薯片罐失败目标的可选128/256 seeds重试，保持原目标和碰撞状态，按单目标运行、克隆返回缓冲、独立复核配置有效性。`ik_candidate_gallery.py` 及导出工具保存同一候选关系各阶段的实际IK行。

这并未把“一个release端点成功”变成“完整抓放成功”。已有记录里胶棒新增release解但未形成四阶段全通过关系；薯片罐已有端点关系，仍卡在带载起始抬升。后者固定目标的负载/底座球相对关系不会因更换冗余IK分支而改变。

### Jimu

本提交对 `jimu_roof_ik_diagnostics.py` 是少量诊断字段补充，没有新的LLM生成或底板前端。此前103 mm显式试验12/12的结果属于原完整builder和其特定配置，不代表任意生成结构均可执行。生成工作流来自后续 `fae8a5e`，不能混称为用户本次已经实现。

### PushT

`batch_search.py`、`response.py`、`motion.py`、`physics.py` 构成实际改进：默认20个接触特征、3个方向偏角、6种推距；NumPy多步候选排序；接近目标限制长度；GPU分批筛选并验证完整动作链；推完用新观测拟合推进/侧滑/转角响应。原50 mm上限与6 mm微调候选都应保留，不退回固定12 mm版本。

`gripper_collision.py` / `curobo2.py` 还修正了自碰撞平方距离与线性重叠的单位，以及实际夹爪几何处理。不能再把原平方数值当米，也不能把规划球重叠说成实体网格穿透。

现有记录称同一仿真场景经校准后30 mm+6 mm两次推移满足原阈值。原始运行视频、轨迹及15条校准转换仍是本机文件；本轮只读到代码和提交内记录，没有下载或重跑这些物理证据，不能据此声称随机场景已通过。

## 3. 本轮已修复的接口问题

### 3.1 连续抓放摘要读取了不存在的原生字段

原生产端 `FrozenWorldValidation.result()` 输出：

```text
frozen_world_validation.source_outcomes
```

而 `iteration_workflows.completion_summary()` 读取 `native_full_world_validation`。因此实际完成5个物体的作业可能在新UI里显示0个完成。旧模拟fixture也沿用了错误字段，没检出这个生产/消费端不匹配。

新增 `sequence_evidence.py` 并接到摘要函数：优先读取真实字段，按实际前台source与显式非预取标记计数，列出pending、重复成功和意外source。即使已有部分进展，只有原生完整门、输入不变、所有source搬运、独立退让验收都为真，且成功次数与请求匹配，才标记整序列通过；独立物理成功仍为None。

为兼容此前客户端fixture保留旧字段的诊断展示，但该字段永远不能授予整序列通过；响应标出evidence_key及是否真实原生接口。两种字段同时出现时只使用真实字段。

### 3.2 多选框只选择一个物体时传了无效循环

前端允许选择1..12个物体，但原 `read_contract` 对显式 `source_order` 要求至少2个。现在只选择一个时转换为原单对象请求，并从作业profile副本移除旧循环字段；多选仍使用同一场景原生循环。原profile不修改。

### 3.3 LLM联网与仿真隔离需要分进程

把整个工作台放进非Unix网络隔离会阻止浏览器HTTP和LLM请求；不隔离任何进程又不应让原生SIM依赖获得设备网络访问。本轮新增 `offline_boundary.py`，在实际 `worker.main` 的任务分发之前强制preview/sim worker安装现有seccomp过滤器。

```text
仅回环地址的独立Web/API进程 → 已配置LLM端点
                         ↓ 作业文件，不传密钥
preview/sim worker → 安装/自检网络隔离 → 原生引擎或物理后端
                                  ↓
                           子进程继承隔离
```

隔离失败立即返回任务失败，不继续导入或运行原生引擎。没有浏览器/profile关闭开关。没有访问真实地址来测试隔离。真实模式保留原有单次许可和现场确认门，不因本函数获得授权。

这仅是网络隔离，不是串口/USB或完整文件系统沙箱；本轮仍没有硬件动作许可。所有命令行回归及仿真测试继续按AGENTS通过 `tools/run_network_isolated.py` 启动。本轮没有改变该隔离脚本。

## 4. 三条主线的下一轮目标

### 4.1 PickPlace：先让大部分顺畅，不无差别加seeds

优先跑同一current_table、同一完整请求的顺序任务，保留七物体分母。先完成原来可完成的刷子、笔、木块、胡萝卜、网球，再处理胶棒/薯片罐；这是调度建议，不是通过保证。前台成功、后台预取、端点通过、完整链通过必须分开。

胶棒优先对照 `paired_endpoint_repair` 关闭/开启。它只让同一抓取关系的合格hover给release提供seed（或反向），最多12次额外查询、5 s查询间软预算；不改位置、姿态、负载、碰撞或原始seeds数量。不要同时开启128/256-seed搜索再声称单因素提升。

薯片罐优先验证带载退出路径。先复核原代码已有的world-Z lift与approach-line retreat，定位首个无效配置。若原抬升目标在不变负载几何下与底座冲突，应比较原允许抓取关系或有界的新退出中间点，并重验带载全路径；不能关掉底座碰撞。最终放置目标不变，不要修改默认算法直到冻结SIM对照支持。

网球顺序中可成功不等于独立首槽已解决；原记录还出现过下降窗口瞬时姿态偏差。后续要检查整条下降的FK方向，不能只看两个端点水平就标记全程水平。

### 4.2 Jimu：两种旧底板都接上，LLM设计直接进入同一原生入口

当前已有代码：`magnetic/generation.py`、`llm_client.py`、`tools/import_jimu_design_library.py`、`configure_iteration_workflows.py`、前端iteration面板与原manifest调用适配。

现在生成空间是原模板槽位的依赖完整子结构：LLM选择原模板和活动件角色；底板/坐标/局部轴/相对变换由原数据确定，最多12件。它是可落地的第一版，不是自由创造任意新6D装配。支撑闭包和库存通过也不是磁吸强度或整臂可达证明。

Codex只读查找本地旧 `jimu_task_manifest_v1`，分别确认3×3与arc/tag2任务，不要求用户再提供路径。已尝试读取旧远端 `Beta_demo-codex-v0.9/jimu_tasks`（ref 1.0.25），该路径返回404，不能说已从远端恢复了两份实际任务。不要用合成测试夹具冒充旧弧形底板，也不要按猜测半径重建。

导入真实manifest、builder、fixed scene后，保留tag/底板坐标、料槽角色/三角槽位以及原启动参数。原完整模板先跑通，再测试较小的生成子结构：未选角色不得仍被调度，选中角色的父件不能缺失，料槽实际库存/身份与设计角色必须对应。

至少两类底板各三个不同结构，包含原完整12件方案；记录“用户描述→真实LLM返回→编译后设计hash→前端预览→传给native的相同JSON→实际完成角色”。生成错误可以有界修正，不能把固定示例偷偷作为LLM返回；规划失败明确反馈哪一件、哪个阶段。

用户目标是前端可用，所以不能只测Python函数。实际API密钥由本机既有环境读取，不写进Git、浏览器或回传。

### 4.3 PushT：随机状态、长推、移动后重定位继续

保留用户最新候选/响应模型/完整物理分支；后续 `fae8a5e` 已有 `run_until_goal`、随机案例、服务端几何变体、pause/relocate/resume队列和干预后丢弃旧计划。先回归当前已通过案例，再扩大测试集。

第一层随机化只改原T的位置、yaw和目标（平移/旋转/混合）；第二层再测不同尺寸，丢弃不同几何的旧响应拟合。不要把几何边界有效误写成机械臂可达，也不要只留下成功seed。先12个原T案例，不通过先定位，不立即跑大量GPU作业。

交互主流程：推完并撤离→暂停确认→移动仿真T→恢复→读取新稳定观测→按原目标继续。干预不计推送、不进入响应拟合；原观测会话、帧序号、时间戳检查保留。新代码的pause是动作边界暂停，不是正在接触时立即制动，更没有授权用户把手伸进运动臂附近。

“未到位就继续”允许多轮动作，但无进展应等待/重新观测，不能空转GPU；碰撞、坏观测、未知执行错误、Stop仍终止。还需特别实测：

- 新交互控制器在准备轨迹后再次observe，会推进物理步；核对完整臂实际q与计划起点检查是否因此频繁失败。应从新实际状态重新规划或恢复已定义的保持协议，不直接放宽跟踪门。
- 位姿改变发生在成功确认窗口时，必须丢弃本次成功确认并重新获取稳定观测，不能下一步使用窗口前的旧状态。
- `PhysicsSession.observations` 和环境tracking/contact数组仍可能随长会话增长。正式长时demo前改为流式证据+有界展示缓存，保持累计计数/峰值/全部磁盘证据；不能因“一直推”重新触发内存问题。
- 当前退让碰撞豁免是用户既有显式实验设置，本轮未扩大。随机新场景的物理禁止接触监测必须保留；不能描述为全程规划碰撞检查均开启，更不能移植为真机资格。

## 5. 本轮实际验证

测试环境为部分源码树，不是用户完整仓库/GPU环境。读到的 `io.py`、`native_frozen_world.py`、原worker、原iteration_workflows以及隔离脚本均在本地核对Git blob一致后再修改/测试；没有导入vendor。

运行：

```bash
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python tools/run_network_isolated.py -- python -m pytest \
  tests/test_iteration_sequence_reconciliation.py \
  tests/test_iteration_offline_boundary.py -q
```

结果：31 passed，0 failed（0.67 s）。新增/修改Python文件compileall通过。

测试包含真实 `FrozenWorldValidation.result` 方法的AST与消费者对照（仅编译方法，不加载其原生依赖；碰撞/源统计帮助函数为替身）、单选/多选转换、前台/预取/替代source/重复成功、缺失门检查、真实内核子进程IPv4/IPv6 socket拒绝及Unix IPC保留、隔离失败和worker调用位置。

本轮没有跑整仓tests、真实浏览器点击、商业LLM、GPU、PhysX随机矩阵、相机或真机。不要把31项契约测试解释为“抓放成功率提高”或“生成的结构已经搭出”。验证JSON记录此边界。

## 6. Codex执行顺序

先保留本地未提交差异，fetch并ff-only pull；不reset/clean，不修改旧目录/vendor。最新文件需同时读取：本MD，以及上一版 `CHATGPT_THREE_WORKFLOW_ITERATION_20260910.md` 的配置/数据结构说明。冲突时以本文件更新的字段与隔离方式为准。

先跑新增31项和现有 `tests/iteration`，再按AGENTS显式运行完整 `tests/`。禁止收集旧vendor测试。GPU仍串行，并沿用当前内存/CPU资源上限。

先用真实旧任务导入两种底板、创建新的SIM profile，保留已有模型和原成功参数。独立工作台启动：

```bash
PYTHONPATH=. python -m rm75_app.workcell serve \
  --profile runtime_data/iteration/machine.json --port 7861
```

不传 `--allow-real`。此Web/API进程需要回环HTTP并可访问用户配置的LLM；preview/sim作业进入worker后会自动安装网络隔离。若要求连API父进程也完全断网，商业LLM生成必须分离为另一服务/阶段，不能关闭worker隔离来解决。命令行测试/原生SIM仍使用外层隔离启动器。

随后按PickPlace连续抓放、Jimu两底板生成链路、PushT单例回归+随机与干预的顺序测试。写一份 `docs/CODEX_THREE_WORKFLOW_ITERATION_20260910.md` 与一个有界JSON回传，保留全部失败分母。不要再把“未生成MD”当阻止代码审查或继续工作的前置条件。

结束标记：ECA8ECE-RECONCILIATION-END
