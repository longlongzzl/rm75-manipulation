# RM75 统一操作控制台：使用、实现与本机验收

日期：2026-09-11。实现基线：`7086fbd86698299013ff3aef4aa077e2aad76dee`。协作分支：`chatgpt/three-scene-software-closeout`。

这版交付将三项任务收敛到一个操作控制台。它不改变抓放、装配、推移算法、碰撞规则、原始成功阈值、原版 vendor 文件或真机资格。当前新控制台只提交 preview/SIM，不能把 UI 完成解释为硬件已经验收。

## 1. 先打开界面

仓库根目录执行：

```bash
python tools/launch_workcell_console.py
```

浏览器打开：

```text
http://127.0.0.1:7861/workcell/console/
```

不带 profile 时使用仓库示例配置。界面会显示“尚未配置”的项目；它适合先看页面、导入设计、检查操作方式，不会自动找到或假定本机原生/物理环境。示例配置不是 R2 的实际机器配置，不能据它保证原生仿真可启动。

读取已有本机配置候选，不修改、不自动选择：

```bash
python tools/launch_workcell_console.py --list-profiles
```

使用 R2 已跑通的完整配置：

```bash
python tools/launch_workcell_console.py --profile /绝对路径/实际机器配置.json --port 7861
```

只读检查路径与配置：

```bash
python tools/launch_workcell_console.py --profile /绝对路径/实际机器配置.json --doctor
```

`--doctor` 不加载机器人 SDK、连接相机、探测设备或运行 CUDA；“配置存在”不等于依赖已可运行、路径可达或硬件安全。新启动器没有 `--allow-real`，也不会修改任何资格字段。端口被占用时报告错误，不擅自杀掉占用进程，可换 `--port 7863`。

原 `/workcell/` 页面、旧编辑器和原 API 保留，用于兼容；日常使用选择 `/workcell/console/`。如果从其它机器访问，应按现有 loopback/SSH 隧道约定，不为方便改成对外开放的无鉴权服务。

## 2. 这版界面怎么操作

### 2.1 三项任务共用一个任务管理器

左侧切换“桌面整理 / 磁吸搭建 / 闭环推移”，右侧统一显示当前作业、检查状态和最近记录。顶栏只有一个共享 Stop。切换标签不会产生第二个任务，也不清除正在运行的作业。

提交前先做类型与配置检查，SIM 再显示确认对话框；取消确认不发送运行请求。预览、近似模型到位、物理状态验证、原生命令结束与真正任务成功使用不同标签。不把 `command_completed_unverified` 显示为“全部成功”。

支持原生程序的 Enter/r/q 输入提示；只有实际未消费的 nonce 才能回应。任务状态、物体完成摘要、PushT 帧和推移次数、事件及日志在一个页面显示。日志不增长为无限浏览器数组，后端沿用现有尾部读取限制。

### 2.2 刷新、断线与重复点击

浏览器按当前 profile 内容摘要保存任务草稿、已采用设计、正在编辑但未应用的 JSON、生成 ID 和作业 ID。不存 API 密钥、arm token、CSRF 或整个机器配置。

恢复仅查询现有 ID，不自动重发模型调用或机器人任务。断线显示“状态未知 / 正在重连”，不能把网络失败当作任务已经停止。服务重启后未被当前进程管理的旧作业显示“进程状态待确认”，不删除锁文件、不自动重启、不宣称已安全停止。

新的任务和生成请求使用客户端随机 128-bit ID。服务器先持久化提交意图，同 ID 同输入返回原接收记录；同 ID 换输入拒绝。确认丢失时页面查询记录，不盲目第二次提交。启动异常保留 `uncertain`，需核对历史后显式清除本地待确认记录；不承诺网络/进程任意故障下的外部 exactly-once 执行。

### 2.3 PickPlace

多选对象、显示建议顺序，或关闭建议顺序保留所选顺序。单选仍走原 `object_name`，多选走原同场景序列。预览不执行动作，SIM 使用原运行入口。

物体摘要使用上一轮已修复的真实原生证据字段，不将后台预取、替代物体或部分完成记为全成功。胶棒与薯片罐的算法问题没有因前端改版自动解决；七物体分母、失败阶段和原始路径判断仍保留。

### 2.4 Jimu

选择已安装的 3×3 / 弧形底板，设置活动件预算并输入描述。沿用当前 `IterationAPI` → 统一 provider → 受约束生成器 → 原 manifest/装配入口，没有新建第三套生成算法。

模型结果只采用一次；超过 135 秒不再由页面提前判失败。页面显示等待时间、预计预算和持续状态，直到服务端明确结束或用户放弃结果。刷新后用同一 generation_id 恢复查询。

“放弃结果”只保证之后不会自动采用该结果，不能取消已经发往提供方的网络计算或费用。提供方线程仍运行时禁止再次开始一个生成；Stop 与作业控制不因慢速模型调用失效。

载入原模板明确标成“未调用 LLM”，同时保留对应的 proof 和底板运行配方，防止把弧形设计交给另一个底板的场景配置。生成、原模板、手动设计三种来源明确区分。

支持原 JSON 和含 proof 的设计包导入/导出。编辑器修改受证明的坐标后不能复用旧 proof；需重新生成，或显式切换为手动设计并重新检查。手动设计不声称满足 LLM 库存/固定底板来源约束，运行配置仍由本机 profile 决定。

右侧画布为本地几何示意，不是相机/仿真直播。按原 Y-up 与 `[u,n,v]` 变换绘制；可拖拽旋转、滚轮缩放、切换俯视/正视/等轴视角。视图操作不修改设计坐标。

#### 一个实际浏览器数据问题的修复

原生成 proof 使用 Python 规范化 JSON 摘要，原文件中可能含 `0.0`，浏览器 `JSON.stringify` 会写成 `0`。它们数值完全相等，但字节摘要不同。后端单独调用通过，不代表真实浏览器往返也能通过。

新 Console API 在提交带 proof 的设计前，从可信库按原提案重新编译，逐字段验证 JSON 数值和结构完全相同，然后恢复可信编译器的原数值表示再交给原校验和 Service。只允许精确的有限 int/float 数值相等；不允许坐标容差、布尔值转数值、数组重排、增删字段或换 locked 件。`1e-9` 的坐标变化也拒绝。没有改变全局 `io.digest` 或降低原 provenance 校验。

### 2.5 PushT

初态、目标、角度、推速、最长推距、后端和连续模式在同一张卡上。画布区分草稿初态、当前作业实际观测与目标；可切换局部适配视图 / 全工作区。随机样本显示 seed 和案例身份，几何边界通过不标为机器人可达。

暂停走原会话 API，只有 worker 确认 `paused` 才允许“移动仿真 T”；等待上条控制指令确认后才能继续。暂停表示本段推动/撤离结束后的边界暂停，不是物理急停。移动和恢复不会直接写 worker 命令文件，也不更改控制器/物理执行器。

“未到位继续”保留现有策略：无进展可等待场景变化，异常和 Stop 仍有效，不是无限盲推。新界面没有修复 seed77 随机矩阵失败，也没有把原退让碰撞豁免变成全程碰撞已检查的保证。

## 3. 后端范围与兼容性

新增 `ConsoleAPI` 是原 `WorkcellService`、`IterationAPI` 上面的展示/提交协调层，不新建运动队列。所有 accepted preview/SIM 继续由原隔离 worker 执行。

新静态文件显式白名单，无 CDN、无构建依赖。原 WSGI CSRF、同源、loopback、请求大小和原停止/输入/会话 API 保留。新控制台 GET 元数据也限制 loopback/same-origin，不向浏览器返回机器配置、解释器路径集合、环境或凭据。

增加检查跨进程 `robot.lock` 是否占用：只读查看并尝试非阻塞锁，不解除其它进程的锁，不给占用者发任何命令。原 `ResourceLease` 仍是实际任务互斥门。

诊断导出仅含该作业请求、结果、进度及尾部事件，不含 machine_profile 或完整 stdout；敏感字段与当前已知环境密钥值会脱敏。这不能保证识别任意未知秘密，分享前仍需查看内容。

本轮改动的旧文件只有 `workcell/server.py` 与 `workcell/cli.py`：挂载新路由和启动提示。没有修改 service、worker、spec、iteration_api、completion_provider、generation、planner、executor、physics、collision、vendor 或资格限制。

## 4. 实际验证及诚实边界

环境是受限的部分源码树，不是用户完整仓库。当前基线 server/cli/io/design 与网络隔离工具的 Git blob 已核对；测试没有导入 vendor。服务/生成/运行依赖在新增展示层测试中使用明确替身，不冒充已完成原生调用。

| 验证 | 本轮结果 | 覆盖边界 |
|---|---|---|
| Python 展示/API测试 | 43 passed / 0 failed / 0 skipped | 同 ID重试与重启、并发、配置/权限、元数据、设计精确JSON往返、失联、历史、脱敏等；运行与生成依赖替身 |
| JavaScript 纯状态测试 | 32 passed / 0 failed / 0 skipped | 输入、状态标签、会话按钮、draft恢复、画布坐标等；Node built-in runner |
| 离线 DOM/组件工作流 | 11 PASS | Chromium 空白页内挂载本地组件，内存API桥+WSGI与运行替身；无外网、无真实模型或worker |
| Python 编译、JS语法 | PASS | 新增/修改文件 |
| 正常浏览器HTTP导航 | BLOCKED | `ERR_BLOCKED_BY_ADMINISTRATOR`，停在页面加载前，未绕过策略 |
| 完整仓库1334项 / GPU / PhysX / 相机 / 真机 | NOT_RUN | 必须在本地对应环境复验 |

11项组件工作流包含：首次不自动启动；预览与重新挂载不重复提交；模板画布；服务端逻辑等待超过135秒后继续轮询且恢复不再次调用；SIM确认取消与同设计提交；暂停确认/移动/恢复；共享Stop；报告导出；断线重连；390px布局无横向溢出；无脚本错误/arm请求。

“重新挂载”使用替代浏览器存储，不等同于已测真实HTTP页面刷新；慢生成采用模拟时钟/状态，不等于本轮真的付费等待模型几分钟。离线组件截图只是UI夹具渲染，不是用户电脑、真实相机或机器人运行截图。正常导航、CSP和网络集成仍未在这里验收。

验证记录：`benchmarks/unified_scenarios/operator_console_validation_20260911.json`。

## 5. 本机复验顺序

先保留本地未提交内容，再 ff-only pull。不要 reset/clean 或覆盖旧文件。继续沿用 AGENTS 网络隔离和测试收集范围：

```bash
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- \
  python3 -m pytest tests/console -q

python3 tools/run_network_isolated.py -- \
  node --test --experimental-default-type=module tests/console/client_state.test.mjs

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- \
  python3 -m pytest tests -q
```

本地 Node 需支持 ES modules 与内置 test runner。不同版本不支持上述 flag 时先明确本机版本，不删除测试来算通过。

启动控制台本身需要 loopback TCP，LLM 需要用户配置的提供方网络；不要把整个 Web 父进程放进禁止 TCP 的测试沙箱，再关闭 worker 隔离来解决。preview/SIM worker 保留内部隔离，仍禁止任何真实设备访问。

在一个没有活动作业、未启用 real 的控制台上，用本机 Playwright/Chromium执行：

```bash
python tools/check_workcell_console.py \
  --url http://127.0.0.1:7861/workcell/console/ \
  --output runtime_data/console_acceptance_20260911
```

默认只读页面和模板，不发送模型调用或SIM；增加 `--preview-object bi` 才执行一次所选物体的 preview，不运行运动。脚本限制同源，拒绝 busy/real-enabled 服务，新的输出目录不会覆盖旧证据。浏览器若缺失可用 `--chromium` 指定本机已安装可执行文件；不为测试连接实体机器人。

随后人工核对一次真正慢速 LLM：生成开始后刷新页面，最终只出现一个 generation_id，对应同一份 proof；再点“预检查”与SIM确认。核对当前R2配置，不将示例文件当正式配置。PushT先当前已知单例，不立即重复大量GPU随机任务。外部占用/锁状态异常先确认，不自动清锁或重启。

只需一份本地回传 `docs/CODEX_OPERATOR_CONSOLE_RESULTS_20260911.md`：记录 HEAD、启动配置名、正常导航、LLM刷新恢复、单对象/多对象预览、同设计SIM、Stop/会话操作、失败项和真实设备NOT_RUN。功能扩展和真机验收另列，不靠布尔资格开关绕过。

结束标记：OPERATOR-CONSOLE-20260911-END
