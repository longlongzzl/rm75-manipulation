# RM75 三场景迁移与本地验收交接

日期：2026-09-06  
交付范围：PickPlace 已完成版本迁移、磁吸 Jimu 已完成版本迁移、PushT 新前端与真机链路。  
不是 Lego 第三任务，也不是仅针对 PickPlace 的 E1C 诊断包。

## 0. 先读：本次到底交付了什么

本包已经包含三场景工作台、任务服务、迁移器、原版运行适配器、PushT 观测/预测/运动执行实现、测试与安装脚本。不是让 Codex 再从文字方案开始写三套功能。

但必须区分以下三个事实：

1. **原版大体积源码/模型资产不在这个增量 ZIP 内。** 已通过 GitHub 阅读旧仓库的对应实现；本地迁移器会从用户已有旧 Git 仓库的固定提交，把已提交源码及所选资产实际复制到新仓库内部。必须运行迁移步骤之后，原版引擎才会出现在新仓库。这里没有把“写了迁移器”说成“已在用户电脑完成迁移”。
2. **本轮采用“保留工作引擎、迁移外层架构”的边界。** 原版候选生成、抓放、部分张开、料槽重试、磁吸 capture/release 等逻辑不被重写；任务契约、前端、进程管理、日志与启动边界统一。不是已经把两个超大运行脚本逐函数拆进新的 cuRobo2 `planning/` 接口。
3. **本轮没有连接相机或机械臂，没有运行真正的 cuRobo/ManiSkill。** CPU 测试、真实子进程和局部浏览器检查通过，不代表原版实机回归或 PushT 物理效果已经通过。本地仍可能出现原运行环境、资产依赖或 SDK 版本适配问题，须用下面的命令定位，不能通过取消校验来“跑通”。

此前 `rm75_software_completion_20260906.zip` 是独立的 PickPlace 诊断增量，不是本三场景迁移的前置依赖。本包不自动安装或重复覆盖该包。

## 1. 冻结来源与保留原则

### 1.1 新仓库基点

```text
longlongzzl/rm75-manipulation
263cd7eca41b25cbafd1f365050f022c33285362
```

### 1.2 旧工作版本

```text
longlongzzl/lerobot-realman
branch: 1.0.25
commit: 7aaff9da22486b7d25557b3795dd258f9b65f10d
```

入口与已检查的关键源文件：

```text
pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place_sam6d.py
pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py
pick_jiaobang/rm75_jiaobang_pick_real_with_foundationpose.py
pick_jiaobang/curobo_rm75_planner.py
Beta_demo-codex-v0.9/rm75_jimu_four_wall_portable.py
Beta_demo-codex-v0.9/rm75_jimu_triangle_roof_apriltag_portable.py
Beta_demo-codex-v0.9/magnetic_snap.py
Beta_demo-codex-v0.9/jimu_sam6d_pose_provider.py
```

六个关键入口的 Git blob SHA 由 `workcell/migration.py::KEY_BLOBS` 硬性校验。迁移器只读取提交中的 blob，不 checkout、不 reset、不 clean、不覆盖旧工作区。

**先确认这一提交确实包含你最后完成的版本。** 旧目录里的未提交修复不会被偷偷收入快照，也不会被覆盖。若真正完成版本在另一提交，必须先比较差异并更新来源审计，不能绕过 SHA 校验。

迁移后的实际目录：

```text
rm75-manipulation/
  rm75_app/_vendor/working_snapshot/
    MIGRATION_MANIFEST.json
    pick_jiaobang/...
    Beta_demo-codex-v0.9/...
    RM75_gripper/...
    ...其他被清单选中的源码和资产
```

`MIGRATION_MANIFEST.json` 逐文件记录原 blob、原大小、安装后 SHA256、路径替换次数及源提交。每次原版任务启动前验证文件是否被改动。移动新仓库目录会使写入源码的绝对路径失效；应重新迁移到最终路径，不能仅移动快照并改清单骗过校验。

只改已知旧项目绝对路径，不改原算法、不减候选、不改碰撞容差、不修改抓放成功条件。不会打包模型权重、字体或大批运行日志。模型环境继续使用本机已验证安装。清单的 `dependency_completeness_verified` 初始为 false，必须经本地原版启动验证后才能另行给出依赖闭合结论。

## 2. 三条功能线分别推进到哪里

| 功能线 | 本包新增/接入 | 仍需本地验收 |
|---|---|---|
| PickPlace | 原版 SAM6D/direct 入口适配；原放置规则与运动实现保留；网页选物体、请求预览、原版仿真/真机启动、日志、原确认输入、停止 | 固定场景原版前后对照；相机和模型启动；多物体/料槽等原版参数在本机 profile 中恢复；实机回归 |
| 磁吸 Jimu | 原版 triangle-roof→four-wall 调用链迁移；原 `jimu_builder_scene_v1` 导入/导出/编辑；12 活动件、坐标、父子关系校验；网页提交设计到真实原版 CLI；原部分张开与重试保留 | 原完整设计与新导出的 JSON 对照；方板/屋顶/料槽重试/释放后退让；仿真和实机结果 |
| PushT | 目标编辑与状态画布；实时 AprilTag/现有 tracker JSON；有限短程预测与排序；cuRobo2 全动作链检查；RM75 关节透传与反馈；逐次新观测、停滞退出、稳定到达验证 | 相机外参、T 形件原点、推头碰撞几何、桌面与障碍物；真正 GPU 规划；真实推移模型与跟踪效果 |

新 PickPlace 页面使用原版引擎，不把当前尚有失败例的 cuRobo2 实验链当成已完成原版的替代品。原新架构 PickPlace 默认 `curobo2` 不变，便于继续 E1C 对比。磁吸也不改成通用放置后强行声明 snap 成功。

**前端不是三个空标签。** 页面按钮实际调用同一任务服务，服务落盘请求并启动子进程；结果从工作进程回流。此前原扫描台与高级配置页保留，新页增加入口。但新页没有复制原高级面板的每一个开关，也没有把原 OpenCV/ManiSkill GUI 窗口全部变成浏览器视频流。高级原参数由机器 profile 的 `native_args` 保留，原相机页面仍可使用。

## 3. 新代码结构与各自职责

```text
rm75_app/workcell/
  migration.py          固定提交复制、路径迁移、清单校验
  install_patches.py    5 个原文件的 SHA 门控集成补丁
  legacy.py             原版 PickPlace/Jimu 入口与参数适配
  input_bridge.py       原 input() 等待点 → nonce → 原 stdin
  child_entry.py        Python .py 感知子进程的显式输入桥接
  spec.py               前端请求白名单与数据契约
  service.py            互斥、授权、任务子进程、停止与日志
  server.py             独立 WSGI / 原 Flask 挂载
  worker.py             preview / 原版 / PushT 执行分派
  realman.py            PushT API2 适配、反馈与参考轨迹定时
  verification.py       可选的原版任务末态实测验证
  cli.py                serve / run / doctor
rm75_app/magnetic/design.py
rm75_app/pusht/{model,observation,controller,motion}.py
rm75_app/tasks/pusht.py
rm75_app/web/static/workcell/{index.html,style.css,app.js}
tests/three_scene/       独立测试目录，不覆盖原 conftest.py
```

受控修改的原文件：`tasks/registry.py`、`tasks/pickplace.py`、`tasks/jimu.py`、`legacy/backends.py`、`web/control_panel.py`。新模式为 PickPlace/Jimu 的 `working-preview/working-sim/working-real`；新增 PushT adapter。保留 Lego 原入口，但不把 Lego 当本轮三场景之一。

原文件版本不匹配时安装器拒绝覆盖。备份位于 `runtime_data/workcell/install_backup/`。不修改 `planning/backends/curobo2.py` 或原模型参数。

## 4. 安装：Codex 先执行这些

### 4.1 找到两个真实本地 Git 目录

下面变量是占位路径，请定位实际目录后设置。不要为了匹配本文把用户目录重命名。

```bash
export OLD=/实际的旧lerobot-realman仓库路径
export NEW=/实际的rm75-manipulation仓库路径
export BUNDLE=/解压后的rm75_three_scene_migration

# 必须检查输出，不只看目录名
git -C "$OLD" remote -v
git -C "$OLD" status --short
git -C "$OLD" cat-file -e 7aaff9da22486b7d25557b3795dd258f9b65f10d^{commit}
git -C "$NEW" status --short
git -C "$NEW" rev-parse HEAD
```

若旧 Git 尚无这一提交，可以从其已核实的 origin 获取 `1.0.25`，不切换工作区；不要修改未知仓库的 remote。新仓库有本地改动时先保留，SHA 门控冲突必须审阅合并，不要使用 `git reset --hard`。

### 4.2 安装代码并真正复制原实现

```bash
python "$BUNDLE/tools/install_three_scene.py" \
  --target "$NEW" \
  --legacy-source "$OLD"
```

这个命令不只是建立 sibling 路径链接，而是读取固定 Git blob、写入新仓库内部快照并产生清单。若源码复制失败，部分新增文件可能已经拷入，但原集成补丁尚未应用；查明原因后重试，不要把第一次失败当成已安装。

若此前只安装了增量代码、尚未迁移源文件：

```bash
cd "$NEW"
python tools/migrate_working_sources.py --source-repo "$OLD" --target-repo "$NEW"
```

迁移器拒绝覆盖已有快照，这是有意的。若需要重装，先保存旧快照与安装清单并人工确认，不提供 force 覆盖参数。

### 4.3 创建本机配置

```bash
cd "$NEW"
python tools/create_workcell_profile.py \
  --output runtime_data/workcell/machine.json
```

已有配置不会被覆盖。生成器尝试定位 `foundationpose310` 和 `curobo2` Python；若不存在，只写当前 Python 作为待检查值，**不代表环境选对**。请把 PickPlace/Jimu 指向原来确实跑通的环境，PushT 指向 cuRobo2 + 相机 + API2 可用环境。

默认 `hardware_reviewed=false`，三个任务 `integration_qualified=false`；机械臂 IP、关节限位、相机外参、推头姿态均不能靠示例猜填。

`native_args` 是受信任本机参数列表，沿用原成功命令中的模型路径、抓放循环/目标池、料槽与控制配置；不能包含由任务服务控制的 `--execute-real`、`--auto-execute`、`--object-name`、`--jimu-builder-scene-json`。不允许从浏览器传任意 shell 命令。

`fixed_scene` 只用于原版非真机模式。默认为 null，不猜造一个不存在的 fixture 路径。若本地要离线仿真，从迁移清单/原成功命令中选真实文件；非真机模式未配固定场景时，原版可能正常启动相机感知。

### 4.4 不接机器人，先查原版接口

```bash
python -m rm75_app.workcell doctor \
  --profile runtime_data/workcell/machine.json

python tools/check_working_entrypoints.py \
  --task pickplace --profile runtime_data/workcell/machine.json \
  --request examples/workcell/pickplace_preview.json \
  --output runtime_data/workcell/validation/pickplace_native_contract.json

python tools/check_working_entrypoints.py \
  --task magnetic --profile runtime_data/workcell/machine.json \
  --request examples/workcell/magnetic_preview.json \
  --output runtime_data/workcell/validation/magnetic_native_contract.json
```

接口检查在各自配置的真实 Python 环境中 import 原模块并构建参数解析器，不调用原 `main()` 或主动运动/采集接口。原模块的 import 依赖仍要完整，因此失败日志是迁移依赖诊断，不能伪造成已经通过原版仿真。

特别注意：triangle 入口的 parser 名为 `build_arg_parser_triangle`，不是 `build_arg_parser`。本包已按实际源码处理，不使用未经安装 patch 的 portable parser 冒充。

## 5. 三场景前端启动与实际检查

### 5.1 推荐：挂到原扫描台，同一端口

```bash
cd "$NEW"
python -m rm75_app.workcell.flask_entry \
  --profile runtime_data/workcell/machine.json --port 7860
```

访问：

```text
http://127.0.0.1:7860/workcell/
http://127.0.0.1:7860/              原扫描/高级工作台
```

也可使用原 `python -m rm75_app web -- ...` 入口；安装补丁后，当默认 profile 存在时会挂载新页。不要同时开两个控制服务指向同一机器人。

### 5.2 只测新前端，不加载旧 Flask 依赖

```bash
python -m rm75_app.workcell serve \
  --profile runtime_data/workcell/machine.json --port 7861
```

独立模式没有旧扫描台路由；页上的“原场景扫描台”链接只有在 Flask 集成模式下才有旧台含义。完整项目验收仍须测试 7860 集成模式。

### 5.3 浏览器验收

在不带任何 real 许可的服务上执行：

```bash
python tools/check_workcell_browser.py \
  --url http://127.0.0