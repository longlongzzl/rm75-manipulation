# 本地 Codex 写入：LEGO / PickCube 迁移说明

`examples/lego_house/` 是从 `Beta_demo-codex-v1.1` 迁入的独立源码快照，包含 PickCube 环境、LEGO/Jimu 四墙装配规划、装配状态机、RM75 cuRobo 配置和可复现的 cuRobo snapshot。

## 独立性边界

- 迁入代码的默认 cuRobo 路径是仓库内 `examples/lego_house/repro/curobo_snapshot`。如需使用另一份 cuRobo，可设置 `RM75_MANIPULATION_CUROBO_ROOT`。
- RM75 URDF 默认读取本仓 `assets/robot_models/RM75_gripper/RM75-B/urdf/RM75-B.urdf`。
- 不引用旧 `lerobot`、`Beta_demo-*` 或 `pick_jiaobang` 目录。
- ManiSkill、PyTorch、cuRobo 是外部 Python 依赖；若当前 ManiSkill 安装不包含 RealMan robot 扩展，设置 `JIMU_EXTRA_MANISKILL_PACKAGE_ROOT` 指向安装该扩展的环境目录。它不是旧仓库路径。

## 轻量验证

```bash
cd ~/Desktop/rm75-manipulation
python -m py_compile examples/lego_house/*.py examples/lego_house/assembly_planner/*.py
PYTHONPATH=. python -m pytest tests/test_pickplace_coordinator.py -q
```

不应以真实机械臂或 GPU 规划作为迁移验证；这些命令仅验证源码可发现、语法和现有协调器回归。
