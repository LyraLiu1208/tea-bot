# TeaBot 完整开发指南

## 项目完整状态总结

经过深入研究 Alicia-D SDK 和 RoboCore，TeaBot 现在拥有完整的开发和仿真能力。

---

## 🎯 核心发现

### 1. Alicia-D SDK 架构

```
用户层: SynriaRobotAPI
          ↓
  RoboCore (运动学库)
          ↓
   硬件执行层
          ↓
   串口通信层
          ↓
    机械臂硬件
```

### 2. RoboCore - 高性能运动学库

**RoboCore** 是 Synria Robotics 开发的统一机器人库，是 Alicia-D SDK 的核心依赖：

- **性能**: 比纯 Python 快 50-100 倍（NumPy 后端）
- **GPU加速**: PyTorch 后端支持批处理（14x 加速）
- **功能**:
  - ✅ URDF/MJCF 解析
  - ✅ 正向/逆向运动学
  - ✅ 雅可比矩阵计算
  - ✅ SE(3)/SO(3) 变换
  - ✅ 运动学树可视化
  - 🚧 轨迹规划（开发中）

### 3. synriard 包

`synriard` 是提供 Alicia-D 机器人模型的包：
- 包含 URDF/MJCF 模型文件
- 提供 `get_model_path()` 函数
- **可能需要单独安装**（联系官方获取）

---

## 📦 完整依赖关系

### 基础运行（Mock 模式）

```bash
fastapi
uvicorn
pydantic
PyYAML
```

### 真实硬件

```bash
# Alicia-D SDK（包含 RoboCore）
git+https://github.com/Synria-Robotics/Alicia-D-SDK.git

# RoboCore（如果单独安装）
git+https://github.com/Synria-Robotics/RoboCore.git

# 模型文件包（需要联系官方）
synriard  # 提供 URDF/MJCF 模型
```

### 仿真

```bash
# MuJoCo 物理仿真
mujoco>=3.0.0

# 可选：GPU 加速
torch torchvision
```

---

## 🚀 完整使用流程

### 阶段 1: 当前状态（无硬件）

你已经有：
- ✅ Mock 双臂控制器
- ✅ FastAPI 后端
- ✅ Matplotlib 可视化
- ✅ MuJoCo 仿真框架

**现在可以做**：

```bash
# 1. 启动 API 服务
python run.py

# 2. 可视化轨迹设计
python tools/visualize_trajectory.py --demo

# 3. 测试动作序列
python tools/execute_sequence.py data/sequences/demo.json

# 4. 准备好仿真（等待 MJCF）
python tools/run_simulation.py data/models/simple_dual_arm_example.xml --demo
```

### 阶段 2: 获取模型后

**联系 Synria Robotics** 获取：
1. `synriard` Python 包
2. Alicia-D 的 MJCF 模型文件

```bash
# 安装模型包
pip install synriard  # 或从官方提供的 wheel 安装

# 启动完整仿真
python tools/run_simulation.py data/models/alicia_d_dual_arm.xml
```

### 阶段 3: 连接真实硬件

```bash
# 1. 安装 Alicia-D SDK
pip install git+https://github.com/Synria-Robotics/Alicia-D-SDK.git

# 2. 修改配置
vim config/robot_config.yaml
# mode: real
# port: /dev/tty.usbserial-xxx

# 3. 启动服务
python run.py
```

---

## 🎨 可视化和仿真能力

### 方案对比

| 方案 | 状态 | 适用场景 | 性能 |
|------|------|---------|------|
| **Matplotlib** | ✅ 已实现 | 轨迹验证 | ⭐⭐⭐ |
| **MuJoCo** | ✅ 已集成 | 物理仿真 | ⭐⭐⭐⭐⭐ |
| **RoboCore Viz** | 可用（需RoboCore） | 运动学树显示 | ⭐⭐⭐⭐ |
| **SparkVis** | 官方工具 | Web 实时可视化 | ⭐⭐⭐⭐⭐ |
| **SDK plot_3d** | ✅ 可用 | 高质量轨迹图 | ⭐⭐⭐⭐ |

### 使用 RoboCore 可视化

一旦安装了 RoboCore 和模型：

```python
from robocore.modeling import RobotModel

# 加载模型
robot_model = RobotModel("path/to/alicia_d.urdf")

# 显示模型信息
robot_model.summary(show_chain=True)
robot_model.print_tree()

# 显示运动学树
robot_model.visualize_kinematic_tree()
```

---

## 🔧 如何获取缺失的组件

### 1. synriard 包和 URDF/MJCF 模型

**联系官方**:
- Email: support@synriarobotics.ai
- 说明: "我购买了 Alicia-D 双臂机械臂，需要 synriard 包和 MJCF 模型用于 MuJoCo 仿真"

**可能的获取方式**:
- 随硬件提供的 SDK 包
- 官方 GitHub 私有仓库
- 单独的安装包（wheel 文件）

### 2. SparkVis Web 可视化

**查询**:
- 检查购买的硬件包中是否包含
- 询问官方是否有开源版本
- 可能在独立仓库：https://github.com/Synria-Robotics/SparkVis（待确认）

### 3. ROS 包（可选）

如果你想用 ROS/RViz：
- ROS1: https://github.com/Synria-Robotics/Alicia-D-ROS1
- ROS2: https://github.com/Synria-Robotics/Alicia-D-ROS2

**注意**: macOS 上安装 ROS 困难，不推荐。

---

## 💡 开发建议

### 当前阶段（推荐顺序）

1. **定义任务流程**
   - 与 AI Agent 同事确认 JSON 格式
   - 设计奶茶制作的动作序列

2. **验证设计**
   - 使用 Matplotlib 可视化轨迹
   - 在 Mock 模式下测试完整流程

3. **获取模型**
   - 联系官方获取 `synriard` 和 MJCF
   - 在 MuJoCo 中测试碰撞

4. **连接硬件**
   - 安装 Alicia-D SDK
   - 录制真实动作
   - 优化参数

### 远程协作建议

由于你无法直接接触硬件：

1. **准备工具集**:
   ```bash
   # 录制工具
   python tools/read_state.py --arm both --continuous

   # 回放工具
   python tools/execute_sequence.py --file recorded.json
   ```

2. **使用仿真替代**:
   - 在 MuJoCo 中完成大部分开发
   - 让有硬件权限的同事验证

3. **数据共享**:
   - 通过 Git 共享录制的 JSON 文件
   - 共享仿真验证的结果

---

## 📚 完整文档索引

### 快速开始
- [README.md](README.md) - 项目概览
- [QUICKSTART.md](QUICKSTART.md) - 快速开始指南
- [ENVIRONMENT_SETUP.md](ENVIRONMENT_SETUP.md) - 环境设置

### 详细指南
- [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) - 项目总结
- [MUJOCO_SIMULATION.md](MUJOCO_SIMULATION.md) - MuJoCo 仿真指南
- [VISUALIZATION.md](VISUALIZATION.md) - 可视化方案
- **COMPLETE_GUIDE.md** (本文档) - 完整开发指南

### API 文档
- http://localhost:8000/docs (启动服务后访问)

---

## 🎓 学习资源

### 官方资源
- [Alicia-D SDK](https://github.com/Synria-Robotics/Alicia-D-SDK)
- [RoboCore](https://github.com/Synria-Robotics/RoboCore)
- [官方文档](https://docs.sparklingrobo.com/)
- [ROS 包](https://github.com/Synria-Robotics/Alicia-D-ROS2)

### MuJoCo 资源
- [MuJoCo 文档](https://mujoco.readthedocs.io/)
- [MJCF 格式](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [示例模型](https://github.com/deepmind/mujoco/tree/main/model)

### 运动学资源
- [现代机器人学](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- [Peter Corke Robotics Toolbox](https://github.com/petercorke/robotics-toolbox-python)

---

## 🚨 常见问题

### Q: 我现在能做什么？

A: 你可以：
1. ✅ 开发和测试 API 接口（Mock 模式）
2. ✅ 设计动作序列并可视化
3. ✅ 使用简化模型测试 MuJoCo 仿真框架
4. ✅ 与 AI Agent 同事联调接口
5. ⏳ 准备好真实模型后立即启动完整仿真

### Q: synriard 包在哪里？

A: `synriard` 不在 PyPI 上，可能：
- 随 Alicia-D 硬件提供
- 需要联系官方单独获取
- 在私有仓库中

**行动**: 发邮件给 support@synriarobotics.ai

### Q: 没有 MJCF 能用 MuJoCo 吗？

A: 可以，但功能受限：
1. 使用我提供的简化模型（`simple_dual_arm_example.xml`）
2. 或从 URDF 转换（如果有 URDF）
3. 或先用 PyBullet（支持 URDF）

### Q: RoboCore 和 Alicia-D SDK 什么关系？

A:
- **RoboCore**: 通用运动学库（底层）
- **Alicia-D SDK**: 专用于 Alicia-D 的控制库（上层）
- **关系**: SDK 依赖 RoboCore 进行运动学计算

安装 Alicia-D SDK 会自动安装 RoboCore。

### Q: 我需要 ROS 吗？

A: **不需要**。TeaBot 是纯 Python 项目：
- 开发：Mock 模式 + Matplotlib
- 仿真：MuJoCo
- 真机：Alicia-D SDK（串口通信）

ROS 是可选的，主要用于与其他ROS系统集成。

---

## ✅ 下一步行动清单

### 立即可做（无需等待）

- [ ] 与 AI Agent 同事确认 JSON 接口格式
- [ ] 定义奶茶制作的动作序列
- [ ] 在 Matplotlib 中可视化并验证
- [ ] 测试 FastAPI 端点

### 等待模型

- [ ] 联系 Synria Robotics 获取 synriard 和 MJCF
- [ ] 替换简化模型为真实模型
- [ ] 在 MuJoCo 中测试完整仿真
- [ ] 检测潜在碰撞问题

### 连接硬件后

- [ ] 安装 Alicia-D SDK
- [ ] 配置串口连接
- [ ] 录制真实关键点位
- [ ] 优化动作参数
- [ ] 完整系统联调

---

**祝开发顺利！** 🎉

有任何问题随时询问 support@synriarobotics.ai 或在项目中提 issue。

---

**文档版本**: v1.0
**更新日期**: 2025-12-05
**作者**: Claude Code Team
