# TeaBot 可视化指南

## 概述

经过研究 Alicia-D SDK，我们发现 SDK **已经内置了可视化功能**。TeaBot 现在支持多种可视化方式。

---

## 方案 1: 内置 Matplotlib 可视化（已实现）⭐⭐⭐⭐

### 特点
- ✅ 无需硬件连接
- ✅ 可视化关节角度和末端轨迹
- ✅ 3D 图形显示
- ✅ 轻量级，易用

### 使用方法

#### 查看演示

```bash
python tools/visualize_trajectory.py --demo
```

会显示：
- 3D 圆形轨迹
- 6 个关节的角度变化曲线

#### 可视化序列文件

```bash
python tools/visualize_trajectory.py --file data/sequences/demo.json
```

会根据序列中的动作类型自动生成可视化：
- `move_joint` → 关节角度曲线
- `move_pose` → 3D 末端轨迹

---

## 方案 2: Alicia-D SDK 内置可视化（真实硬件）

### plot_3d() - 3D 轨迹

Alicia-D SDK 提供了专业的轨迹可视化功能。

当连接真实硬件后，可以这样使用：

```python
from alicia_d_sdk.utils.vislab import plot_3d, plot_joint_angles

# 假设你录制了一段轨迹
pose_trajectory = [...]  # [[x,y,z,qx,qy,qz,qw], ...]

# 可视化（带姿态箭头）
plot_3d(
    [pose_trajectory],
    legend="Tea Making",
    show_ori=True,  # 显示姿态方向
    title="End Effector Trajectory",
    axis_length=0.08
)
```

**效果**：
- 3D 曲线轨迹
- 起点（绿色）和终点（红色）标记
- RGB 坐标轴显示姿态方向
- 自动等比例坐标轴

### plot_joint_angles() - 关节角度

```python
from alicia_d_sdk.utils.vislab import plot_joint_angles
import numpy as np

# 录制关节角度
joint_trajectory = []
for i in range(100):
    state = robot.get_joints()
    joint_trajectory.append(state)
    time.sleep(0.1)

# 可视化
plot_joint_angles(
    np.array(joint_trajectory),
    title="Joint Angles During Tea Making"
)
```

---

## 方案 3: SparkVis Web 可视化（官方）⭐⭐⭐⭐⭐

Alicia-D 官方提供了 **SparkVis** - 基于 Web 的实时可视化工具。

### 特点
- ✅ 浏览器中的 3D 可视化
- ✅ 实时双向同步（UI ↔ 机械臂）
- ✅ 可以在网页上拖动机械臂，直接控制真机
- ✅ 记录数据到 CSV
- ✅ WebSocket 实时通信

### 使用流程

**注意**: SparkVis 是独立的项目，需要单独下载。

```bash
# 1. 下载 SparkVis（需要从官方获取）
# https://github.com/Synria-Robotics/SparkVis （待确认）

# 2. 启动 SparkVis 后端
cd SparkVis
python backend_server.py

# 3. 启动 Web 服务器
python -m http.server 8080

# 4. 启动机械臂桥接（需要真实硬件）
# 这个脚本在 Alicia-D-SDK 中
python examples/10_demo_sparkvis.py --port /dev/ttyUSB0

# 5. 打开浏览器
http://localhost:8080
```

### 与 TeaBot 集成

可以将 SparkVis 集成到 TeaBot：

```python
# 在 robot/alicia_controller.py 中添加
from alicia_d_sdk.execution.sparkvis import SparkVisBridge

class AliciaDualArmController:
    def enable_web_visualization(self):
        """启用 Web 可视化"""
        self.left_bridge = SparkVisBridge(
            robot=self.left_robot,
            host="localhost",
            port=8765,
            enable_robot_sync=True
        )
        self.left_bridge.start_server()
```

---

## 方案 4: RViz + ROS（不推荐 macOS）

Alicia-D 有 ROS1/ROS2 支持包，可以使用 RViz 可视化。

### 问题
- ❌ macOS 安装 ROS 困难
- ❌ 配置复杂
- ❌ 需要 Linux 环境

### 仓库
- ROS1: https://github.com/Synria-Robotics/Alicia-D-ROS1
- ROS2: https://github.com/Synria-Robotics/Alicia-D-ROS2

---

## 推荐使用方案

### 当前阶段（Mock 模式，无硬件）

**方案**: 使用 `tools/visualize_trajectory.py`

```bash
# 1. 定义动作序列
# 编辑 data/sequences/tea_making.json

# 2. 可视化验证
python tools/visualize_trajectory.py --file data/sequences/tea_making.json

# 3. 测试执行（Mock 模式）
python tools/execute_sequence.py data/sequences/tea_making.json
```

**优点**：
- 快速验证轨迹设计
- 发现碰撞风险
- 优化运动路径

---

### 连接真实硬件后

**方案 A**: 使用 SDK 内置的 `plot_3d()` 和 `plot_joint_angles()`

录制实际运行的轨迹并可视化：

```python
# tools/record_and_visualize.py
from alicia_d_sdk.utils.vislab import plot_3d
import time

# 连接机械臂
robot.connect()

# 录制轨迹
poses = []
for i in range(100):
    pose = robot.get_pose()
    poses.append([
        *pose['position'],
        *pose['quaternion_xyzw']
    ])
    time.sleep(0.1)

# 可视化
plot_3d([poses], legend="Recorded", show_ori=True)
```

**方案 B**: 启用 SparkVis Web 界面

实时查看和控制机械臂。

---

## 可视化对比

| 方案 | 难度 | 实时性 | 3D 效果 | 双臂支持 | macOS 支持 |
|------|------|--------|---------|----------|-----------|
| Matplotlib (内置) | ⭐ | ❌ | ⭐⭐⭐ | ✅ | ✅ |
| SDK plot_3d | ⭐⭐ | ❌ | ⭐⭐⭐⭐ | ✅ | ✅ |
| SparkVis Web | ⭐⭐⭐ | ✅ | ⭐⭐⭐⭐⭐ | ✅ | ✅ |
| RViz + ROS | ⭐⭐⭐⭐⭐ | ✅ | ⭐⭐⭐⭐⭐ | ✅ | ❌ |

---

## 下一步

### 立即可做

1. **运行演示**:
   ```bash
   python tools/visualize_trajectory.py --demo
   ```

2. **可视化你的序列**:
   ```bash
   python tools/visualize_trajectory.py --file data/sequences/demo.json
   ```

### 连接硬件后

1. **录制真实轨迹**:
   使用 SDK 的示教模式录制，然后用 `plot_3d()` 可视化

2. **启用 SparkVis**:
   获取 SparkVis 工具，实时查看机械臂状态

3. **集成到 FastAPI**:
   添加 `/visualization` 端点，从 API 触发可视化

---

## 问题排查

### Q: 运行可视化时没有图形窗口

A: 检查 Matplotlib 后端：
```python
import matplotlib
print(matplotlib.get_backend())
```

在 macOS 上应该是 `MacOSX` 或 `TkAgg`。

### Q: SparkVis 在哪里下载？

A: SparkVis 可能是：
- 独立的开源项目
- 随 Alicia-D 硬件提供
- 联系官方: support@synriarobotics.ai

### Q: 可以同时可视化双臂吗？

A: 可以！`plot_3d()` 支持多轨迹：
```python
plot_3d([left_traj, right_traj], legend="Arm")
```

---

**文档版本**: v1.0
**更新日期**: 2025-12-05
