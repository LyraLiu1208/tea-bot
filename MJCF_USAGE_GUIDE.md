# Alicia-D MJCF 模型使用指南

## 📁 可用的模型文件

项目现在包含3个 MJCF 模型文件：

### 1. `alicia_duo_with_gripper.xml` (单臂，需要 mesh 文件)
- **描述**: 官方提供的单臂 Alicia-D v5.5 模型
- **特点**: 使用真实的 STL mesh 文件，物理参数准确
- **要求**: 需要在 `data/meshes/Alicia-D_v5_4/` 目录下放置 STL 文件
- **用途**: 等待官方 mesh 文件后用于高精度仿真

### 2. `alicia_dual_arm.xml` (双臂，需要 mesh 文件)
- **描述**: 基于官方模型扩展的双臂版本
- **特点**: 左臂(蓝色)和右臂(红色)并排放置，间距 0.7m
- **要求**: 需要 mesh 文件
- **用途**: 完整的双臂奶茶制作仿真

### 3. `alicia_dual_arm_simple.xml` ✅ **推荐用于测试**
- **描述**: 使用简单几何体的双臂模型
- **特点**:
  - 不依赖外部 mesh 文件
  - 使用圆柱体、胶囊体、盒子等基本几何形状
  - 保持真实的运动学参数和关节限位
  - 可以立即运行
- **用途**: 开发、测试、演示

---

## 🚀 快速开始

### 方法 1: 使用简化模型（推荐）

```bash
# 激活虚拟环境
source venv/bin/activate

# 运行仿真（无 GUI，macOS headless 模式）
python tools/run_simulation.py data/models/alicia_dual_arm_simple.xml --demo

# 或者使用 mjpython 获得 GUI（需要先安装 mujoco 包）
mjpython tools/run_simulation.py data/models/alicia_dual_arm_simple.xml --demo
```

### 方法 2: 使用真实模型（需要 mesh 文件）

**第一步**: 获取 mesh 文件

联系 Synria Robotics 获取官方 mesh 文件：
- Email: support@synriarobotics.ai
- 询问: "Alicia-D v5.4 STL mesh files for MuJoCo simulation"

**第二步**: 放置 mesh 文件

```bash
# 创建目录
mkdir -p data/meshes/Alicia-D_v5_4

# 将以下 STL 文件复制到该目录：
# - base_link.STL
# - Link1.STL
# - Link2.STL
# - Link3.STL
# - Link4.STL
# - Link5.STL
# - Link6.STL
# - Grasp_base.STL
# - Link7.STL
# - Link8.STL
```

**第三步**: 运行仿真

```bash
source venv/bin/activate

# 单臂模型
python tools/run_simulation.py data/models/alicia_duo_with_gripper.xml --demo

# 双臂模型
python tools/run_simulation.py data/models/alicia_dual_arm.xml --demo
```

---

## 🎮 仿真控制

### 关节映射

双臂模型的关节顺序（`alicia_dual_arm_simple.xml`）：

| 索引 | 关节名 | 描述 |
|------|--------|------|
| 0-5 | left_Joint1-6 | 左臂 6 个旋转关节 |
| 6 | left_finger | 左臂左指（滑动关节）|
| 7 | left_right_finger | 左臂右指（滑动关节）|
| 8-13 | right_Joint1-6 | 右臂 6 个旋转关节 |
| 14 | right_left_finger | 右臂左指（滑动关节）|
| 15 | right_right_finger | 右臂右指（滑动关节）|

### Python API 使用

```python
from simulator import MuJoCoSimulator
from robot import ArmSide

# 初始化仿真器
sim = MuJoCoSimulator("data/models/alicia_dual_arm_simple.xml")
sim.start(gui=True)  # macOS 上会自动切换到 headless 模式

# 控制左臂关节
sim.set_joint_angles(ArmSide.LEFT, [0.5, 0.3, 0.2, 0, 0, 0])

# 控制夹爪（0=闭合, 100=完全打开）
sim.set_gripper(ArmSide.LEFT, 50)  # 打开 50%

# 推进仿真
for _ in range(1000):
    sim.step()

# 获取末端执行器位姿
pose = sim.get_end_effector_pose(ArmSide.LEFT)
print(f"Position: {pose['position']}")
print(f"Quaternion: {pose['quaternion']}")

# 停止仿真
sim.stop()
```

---

## 🔧 关节限位

根据真实 Alicia-D 规格：

| 关节 | 最小值 (rad) | 最大值 (rad) | 度数范围 |
|------|-------------|-------------|---------|
| Joint1 | -2.16 | 2.16 | ±124° |
| Joint2 | -1.57 | 1.57 | ±90° |
| Joint3 | -0.5 | 2.36 | -29° ~ 135° |
| Joint4 | -3.14 | 3.14 | ±180° |
| Joint5 | -1.57 | 1.5 | -90° ~ 86° |
| Joint6 | -3.14 | 3.14 | ±180° |

夹爪范围：
- `left_finger`: -0.05 ~ 0 m
- `right_finger`: 0 ~ 0.05 m

---

## 📊 模型信息

运行后可以看到模型详细信息：

```bash
python tools/run_simulation.py data/models/alicia_dual_arm_simple.xml
```

输出示例：
```
✓ Model loaded successfully
  Bodies: 18
  Joints: 16
  Actuators: 16
```

---

## 🐛 常见问题

### Q1: 提示 "requires mjpython on macOS"

**原因**: macOS 上 MuJoCo 的 GUI 查看器需要使用 `mjpython` 而不是普通的 `python`

**解决方案**:
1. 使用 headless 模式（自动启用）：
   ```bash
   python tools/run_simulation.py data/models/alicia_dual_arm_simple.xml --demo
   ```

2. 或安装并使用 `mjpython`：
   ```bash
   pip install mujoco
   mjpython tools/run_simulation.py data/models/alicia_dual_arm_simple.xml --demo
   ```

### Q2: 找不到 mesh 文件

**错误**: `Error loading mesh: meshes/...STL`

**解决方案**:
- 使用简化模型：`alicia_dual_arm_simple.xml`
- 或从官方获取 STL 文件并放置到 `data/meshes/Alicia-D_v5_4/`

### Q3: 夹爪控制不对

**检查**:
```python
# 确认使用正确的 API
sim.set_gripper(ArmSide.LEFT, 0)    # 完全闭合
sim.set_gripper(ArmSide.LEFT, 100)  # 完全打开
```

夹爪内部有两个独立的滑动关节（左指和右指），`set_gripper` 会自动控制两者。

---

## 🎯 下一步

1. **无 mesh 文件**: 使用 `alicia_dual_arm_simple.xml` 进行开发和测试
2. **有 mesh 文件**: 使用 `alicia_dual_arm.xml` 进行高精度仿真
3. **集成到工作流**: 在开发动作序列时实时查看仿真
4. **碰撞检测**: 使用 `sim.has_collision()` 检测双臂是否相互碰撞

---

## 📝 模型差异对比

| 特性 | simple | duo/dual_arm |
|------|--------|--------------|
| Mesh 依赖 | ❌ 无需 | ✅ 需要 STL |
| 视觉精度 | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 物理精度 | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 立即可用 | ✅ 是 | ❌ 需要 mesh |
| 适用场景 | 开发/测试 | 最终仿真 |

---

**文档版本**: v1.0
**更新日期**: 2025-12-05
**联系方式**: support@synriarobotics.ai
