# MuJoCo 仿真指南

## 概述

TeaBot 现已集成 **MuJoCo** 物理仿真引擎，提供高性能的双臂机械臂仿真。

### 为什么选择 MuJoCo？

- ✅ **最先进的物理引擎** - 由 DeepMind 开发，用于 AI 训练
- ✅ **高性能** - 比 Gazebo/PyBullet 快 10-100 倍
- ✅ **精确** - 接触力学、摩擦力建模
- ✅ **macOS 友好** - 原生支持
- ✅ **美观的可视化** - 现代 OpenGL 渲染
- ✅ **免费开源** - 自 2021 年起

---

## 安装

### 1. 安装 MuJoCo

```bash
# 激活虚拟环境
source venv/bin/activate

# 安装 MuJoCo
pip install -r requirements_sim.txt
```

### 2. 验证安装

```bash
python -c "import mujoco; print(mujoco.__version__)"
```

应该输出版本号，如 `3.1.0`。

---

## 准备模型文件

### 获取 Alicia-D 的 MJCF 模型

你需要 Alicia-D 的 MJCF（MuJoCo 格式）模型文件。

#### 方法 1: 从官方获取（推荐）

联系 Synria Robotics:
- Email: support@synriarobotics.ai
- 询问: "Alicia-D MJCF model for MuJoCo simulation"
- 可能随 `synriard` 包提供

#### 方法 2: 从 URDF 转换

如果你有 URDF（从 ROS 包或 SDK）：

```bash
# 安装 MuJoCo 的 compile 工具（已随 mujoco 包安装）
python -c "import mujoco; print(mujoco.MjModel.from_xml_path('path/to/alicia_d.urdf'))"
```

或使用命令行工具：

```bash
# MuJoCo 提供的 compile 工具
compile path/to/alicia_d.urdf alicia_d_dual_arm.xml
```

#### 方法 3: 从 synriard 包提取

如果你安装了 `synriard`：

```python
from synriard import get_model_path

urdf_path = get_model_path("Alicia_D", version="v5_6", variant="gripper_50mm")
# 转换为 MJCF
```

### 模型文件放置

将模型文件放在：

```
data/models/
├── alicia_d_dual_arm.xml     # 主 MJCF 文件
├── meshes/                    # 网格文件
│   ├── left_base.stl
│   ├── left_link1.stl
│   └── ...
└── textures/                  # 纹理（可选）
```

---

## 使用方法

### 基础使用

```bash
# 启动仿真（带 GUI）
python tools/run_simulation.py data/models/alicia_d_dual_arm.xml
```

### 演示模式

```bash
# 运行预定义的演示动作
python tools/run_simulation.py data/models/alicia_d_dual_arm.xml --demo
```

### 不同步 Mock 控制器

```bash
# 只显示模型，不同步控制器
python tools/run_simulation.py data/models/alicia_d_dual_arm.xml --no-sync
```

---

## Python API 使用

### 基础示例

```python
from simulator import MuJoCoSimulator
from robot import ArmSide

# 创建仿真器
simulator = MuJoCoSimulator("data/models/alicia_d_dual_arm.xml")

# 启动（带 GUI）
simulator.start(gui=True)

# 设置关节角度
simulator.set_joint_angles(ArmSide.LEFT, [0.5, 0.3, 0.0, 0.0, 0.0, 0.0])

# 推进仿真
for _ in range(1000):
    simulator.step()

# 获取末端位姿
pose = simulator.get_end_effector_pose(ArmSide.LEFT)
print(f"Position: {pose['position']}")
print(f"Quaternion: {pose['quaternion']}")

# 停止
simulator.stop()
```

### 与 Mock 控制器同步

```python
from simulator import MuJoCoSimulator
from robot import MockDualArmController, ArmSide

# 创建控制器和仿真器
controller = MockDualArmController(config)
controller.connect()

simulator = MuJoCoSimulator("data/models/alicia_d_dual_arm.xml")
simulator.start()

# 控制器执行动作
controller.move_joint(ArmSide.LEFT, [0.5, 0, 0, 0, 0, 0], wait=False)

# 同步到仿真
while True:
    simulator.sync_from_controller(controller)
    simulator.step()
```

### 碰撞检测

```python
# 检测碰撞
if simulator.has_collision():
    print("⚠ Collision detected!")
    contacts = simulator.get_contact_info()
    for c in contacts:
        print(f"  Geom {c['geom1']} ↔ Geom {c['geom2']}")
        print(f"  Distance: {c['dist']:.4f}")
```

---

## 配置调整

### 关节映射

如果模型的关节顺序与默认不同，需要调整映射：

```python
# 查看模型信息
simulator.print_model_info()

# 输出示例：
# --- Joint Names ---
#   [0] left_shoulder
#   [1] left_elbow
#   ...

# 更新映射
new_mapping = {
    ArmSide.LEFT: {
        "joints": [0, 1, 2, 3, 4, 5],  # 根据实际 ID 调整
        "gripper": 6
    },
    ArmSide.RIGHT: {
        "joints": [7, 8, 9, 10, 11, 12],
        "gripper": 13
    }
}
simulator.update_joint_mapping(new_mapping)
```

### 末端执行器名称

```python
# 如果模型中的 body 名称不同
new_names = {
    ArmSide.LEFT: "left_ee_link",
    ArmSide.RIGHT: "right_ee_link"
}
simulator.update_ee_body_names(new_names)
```

---

## 与 FastAPI 集成

### 添加仿真端点

在 `api/main.py` 中添加：

```python
from simulator import MuJoCoSimulator

# 全局仿真器实例
sim = None

@app.post("/simulation/start")
async def start_simulation(model_path: str):
    """启动仿真"""
    global sim
    sim = MuJoCoSimulator(model_path)
    success = sim.start(gui=True)
    return {"success": success}

@app.post("/simulation/sync")
async def sync_simulation():
    """同步控制器状态到仿真"""
    global sim, controller
    if sim and controller:
        sim.sync_from_controller(controller)
        return {"message": "Synced"}
    return {"error": "Simulator or controller not initialized"}
```

---

## 高级功能

### 1. 轨迹录制

```python
# 录制仿真中的轨迹
trajectory = []

for _ in range(1000):
    simulator.step()

    # 记录状态
    left_pose = simulator.get_end_effector_pose(ArmSide.LEFT)
    trajectory.append(left_pose['position'])

# 保存轨迹
import json
with open('data/trajectories/recorded.json', 'w') as f:
    json.dump(trajectory, f)
```

### 2. 力反馈

```python
# 获取接触力
contacts = simulator.get_contact_info()
for contact in contacts:
    force = contact['force']
    print(f"Contact force: {force}")
```

### 3. 传感器数据

MuJoCo 支持各种传感器：

```python
# 在 MJCF 中定义传感器
<sensor>
  <force name="left_force" site="left_ee_site"/>
  <torque name="left_torque" site="left_ee_site"/>
</sensor>

# Python 中读取
force_data = simulator.data.sensordata[0:3]  # 力传感器
torque_data = simulator.data.sensordata[3:6]  # 力矩传感器
```

---

## 故障排除

### 问题 1: 模型加载失败

**错误**: `Error: unknown element 'xxx'`

**解决**:
- 检查 MJCF 语法
- 确保所有 mesh 文件路径正确
- 使用 MuJoCo 的 `compile` 工具验证

### 问题 2: 可视化窗口不显示

**原因**: macOS 的权限问题

**解决**:
```bash
# 确保 Python 有屏幕录制权限
# 系统偏好设置 > 安全性与隐私 > 屏幕录制
```

### 问题 3: 关节不动

**原因**: 关节映射错误

**解决**:
```python
# 打印模型信息
simulator.print_model_info()

# 根据输出调整 joint_mapping
```

### 问题 4: 性能慢

**优化**:
```python
# 降低仿真频率
time.sleep(1.0 / 30.0)  # 30Hz 而不是 60Hz

# 或减少仿真步数
simulator.step(2)  # 每次推进 2 步
```

---

## 与其他工具对比

| 特性 | MuJoCo | PyBullet | Gazebo |
|------|--------|----------|--------|
| 速度 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |
| 精度 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| macOS | ✅ | ✅ | ❌ |
| 可视化 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ |
| 易用性 | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐ |
| AI 集成 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ |

---

## 下一步

1. **获取 MJCF 模型** - 联系官方或转换 URDF
2. **调整映射** - 根据模型调整关节映射
3. **测试仿真** - 运行演示模式
4. **集成到工作流** - 在开发时同步仿真查看

---

## 资源

- [MuJoCo 官方文档](https://mujoco.readthedocs.io/)
- [MuJoCo Python 绑定](https://github.com/deepmind/mujoco/tree/main/python)
- [MJCF 格式说明](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [示例模型](https://github.com/deepmind/mujoco/tree/main/model)

---

**文档版本**: v1.0
**更新日期**: 2025-12-05
