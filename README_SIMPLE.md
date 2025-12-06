# TeaBot - 双臂奶茶机器人控制系统

基于 Alicia-D 双臂机械臂的奶茶制作机器人，支持两种运行模式。

---

## 🎯 运行模式

| 模式 | 说明 | 用途 |
|------|------|------|
| **real** | 真实硬件控制 | 生产运行 |
| **mujoco** | 物理仿真 | 开发、验证动作序列 |

---

## ⚡ 快速开始

### 1. 安装依赖

```bash
# 创建虚拟环境
python3 -m venv venv
source venv/bin/activate

# 安装基础依赖
pip install -r requirements.txt

# 如需仿真，安装 MuJoCo
pip install -r requirements_sim.txt

# 如需真机，安装 Alicia-D SDK
pip install alicia-d-sdk
```

### 2. 配置模式

编辑 `config/robot_config.yaml`:

```yaml
# 选择运行模式
mode: mujoco  # 或 real
```

### 3. 启动服务

```bash
python run.py
```

API 地址: http://localhost:8000

---

## 📡 API 使用

### 发送任务

```python
import requests

task = {
    "task_id": "order_001",
    "actions": [
        {
            "type": "move_joint",
            "arm": "left",
            "params": {"joint_angles": [0, 0, 0, 0, 0, 0]}
        },
        {
            "type": "control_gripper",
            "arm": "left",
            "params": {"value": 50}
        }
    ]
}

response = requests.post("http://localhost:8000/task", json=task)
print(response.json())
```

### 动作类型

| 类型 | 参数 | 示例 |
|------|------|------|
| `move_joint` | `joint_angles` (6个角度，弧度) | `[0.5, 0, 0, 0, 0, 0]` |
| `move_pose` | `pose` ([x,y,z,qx,qy,qz,qw]) | `[0.3, 0.2, 0.4, 0,0,0,1]` |
| `control_gripper` | `value` (0-100) | `50` |
| `wait` | `duration` (秒) | `2.0` |

---

## 🏠 Home 位置

双臂底座垂直于地面且相对，Home 位置为：
- **关节角度**: 全部为 0（垂直姿态）
- **夹爪**: 半开状态 (50)

在 `config/robot_config.yaml` 中定义：

```yaml
home_position:
  joint_angles: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  gripper: 50
```

---

## 🔧 模式配置

### Real 模式（真实硬件）

```yaml
mode: real
real:
  left_arm:
    port: ""  # 留空自动搜索
    baudrate: 1000000
    robot_version: v5_6
    gripper_type: 50mm
    speed_deg_s: 20.0
  right_arm:
    port: ""
    baudrate: 1000000
    robot_version: v5_6
    gripper_type: 50mm
    speed_deg_s: 20.0
```

### MuJoCo 模式（物理仿真）

```yaml
mode: mujoco
mujoco:
  model_path: data/models/alicia_dual_arm.xml
  timestep: 0.002
  gui: true  # macOS 需要 mjpython
```

---

## 🐍 Python API

### 直接使用控制器

```python
import yaml
from robot import ArmSide

# 加载配置
with open("config/robot_config.yaml") as f:
    config = yaml.safe_load(f)

# 根据配置创建控制器
if config["mode"] == "real":
    from robot import AliciaDualArmController
    controller = AliciaDualArmController(config)
elif config["mode"] == "mujoco":
    from robot import MuJoCoController
    controller = MuJoCoController(config)

# 连接
controller.connect()

# 回到 Home 位置
controller.set_home(ArmSide.LEFT)
controller.set_home(ArmSide.RIGHT)

# 移动关节
controller.move_joint(ArmSide.LEFT, [0.5, 0, 0, 0, 0, 0])

# 控制夹爪
controller.control_gripper(ArmSide.LEFT, 80)

# 获取状态
state = controller.get_state(ArmSide.LEFT)
print(f"Joints: {state.joint_angles}")
print(f"Gripper: {state.gripper_value}")

# 断开连接
controller.disconnect()
```

### 切换模式

只需修改配置文件中的 `mode` 字段，代码无需改动：

```python
# 统一的接口
if config["mode"] == "real":
    from robot import AliciaDualArmController
    controller = AliciaDualArmController(config)
elif config["mode"] == "mujoco":
    from robot import MuJoCoController
    controller = MuJoCoController(config)

# 所有控制器使用相同的 API
controller.connect()
controller.set_home(ArmSide.LEFT)
controller.move_joint(ArmSide.LEFT, [0, 0, 0, 0, 0, 0])
controller.disconnect()
```

---

## 🎮 MuJoCo 仿真

### 启动仿真（GUI）

macOS 需要使用 `mjpython`:

```bash
mjpython -m robot.mujoco_controller
```

或设置配置文件 `mujoco.gui: false` 使用 headless 模式。

### 模型文件

- **双臂模型**: `data/models/alicia_dual_arm.xml`
  - 统一使用此模型进行仿真
  - 包含完整的双臂机械臂结构
  - 如需 STL mesh 文件，联系官方: support@synriarobotics.ai

---

## 🔐 安全限位

基于 Alicia-D v5.6 规格（配置文件中定义）：

| Joint | 范围 (rad) | 范围 (度) |
|-------|-----------|----------|
| 1 | ±2.16 | ±124° |
| 2 | ±1.57 | ±90° |
| 3 | -0.5 ~ 2.36 | -29° ~ 135° |
| 4 | ±3.14 | ±180° |
| 5 | -1.57 ~ 1.5 | -90° ~ 86° |
| 6 | ±3.14 | ±180° |

**夹爪**: 0-100 (0=闭合, 100=打开)

---

## 📂 项目结构

```
tea-bot/
├── api/                # FastAPI 后端
│   ├── main.py        # 主应用
│   └── models.py      # 数据模型
├── robot/              # 控制器
│   ├── base_controller.py      # 抽象接口
│   ├── mock_controller.py      # Mock 模式
│   ├── alicia_controller.py    # Real 模式
│   └── mujoco_controller.py    # MuJoCo 模式
├── simulator/          # 仿真器
│   └── mujoco_simulator.py
├── config/             # 配置文件
│   └── robot_config.yaml
├── data/
│   └── models/        # MuJoCo 模型
└── run.py             # 启动脚本
```

---

## 🚀 常用命令

```bash
# 启动 API 服务
python run.py

# 测试连接 (需自己实现或使用 Python API)
python
>>> from robot import MockDualArmController
>>> controller = MockDualArmController({...})
>>> controller.connect()

# 查看 API 文档
# 打开浏览器访问 http://localhost:8000/docs
```

---

## ❓ 常见问题

### Q: 如何切换模式？

A: 编辑 `config/robot_config.yaml`，修改 `mode` 字段为 `real` 或 `mujoco`。

### Q: MuJoCo 没有窗口？

A: macOS 上使用 `mjpython` 而不是 `python`，或设置 `mujoco.gui: false`。

### Q: 真机如何连接？

A: 将 `real.left_arm.port` 和 `real.right_arm.port` 设为空字符串自动搜索，或指定串口如 `/dev/ttyUSB0`。

### Q: 如何定义动作序列？

A: 创建 JSON 文件，使用 POST /task 接口发送：

```json
{
  "task_id": "make_tea",
  "actions": [
    {"type": "move_joint", "arm": "left", "params": {"joint_angles": [...]}},
    {"type": "control_gripper", "arm": "left", "params": {"value": 80}},
    {"type": "wait", "params": {"duration": 1.0}}
  ]
}
```

---

## 📖 更多文档

- **完整指南**: [COMPLETE_GUIDE.md](COMPLETE_GUIDE.md)
- **MuJoCo 仿真**: [MUJOCO_SIMULATION.md](MUJOCO_SIMULATION.md)
- **快速参考**: [QUICK_REFERENCE.md](QUICK_REFERENCE.md)

---

**版本**: v0.2.0
**更新**: 2025-12-05
**联系**: support@synriarobotics.ai
