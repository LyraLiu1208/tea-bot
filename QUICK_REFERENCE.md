# TeaBot 快速参考

## 🎯 项目概览

**TeaBot** - 基于 Alicia-D 双臂机械臂的奶茶制作机器人控制系统

- **AI Agent 同事**: 处理订单，生成任务 JSON
- **你的部分**: 机器人控制、动作执行、仿真

---

## 📁 项目结构

```
tea-bot/
├── api/                    # FastAPI 后端
│   └── main.py            # 接收 AI Agent 的 POST 请求
├── robot/                 # 机器人控制器
│   ├── base_controller.py # 抽象接口
│   ├── mock_controller.py # Mock 模式（开发用）✅
│   └── alicia_controller.py # 真实硬件控制
├── simulator/             # MuJoCo 仿真
│   └── mujoco_simulator.py # 物理仿真器 ✅
├── tools/                 # 辅助工具
│   ├── run_simulation.py  # 启动仿真
│   ├── visualize_trajectory.py # 轨迹可视化
│   ├── read_state.py      # 读取机器人状态
│   └── execute_sequence.py # 执行动作序列
├── data/
│   ├── models/            # MJCF 模型文件
│   │   ├── alicia_dual_arm_simple.xml ⭐ 立即可用
│   │   ├── alicia_dual_arm.xml (需要 mesh)
│   │   └── alicia_duo_with_gripper.xml (需要 mesh)
│   ├── sequences/         # 动作序列 JSON
│   └── trajectories/      # 轨迹数据
└── config/
    └── robot_config.yaml  # 配置文件
```

---

## ⚡ 常用命令

### 环境管理

```bash
# 激活虚拟环境
source venv/bin/activate

# 安装依赖
pip install -r requirements.txt        # 基础依赖
pip install -r requirements_sim.txt    # 仿真依赖 (已安装 MuJoCo ✅)
```

### 启动服务

```bash
# 启动 FastAPI 服务器 (Mock 模式)
python run.py

# 或指定端口
uvicorn api.main:app --reload --port 8000
```

API 地址: http://localhost:8000
文档地址: http://localhost:8000/docs

### 运行仿真

```bash
# 使用简化模型（推荐）
python tools/run_simulation.py data/models/alicia_dual_arm_simple.xml --demo

# 使用真实模型（需要 mesh 文件）
python tools/run_simulation.py data/models/alicia_dual_arm.xml --demo

# 使用 mjpython 获得 GUI（macOS）
mjpython tools/run_simulation.py data/models/alicia_dual_arm_simple.xml --demo
```

### 可视化轨迹

```bash
# 演示模式
python tools/visualize_trajectory.py --demo

# 可视化自定义序列
python tools/visualize_trajectory.py data/sequences/demo.json
```

### 测试连接

```bash
# 测试控制器连接
python tools/test_connection.py

# 读取机器人状态
python tools/read_state.py --arm left
python tools/read_state.py --arm both --continuous

# 执行动作序列
python tools/execute_sequence.py data/sequences/demo.json
```

---

## 🔧 配置切换

### Mock 模式（当前） ✅

编辑 `config/robot_config.yaml`:

```yaml
mode: mock  # 模拟模式，无需硬件
```

### 真实硬件模式

```yaml
mode: real
left_arm:
  port: "/dev/tty.usbserial-左臂序列号"
  baudrate: 1000000
right_arm:
  port: "/dev/tty.usbserial-右臂序列号"
  baudrate: 1000000
```

---

## 🎮 Python API 使用

### Mock 控制器

```python
from robot import MockDualArmController, ArmSide

# 初始化
controller = MockDualArmController(config)
controller.connect()

# 移动关节
controller.move_joint(ArmSide.LEFT, [0.5, 0.3, 0.2, 0, 0, 0])

# 移动到位姿
controller.move_pose(ArmSide.LEFT, [0.3, 0.2, 0.4, 0, 0, 0])

# 控制夹爪 (0-100)
controller.control_gripper(ArmSide.LEFT, 50)

# 获取状态
state = controller.get_state(ArmSide.LEFT)
print(f"Joints: {state.joint_angles}")
print(f"Gripper: {state.gripper_value}")
```

### MuJoCo 仿真器

```python
from simulator import MuJoCoSimulator
from robot import ArmSide

# 初始化
sim = MuJoCoSimulator("data/models/alicia_dual_arm_simple.xml")
sim.start(gui=True)  # macOS 会自动 headless

# 设置关节
sim.set_joint_angles(ArmSide.LEFT, [0.5, 0, 0, 0, 0, 0])

# 设置夹爪
sim.set_gripper(ArmSide.LEFT, 50)

# 推进仿真
for _ in range(1000):
    sim.step()

# 获取末端位姿
pose = sim.get_end_effector_pose(ArmSide.LEFT)

# 检测碰撞
if sim.has_collision():
    print("碰撞!")

sim.stop()
```

### FastAPI 端点

```python
import requests

# 提交任务
task = {
    "task_id": "order_001",
    "actions": [
        {
            "type": "move_joint",
            "arm": "left",
            "params": {"joint_angles": [0.5, 0, 0, 0, 0, 0], "speed": 50}
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

---

## 📊 动作类型

| 类型 | 参数 | 示例 |
|------|------|------|
| `move_joint` | `joint_angles`, `speed` | `[0.5, 0, 0, 0, 0, 0]` |
| `move_pose` | `pose`, `speed` | `[x, y, z, rx, ry, rz]` |
| `control_gripper` | `value` | `0-100` |
| `wait` | `duration` | 秒数 |

---

## 🎯 关节限位（rad）

| Joint | Min | Max | 度数 |
|-------|-----|-----|------|
| 1 | -2.16 | 2.16 | ±124° |
| 2 | -1.57 | 1.57 | ±90° |
| 3 | -0.5 | 2.36 | -29°~135° |
| 4 | -3.14 | 3.14 | ±180° |
| 5 | -1.57 | 1.5 | -90°~86° |
| 6 | -3.14 | 3.14 | ±180° |

---

## 📚 文档索引

### 快速开始
- [README.md](README.md) - 项目概览
- [QUICKSTART.md](QUICKSTART.md) - 快速开始
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - 本文档 ⭐

### 详细指南
- [COMPLETE_GUIDE.md](COMPLETE_GUIDE.md) - 完整开发指南
- [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) - 项目总结
- [ENVIRONMENT_SETUP.md](ENVIRONMENT_SETUP.md) - 环境设置

### 仿真相关
- [MUJOCO_SIMULATION.md](MUJOCO_SIMULATION.md) - MuJoCo 仿真指南
- [MJCF_USAGE_GUIDE.md](MJCF_USAGE_GUIDE.md) - MJCF 模型使用 ⭐
- [MJCF_INTEGRATION_SUMMARY.md](MJCF_INTEGRATION_SUMMARY.md) - MJCF 集成总结
- [VISUALIZATION.md](VISUALIZATION.md) - 可视化方案

---

## 🐛 常见问题快速解决

### 问题: MuJoCo 提示需要 mjpython

**现象**: `RuntimeError: requires mjpython on macOS`

**解决**:
- 正常现象，会自动切换到 headless 模式
- 仿真仍在运行，只是看不到窗口
- 需要 GUI 时使用: `mjpython tools/run_simulation.py ...`

### 问题: 找不到 mesh 文件

**现象**: `Error loading mesh`

**解决**:
```bash
# 使用简化模型
python tools/run_simulation.py data/models/alicia_dual_arm_simple.xml --demo
```

### 问题: FastAPI 启动失败

**检查**:
```bash
# 确保在虚拟环境中
source venv/bin/activate

# 检查依赖
pip install -r requirements.txt
```

### 问题: 端口被占用

**解决**:
```bash
# 使用不同端口
uvicorn api.main:app --port 8001
```

---

## ✅ 当前可用功能

- ✅ Mock 双臂控制器
- ✅ FastAPI 后端服务
- ✅ MuJoCo 物理仿真（headless 模式）
- ✅ 简化双臂模型（无需外部文件）
- ✅ Matplotlib 轨迹可视化
- ✅ 动作序列执行
- ✅ 状态读取工具

---

## ⏳ 等待/TODO

- ⏳ 获取官方 STL mesh 文件
- ⏳ 真实硬件连接测试
- ⏳ 与 AI Agent 联调
- ⏳ 定义奶茶制作动作序列
- ⏳ 碰撞检测优化

---

## 📞 获取帮助

### Mesh 文件
- **联系**: support@synriarobotics.ai
- **询问**: "Alicia-D v5.4 STL mesh files for MuJoCo simulation"

### SDK 相关
- **官方文档**: https://docs.sparklingrobo.com/
- **GitHub**: https://github.com/Synria-Robotics/

---

**更新日期**: 2025-12-05
**项目状态**: ✅ 开发就绪，等待硬件和 AI Agent 集成
