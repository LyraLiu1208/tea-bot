# TeaBot - 机械臂控制系统

基于 Alicia-D 机械臂的控制系统，支持单臂/双臂、真实硬件/MuJoCo 仿真。

## 🎯 当前开发重点

**优先验证单臂功能** - 配置在 `config/robot_config.yaml`:
- `arm_mode: single_arm` - 单臂模式（当前）
- `arm_mode: dual_arm` - 双臂模式（后续）

## 🚀 快速开始

### 单臂仿真测试 (Mac)

```bash
# 使用单臂模型
python test_real_arm.py --mode mujoco
# 或
python tools/run_simulation.py --single-arm
```

### 真机测试 (Linux)

```bash
source venv/bin/activate
python test_real_arm.py --mode real
```

---

## ✨ 主要特性

- ✅ **单臂/双臂支持**: 优先验证单臂，后续扩展双臂
- ✅ **双运行模式**: Real (真实硬件) + MuJoCo (物理仿真)
- ✅ **平滑运动**: 大角度轨迹规划 (`move_joint_smooth`)
- ✅ **REST API**: FastAPI 接口
- ✅ **交互式测试**: CLI 工具

---

## 📁 项目结构

```
tea-bot/
├── robot/                  # 控制器模块
│   ├── base_controller.py  # 抽象基类 + move_joint_smooth
│   ├── alicia_controller.py # Real 模式 (Alicia-D SDK)
│   └── mujoco_controller.py # MuJoCo 仿真模式
├── simulator/              # MuJoCo 仿真器
├── api/                    # FastAPI REST 接口
├── config/
│   └── robot_config.yaml   # 主配置 (arm_mode: single_arm/dual_arm)
├── data/models/
│   ├── alicia_duo_with_gripper.xml  # 单臂模型（当前）
│   ├── alicia_dual_arm.xml          # 双臂模型（后续）
│   └── meshes/Alicia-D_v5_4/        # STL 网格文件
├── tools/
│   ├── run_simulation.py   # 仿真启动 (--single-arm/--dual-arm)
│   └── test_serial_connection.py  # 串口测试
└── test_real_arm.py        # 交互式测试 (Real/MuJoCo)
```

---

## 🔧 硬件配置 (Real 模式)

**已验证配置**:
```yaml
port: ""              # 自动搜索或 /dev/ttyACM0
baudrate: 1000000     # ✅ 已验证
```

**Linux 权限**:
```bash
sudo usermod -a -G dialout $USER  # 注销重新登录
python tools/test_serial_connection.py  # 测试连接
```

---

## 📖 核心 API

### move_joint() - 直接运动 (小角度)

```python
controller.move_joint(ArmSide.LEFT, [0, 0.3, 0.5, 0, 0, 0], wait=True)
```

### move_joint_smooth() - 平滑运动 (大角度) ⭐

```python
import math
target = [0, math.radians(90), math.radians(90), 0, 0, 0]
controller.move_joint_smooth(ArmSide.LEFT, target, steps=30)
```

### 其他常用 API

```python
controller.set_home(ArmSide.LEFT)              # 回 Home
controller.control_gripper(ArmSide.LEFT, 50)   # 夹爪控制
state = controller.get_state(ArmSide.LEFT)     # 获取状态
controller.emergency_stop()                     # 紧急停止
```

---

## 🧪 测试工具

### test_real_arm.py - 交互式测试

```bash
python test_real_arm.py --mode real  # 或 mujoco

# 菜单 (核心基础能力):
# 基础运动:
#   1. 回到 Home 位置 (平滑)
#   2. 预设位置测试 (轻微弯曲)
#   3. 自定义关节角度 (梯形轨迹)
#
# 笛卡尔空间:
#   4. 移动到笛卡尔位置 (XYZ)
#
# 夹爪:
#   5. 夹爪开合测试
#
# 状态与安全:
#   6. 查看当前状态
#   7. 紧急停止 (立即停止当前动作)
#
# 说明:
# - 真机模式会提示安全注意事项，所有输入均自动执行，需准备物理急停。
# - 关节/笛卡尔目标会在执行前校验配置的安全限位。
```

### tools/run_simulation.py - 仿真

```bash
python tools/run_simulation.py --single-arm     # 单臂
python tools/run_simulation.py --dual-arm       # 双臂
python tools/run_simulation.py --demo           # 演示模式
```

---

## 🔍 配置说明

编辑 `config/robot_config.yaml`:

```yaml
# 运行模式
mode: mujoco  # real / mujoco

# 机械臂配置
arm_mode: single_arm  # single_arm / dual_arm

# MuJoCo 配置
mujoco:
  single_arm_model: data/models/alicia_duo_with_gripper.xml
  dual_arm_model: data/models/alicia_dual_arm.xml
  gui: true
  timestep: 0.002
  pose_steps: 80
  pose_step_delay: 0.006

# Real 模式配置
real:
  left_arm:
    port: ""  # /dev/ttyACM0
    baudrate: 1000000
    robot_version: v5_6
    gripper_type: 50mm
    speed_deg_s: 20.0
  wait_timeout: 6.0
  wait_poll_interval: 0.05
  wait_tolerance_deg: 3.0
  pose_control:
    backend: numpy
    method: dls
    tolerance: 0.0005
    max_iters: 200
    speed_factor: 1.0
    execute: true

# 安全限位
safety:
  joint_limits:
    - [-2.16, 2.16]
    - [-1.57, 1.57]
    - [-0.5, 2.36]
    - [-3.14, 3.14]
    - [-1.57, 1.5]
    - [-3.14, 3.14]
  workspace_limits:
    x: [0.15, 0.60]
    y: [-0.25, 0.25]
    z: [0.10, 0.80]
```
```

---

## 🛠️ 安装

```bash
# 创建虚拟环境
python3 -m venv venv
source venv/bin/activate

# 基础依赖
pip install -r requirements.txt

# Real 模式
pip install alicia-d-sdk

# MuJoCo 仿真
pip install -r requirements_sim.txt
```

---

## 📝 开发状态

**已完成**:
- ✅ 单臂/双臂配置分离
- ✅ 串口连接验证 (ttyACM0, 1Mbps)
- ✅ 平滑运动 API (`move_joint_smooth`)
- ✅ 单臂仿真模型 (竖直底座, 0.8m 高度)
- ✅ 奶茶机器人基础功能:
  - 夹爪测试序列
  - 抓取-移动-放置工作流
  - 倾倒动作 (手腕旋转)
  - 搅拌动作 (循环路径)

**进行中**:
- ⏳ 笛卡尔空间直线运动 (需逆运动学)
- ⏳ 轨迹录制/回放功能 (示教模式)
- ⏳ 真机硬件测试验证

**待开发**:
- ⏳ 双臂协同功能
