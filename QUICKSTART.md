# TeaBot 快速开始

## 安装依赖

```bash
cd tea-bot
pip install -r requirements.txt
```

## 运行模式

### Mock 模式（默认）

无需硬件连接，用于开发和测试。

1. 启动 API 服务：

```bash
python run.py
```

2. 访问 API 文档：http://localhost:8000/docs

3. 测试连接：

```bash
python tools/test_connection.py
```

### 真实硬件模式

连接 Alicia D 双臂机械臂。

1. 安装 Alicia-D SDK：

```bash
pip install git+https://github.com/Synria-Robotics/Alicia-D-SDK.git
```

2. 修改配置文件 `config/robot_config.yaml`：

```yaml
mode: real  # 从 mock 改为 real

real:
  left_arm:
    port: /dev/tty.usbserial-left  # 修改为实际串口
  right_arm:
    port: /dev/tty.usbserial-right
```

3. 启动服务：

```bash
python run.py
```

## API 使用示例

### 发送任务

```bash
curl -X POST http://localhost:8000/task \
  -H "Content-Type: application/json" \
  -d '{
    "task_id": "task-001",
    "actions": [
      {
        "type": "set_home",
        "arm": "left",
        "params": {},
        "wait": true
      },
      {
        "type": "move_joint",
        "arm": "left",
        "params": {
          "joints": [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
        },
        "wait": true
      },
      {
        "type": "control_gripper",
        "arm": "left",
        "params": {
          "value": 50.0
        },
        "wait": true
      }
    ]
  }'
```

### 查询状态

```bash
curl http://localhost:8000/status
```

### 紧急停止

```bash
curl -X POST http://localhost:8000/stop
```

## 辅助工具使用

### 1. 测试连接

```bash
python tools/test_connection.py
```

### 2. 读取机械臂状态

单次读取并保存：
```bash
python tools/read_state.py --arm both --save data/states/current.json
```

连续读取：
```bash
python tools/read_state.py --arm left --continuous
```

### 3. 执行动作序列

预览模式（不执行）：
```bash
python tools/execute_sequence.py data/sequences/demo.json --dry-run
```

实际执行：
```bash
python tools/execute_sequence.py data/sequences/demo.json
```

## 动作类型

支持的动作类型（`type` 字段）：

1. **move_joint** - 关节空间运动
   ```json
   {
     "type": "move_joint",
     "arm": "left",
     "params": {
       "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  // 6个关节角度（弧度）
     },
     "wait": true
   }
   ```

2. **move_pose** - 笛卡尔空间运动
   ```json
   {
     "type": "move_pose",
     "arm": "right",
     "params": {
       "pose": [0.3, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0]  // [x,y,z,qx,qy,qz,qw]
     },
     "wait": true
   }
   ```

3. **control_gripper** - 控制夹爪
   ```json
   {
     "type": "control_gripper",
     "arm": "left",
     "params": {
       "value": 50.0  // 0(闭合) - 100(张开)
     },
     "wait": true
   }
   ```

4. **set_home** - 回到初始位置
   ```json
   {
     "type": "set_home",
     "arm": "left",
     "params": {},
     "wait": true
   }
   ```

5. **enable_torque** - 启用/禁用力矩（示教模式）
   ```json
   {
     "type": "enable_torque",
     "arm": "right",
     "params": {
       "enable": false  // false=禁用力矩（可手动拖动）
     }
   }
   ```

## 与 AI Agent 对接

你的同事只需要 POST JSON 到 `/task` 端点即可：

```python
import requests

# AI Agent 生成的订单
order = {
    "task_id": "order-001",
    "actions": [
        # 动作序列...
    ]
}

# 发送到机械臂后端
response = requests.post(
    "http://localhost:8000/task",
    json=order
)

print(response.json())
```

## 下一步

1. **录制动作**：使用 `tools/read_state.py` 在真实硬件上录制关键点位
2. **定义配方**：创建具体任务的动作序列 JSON 文件
3. **集成测试**：与 AI Agent 同事联调接口

## 故障排除

### Mock 模式运行缓慢
在 `config/robot_config.yaml` 中调整延迟：
```yaml
mock:
  simulate_delay: true
  delay_range: [0.1, 0.5]  # 减小延迟
```

### 找不到串口
Mac 下查看串口：
```bash
ls /dev/tty.*
```

### 依赖安装失败
确保 Python 版本 >= 3.8：
```bash
python --version
```
