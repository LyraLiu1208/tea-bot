# TeaBot - 双臂机械臂控制系统

基于 Alicia D 双臂机械臂的通用控制框架，支持 Mock 模拟和真实硬件。

## 快速开始

### 1. 创建虚拟环境（推荐）

**一键设置（推荐）:**
```bash
cd /Users/lyra/Desktop/StarBot/tea-bot
./setup_env.sh
```

**手动设置 - 使用 venv:**
```bash
cd /Users/lyra/Desktop/StarBot/tea-bot
python3 -m venv venv
source venv/bin/activate
```

**手动设置 - 使用 conda:**
```bash
cd /Users/lyra/Desktop/StarBot/tea-bot
conda create -n teabot python=3.10 -y
conda activate teabot
```

> 📖 详细环境设置说明请查看 [ENVIRONMENT_SETUP.md](ENVIRONMENT_SETUP.md)

### 2. 安装依赖

```bash
pip install -r requirements.txt
```

### 启动服务（Mock 模式）

```bash
python run.py
```

访问 API 文档: http://localhost:8000/docs

### 切换到真实硬件

编辑 `config/robot_config.yaml`，修改 `mode: real`，并配置串口参数。

## 项目结构

```
tea-bot/
├── api/          # FastAPI 后端服务
├── robot/        # 机械臂控制层
├── tools/        # 辅助工具脚本
├── config/       # 配置文件
└── data/         # 数据存储
```

## API 使用

### 发送任务

```bash
curl -X POST http://localhost:8000/task \
  -H "Content-Type: application/json" \
  -d '{
    "task_id": "task-001",
    "actions": [
      {
        "type": "move_joint",
        "arm": "left",
        "params": {"joints": [0, 0, 0, 0, 0, 0]}
      }
    ]
  }'
```

### 查询状态

```bash
curl http://localhost:8000/status
```

## 辅助工具

### 测试连接

```bash
python tools/test_connection.py
```

### 读取机械臂状态

```bash
python tools/read_state.py --arm left --save data/states/current.json
```

### 执行动作序列

```bash
python tools/execute_sequence.py --file data/sequences/demo.json
```

## 配置

编辑 `config/robot_config.yaml` 修改系统配置。

## 开发

- 机械臂接口定义: [robot/base_controller.py](robot/base_controller.py)
- Mock 实现: [robot/mock_controller.py](robot/mock_controller.py)
- 真实硬件实现: [robot/alicia_controller.py](robot/alicia_controller.py)（待实现）
