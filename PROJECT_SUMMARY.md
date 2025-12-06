# TeaBot 项目总结

## 项目信息

- **项目名称**: TeaBot - 双臂机械臂控制系统
- **机械臂**: 悬崖玄雅 Alicia D (双臂六轴)
- **SDK**: Alicia-D-SDK (https://github.com/Synria-Robotics/Alicia-D-SDK)
- **开发语言**: Python 3.8+
- **当前状态**: ✅ 基础框架完成

## 已完成功能

### 1. 核心架构

- ✅ 抽象控制器基类 (`BaseDualArmController`)
- ✅ Mock 模拟控制器 (`MockDualArmController`) - 无需硬件即可开发测试
- ✅ Alicia D 真实控制器 (`AliciaDualArmController`) - 基于官方 SDK
- ✅ 双臂独立控制（左臂/右臂）

### 2. FastAPI 后端

- ✅ RESTful API 服务
- ✅ POST `/task` - 接收任务 JSON
- ✅ GET `/status` - 查询机械臂状态
- ✅ POST `/stop` - 紧急停止
- ✅ POST `/reset` - 复位系统
- ✅ GET `/health` - 健康检查
- ✅ 自动生成 API 文档 (Swagger UI)

### 3. 支持的动作类型

1. **move_joint** - 关节空间运动（6 个关节角度）
2. **move_pose** - 笛卡尔空间运动（位置+四元数）
3. **control_gripper** - 夹爪控制（0-100）
4. **set_home** - 回到初始位置
5. **enable_torque** - 启用/禁用力矩（示教模式）

### 4. 辅助工具

- ✅ `test_connection.py` - 测试连接和读取状态
- ✅ `read_state.py` - 读取并保存机械臂状态（支持连续模式）
- ✅ `execute_sequence.py` - 执行动作序列文件（支持预览模式）

### 5. 配置管理

- ✅ YAML 配置文件
- ✅ Mock/Real 模式切换
- ✅ 双臂独立配置（串口、波特率、速度等）

## 项目结构

```
tea-bot/
├── README.md              # 项目说明
├── QUICKSTART.md          # 快速开始指南
├── PROJECT_SUMMARY.md     # 本文档
├── requirements.txt       # Python 依赖
├── run.py                 # 启动脚本
│
├── api/                   # FastAPI 后端
│   ├── main.py           # 主应用
│   └── models.py         # 数据模型
│
├── robot/                 # 机械臂控制层
│   ├── base_controller.py      # 抽象基类
│   ├── mock_controller.py      # Mock 实现
│   └── alicia_controller.py    # Alicia D 实现
│
├── tools/                 # 辅助工具
│   ├── test_connection.py
│   ├── read_state.py
│   └── execute_sequence.py
│
├── config/               # 配置文件
│   └── robot_config.yaml
│
└── data/                 # 数据存储
    ├── states/          # 保存的状态
    └── sequences/       # 动作序列
        └── demo.json    # 示例序列
```

## 与 AI Agent 对接

你的同事（AI Agent 开发者）只需要：

1. **发送任务 JSON** 到 `POST /task` 端点
2. **JSON 格式**：
   ```json
   {
     "task_id": "order-001",
     "actions": [
       {
         "type": "move_joint",
         "arm": "left",
         "params": {"joints": [...]},
         "wait": true
       }
     ]
   }
   ```
3. **接收响应**：
   ```json
   {
     "task_id": "order-001",
     "status": "completed",
     "message": "All actions executed successfully"
   }
   ```

## 当前工作流程

### 开发阶段（无硬件）

1. 使用 Mock 模式开发和测试
2. 定义动作序列 JSON
3. 通过 FastAPI 调试接口
4. 与 AI Agent 联调

### 硬件集成阶段（有硬件）

1. 安装 Alicia-D SDK
2. 修改配置切换到 Real 模式
3. 配置串口参数
4. 使用工具录制关键点位
5. 优化动作参数

## 下一步开发建议

### 短期（当前可做）

1. **定义具体任务**：
   - 根据奶茶制作流程，定义具体的动作序列
   - 创建更多示例 JSON 文件

2. **与 AI Agent 对接**：
   - 确认 JSON 格式
   - 进行接口联调测试

3. **完善工具**：
   - 添加更多辅助脚本（如轨迹可视化）
   - 添加日志记录功能

### 中期（连接硬件后）

1. **硬件集成**：
   - 安装 Alicia-D SDK
   - 测试真实硬件连接
   - 校准关键点位

2. **动作录制**：
   - 使用示教模式录制动作
   - 优化动作流畅度
   - 测试双臂协调

3. **安全机制**：
   - 添加碰撞检测
   - 完善紧急停止逻辑
   - 添加限位保护

### 长期（优化阶段）

1. **任务管理**：
   - 添加任务队列
   - 支持并发任务
   - 任务优先级

2. **性能优化**：
   - 动作序列优化
   - 双臂协调优化
   - 执行效率提升

3. **监控和日志**：
   - 添加详细日志
   - 性能监控
   - 错误追踪

## 技术要点

### 双臂控制

- Alicia-D SDK 针对单臂设计
- 本项目为每个臂创建独立的 SDK 实例
- 通过 `AliciaDualArmController` 统一管理

### Mock 模式

- 完全模拟机械臂行为
- 可调节延迟参数
- 适合无硬件开发

### 配置设计

- 支持 Mock/Real 模式一键切换
- 双臂独立配置
- 易于扩展

## 测试验证

### 已验证功能

✅ Mock 控制器正常工作
✅ 连接测试工具运行正常
✅ 序列执行工具预览模式正常
✅ 配置文件加载正常

### 待验证（需要硬件）

⏳ 真实硬件连接
⏳ 关节运动控制
⏳ 笛卡尔运动控制
⏳ 夹爪控制
⏳ 双臂协调运动

## 依赖说明

### 当前依赖

```
fastapi==0.104.1
uvicorn[standard]==0.24.0
pydantic==2.5.0
pydantic-settings==2.1.0
PyYAML==6.0.1
python-dotenv==1.0.0
```

### 连接真实硬件时需要

```
# 从 GitHub 安装
pip install git+https://github.com/Synria-Robotics/Alicia-D-SDK.git
```

## 远程协作说明

由于你无法直接接触硬件，建议工作流程：

1. **开发阶段**：使用 Mock 模式在本地开发
2. **录制数据**：让有硬件权限的同事使用工具录制数据
3. **数据共享**：通过 Git 共享录制的 JSON 文件
4. **远程调试**：使用工具的网络接口进行远程控制
5. **日志分析**：通过详细日志排查问题

## 联系人

- **机械臂部分**：你（本项目开发者）
- **AI Agent**：同事（负责订单生成和任务下发）

## 文档

- [README.md](README.md) - 项目概览
- [QUICKSTART.md](QUICKSTART.md) - 快速开始指南
- [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) - 本文档
- API 文档: http://localhost:8000/docs (启动服务后访问)

---

**创建日期**: 2025-12-05
**版本**: v0.1.0
**状态**: 基础框架完成，待硬件集成
