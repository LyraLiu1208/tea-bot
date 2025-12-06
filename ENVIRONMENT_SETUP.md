# TeaBot 环境设置指南

## 快速设置（推荐）

使用自动化脚本一键设置：

```bash
cd /Users/lyra/Desktop/StarBot/tea-bot
./setup_env.sh
```

## 手动设置

### 方法 1: 使用 venv（Python 内置）

#### macOS/Linux:

```bash
# 1. 进入项目目录
cd /Users/lyra/Desktop/StarBot/tea-bot

# 2. 创建虚拟环境
python3 -m venv venv

# 3. 激活虚拟环境
source venv/bin/activate

# 4. 升级 pip
pip install --upgrade pip

# 5. 安装依赖
pip install -r requirements.txt
```

#### Windows:

```bash
# 1. 进入项目目录
cd C:\path\to\tea-bot

# 2. 创建虚拟环境
python -m venv venv

# 3. 激活虚拟环境
venv\Scripts\activate

# 4. 升级 pip
pip install --upgrade pip

# 5. 安装依赖
pip install -r requirements.txt
```

### 方法 2: 使用 conda

```bash
# 1. 进入项目目录
cd /Users/lyra/Desktop/StarBot/tea-bot

# 2. 创建 conda 环境
conda create -n teabot python=3.10 -y

# 3. 激活环境
conda activate teabot

# 4. 安装依赖
pip install -r requirements.txt
```

## 验证安装

激活环境后，运行测试：

```bash
# 测试连接
python tools/test_connection.py

# 启动服务
python run.py
```

如果看到以下输出，说明安装成功：

```
╔══════════════════════════════════════╗
║        TeaBot API Server             ║
╚══════════════════════════════════════╝

Mode: mock
Host: 0.0.0.0
Port: 8000
Docs: http://localhost:8000/docs
```

## 环境管理

### 激活环境

**venv (macOS/Linux):**
```bash
source venv/bin/activate
```

**venv (Windows):**
```bash
venv\Scripts\activate
```

**conda:**
```bash
conda activate teabot
```

### 退出环境

**venv:**
```bash
deactivate
```

**conda:**
```bash
conda deactivate
```

### 删除环境

**venv:**
```bash
rm -rf venv
```

**conda:**
```bash
conda env remove -n teabot
```

## 连接真实硬件时

当你准备连接真实的 Alicia D 机械臂时，需要额外安装 SDK：

```bash
# 激活环境后
pip install git+https://github.com/Synria-Robotics/Alicia-D-SDK.git
```

## 依赖列表

### 核心依赖（Mock 模式）

- `fastapi` - Web 框架
- `uvicorn` - ASGI 服务器
- `pydantic` - 数据验证
- `PyYAML` - 配置文件解析
- `python-dotenv` - 环境变量管理

### 真实硬件额外依赖

- `alicia-d-sdk` - Alicia D 机械臂 SDK
- `robocore` - 运动学库（SDK 依赖）
- `numpy` - 数值计算（SDK 依赖）

## 常见问题

### Q: 激活环境后，命令行提示符没有变化？

A: 这是正常的。你可以运行 `which python` (macOS/Linux) 或 `where python` (Windows) 确认正在使用虚拟环境的 Python。

### Q: pip install 速度很慢？

A: 可以使用国内镜像源：

```bash
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### Q: 提示 "permission denied"？

A: macOS 上可能需要：

```bash
chmod +x setup_env.sh
```

### Q: conda 和 venv 应该用哪个？

A:
- **venv**: Python 内置，轻量级，推荐大多数场景
- **conda**: 适合需要管理多个 Python 版本或非 Python 依赖的场景

### Q: 如何查看已安装的包？

```bash
pip list
```

### Q: 如何更新依赖？

```bash
pip install --upgrade -r requirements.txt
```

## VSCode 配置

如果使用 VSCode 开发，可以配置虚拟环境：

1. 按 `Cmd+Shift+P` (macOS) 或 `Ctrl+Shift+P` (Windows)
2. 输入 "Python: Select Interpreter"
3. 选择 `./venv/bin/python` 或 `./venv/Scripts/python.exe`

## 环境变量

复制环境变量模板：

```bash
cp .env.example .env
```

然后编辑 `.env` 文件（如果需要）。

## 开发工作流

```bash
# 1. 激活环境
source venv/bin/activate

# 2. 开发代码
# ...

# 3. 测试
python tools/test_connection.py

# 4. 运行服务
python run.py

# 5. 完成后退出环境
deactivate
```

## 团队协作

如果有新的依赖添加：

```bash
# 添加依赖
pip install <new-package>

# 更新 requirements.txt
pip freeze > requirements.txt
```

团队成员只需：

```bash
pip install -r requirements.txt
```

## 清理和重置

完全重置环境：

```bash
# 1. 删除虚拟环境
rm -rf venv

# 2. 重新创建
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

---

**提示**: 每次打开新的终端窗口工作时，都需要重新激活虚拟环境！
