#!/bin/bash
# TeaBot 环境设置脚本

echo "╔══════════════════════════════════════╗"
echo "║   TeaBot Environment Setup           ║"
echo "╚══════════════════════════════════════╝"
echo ""

# 检查是否在项目目录
if [ ! -f "requirements.txt" ]; then
    echo "❌ Error: Please run this script from the tea-bot directory"
    exit 1
fi

# 创建虚拟环境
if [ ! -d "venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv venv
    echo "✅ Virtual environment created"
else
    echo "✅ Virtual environment already exists"
fi

# 激活虚拟环境并安装依赖
echo ""
echo "📥 Installing dependencies..."
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo ""
echo "╔══════════════════════════════════════╗"
echo "║   Setup Complete!                    ║"
echo "╚══════════════════════════════════════╝"
echo ""
echo "To activate the environment, run:"
echo "  source venv/bin/activate"
echo ""
echo "To start the server, run:"
echo "  python run.py"
echo ""
