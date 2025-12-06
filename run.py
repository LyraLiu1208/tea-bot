#!/usr/bin/env python3
"""启动 TeaBot API 服务"""

import uvicorn
import yaml
from pathlib import Path


def load_config():
    """加载配置"""
    config_path = Path(__file__).parent / "config" / "robot_config.yaml"
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


if __name__ == "__main__":
    config = load_config()
    api_config = config.get("api", {})

    host = api_config.get("host", "0.0.0.0")
    port = api_config.get("port", 8000)

    print(f"""
╔══════════════════════════════════════╗
║        TeaBot API Server             ║
╚══════════════════════════════════════╝

Mode: {config.get('mode', 'unknown')}
Host: {host}
Port: {port}
Docs: http://localhost:{port}/docs

Starting server...
""")

    uvicorn.run(
        "api.main:app",
        host=host,
        port=port,
        reload=True,
        log_level="info"
    )
