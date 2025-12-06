#!/usr/bin/env python3
"""Open MuJoCo's interactive viewer for manual joint tuning.

Compared to `tools/run_simulation.py`, this script launches the *interactive*
viewer (`mujoco.viewer.launch`) instead of the passive viewer. That means the
Joint/Control 面板里的 slider 不会变灰，你可以直接拖动或输入角度来自由设置双臂和夹爪。
"""

import argparse
import sys
import time
from pathlib import Path

import subprocess

import mujoco
import mujoco.viewer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Open MuJoCo interactive viewer for manual joint editing"
    )
    parser.add_argument(
        "model",
        nargs="?",
        default="data/models/alicia_dual_arm.xml",
        help="MJCF model path (default: data/models/alicia_dual_arm.xml)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"✗ 模型文件不存在: {model_path}")
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    print(
        "提示：\n"
        "  • Joint 面板的输入框现在可编辑，可直接输入角度或拖动 slider。\n"
        "  • 按 Space 可以暂停/继续仿真，便于调姿后再运行。\n"
        "  • Ctrl/Shift + 鼠标可选择单关节或多关节。\n"
        "关闭窗口即可结束。"
    )

    try:
        with mujoco.viewer.launch(model, data) as viewer:
            # 在 viewer 打开期间持续推进仿真
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(1.0 / 120.0)
    except RuntimeError as exc:
        print("⚠ 在当前进程中启动交互式 viewer 失败，尝试使用官方备用命令。")
        print(f"  原因: {exc}")
        print("  正在执行：mjpython -m mujoco.viewer <model>")
        subprocess.run(
            ["mjpython", "-m", "mujoco.viewer", str(model_path)],
            check=False,
        )


if __name__ == "__main__":
    main()
