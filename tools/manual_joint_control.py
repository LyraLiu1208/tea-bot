#!/usr/bin/env python3
"""Interactive MuJoCo joint control helper.

Usage:
    python tools/manual_joint_control.py data/models/alicia_dual_arm.xml

You can then type commands like:
    left 0 0 0 0 0 0          # set left arm joint angles (rad)
    right 0.5 -0.3 0 0 0 0    # set right arm
    gripper left 50           # set left gripper opening (0-100)
    home                      # reset both arms to zero pose
    quit                      # exit the tool
"""

import argparse
import sys
import threading
import time
from pathlib import Path

# Make project modules importable when running as script
sys.path.insert(0, str(Path(__file__).parent.parent))

from robot.base_controller import ArmSide  # noqa: E402
from simulator import MuJoCoSimulator  # noqa: E402


def _run_simulation_loop(simulator: MuJoCoSimulator, stop_event: threading.Event):
    """Background loop that keeps stepping MuJoCo so the viewer stays live."""
    while not stop_event.is_set():
        simulator.step()
        time.sleep(1.0 / 120.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive MuJoCo joint controller")
    parser.add_argument(
        "model",
        nargs="?",
        default="data/models/alicia_dual_arm.xml",
        help="Path to MJCF model (default: data/models/alicia_dual_arm.xml)",
    )
    return parser.parse_args()


def prompt_help():
    print(
        """
Commands:
  left <j1..j6>            设置左臂 6 个关节角（单位：弧度）
  right <j1..j6>           设置右臂关节角
  gripper <left|right> <0-100>   设置夹爪开合百分比
  home                     两只手臂均归零
  info                     打印当前关节角
  help                     显示帮助
  quit / exit              退出
"""
    )


def parse_joint_values(parts):
    if len(parts) != 7:
        print("✗ 需要 6 个关节角，例如：left 0 0 0 0 0 0")
        return None
    try:
        return [float(v) for v in parts[1:]]
    except ValueError:
        print("✗ 关节角必须是数字")
        return None


def interactive_loop(simulator: MuJoCoSimulator):
    prompt_help()

    while True:
        try:
            raw = input("tea-bot> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n退出")
            break

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        if cmd in {"quit", "exit"}:
            break
        elif cmd == "help":
            prompt_help()
        elif cmd == "home":
            simulator.set_joint_angles(ArmSide.LEFT, [0.0] * 6)
            simulator.set_joint_angles(ArmSide.RIGHT, [0.0] * 6)
            simulator.set_gripper(ArmSide.LEFT, 0.0)
            simulator.set_gripper(ArmSide.RIGHT, 0.0)
            print("✓ 已归零")
        elif cmd == "left":
            values = parse_joint_values(parts)
            if values is not None:
                simulator.set_joint_angles(ArmSide.LEFT, values)
                print("✓ 左臂已更新")
        elif cmd == "right":
            values = parse_joint_values(parts)
            if values is not None:
                simulator.set_joint_angles(ArmSide.RIGHT, values)
                print("✓ 右臂已更新")
        elif cmd == "gripper":
            if len(parts) != 3:
                print("✗ 用法：gripper left 50")
                continue
            arm_name = parts[1].lower()
            try:
                value = float(parts[2])
            except ValueError:
                print("✗ 开合度必须是数字 (0-100)")
                continue
            value = max(0.0, min(100.0, value))
            if arm_name == "left":
                simulator.set_gripper(ArmSide.LEFT, value)
                print(f"✓ 左夹爪: {value:.1f}")
            elif arm_name == "right":
                simulator.set_gripper(ArmSide.RIGHT, value)
                print(f"✓ 右夹爪: {value:.1f}")
            else:
                print("✗ gripper 只接受 left 或 right")
        elif cmd == "info":
            left_angles = simulator.get_joint_angles(ArmSide.LEFT)
            right_angles = simulator.get_joint_angles(ArmSide.RIGHT)
            print("左臂:", " ".join(f"{a:.3f}" for a in left_angles))
            print("右臂:", " ".join(f"{a:.3f}" for a in right_angles))
        else:
            print("✗ 未知命令，输入 help 查看可用命令")


def main():
    args = parse_args()
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"✗ 模型文件不存在: {model_path}")
        sys.exit(1)

    simulator = MuJoCoSimulator(str(model_path))
    if not simulator.start(gui=True):
        print("✗ 无法启动 MuJoCo 仿真")
        sys.exit(1)

    stop_event = threading.Event()
    loop_thread = threading.Thread(
        target=_run_simulation_loop, args=(simulator, stop_event), daemon=True
    )
    loop_thread.start()

    try:
        interactive_loop(simulator)
    finally:
        stop_event.set()
        loop_thread.join(timeout=1.0)
        simulator.stop()
        print("✓ 仿真已结束")


if __name__ == "__main__":
    main()
