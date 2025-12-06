#!/usr/bin/env python3
"""启动 MuJoCo 仿真

可以直接运行（从配置文件读取），或指定模型路径
"""

import sys
import time
import argparse
import yaml
from pathlib import Path

# 添加父目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from robot import ArmSide
from simulator import MuJoCoSimulator


def load_config():
    """加载配置"""
    config_path = Path(__file__).parent.parent / "config" / "robot_config.yaml"
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def get_model_path_from_config(config):
    """从配置文件获取模型路径"""
    mujoco_config = config.get("mujoco", {})
    model_path = mujoco_config.get("model_path", "data/models/alicia_dual_arm.xml")
    # 转换为绝对路径
    if not Path(model_path).is_absolute():
        model_path = str(Path(__file__).parent.parent / model_path)
    return model_path


def run_simulation(model_path: str):
    """运行仿真

    Args:
        model_path: MJCF 模型文件路径
    """
    print(f"╔══════════════════════════════════════╗")
    print(f"║   MuJoCo Simulation for TeaBot      ║")
    print(f"╚══════════════════════════════════════╝\n")

    # 加载配置
    config = load_config()

    # 创建仿真器
    print(f"Initializing MuJoCo simulator...")
    simulator = MuJoCoSimulator(model_path, config)

    # 启动仿真
    if not simulator.start(gui=True):
        print("✗ Failed to start simulator")
        return

    print("✓ Simulator started\n")

    # 打印模型信息
    simulator.print_model_info()

    try:
        print("\n" + "="*50)
        print("Simulation running (Press Ctrl+C to stop)")
        print("="*50 + "\n")

        step_count = 0
        while True:
            # 推进仿真
            simulator.step(1)

            # 检测碰撞
            if simulator.has_collision():
                print(f"⚠ Collision detected at step {step_count}")

            step_count += 1

            # 控制频率（约 60Hz）
            time.sleep(1.0 / 60.0)

    except KeyboardInterrupt:
        print("\n\n✓ Simulation stopped by user")

    finally:
        simulator.stop()
        print("\n✓ Cleanup complete")


def demo_simulation(model_path: str):
    """演示仿真（使用预定义动作）

    Args:
        model_path: MJCF 模型文件路径
    """
    print(f"╔══════════════════════════════════════╗")
    print(f"║   MuJoCo Demo Simulation            ║")
    print(f"╚══════════════════════════════════════╝\n")

    # 创建仿真器
    simulator = MuJoCoSimulator(model_path)

    if not simulator.start(gui=True):
        print("✗ Failed to start simulator")
        return

    print("✓ Simulator started\n")
    simulator.print_model_info()

    try:
        print("\n" + "="*50)
        print("Running demo sequence...")
        print("="*50 + "\n")

        # 演示动作序列
        import numpy as np

        # 1. 回到初始位置
        print("[1/4] Moving to home position...")
        simulator.set_joint_angles(ArmSide.LEFT, [0.0] * 6)
        simulator.set_joint_angles(ArmSide.RIGHT, [0.0] * 6)

        for _ in range(120):  # 2秒
            simulator.step()
            time.sleep(1.0 / 60.0)

        # 2. 左臂运动
        print("[2/4] Moving left arm...")
        target = [0.5, 0.3, -0.2, 0.1, 0.0, 0.0]
        simulator.set_joint_angles(ArmSide.LEFT, target)

        for _ in range(120):
            simulator.step()
            time.sleep(1.0 / 60.0)

        # 3. 右臂运动
        print("[3/4] Moving right arm...")
        target = [-0.5, -0.3, 0.2, -0.1, 0.0, 0.0]
        simulator.set_joint_angles(ArmSide.RIGHT, target)

        for _ in range(120):
            simulator.step()
            time.sleep(1.0 / 60.0)

        # 4. 夹爪操作
        print("[4/4] Operating grippers...")
        simulator.set_gripper(ArmSide.LEFT, 50.0)
        simulator.set_gripper(ArmSide.RIGHT, 100.0)

        for _ in range(120):
            simulator.step()
            time.sleep(1.0 / 60.0)

        print("\n✓ Demo sequence complete!")
        print("\nPress Ctrl+C to exit...")

        # 继续运行直到用户停止
        while True:
            simulator.step()
            time.sleep(1.0 / 60.0)

    except KeyboardInterrupt:
        print("\n\n✓ Demo stopped by user")

    finally:
        simulator.stop()
        print("✓ Cleanup complete")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="MuJoCo 仿真器 - 从配置文件或命令行启动"
    )

    parser.add_argument(
        "--model",
        type=str,
        default=None,
        help="MJCF 模型文件路径（可选，默认从配置文件读取）"
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="运行演示模式"
    )

    args = parser.parse_args()

    # 加载配置
    config = load_config()

    # 获取模型路径
    if args.model:
        model_path = Path(args.model)
    else:
        # 从配置文件读取
        model_path = Path(get_model_path_from_config(config))

    # 检查模型文件是否存在
    if not model_path.exists():
        print(f"✗ Model file not found: {model_path}")
        print("\n提示：")
        print("  1. 确保 MJCF 文件路径正确")
        print("  2. 将 Alicia-D 的 MJCF 文件放在 data/models/ 目录")
        sys.exit(1)

    # 运行仿真
    try:
        if args.demo:
            demo_simulation(str(model_path))
        else:
            run_simulation(str(model_path))
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
