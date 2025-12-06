#!/usr/bin/env python3
"""简单的 MuJoCo 关节调试器。

该工具会：
  1. 启动 TeaBot 的 MuJoCo 仿真与 passive viewer（稳定、不会崩溃）。
  2. 打开一个 Tkinter 窗口，提供左右臂 6 个关节与 2 个夹爪的滑块。
  3. 滑块拖动或输入数值时，立即同步到仿真，便于直观看效果。

使用方式：
  source venv/bin/activate
  python tools/joint_tuner.py               # 默认加载 data/models/alicia_dual_arm.xml
  python tools/joint_tuner.py <your_model>  # 指定其他 MJCF
"""

import argparse
import sys
import threading
import time
from pathlib import Path
from typing import List

import tkinter as tk
from tkinter import ttk

# 将项目根目录加入 sys.path（方便直接运行脚本）
sys.path.insert(0, str(Path(__file__).parent.parent))

from robot.base_controller import ArmSide  # noqa: E402
from simulator import MuJoCoSimulator  # noqa: E402


JOINT_LIMIT = (-3.14, 3.14)


def parse_args():
    parser = argparse.ArgumentParser(description="MuJoCo 双臂关节调试器")
    parser.add_argument(
        "model",
        nargs="?",
        default="data/models/alicia_dual_arm.xml",
        help="MJCF 模型路径（默认 data/models/alicia_dual_arm.xml）",
    )
    return parser.parse_args()


def start_simulator(model_path: str) -> MuJoCoSimulator:
    sim = MuJoCoSimulator(model_path)
    if not sim.start(gui=True):
        raise RuntimeError("无法启动 MuJoCo 仿真器")
    return sim


def simulation_loop(sim: MuJoCoSimulator, stop_event: threading.Event):
    while not stop_event.is_set():
        sim.step()
        time.sleep(1.0 / 240.0)


def create_joint_row(parent, label: str, command, initial_value: float):
    frame = ttk.Frame(parent)
    ttk.Label(frame, text=label, width=14).pack(side=tk.LEFT, padx=2)
    slider = ttk.Scale(
        frame,
        from_=JOINT_LIMIT[0],
        to=JOINT_LIMIT[1],
        orient=tk.HORIZONTAL,
        value=initial_value,
        command=lambda v: command(float(v)),
    )
    slider.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)

    value_var = tk.StringVar(value=f"{initial_value:.3f}")

    def on_entry_change(*_):
        try:
            val = float(value_var.get())
        except ValueError:
            return
        val = max(JOINT_LIMIT[0], min(JOINT_LIMIT[1], val))
        slider.set(val)
        command(val)

    entry = ttk.Entry(frame, textvariable=value_var, width=7)
    entry.pack(side=tk.LEFT, padx=4)
    value_var.trace_add("write", on_entry_change)

    def update_value(new_val: float):
        value_var.set(f"{new_val:.3f}")
        slider.set(new_val)

    return frame, update_value


def build_ui(sim: MuJoCoSimulator):
    root = tk.Tk()
    root.title("TeaBot Joint Tuner")

    stop_event = threading.Event()
    loop_thread = threading.Thread(
        target=simulation_loop, args=(sim, stop_event), daemon=True
    )
    loop_thread.start()

    sliders = {}

    def make_joint_setter(arm: ArmSide, idx: int):
        def setter(value: float):
            angles = sim.get_joint_angles(arm)
            if len(angles) != 6:
                return
            angles[idx] = value
            sim.set_joint_angles(arm, angles)
        return setter

    def make_gripper_setter(arm: ArmSide):
        def setter(value: float):
            sim.set_gripper(arm, value)
        return setter

    notebook = ttk.Notebook(root)
    notebook.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

    for arm, label in [(ArmSide.LEFT, "左臂 (Left)"), (ArmSide.RIGHT, "右臂 (Right)")]:
        frame = ttk.Frame(notebook, padding=8)
        notebook.add(frame, text=label)
        current_angles = sim.get_joint_angles(arm)
        for i in range(6):
            row, updater = create_joint_row(
                frame,
                f"Joint{i+1}",
                make_joint_setter(arm, i),
                current_angles[i] if i < len(current_angles) else 0.0,
            )
            row.pack(fill=tk.X, pady=2)
            sliders[(arm, i)] = updater

        # 夹爪
        grip_frame = ttk.Frame(frame, padding=(0, 6))
        grip_frame.pack(fill=tk.X)
        ttk.Label(grip_frame, text="Gripper (0-100)", width=14).pack(side=tk.LEFT)
        grip_scale = ttk.Scale(
            grip_frame,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            command=lambda v, a=arm: make_gripper_setter(a)(float(v)),
        )
        grip_scale.set(0)
        grip_scale.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)

    def reset():
        sim.set_joint_angles(ArmSide.LEFT, [0.0] * 6)
        sim.set_joint_angles(ArmSide.RIGHT, [0.0] * 6)
        sim.set_gripper(ArmSide.LEFT, 0.0)
        sim.set_gripper(ArmSide.RIGHT, 0.0)
        for arm in (ArmSide.LEFT, ArmSide.RIGHT):
            angles = sim.get_joint_angles(arm)
            for i in range(6):
                sliders[(arm, i)](angles[i])

    ttk.Button(root, text="Reset (Zero Pose)", command=reset).pack(pady=6)

    def on_close():
        stop_event.set()
        loop_thread.join(timeout=1.0)
        sim.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    return root


def main():
    args = parse_args()
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"✗ 模型文件不存在: {model_path}")
        sys.exit(1)

    simulator = start_simulator(str(model_path))
    ui = build_ui(simulator)
    print(
        "已启动 Joint Tuner。\n"
        " - 在 Tk 窗口里拖动滑块即可调整对应关节\n"
        " - MuJoCo viewer 会实时显示效果\n"
        " - 点击 Reset 可快速回到零位"
    )
    ui.mainloop()


if __name__ == "__main__":
    main()
