#!/usr/bin/env python3
"""可视化机械臂轨迹"""

import sys
import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path

# 添加父目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))


def plot_joint_trajectory(joint_traj, title="Joint Trajectory"):
    """绘制关节角度轨迹

    Args:
        joint_traj: List of joint angles, shape (N, 6)
        title: 图标题
    """
    joint_traj = np.array(joint_traj)

    plt.figure(figsize=(12, 6))

    # 弧度图
    plt.subplot(1, 2, 1)
    for i in range(6):
        plt.plot(joint_traj[:, i], label=f'Joint {i+1}')
    plt.title(f"{title} (Radians)")
    plt.xlabel("Step")
    plt.ylabel("Angle (rad)")
    plt.legend()
    plt.grid(True)

    # 角度图
    plt.subplot(1, 2, 2)
    joint_traj_deg = np.rad2deg(joint_traj)
    for i in range(6):
        plt.plot(joint_traj_deg[:, i], label=f'Joint {i+1}')
    plt.title(f"{title} (Degrees)")
    plt.xlabel("Step")
    plt.ylabel("Angle (deg)")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


def plot_3d_trajectory(poses, title="3D Trajectory", show_axes=True):
    """绘制 3D 末端轨迹

    Args:
        poses: List of poses [x, y, z] or [x, y, z, qx, qy, qz, qw]
        title: 图标题
        show_axes: 是否显示坐标轴方向
    """
    poses = np.array(poses)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 绘制轨迹
    ax.plot(poses[:, 0], poses[:, 1], poses[:, 2], 'b-', linewidth=2, label='Trajectory')

    # 起点和终点
    ax.scatter(poses[0, 0], poses[0, 1], poses[0, 2],
               c='green', s=100, marker='o', label='Start')
    ax.scatter(poses[-1, 0], poses[-1, 1], poses[-1, 2],
               c='red', s=100, marker='x', label='End')

    # 如果有姿态信息，绘制方向箭头
    if show_axes and poses.shape[1] >= 7:
        interval = max(1, len(poses) // 10)  # 每 10% 显示一个
        for i in range(0, len(poses), interval):
            pos = poses[i, :3]
            # 这里简化：只绘制一个方向向量
            # 真实情况需要从四元数转换为旋转矩阵
            ax.quiver(pos[0], pos[1], pos[2], 0.05, 0, 0, color='r', alpha=0.6)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    ax.legend()

    # 设置等比例坐标轴
    _set_axes_equal(ax)

    plt.tight_layout()
    plt.show()


def _set_axes_equal(ax):
    """设置 3D 图等比例"""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])
    max_range = max(x_range, y_range, z_range)

    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)

    ax.set_xlim3d([x_middle - max_range / 2, x_middle + max_range / 2])
    ax.set_ylim3d([y_middle - max_range / 2, y_middle + max_range / 2])
    ax.set_zlim3d([z_middle - max_range / 2, z_middle + max_range / 2])


def visualize_sequence_file(sequence_file: str):
    """可视化动作序列文件"""
    with open(sequence_file, 'r') as f:
        sequence = json.load(f)

    actions = sequence.get("actions", [])

    # 提取关节运动
    joint_trajectories = {'left': [], 'right': []}
    pose_trajectories = {'left': [], 'right': []}

    for action in actions:
        action_type = action.get("type")
        arm = action.get("arm")
        params = action.get("params", {})

        if action_type == "move_joint":
            joints = params.get("joints")
            if joints:
                joint_trajectories[arm].append(joints)

        elif action_type == "move_pose":
            pose = params.get("pose")
            if pose:
                pose_trajectories[arm].append(pose)

    # 绘制关节轨迹
    for arm, traj in joint_trajectories.items():
        if traj:
            plot_joint_trajectory(traj, title=f"{arm.upper()} Arm Joint Trajectory")

    # 绘制末端轨迹
    for arm, traj in pose_trajectories.items():
        if traj:
            plot_3d_trajectory(traj, title=f"{arm.upper()} Arm End Effector Trajectory")


def visualize_demo():
    """演示：生成并可视化示例轨迹"""
    print("Generating demo trajectory...")

    # 生成一个圆形轨迹
    N = 50
    t = np.linspace(0, 2 * np.pi, N)
    x = 0.3 + 0.1 * np.cos(t)
    y = 0.1 * np.sin(t)
    z = 0.2 + 0.05 * np.sin(2 * t)

    poses = np.stack([x, y, z], axis=1)

    # 生成关节轨迹（示例）
    joint_traj = np.zeros((N, 6))
    for i in range(6):
        joint_traj[:, i] = np.sin(t * (i + 1) / 3) * 0.5

    # 可视化
    plot_3d_trajectory(poses, title="Demo Circle Trajectory")
    plot_joint_trajectory(joint_traj, title="Demo Joint Angles")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="可视化机械臂轨迹")
    parser.add_argument(
        "--file",
        type=str,
        help="序列文件路径 (JSON)"
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="运行演示"
    )

    args = parser.parse_args()

    if args.demo:
        visualize_demo()
    elif args.file:
        visualize_sequence_file(args.file)
    else:
        print("请指定 --file 或 --demo")
        print("\n使用示例:")
        print("  python tools/visualize_trajectory.py --demo")
        print("  python tools/visualize_trajectory.py --file data/sequences/demo.json")
