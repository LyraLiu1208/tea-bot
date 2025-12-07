#!/usr/bin/env python3
"""测试关节角度读取"""

import sys
import time
sys.path.insert(0, '../Alicia-D-SDK')

from robot import ArmSide
from robot.alicia_controller import AliciaDualArmController
import yaml
from pathlib import Path

# 加载配置
config_path = Path(__file__).parent / "config" / "robot_config.yaml"
with open(config_path, 'r', encoding='utf-8') as f:
    config = yaml.safe_load(f)

config["mode"] = "real"

print("="*60)
print("关节角度读取测试")
print("="*60)

# 创建控制器
controller = AliciaDualArmController(config)

# 连接
print("\n正在连接机械臂...")
if not controller.connect():
    print("✗ 连接失败")
    sys.exit(1)

print("✓ 已连接")

# 启用力矩
print("\n启用力矩...")
controller.enable_torque(ArmSide.LEFT, True)

arm = ArmSide.LEFT
robot = controller._get_robot(arm)

print("\n" + "="*60)
print("测试 1: 连续读取关节角度 (10次)")
print("="*60)
print("\n请手动移动机械臂,观察读数是否变化...")

for i in range(10):
    # 直接从SDK读取
    joints_sdk = robot.get_joints()

    # 通过controller读取
    state = controller.get_state(arm)

    print(f"\n第 {i+1} 次读取:")
    if joints_sdk:
        print(f"  SDK直接读取 (弧度): {[f'{j:.3f}' for j in joints_sdk]}")
        print(f"  SDK直接读取 (度):   {[f'{j*57.3:.1f}°' for j in joints_sdk]}")
    else:
        print("  SDK直接读取: None")

    if state:
        print(f"  Controller读取 (弧度): {[f'{a:.3f}' for a in state.joint_angles]}")
        print(f"  Controller读取 (度):   {[f'{a*57.3:.1f}°' for a in state.joint_angles]}")
    else:
        print("  Controller读取: None")

    time.sleep(1)

print("\n" + "="*60)
print("测试 2: 发送运动命令并验证")
print("="*60)

print("\n当前关节角度:")
state = controller.get_state(arm)
if state:
    print(f"  {[f'{a*57.3:.1f}°' for a in state.joint_angles]}")

print("\n→ 发送命令: Joint1 = 30° (0.524 rad)")
target = [0.524, 0.0, 0.0, 0.0, 0.0, 0.0]
robot.set_joint_target(target_joints=target, joint_format='rad')

print("  等待 3 秒...")
time.sleep(3)

print("\n读取新的角度:")
state = controller.get_state(arm)
if state:
    print(f"  {[f'{a*57.3:.1f}°' for a in state.joint_angles]}")
    error = abs(state.joint_angles[0] - 0.524) * 57.3
    print(f"  Joint1 误差: {error:.1f}°")
    if error < 5:
        print("  ✓ 读取正确!")
    else:
        print("  ✗ 读取可能有问题")

print("\n→ 回到 Home 位置")
robot.set_joint_target(target_joints=[0.0]*6, joint_format='rad')
time.sleep(3)

state = controller.get_state(arm)
if state:
    print(f"  {[f'{a*57.3:.1f}°' for a in state.joint_angles]}")

# 断开连接
print("\n正在断开连接...")
controller.disconnect()
print("✓ 已断开")
