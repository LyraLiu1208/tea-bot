#!/usr/bin/env python3
"""SDK调试模式测试 - 查看底层通信"""

import sys
import time
sys.path.insert(0, '../Alicia-D-SDK')

from alicia_d_sdk import create_robot

print("="*60)
print("SDK 调试模式测试")
print("="*60)

# 创建机器人实例 - 开启调试模式
print("\n创建机器人实例 (调试模式开启)...")
robot = create_robot(
    port='',  # 自动搜索
    baudrate=1000000,
    robot_version='v5_6',
    gripper_type='50mm',
    debug_mode=True  # 开启调试模式
)

# 连接
print("\n正在连接...")
if not robot.connect():
    print("✗ 连接失败")
    sys.exit(1)

print("✓ 已连接")

# 启用力矩
print("\n启用力矩...")
robot.torque_control('on')

# 等待数据
print("\n等待3秒接收数据...")
time.sleep(3)

# 读取关节角度
print("\n" + "="*60)
print("读取关节角度:")
print("="*60)

for i in range(5):
    joints = robot.get_joints()
    print(f"\n第 {i+1} 次读取:")
    if joints:
        print(f"  弧度: {joints}")
        print(f"  度:   {[f'{j*57.3:.1f}°' for j in joints]}")
    else:
        print("  None (未接收到数据)")
    time.sleep(1)

# 尝试发送一个运动命令
print("\n" + "="*60)
print("发送运动命令测试:")
print("="*60)

print("\n→ 命令: Joint1 = 15° (0.262 rad)")
robot.set_joint_target([0.262, 0, 0, 0, 0, 0], joint_format='rad')

print("  等待 2 秒...")
time.sleep(2)

joints = robot.get_joints()
if joints:
    print(f"  读取: {[f'{j*57.3:.1f}°' for j in joints]}")
else:
    print("  读取: None")

# 断开
print("\n正在断开连接...")
robot.disconnect()
print("✓ 已断开")
