#!/usr/bin/env python3
"""夹爪诊断工具"""

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
print("夹爪诊断工具")
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

print("\n" + "="*60)
print("测试 1: 读取当前夹爪状态")
print("="*60)

for i in range(5):
    state = controller.get_state(arm)
    if state:
        print(f"  第 {i+1} 次读取: 夹爪值 = {state.gripper_value}")
    else:
        print(f"  第 {i+1} 次读取: 无法获取状态")
    time.sleep(0.5)

print("\n" + "="*60)
print("测试 2: 发送夹爪命令 (不等待完成)")
print("="*60)

robot = controller._get_robot(arm)

print("\n→ 发送命令: 夹爪到 100 (完全打开)")
robot.set_gripper_target(value=100, wait_for_completion=False)

print("   等待 2 秒...")
time.sleep(2)

state = controller.get_state(arm)
if state:
    print(f"   当前夹爪值: {state.gripper_value}")
else:
    print("   无法读取夹爪值")

print("\n→ 发送命令: 夹爪到 0 (完全闭合)")
robot.set_gripper_target(value=0, wait_for_completion=False)

print("   等待 2 秒...")
time.sleep(2)

state = controller.get_state(arm)
if state:
    print(f"   当前夹爪值: {state.gripper_value}")
else:
    print("   无法读取夹爪值")

print("\n→ 发送命令: 夹爪到 50 (半开)")
robot.set_gripper_target(value=50, wait_for_completion=False)

print("   等待 2 秒...")
time.sleep(2)

state = controller.get_state(arm)
if state:
    print(f"   当前夹爪值: {state.gripper_value}")
else:
    print("   无法读取夹爪值")

print("\n" + "="*60)
print("测试 3: 使用等待模式 (超时10秒)")
print("="*60)

print("\n→ 发送命令: 夹爪到 80 (等待完成)")
result = robot.set_gripper_target(value=80, wait_for_completion=True, timeout=10.0, tolerance=2.0)
if result:
    print("   ✓ 命令成功完成")
else:
    print("   ✗ 命令超时或失败")

state = controller.get_state(arm)
if state:
    print(f"   最终夹爪值: {state.gripper_value}")

print("\n" + "="*60)
print("诊断结论:")
print("="*60)

state = controller.get_state(arm)
if state:
    if state.gripper_value is None or state.gripper_value == 0:
        print("❌ 夹爪可能未安装或未连接")
        print("   - 检查夹爪硬件连接")
        print("   - 确认夹爪电源供电")
        print("   - 检查夹爪通信线缆")
    else:
        print("✓ 夹爪硬件正常,可以读取状态")
        print(f"   当前值: {state.gripper_value}")
        if abs(state.gripper_value - 80) > 5:
            print("⚠ 但夹爪未能到达目标位置")
            print("   - 可能是运动速度较慢")
            print("   - 建议增加超时时间或降低 tolerance")
else:
    print("❌ 无法读取机械臂状态")

# 断开连接
print("\n正在断开连接...")
controller.disconnect()
print("✓ 已断开")
