#!/usr/bin/env python3
"""单臂测试脚本 (支持 Real / MuJoCo 模式)

简单的命令行工具，用于测试 Alicia-D 机械臂
- Real 模式: 真实硬件控制
- MuJoCo 模式: 物理仿真测试
"""

import yaml
import time
import sys
import argparse
import math
from pathlib import Path
from robot import ArmSide

# 加载配置
config_path = Path(__file__).parent / "config" / "robot_config.yaml"
with open(config_path, 'r', encoding='utf-8') as f:
    config = yaml.safe_load(f)


def warn_real_mode_if_needed(controller_mode: str):
    """真机模式下提示安全注意事项"""
    if controller_mode == "real":
        print("\n⚠ 真机测试提示: 请确保工作区域清空、急停按钮就绪。")


def is_motion_safe(joint_angles):
    """检查关节角是否在安全范围内"""
    limits = config.get("safety", {}).get("joint_limits", [])
    if len(limits) != 6 or len(joint_angles) != 6:
        return True
    for angle, limit in zip(joint_angles, limits):
        if not (limit[0] <= angle <= limit[1]):
            return False
    return True


def is_pose_safe(pose):
    """检查笛卡尔空间目标是否位于安全工作空间"""
    workspace = config.get("safety", {}).get("workspace_limits", {})
    axes = ["x", "y", "z"]
    for idx, axis in enumerate(axes):
        limits = workspace.get(axis)
        if limits and len(limits) == 2:
            if not (limits[0] <= pose[idx] <= limits[1]):
                return False
    return True


def normalize_quaternion(q):
    norm = math.sqrt(sum(component * component for component in q))
    if norm < 1e-6:
        return None
    return [component / norm for component in q]


def print_menu():
    """打印菜单"""
    print("\n" + "="*60)
    print("单臂功能测试菜单 (奶茶机器人基础功能)")
    print("="*60)
    print("基础运动:")
    print("  1. 回到 Home 位置 (平滑)")
    print("  2. 预设位置测试 (轻微弯曲)")
    print("  3. 自定义关节角度 (梯形轨迹)")
    print("\n笛卡尔空间:")
    print("  4. 移动到笛卡尔位置 (XYZ)")
    print("\n夹爪:")
    print("  5. 夹爪开合测试")
    print("\n状态与安全:")
    print("  6. 查看当前状态")
    print("  7. 紧急停止 (立即停止当前动作)")
    print("\n  0. 退出")
    print("="*60)


def print_current_state(controller, arm):
    """打印当前状态"""
    state = controller.get_state(arm)
    if state:
        print(f"\n当前状态 [{arm.value}]:")
        print(f"  关节角度 (弧度): {[f'{a:.3f}' for a in state.joint_angles]}")
        print(f"  关节角度 (度):   {[f'{a*57.3:.1f}°' for a in state.joint_angles]}")
        print(f"  夹爪值: {state.gripper_value:.1f}")
        if state.end_effector_pose:
            print(f"  末端位置: {state.end_effector_pose}")
    else:
        print("✗ 无法获取状态")


def move_to_home(controller, arm):
    """回到 Home 位置"""
    print(f"\n正在移动 {arm.value} 臂到 Home 位置...")
    if controller.set_home(arm):
        print("✓ 已到达 Home 位置")
        time.sleep(1)
        print_current_state(controller, arm)
    else:
        print("✗ 移动失败")


def move_to_preset_1(controller, arm):
    """预设位置 1: 轻微弯曲"""
    print(f"\n正在移动 {arm.value} 臂到预设位置 1 (轻微弯曲)...")
    # Joint2 和 Joint3 轻微弯曲
    angles = [0.0, 0.3, 0.5, 0.0, 0.0, 0.0]  # 约 17°, 29°
    print(f"目标角度 (度): {[f'{a*57.3:.1f}°' for a in angles]}")

    if controller.move_joint_smooth(arm, angles, steps=80, step_delay=0.006):
        print("✓ 已到达预设位置 1")
        time.sleep(1)
        print_current_state(controller, arm)
    else:
        print("✗ 移动失败")


def move_to_preset_2(controller, arm):
    """预设位置 2: 更大弯曲"""
    print(f"\n正在移动 {arm.value} 臂到预设位置 2 (更大弯曲)...")
    # Joint2 和 Joint3 更大弯曲
    angles = [0.0, 0.6, 1.0, 0.0, 0.0, 0.0]  # 约 34°, 57°
    print(f"目标角度 (度): {[f'{a*57.3:.1f}°' for a in angles]}")

    if controller.move_joint_smooth(arm, angles, steps=120, step_delay=0.006):
        print("✓ 已到达预设位置 2")
        time.sleep(1)
        print_current_state(controller, arm)
    else:
        print("✗ 移动失败")


def move_custom(controller, arm):
    """自定义关节角度（直接运动）"""
    print(f"\n自定义关节角度 - 直接运动 (适合小角度变化)")
    print("⚠️  注意: 大角度运动可能失败，建议使用平滑运动功能")
    print("\n安全范围提示:")
    print("  Joint1: ±124°")
    print("  Joint2: ±90°")
    print("  Joint3: -29° ~ 135°")
    print("  Joint4: ±180°")
    print("  Joint5: -90° ~ 86°")
    print("  Joint6: ±180°")

    try:
        input_str = input("\n输入 6 个角度 (用空格分隔，例如: 0 30 45 0 0 0): ").strip()
        degrees = [float(x) for x in input_str.split()]

        if len(degrees) != 6:
            print(f"✗ 需要输入 6 个值，您输入了 {len(degrees)} 个")
            return

        # 转换为弧度
        radians = [d * 3.14159 / 180.0 for d in degrees]

        print(f"目标角度 (度): {[f'{d:.1f}°' for d in degrees]}")

        if not is_motion_safe(radians):
            print("✗ 目标超出安全范围，已取消执行")
            return

        print("自动执行（无确认）。如需中止，请立即按 Ctrl+C 或使用紧急停止。")

        if controller.move_joint(arm, radians, wait=True):
            print("✓ 移动完成")
            time.sleep(1)
            print_current_state(controller, arm)
        else:
            print("✗ 移动失败")

    except ValueError:
        print("✗ 输入格式错误，请输入数字")
    except Exception as e:
        print(f"✗ 错误: {e}")


def move_custom_smooth(controller, arm):
    """自定义关节角度（梯形速度轨迹规划）"""
    print(f"\n自定义关节角度 - 梯形速度轨迹规划")
    print("✓ 使用工业机器人标准方法：加速-匀速-减速")
    print("✓ 平滑且高效，适合所有角度范围")
    print("\n安全范围提示:")
    print("  Joint1: ±124°")
    print("  Joint2: ±90°")
    print("  Joint3: -29° ~ 135°")
    print("  Joint4: ±180°")
    print("  Joint5: -90° ~ 86°")
    print("  Joint6: ±180°")

    try:
        input_str = input("\n输入 6 个角度 (用空格分隔，例如: 0 60 90 0 0 0): ").strip()
        degrees = [float(x) for x in input_str.split()]

        if len(degrees) != 6:
            print(f"✗ 需要输入 6 个值，您输入了 {len(degrees)} 个")
            return

        # 转换为弧度
        radians = [d * 3.14159 / 180.0 for d in degrees]

        print(f"\n目标角度 (度): {[f'{d:.1f}°' for d in degrees]}")

        if not is_motion_safe(radians):
            print("✗ 目标超出安全范围，已取消执行")
            return

        steps = 100
        step_delay = 0.005
        print(f"\n配置: {steps} 步，每步间隔 {step_delay}s，预计耗时 ~{steps*step_delay:.1f}s")
        print("自动执行（无确认）。如需中止，请立即按 Ctrl+C 或使用紧急停止。")

        print(f"\n正在执行梯形速度轨迹...")
        if controller.move_joint_smooth(arm, radians, steps=steps, step_delay=step_delay):
            print("✓ 运动完成")
            time.sleep(0.5)
            print_current_state(controller, arm)
        else:
            print("✗ 运动失败")

    except ValueError:
        print("✗ 输入格式错误，请输入数字")
    except Exception as e:
        print(f"✗ 错误: {e}")


def control_gripper(controller, arm):
    """控制夹爪"""
    print(f"\n控制夹爪")
    print("范围: 0-100 (0=完全闭合, 100=完全打开)")

    try:
        value = float(input("输入夹爪值: ").strip())

        if not (0 <= value <= 100):
            print("✗ 值必须在 0-100 之间")
            return

        if controller.control_gripper(arm, value, wait=True):
            print(f"✓ 夹爪已设置为 {value}")
        else:
            print("✗ 控制失败")

    except ValueError:
        print("✗ 输入格式错误，请输入数字")
    except Exception as e:
        print(f"✗ 错误: {e}")


def move_cartesian_position(controller, arm):
    """移动到笛卡尔坐标位置"""
    print(f"\n移动到笛卡尔位置 (XYZ + 姿态)")
    print("输入目标位置和姿态 (7个值)")
    print("格式: x y z qx qy qz qw")
    print("示例: 0.3 0.0 0.5 0 0 0 1")

    try:
        input_str = input("\n输入坐标: ").strip()
        pose = [float(x) for x in input_str.split()]

        if len(pose) != 7:
            print(f"✗ 需要 7 个值 (x y z qx qy qz qw)，您输入了 {len(pose)} 个")
            return

        quat = normalize_quaternion(pose[3:])
        if quat is None:
            print("✗ 四元数无效，已取消执行")
            return
        pose[3:] = quat

        if not is_pose_safe(pose):
            print("✗ 目标超出安全工作空间，已取消执行")
            return

        print(f"\n目标位置: X={pose[0]:.3f}, Y={pose[1]:.3f}, Z={pose[2]:.3f}")
        print(f"目标姿态 (四元数): qx={pose[3]:.3f}, qy={pose[4]:.3f}, qz={pose[5]:.3f}, qw={pose[6]:.3f}")
        print("自动执行（无确认）。如需中止，请立即按 Ctrl+C 或使用紧急停止。")

        if controller.move_pose(arm, pose, wait=True):
            print("✓ 到达目标位置")
            time.sleep(1)
            print_current_state(controller, arm)
        else:
            print("✗ 移动失败")

    except ValueError:
        print("✗ 输入格式错误，请输入数字")
    except Exception as e:
        print(f"✗ 错误: {e}")


def gripper_test_sequence(controller, arm):
    """夹爪开合测试序列"""
    print(f"\n夹爪开合测试")
    print("将执行: 完全打开 → 半开 → 完全闭合 → 半开")

    sequence = [
        (100, "完全打开"),
        (50, "半开"),
        (0, "完全闭合"),
        (50, "半开 (准备)")
    ]

    for value, desc in sequence:
        print(f"\n→ {desc} ({value})")
        if controller.control_gripper(arm, value, wait=True):
            time.sleep(1)
        else:
            print(f"✗ 失败于 {desc}")
            return

    print("\n✓ 夹爪测试完成")


def create_controller(mode: str):
    """根据模式创建控制器

    Args:
        mode: "real" 或 "mujoco"
    """
    test_config = config.copy()
    test_config["mode"] = mode

    if mode == "real":
        # Real 模式配置
        test_config["real"] = {
            "left_arm": config["real"]["left_arm"],
            "right_arm": config["real"]["right_arm"]
        }
        test_config["safety"] = config.get("safety", {})

        try:
            from robot.alicia_controller import AliciaDualArmController
            print("✓ Alicia-D SDK 已导入")
            return AliciaDualArmController(test_config)
        except ImportError as e:
            print(f"✗ 无法导入 Alicia-D SDK: {e}")
            print("请先安装: pip install alicia-d-sdk")
            sys.exit(1)

    elif mode == "mujoco":
        # MuJoCo 模式配置
        test_config["mujoco"] = config.get("mujoco", {})

        try:
            from robot.mujoco_controller import MuJoCoController
            print("✓ MuJoCo Controller 已导入")
            return MuJoCoController(test_config)
        except ImportError as e:
            print(f"✗ 无法导入 MuJoCo: {e}")
            print("请先安装: pip install mujoco")
            sys.exit(1)

    else:
        print(f"✗ 未知模式: {mode}")
        sys.exit(1)


def main():
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="单臂机械臂测试工具 (Real / MuJoCo)")
    parser.add_argument(
        "--mode",
        type=str,
        choices=["real", "mujoco"],
        default=None,
        help="运行模式: real (真实硬件) 或 mujoco (仿真)"
    )
    args = parser.parse_args()

    print("╔══════════════════════════════════════════╗")
    print("║   TeaBot 机械臂测试工具                 ║")
    print("╚══════════════════════════════════════════╝")

    # 选择模式
    if args.mode:
        mode = args.mode
        print(f"\n运行模式: {mode}")
    else:
        print("\n选择运行模式:")
        print("1. Real    - 真实硬件")
        print("2. MuJoCo  - 物理仿真")

        mode_choice = input("请选择 (1/2): ").strip()
        if mode_choice == "1":
            mode = "real"
        elif mode_choice == "2":
            mode = "mujoco"
        else:
            print("✗ 无效选择")
            return

    print(f"✓ 已选择模式: {mode}")
    warn_real_mode_if_needed(mode)

    # 根据 arm_mode 配置决定是否需要选择手臂
    arm_mode = config.get("arm_mode", "dual_arm")

    if arm_mode == "single_arm":
        # 单臂模式直接使用左臂
        arm = ArmSide.LEFT
        print(f"\n单臂模式 (使用左臂)")
    else:
        # 双臂模式需要选择
        print("\n选择要测试的手臂:")
        print("1. 左臂 (Left)")
        print("2. 右臂 (Right)")

        choice = input("请选择 (1/2): ").strip()
        if choice == "1":
            arm = ArmSide.LEFT
        elif choice == "2":
            arm = ArmSide.RIGHT
        else:
            print("✗ 无效选择")
            return

        print(f"✓ 已选择手臂: {arm.value}")

    # 创建控制器
    print(f"\n正在初始化控制器 ({mode} 模式)...")
    try:
        controller = create_controller(mode)
    except Exception as e:
        print(f"✗ 控制器初始化失败: {e}")
        return

    # 连接机械臂/仿真器
    if mode == "real":
        print("\n正在连接机械臂...")
        print("提示: 请确保机械臂已通过 USB 连接到电脑")
    else:
        print("\n正在启动 MuJoCo 仿真器...")

    if not controller.connect():
        if mode == "real":
            print("✗ 连接失败!")
            print("\n故障排查:")
            print("1. 检查 USB 连接")
            print("2. 检查机械臂是否上电")
            print("3. 检查串口权限 (Linux 需要 dialout 组)")
            print("   sudo usermod -a -G dialout $USER  # 然后注销重新登录")
            print("4. 检查串口设备: ls /dev/ttyACM* /dev/ttyUSB*")
            print("5. 尝试指定串口: 修改 config/robot_config.yaml")
            print("   left_arm.port: '/dev/ttyACM0'  # 常见为 ttyACM0")
        else:
            print("✗ 仿真器启动失败!")
            print("\n故障排查:")
            print("1. 检查模型文件是否存在: data/models/alicia_dual_arm.xml")
            print("2. 检查 MuJoCo 是否正确安装")
        return

    print("✓ 连接成功!" if mode == "real" else "✓ 仿真器已启动!")

    # 启用力矩 (仅 Real 模式需要)
    if mode == "real":
        print("\n正在启用力矩...")
        if controller.enable_torque(arm, True):
            print("✓ 力矩已启用")
        else:
            print("⚠ 力矩启用可能失败")

    # 主循环
    try:
        while True:
            print_menu()
            choice = input("\n请选择操作: ").strip()

            # 基础运动
            if choice == "1":
                move_to_home(controller, arm)
            elif choice == "2":
                move_to_preset_1(controller, arm)
            elif choice == "3":
                move_custom_smooth(controller, arm)

            elif choice == "4":
                move_cartesian_position(controller, arm)

            # 夹爪
            elif choice == "5":
                gripper_test_sequence(controller, arm)

            # 状态与安全
            elif choice == "6":
                print_current_state(controller, arm)
            elif choice == "7":
                print("\n⚠ 紧急停止 - 所有运动立即停止!")
                controller.emergency_stop()
                if mode == "real":
                    print("力矩已关闭，请确认安全后重新启用。")
                    try:
                        controller.reset()
                        controller.enable_torque(arm, True)
                        print("✓ 已尝试重新启用力矩")
                    except Exception as e:
                        print(f"⚠ 力矩重新启用失败: {e}")
                else:
                    print("仿真已停止运动，可继续执行其他操作。")

            elif choice == "0":
                print("\n正在退出...")
                break
            else:
                print("✗ 无效选择，请输入 0-7")

    except KeyboardInterrupt:
        print("\n\n⚠ 收到中断信号")

    finally:
        # 断开连接
        print("\n正在断开连接...")
        controller.disconnect()
        print("✓ 已断开连接")
        print("\n测试结束!")


if __name__ == "__main__":
    main()
