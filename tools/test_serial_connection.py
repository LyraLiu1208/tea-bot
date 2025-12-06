#!/usr/bin/env python3
"""串口连接快速测试工具

用于验证机械臂串口连接是否正常
"""

import sys
import time

def test_serial_connection(port, baudrate=1000000):
    """测试串口连接

    Args:
        port: 串口设备路径 (如 /dev/ttyACM0)
        baudrate: 波特率 (默认 1000000)
    """
    try:
        import serial
    except ImportError:
        print("✗ pyserial 未安装")
        print("安装: pip install pyserial")
        return False

    print(f"\n测试串口连接...")
    print(f"  端口: {port}")
    print(f"  波特率: {baudrate}")
    print(f"  超时: 1.0s")

    try:
        # 尝试打开串口
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1.0,
            write_timeout=1.0
        )

        print("\n✓ 串口打开成功!")
        print(f"  实际端口: {ser.port}")
        print(f"  实际波特率: {ser.baudrate}")
        print(f"  数据位: {ser.bytesize}")
        print(f"  停止位: {ser.stopbits}")
        print(f"  校验位: {ser.parity}")

        # 检查串口状态
        if ser.is_open:
            print("  状态: 已打开")

        # 关闭串口
        ser.close()
        print("\n✓ 串口测试成功!")
        return True

    except serial.SerialException as e:
        print(f"\n✗ 串口连接失败: {e}")
        print("\n请检查:")
        print("1. USB 是否连接")
        print("2. 机械臂是否上电")
        print("3. 串口权限:")
        print(f"   ls -l {port}")
        print("   sudo usermod -a -G dialout $USER")
        print("4. 设备是否被占用:")
        print(f"   sudo fuser {port}")
        return False

    except Exception as e:
        print(f"\n✗ 未知错误: {e}")
        return False


def auto_detect_ports():
    """自动检测可用的串口设备"""
    import glob

    print("\n正在扫描串口设备...")

    # Linux 常见串口设备
    patterns = [
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
        "/dev/cu.usbmodem*",  # macOS
        "/dev/cu.usbserial*"  # macOS
    ]

    found_ports = []
    for pattern in patterns:
        ports = glob.glob(pattern)
        found_ports.extend(ports)

    if found_ports:
        print(f"\n发现 {len(found_ports)} 个串口设备:")
        for i, port in enumerate(found_ports, 1):
            print(f"  {i}. {port}")
        return found_ports
    else:
        print("\n✗ 未发现串口设备")
        print("\n请检查:")
        print("1. USB 是否连接")
        print("2. 机械臂是否上电")
        print("3. 驱动是否安装")
        return []


def test_alicia_sdk(port=""):
    """测试 Alicia-D SDK 连接

    Args:
        port: 串口设备 (留空自动搜索)
    """
    print("\n测试 Alicia-D SDK 连接...")
    print(f"  端口: {port if port else '自动搜索'}")

    try:
        from alicia_d_sdk import create_robot
    except ImportError:
        print("✗ alicia-d-sdk 未安装")
        print("安装: pip install alicia-d-sdk")
        return False

    try:
        # 创建机器人实例
        robot = create_robot(
            port=port,
            baudrate=1000000,
            robot_version="v5_6",
            gripper_type="50mm",
            speed_deg_s=20.0,
            debug_mode=True  # 开启调试模式
        )

        print("\n正在连接...")
        if robot.connect():
            print("✓ Alicia-D SDK 连接成功!")

            # 尝试获取关节角度
            print("\n获取当前关节角度...")
            joints = robot.get_joints()
            if joints:
                print(f"  关节角度: {joints}")

            # 获取夹爪状态
            gripper = robot.get_gripper()
            if gripper is not None:
                print(f"  夹爪状态: {gripper}")

            # 断开连接
            robot.disconnect()
            print("\n✓ SDK 测试成功!")
            return True
        else:
            print("✗ Alicia-D SDK 连接失败")
            return False

    except Exception as e:
        print(f"✗ SDK 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """主函数"""
    print("╔══════════════════════════════════════════╗")
    print("║   串口连接测试工具                       ║")
    print("╚══════════════════════════════════════════╝")

    print("\n选择测试模式:")
    print("1. 自动检测串口设备")
    print("2. 测试指定串口 (低级)")
    print("3. 测试 Alicia-D SDK (高级)")
    print("0. 退出")

    choice = input("\n请选择 (0-3): ").strip()

    if choice == "1":
        # 自动检测
        ports = auto_detect_ports()
        if ports:
            print("\n是否测试这些设备? (y/n): ", end="")
            if input().strip().lower() == 'y':
                for port in ports:
                    print(f"\n{'='*50}")
                    test_serial_connection(port)
                    time.sleep(0.5)

    elif choice == "2":
        # 测试指定串口
        port = input("\n输入串口设备 (如 /dev/ttyACM0): ").strip()
        if port:
            baudrate = input("波特率 (默认 1000000): ").strip()
            baudrate = int(baudrate) if baudrate else 1000000
            test_serial_connection(port, baudrate)

    elif choice == "3":
        # 测试 Alicia-D SDK
        port = input("\n输入串口设备 (留空自动搜索): ").strip()
        test_alicia_sdk(port)

    elif choice == "0":
        print("退出")

    else:
        print("✗ 无效选择")


if __name__ == "__main__":
    main()
