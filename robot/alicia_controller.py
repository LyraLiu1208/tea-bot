"""Alicia D 双臂机械臂控制器"""

import logging
import math
import time
from typing import List, Dict, Optional
from .base_controller import BaseDualArmController, ArmState, ArmSide

logger = logging.getLogger(__name__)


class AliciaDualArmController(BaseDualArmController):
    """Alicia D 双臂机械臂控制器

    基于 Alicia-D-SDK 控制真实双臂机械臂
    每个臂独立创建一个 SDK 实例
    """

    def __init__(self, config: Dict):
        super().__init__(config)
        self.real_config = config.get("real", {})
        self.wait_params = {
            "timeout": self.real_config.get("wait_timeout", 6.0),
            "poll_interval": self.real_config.get("wait_poll_interval", 0.05),
            "tolerance_deg": self.real_config.get("wait_tolerance_deg", 3.0)
        }
        self.pose_control = self.real_config.get("pose_control", {})
        self.pose_wait_params = {
            "pos_tolerance": self.pose_control.get("pos_tolerance", 0.005),
            "ori_tolerance_deg": self.pose_control.get("ori_tolerance_deg", 5.0)
        }

        # 导入 Alicia D SDK
        try:
            from alicia_d_sdk import create_robot
            self.create_robot = create_robot
        except ImportError:
            logger.error("alicia_d_sdk not found! Please install: pip install alicia-d-sdk")
            raise

        # 创建两个机械臂实例
        left_config = self.real_config.get("left_arm", {})
        right_config = self.real_config.get("right_arm", {})

        self.left_robot = self.create_robot(
            port=left_config.get("port", ""),  # 空字符串表示自动搜索
            baudrate=left_config.get("baudrate", 1000000),
            robot_version=left_config.get("robot_version", "v5_6"),
            gripper_type=left_config.get("gripper_type", "50mm"),
            speed_deg_s=left_config.get("speed_deg_s", 20.0),
            debug_mode=False
        )

        self.right_robot = self.create_robot(
            port=right_config.get("port", ""),
            baudrate=right_config.get("baudrate", 1000000),
            robot_version=right_config.get("robot_version", "v5_6"),
            gripper_type=right_config.get("gripper_type", "50mm"),
            speed_deg_s=right_config.get("speed_deg_s", 20.0),
            debug_mode=False
        )

        logger.info("Alicia D dual-arm controller initialized")

    def _get_robot(self, arm: ArmSide):
        """获取对应机械臂的 SDK 实例"""
        return self.left_robot if arm == ArmSide.LEFT else self.right_robot

    def connect(self) -> bool:
        """连接双臂机械臂"""
        logger.info("Connecting to Alicia D dual-arm robot...")

        # 连接左臂
        if not self.left_robot.connect():
            logger.error("Failed to connect to left arm")
            return False
        logger.info("✓ Left arm connected")

        # 连接右臂
        if not self.right_robot.connect():
            logger.error("Failed to connect to right arm")
            self.left_robot.disconnect()
            return False
        logger.info("✓ Right arm connected")

        self._is_connected = True
        logger.info("✓ Dual-arm connected successfully!")
        return True

    def disconnect(self) -> bool:
        """断开连接"""
        logger.info("Disconnecting from Alicia D robot...")

        self.left_robot.disconnect()
        self.right_robot.disconnect()

        self._is_connected = False
        logger.info("✓ Disconnected")
        return True

    def move_joint(
        self,
        arm: ArmSide,
        joint_angles: List[float],
        wait: bool = True
    ) -> bool:
        """关节空间运动"""
        if not self._is_connected:
            logger.error("Not connected!")
            return False

        if len(joint_angles) != 6:
            logger.error(f"Expected 6 joint angles, got {len(joint_angles)}")
            return False

        robot = self._get_robot(arm)

        try:
            # SDK API: set_joint_target(target_joints, joint_format='rad')
            robot.set_joint_target(target_joints=joint_angles, joint_format='rad')
            if wait:
                return self._wait_for_joint_target(arm, joint_angles)
            return True
        except Exception as e:
            logger.error(f"Failed to move {arm.value} arm: {e}")
            return False

    def move_pose(
        self,
        arm: ArmSide,
        pose: List[float],
        wait: bool = True
    ) -> bool:
        """笛卡尔空间运动

        Args:
            pose: [x, y, z, qx, qy, qz, qw] 位置 + 四元数
        """
        if not self._is_connected:
            logger.error("Not connected!")
            return False

        robot = self._get_robot(arm)

        try:
            # SDK API: set_pose_target(target_pose, ..., execute=True)
            control_kwargs = self._build_pose_control_kwargs()
            robot.set_pose_target(target_pose=pose, **control_kwargs)
            if wait:
                return self._wait_for_pose_target(arm, pose)
            return True
        except Exception as e:
            logger.error(f"Failed to move {arm.value} arm to pose: {e}")
            return False

    def control_gripper(
        self,
        arm: ArmSide,
        value: float,
        wait: bool = True
    ) -> bool:
        """控制夹爪

        Args:
            value: 0-100，0=完全闭合，100=完全打开
        """
        if not self._is_connected:
            logger.error("Not connected!")
            return False

        if not (0 <= value <= 100):
            logger.error(f"Gripper value must be 0-100, got {value}")
            return False

        robot = self._get_robot(arm)

        try:
            # SDK API: set_gripper_target(value=..., wait_for_completion=...)
            robot.set_gripper_target(
                value=value,
                wait_for_completion=wait
            )
            return True
        except Exception as e:
            logger.error(f"Failed to control {arm.value} gripper: {e}")
            return False

    def get_state(self, arm: ArmSide) -> Optional[ArmState]:
        """获取机械臂当前状态"""
        if not self._is_connected:
            return None

        robot = self._get_robot(arm)

        try:
            # 获取关节角度
            joints = robot.get_joints()
            if joints is None:
                return None

            # 获取夹爪状态
            gripper = robot.get_gripper()
            if gripper is None:
                gripper = 0.0

            # 获取末端位姿
            pose_dict = robot.get_pose()

            return ArmState(
                arm=arm,
                joint_angles=joints,
                gripper_value=gripper,
                end_effector_pose=pose_dict,
                is_moving=False,  # SDK 没有提供此状态
                torque_enabled=True
            )
        except Exception as e:
            logger.error(f"Failed to get {arm.value} state: {e}")
            return None

    def enable_torque(self, arm: ArmSide, enable: bool) -> bool:
        """启用/禁用力矩"""
        if not self._is_connected:
            logger.error("Not connected!")
            return False

        robot = self._get_robot(arm)

        try:
            command = "on" if enable else "off"
            success = robot.torque_control(command)
            return success
        except Exception as e:
            logger.error(f"Failed to control {arm.value} torque: {e}")
            return False

    def set_home(self, arm: ArmSide) -> bool:
        """回到初始位置"""
        if not self._is_connected:
            logger.error("Not connected!")
            return False

        logger.info(f"Moving {arm.value} arm to home position")

        robot = self._get_robot(arm)
        try:
            robot.set_home()
        except AttributeError:
            logger.warning("SDK does not provide set_home(), falling back to joint command")
            if not self.move_joint(arm, [0.0] * 6, wait=True):
                return False
        except Exception as e:
            logger.error(f"Failed to execute SDK home: {e}")
            return False

        # 设置夹爪到半开状态
        if not self.control_gripper(arm, 50, wait=True):
            logger.warning(f"Failed to set {arm.value} gripper to home position")

        logger.info(f"✓ {arm.value} arm at home position")
        return True

    def emergency_stop(self) -> bool:
        """紧急停止"""
        logger.warning("⚠ EMERGENCY STOP!")

        # 关闭双臂力矩
        self.enable_torque(ArmSide.LEFT, False)
        self.enable_torque(ArmSide.RIGHT, False)

        self._emergency_stopped = True
        return True

    def reset(self) -> bool:
        """复位系统"""
        logger.info("Resetting system...")

        # 重新启用力矩
        self.enable_torque(ArmSide.LEFT, True)
        self.enable_torque(ArmSide.RIGHT, True)

        self._emergency_stopped = False
        logger.info("✓ System reset")
        return True

    # ======================
    # 辅助函数
    # ======================

    def _wait_for_joint_target(self, arm: ArmSide, target_angles: List[float]) -> bool:
        """轮询状态直到到达目标或超时"""
        tolerance_rad = math.radians(self.wait_params["tolerance_deg"])
        timeout = self.wait_params["timeout"]
        poll = self.wait_params["poll_interval"]
        end_time = time.time() + timeout
        last_error = None

        while time.time() < end_time:
            state = self.get_state(arm)
            if state:
                errors = [
                    abs(state.joint_angles[i] - target_angles[i])
                    for i in range(6)
                ]
                max_error = max(errors)
                last_error = math.degrees(max_error)
                if max_error <= tolerance_rad:
                    return True
            time.sleep(poll)

        if last_error is not None:
            logger.warning(
                f"{arm.value} joint target timeout (max error {last_error:.2f}°)"
            )
        else:
            logger.warning(f"{arm.value} joint target timeout (no state reading)")
        return False

    def _quaternion_distance(self, target, current):
        dot = sum(t * c for t, c in zip(target, current))
        dot = max(min(dot, 1.0), -1.0)
        angle = 2 * math.acos(abs(dot))
        return angle

    def _wait_for_pose_target(self, arm: ArmSide, target_pose: List[float]) -> bool:
        """等待末端到达目标位姿"""
        pos_tol = self.pose_wait_params["pos_tolerance"]
        ori_tol = math.radians(self.pose_wait_params["ori_tolerance_deg"])
        timeout = self.wait_params["timeout"]
        poll = self.wait_params["poll_interval"]
        end_time = time.time() + timeout
        last_pos_err = None
        last_ori_err = None

        target_quat = target_pose[3:]
        norm = math.sqrt(sum(q * q for q in target_quat))
        if norm < 1e-6:
            logger.warning("Invalid target quaternion, skipping pose wait")
            return True
        target_quat = [q / norm for q in target_quat]

        while time.time() < end_time:
            state = self.get_state(arm)
            if state and state.end_effector_pose:
                pos = state.end_effector_pose.get("position")
                quat = state.end_effector_pose.get("quaternion")
                if pos and quat:
                    pos_err = math.sqrt(sum((pos[i] - target_pose[i]) ** 2 for i in range(3)))
                    norm_quat = math.sqrt(sum(q * q for q in quat))
                    if norm_quat > 1e-6:
                        quat = [q / norm_quat for q in quat]
                    ori_err = self._quaternion_distance(target_quat, quat)
                    last_pos_err = pos_err
                    last_ori_err = math.degrees(ori_err)
                    if pos_err <= pos_tol and ori_err <= ori_tol:
                        return True
            time.sleep(poll)

        logger.warning(
            f"{arm.value} pose target timeout (pos err {last_pos_err or 'NA'}, "
            f"ori err {last_ori_err or 'NA'} deg)"
        )
        return False

    def _build_pose_control_kwargs(self):
        """构建传给 set_pose_target 的参数"""
        defaults = {
            "backend": "numpy",
            "method": "dls",
            "display": False,
            "tolerance": self.pose_control.get("tolerance", 0.0005),
            "max_iters": self.pose_control.get("max_iters", 200),
            "multi_start": self.pose_control.get("multi_start", 0),
            "use_random_init": self.pose_control.get("use_random_init", False),
            "speed_factor": self.pose_control.get("speed_factor", 1.0),
            "execute": self.pose_control.get("execute", True)
        }
        return defaults
