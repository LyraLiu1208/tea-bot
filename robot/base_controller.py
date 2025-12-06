"""双臂机械臂控制器抽象基类"""

import time
import logging
from abc import ABC, abstractmethod
from typing import List, Dict, Optional
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class ArmSide(str, Enum):
    """机械臂左右标识"""
    LEFT = "left"
    RIGHT = "right"


@dataclass
class ArmState:
    """单臂状态"""
    arm: ArmSide
    joint_angles: List[float]  # 关节角度（弧度）
    gripper_value: float  # 夹爪开合度 0-100
    end_effector_pose: Optional[Dict] = None  # 末端位姿
    is_moving: bool = False
    torque_enabled: bool = True
    error: Optional[str] = None


class BaseDualArmController(ABC):
    """双臂机械臂控制器抽象基类

    所有控制器（Mock 或 Alicia D）都需要实现此接口
    """

    def __init__(self, config: Dict):
        """初始化控制器

        Args:
            config: 配置字典
        """
        self.config = config
        self._is_connected = False
        self._emergency_stopped = False

    @abstractmethod
    def connect(self) -> bool:
        """连接双臂机械臂

        Returns:
            bool: 连接是否成功
        """
        pass

    @abstractmethod
    def disconnect(self) -> bool:
        """断开连接

        Returns:
            bool: 断开是否成功
        """
        pass

    @abstractmethod
    def move_joint(
        self,
        arm: ArmSide,
        joint_angles: List[float],
        wait: bool = True
    ) -> bool:
        """关节空间运动

        Args:
            arm: 左臂或右臂
            joint_angles: 6个关节角度（弧度）
            wait: 是否等待运动完成

        Returns:
            bool: 执行是否成功
        """
        pass

    @abstractmethod
    def move_pose(
        self,
        arm: ArmSide,
        pose: List[float],
        wait: bool = True
    ) -> bool:
        """笛卡尔空间运动

        Args:
            arm: 左臂或右臂
            pose: [x, y, z, qx, qy, qz, qw] 位置+四元数
            wait: 是否等待运动完成

        Returns:
            bool: 执行是否成功
        """
        pass

    @abstractmethod
    def control_gripper(
        self,
        arm: ArmSide,
        value: float,
        wait: bool = True
    ) -> bool:
        """控制夹爪

        Args:
            arm: 左臂或右臂
            value: 开合度 0(闭合) - 100(张开)
            wait: 是否等待运动完成

        Returns:
            bool: 执行是否成功
        """
        pass

    @abstractmethod
    def get_state(self, arm: ArmSide) -> Optional[ArmState]:
        """获取机械臂当前状态

        Args:
            arm: 左臂或右臂

        Returns:
            ArmState: 机械臂状态，失败返回 None
        """
        pass

    @abstractmethod
    def enable_torque(self, arm: ArmSide, enable: bool) -> bool:
        """启用/禁用力矩（用于示教模式）

        Args:
            arm: 左臂或右臂
            enable: True=启用力矩, False=禁用力矩（可手动拖动）

        Returns:
            bool: 执行是否成功
        """
        pass

    @abstractmethod
    def set_home(self, arm: ArmSide) -> bool:
        """回到初始位置

        Args:
            arm: 左臂或右臂

        Returns:
            bool: 执行是否成功
        """
        pass

    @abstractmethod
    def emergency_stop(self) -> bool:
        """紧急停止（停止所有运动）

        Returns:
            bool: 执行是否成功
        """
        pass

    @abstractmethod
    def reset(self) -> bool:
        """复位系统（清除紧急停止等）

        Returns:
            bool: 执行是否成功
        """
        pass

    @property
    def is_connected(self) -> bool:
        """是否已连接"""
        return self._is_connected

    @property
    def is_emergency_stopped(self) -> bool:
        """是否处于紧急停止状态"""
        return self._emergency_stopped

    def move_joint_smooth(
        self,
        arm: ArmSide,
        target_angles: List[float],
        steps: int = 100,
        step_delay: float = 0.005
    ) -> bool:
        """关节空间平滑运动（带梯形速度轨迹规划）

        使用梯形速度曲线进行轨迹规划，包含加速、匀速、减速三个阶段。
        这是工业机器人最常用的运动规划方法，确保平滑且高效的运动。

        Args:
            arm: 左臂或右臂
            target_angles: 目标关节角度（弧度，6个值）
            steps: 插值步数（默认100步，更平滑）
            step_delay: 每步之间的延迟时间（秒，默认0.005s）

        Returns:
            bool: 执行是否成功
        """
        if not self._is_connected:
            logger.error("Not connected!")
            return False

        if len(target_angles) != 6:
            logger.error(f"Expected 6 joint angles, got {len(target_angles)}")
            return False

        # 获取当前关节角度
        current_state = self.get_state(arm)
        if not current_state:
            logger.error(f"Failed to get current state of {arm.value} arm")
            return False

        current_angles = current_state.joint_angles

        logger.info(f"Planning trapezoidal trajectory with {steps} steps")

        try:
            # 梯形速度曲线参数
            # 加速阶段: 0-30%, 匀速阶段: 30%-70%, 减速阶段: 70%-100%
            accel_ratio = 0.3
            decel_ratio = 0.7

            # 生成梯形速度轨迹
            for i in range(1, steps + 1):
                # 当前步进比例
                t = i / steps

                # 梯形速度曲线的位置映射
                # 使用积分得到位置（速度的积分）
                if t < accel_ratio:
                    # 加速阶段: 二次曲线 (速度线性增加)
                    s = 0.5 * (t / accel_ratio) ** 2 * accel_ratio
                elif t < decel_ratio:
                    # 匀速阶段: 线性
                    s = accel_ratio * 0.5 + (t - accel_ratio)
                else:
                    # 减速阶段: 二次曲线 (速度线性减少)
                    total_accel = accel_ratio * 0.5
                    total_const = (decel_ratio - accel_ratio)
                    decel_progress = (t - decel_ratio) / (1 - decel_ratio)
                    s = total_accel + total_const + (1 - decel_ratio) * (decel_progress - 0.5 * decel_progress ** 2)

                # 限制 s 在 [0, 1] 范围内
                s = max(0.0, min(1.0, s))

                # 计算当前步的目标角度
                interpolated_angles = [
                    current_angles[j] + s * (target_angles[j] - current_angles[j])
                    for j in range(6)
                ]

                # 执行单步运动
                if not self.move_joint(arm, interpolated_angles, wait=False):
                    logger.error(f"Failed at step {i}/{steps}")
                    return False

                # 延迟等待
                time.sleep(step_delay)

            logger.info(f"✓ Smooth trajectory completed for {arm.value} arm")

            # 验证是否到达目标位置
            time.sleep(0.1)  # 等待稳定
            return self.verify_position_reached(arm, target_angles, tolerance_deg=3.0)

        except Exception as e:
            logger.error(f"Smooth motion failed: {e}")
            return False

    def verify_position_reached(
        self,
        arm: ArmSide,
        target_angles: List[float],
        tolerance_deg: float = 2.0
    ) -> bool:
        """验证机械臂是否到达目标位置

        通过比较当前关节角度与目标角度，判断运动是否成功到达。
        适用于检测运动完成后的实际位置。

        Args:
            arm: 左臂或右臂
            target_angles: 目标关节角度（弧度，6个值）
            tolerance_deg: 允许误差（度，默认2°）

        Returns:
            bool: True 表示所有关节都在误差范围内，False 表示至少有一个关节超出误差
        """
        import math

        if not self._is_connected:
            logger.error("Not connected!")
            return False

        # 获取当前状态
        current_state = self.get_state(arm)
        if not current_state:
            logger.error(f"Failed to get current state of {arm.value} arm")
            return False

        current_angles = current_state.joint_angles
        if len(current_angles) != 6 or len(target_angles) != 6:
            logger.error("Invalid joint angles length")
            return False

        # 转换容差到弧度
        tolerance_rad = math.radians(tolerance_deg)

        # 检查每个关节
        all_reached = True
        max_error_deg = 0.0

        for i in range(6):
            error = abs(current_angles[i] - target_angles[i])
            error_deg = math.degrees(error)

            if error > tolerance_rad:
                all_reached = False
                logger.warning(
                    f"Joint{i+1} not reached: current={math.degrees(current_angles[i]):.2f}°, "
                    f"target={math.degrees(target_angles[i]):.2f}°, error={error_deg:.2f}°"
                )

            max_error_deg = max(max_error_deg, error_deg)

        if all_reached:
            logger.info(f"✓ Position reached! Max error: {max_error_deg:.2f}°")
        else:
            logger.warning(f"✗ Position NOT reached! Max error: {max_error_deg:.2f}° (tolerance: {tolerance_deg}°)")

        return all_reached
