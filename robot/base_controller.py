"""双臂机械臂控制器抽象基类"""

from abc import ABC, abstractmethod
from typing import List, Dict, Optional
from dataclasses import dataclass
from enum import Enum


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
