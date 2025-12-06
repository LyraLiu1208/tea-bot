"""仿真器抽象基类"""

from abc import ABC, abstractmethod
from typing import Dict, List, Optional
import sys
from pathlib import Path

# 添加父目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))
from robot.base_controller import ArmSide, ArmState


class BaseSimulator(ABC):
    """仿真器抽象基类

    所有仿真器（MuJoCo, PyBullet 等）都需要实现此接口
    """

    def __init__(self, model_path: str, config: Dict = None):
        """初始化仿真器

        Args:
            model_path: 模型文件路径（MJCF, URDF 等）
            config: 仿真配置
        """
        self.model_path = model_path
        self.config = config or {}
        self._is_running = False

    @abstractmethod
    def start(self, gui: bool = True) -> bool:
        """启动仿真

        Args:
            gui: 是否显示 GUI

        Returns:
            bool: 启动是否成功
        """
        pass

    @abstractmethod
    def stop(self):
        """停止仿真"""
        pass

    @abstractmethod
    def step(self, num_steps: int = 1):
        """推进仿真

        Args:
            num_steps: 仿真步数
        """
        pass

    @abstractmethod
    def reset(self):
        """重置仿真到初始状态"""
        pass

    @abstractmethod
    def set_joint_angles(self, arm: ArmSide, joint_angles: List[float]):
        """设置关节角度

        Args:
            arm: 左臂或右臂
            joint_angles: 6 个关节角度（弧度）
        """
        pass

    @abstractmethod
    def get_joint_angles(self, arm: ArmSide) -> List[float]:
        """获取关节角度

        Args:
            arm: 左臂或右臂

        Returns:
            List[float]: 6 个关节角度（弧度）
        """
        pass

    @abstractmethod
    def set_gripper(self, arm: ArmSide, value: float):
        """设置夹爪

        Args:
            arm: 左臂或右臂
            value: 夹爪开合度 0-100
        """
        pass

    @abstractmethod
    def get_end_effector_pose(self, arm: ArmSide) -> Dict:
        """获取末端执行器位姿

        Args:
            arm: 左臂或右臂

        Returns:
            Dict: {"position": [x,y,z], "quaternion": [qx,qy,qz,qw]}
        """
        pass

    def sync_from_controller(self, controller):
        """从控制器同步状态到仿真

        Args:
            controller: BaseDualArmController 实例
        """
        for arm in [ArmSide.LEFT, ArmSide.RIGHT]:
            state = controller.get_state(arm)
            if state:
                self.set_joint_angles(arm, state.joint_angles)
                self.set_gripper(arm, state.gripper_value)

    def get_state(self, arm: ArmSide) -> Optional[ArmState]:
        """获取仿真中的机械臂状态

        Args:
            arm: 左臂或右臂

        Returns:
            ArmState: 机械臂状态
        """
        try:
            joint_angles = self.get_joint_angles(arm)
            pose = self.get_end_effector_pose(arm)

            return ArmState(
                arm=arm,
                joint_angles=joint_angles,
                gripper_value=0.0,  # 需要子类实现
                end_effector_pose=pose,
                is_moving=False,
                torque_enabled=True
            )
        except Exception as e:
            print(f"Failed to get state: {e}")
            return None

    @property
    def is_running(self) -> bool:
        """仿真是否正在运行"""
        return self._is_running
