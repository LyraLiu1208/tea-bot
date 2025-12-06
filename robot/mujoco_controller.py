"""MuJoCo 仿真控制器

将 MuJoCo 仿真器包装为统一的控制器接口
与 Mock 和 Real 控制器使用相同的 API
"""

import logging
from typing import List, Dict, Optional
from .base_controller import BaseDualArmController, ArmState, ArmSide

logger = logging.getLogger(__name__)


class MuJoCoController(BaseDualArmController):
    """MuJoCo 仿真控制器

    将 MuJoCo 仿真器适配为标准控制器接口
    模拟真实硬件行为，但在物理仿真环境中运行
    """

    def __init__(self, config: Dict):
        super().__init__(config)
        self.mujoco_config = config.get("mujoco", {})
        self.simulator = None

        # 导入仿真器
        try:
            import sys
            from pathlib import Path
            sys.path.insert(0, str(Path(__file__).parent.parent))
            from simulator.mujoco_simulator import MuJoCoSimulator
            self.SimulatorClass = MuJoCoSimulator
        except ImportError as e:
            logger.error(f"Failed to import MuJoCo simulator: {e}")
            raise

        logger.info("MuJoCo controller initialized")

    def connect(self) -> bool:
        """启动 MuJoCo 仿真器"""
        logger.info("Starting MuJoCo simulator...")

        model_path = self.mujoco_config.get(
            "model_path",
            "data/models/alicia_dual_arm.xml"
        )
        gui = self.mujoco_config.get("gui", True)

        try:
            self.simulator = self.SimulatorClass(model_path, self.mujoco_config)
            success = self.simulator.start(gui=gui)

            if success:
                self._is_connected = True
                logger.info("✓ MuJoCo simulator started")
                return True
            else:
                logger.error("Failed to start MuJoCo simulator")
                return False

        except Exception as e:
            logger.error(f"Failed to start simulator: {e}")
            return False

    def disconnect(self) -> bool:
        """停止仿真器"""
        logger.info("Stopping MuJoCo simulator...")

        if self.simulator:
            self.simulator.stop()
            self.simulator = None

        self._is_connected = False
        logger.info("✓ Simulator stopped")
        return True

    def move_joint(
        self,
        arm: ArmSide,
        joint_angles: List[float],
        wait: bool = True
    ) -> bool:
        """关节空间运动（仿真）"""
        if not self._is_connected or not self.simulator:
            logger.error("Simulator not running!")
            return False

        if len(joint_angles) != 6:
            logger.error(f"Expected 6 joint angles, got {len(joint_angles)}")
            return False

        try:
            # 设置关节角度
            self.simulator.set_joint_angles(arm, joint_angles)

            # 推进仿真以反映变化
            if wait:
                for _ in range(100):  # 仿真 0.2 秒
                    self.simulator.step()

            return True
        except Exception as e:
            logger.error(f"Failed to move {arm.value} arm in simulation: {e}")
            return False

    def move_pose(
        self,
        arm: ArmSide,
        pose: List[float],
        wait: bool = True
    ) -> bool:
        """笛卡尔空间运动（仿真）

        注意：MuJoCo 模式下需要先通过 IK 求解再设置关节角度
        """
        if not self._is_connected or not self.simulator:
            logger.error("Simulator not running!")
            return False

        logger.warning(
            "MuJoCo controller does not support direct pose control. "
            "Use move_joint() with IK solution instead."
        )
        return False

    def control_gripper(
        self,
        arm: ArmSide,
        value: float,
        wait: bool = True
    ) -> bool:
        """控制夹爪（仿真）"""
        if not self._is_connected or not self.simulator:
            logger.error("Simulator not running!")
            return False

        if not (0 <= value <= 100):
            logger.error(f"Gripper value must be 0-100, got {value}")
            return False

        try:
            self.simulator.set_gripper(arm, value)

            # 推进仿真
            if wait:
                for _ in range(50):  # 仿真 0.1 秒
                    self.simulator.step()

            return True
        except Exception as e:
            logger.error(f"Failed to control {arm.value} gripper in simulation: {e}")
            return False

    def get_state(self, arm: ArmSide) -> Optional[ArmState]:
        """获取当前状态（仿真）"""
        if not self._is_connected or not self.simulator:
            return None

        try:
            # 从仿真器获取关节角度
            joint_angles = self.simulator.get_joint_angles(arm)

            # 获取末端位姿
            ee_pose = self.simulator.get_end_effector_pose(arm)

            # MuJoCo 没有夹爪状态读取，返回中间值
            gripper_value = 50.0

            return ArmState(
                arm=arm,
                joint_angles=joint_angles,
                gripper_value=gripper_value,
                end_effector_pose=ee_pose,
                is_moving=False,
                torque_enabled=True
            )
        except Exception as e:
            logger.error(f"Failed to get {arm.value} state from simulation: {e}")
            return None

    def enable_torque(self, arm: ArmSide, enable: bool) -> bool:
        """启用/禁用力矩（仿真中无效）"""
        logger.info(f"Torque control not applicable in MuJoCo simulation")
        return True

    def set_home(self, arm: ArmSide) -> bool:
        """回到初始位置"""
        if not self._is_connected or not self.simulator:
            logger.error("Simulator not running!")
            return False

        logger.info(f"Moving {arm.value} arm to home position")

        # 移动到默认位置（全0）
        if not self.move_joint(arm, [0.0] * 6, wait=True):
            return False

        # 设置夹爪到半开状态
        self.control_gripper(arm, 50, wait=True)

        logger.info(f"✓ {arm.value} arm at home position")
        return True

    def emergency_stop(self) -> bool:
        """紧急停止（仿真）"""
        logger.warning("⚠ EMERGENCY STOP (simulation)")
        self._emergency_stopped = True
        return True

    def reset(self) -> bool:
        """复位系统（仿真）"""
        logger.info("Resetting simulation...")

        if self.simulator:
            self.simulator.reset()

        self._emergency_stopped = False
        logger.info("✓ Simulation reset")
        return True
