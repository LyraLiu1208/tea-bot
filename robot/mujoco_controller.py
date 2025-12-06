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

        # 根据 arm_mode 选择模型
        arm_mode = self.config.get("arm_mode", "dual_arm")
        if arm_mode == "single_arm":
            default_model = self.mujoco_config.get(
                "single_arm_model",
                "data/models/alicia_duo_with_gripper.xml"
            )
        else:
            default_model = self.mujoco_config.get(
                "dual_arm_model",
                "data/models/alicia_dual_arm.xml"
            )

        # 允许通过 model_path 覆盖
        model_path = self.mujoco_config.get("model_path", default_model)
        gui = self.mujoco_config.get("gui", True)

        logger.info(f"Loading model: {model_path} (arm_mode: {arm_mode})")

        try:
            self.simulator = self.SimulatorClass(model_path, self.mujoco_config)
            success = self.simulator.start(gui=gui)

            if success:
                self._is_connected = True
                # 初始化夹爪为半开，保持与真机一致
                try:
                    for arm in [ArmSide.LEFT, ArmSide.RIGHT]:
                        self.simulator.set_gripper(arm, 50.0)
                except Exception as init_err:
                    logger.warning(f"Failed to initialize gripper state: {init_err}")
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
            # 设置关节角度（通过 position 执行器）
            self.simulator.set_joint_angles(arm, joint_angles)

            # 推进仿真以反映变化
            if wait:
                # 等待模式: 仿真足够长时间让关节到达目标位置
                # position 执行器会平滑驱动关节，需要更多仿真步
                for _ in range(500):  # 仿真 1.0 秒
                    self.simulator.step()
            else:
                # 非等待模式: 少量仿真步让角度更新可见，避免卡顿
                for _ in range(10):  # 仿真 0.02 秒
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

        通过阻尼伪逆 IK 将目标位姿转换为关节角度，再执行平滑轨迹
        """
        if not self._is_connected or not self.simulator:
            logger.error("Simulator not running!")
            return False

        solution = self.simulator.solve_inverse_kinematics(arm, pose)
        if solution is None:
            logger.error("Failed to solve IK for target pose")
            return False

        # 复用平滑运动，保证与真机体验一致
        steps = self.mujoco_config.get("pose_steps", 80)
        step_delay = self.mujoco_config.get("pose_step_delay", 0.006)
        return self.move_joint_smooth(
            arm,
            solution,
            steps=steps,
            step_delay=step_delay
        )

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

            # 夹爪状态由模拟器维护
            gripper_value = self.simulator.get_gripper_value(arm)

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

    def move_joint_smooth(
        self,
        arm: ArmSide,
        target_angles: List[float],
        steps: int = 100,
        step_delay: float = 0.005
    ) -> bool:
        """平滑运动（MuJoCo 优化版本）

        MuJoCo 仿真需要额外的稳定时间让 position 执行器驱动关节到达目标。
        这个实现在轨迹规划后增加足够的仿真步数。
        """
        import time
        import math

        if not self._is_connected or not self.simulator:
            logger.error("Simulator not running!")
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
        smooth_steps = max(5, steps)
        step_delay = max(step_delay, 0.0)
        logger.info(
            f"Planning trapezoidal trajectory with {smooth_steps} steps "
            f"(step_delay={step_delay}s)"
        )

        # 将真实世界的时间配置映射为仿真步数
        sim_dt = getattr(
            getattr(getattr(self.simulator, "model", None), "opt", None),
            "timestep",
            0.002
        )
        min_sim_steps = self.mujoco_config.get("smooth_min_sim_steps", 50)
        sim_steps_per_interval = max(
            int(round(step_delay / sim_dt)),
            min_sim_steps
        )
        settle_tolerance_deg = self.mujoco_config.get(
            "smooth_tolerance_deg", 3.0
        )
        settle_timeout = self.mujoco_config.get(
            "smooth_settle_timeout", 3.0
        )
        settle_batch_steps = self.mujoco_config.get(
            "smooth_settle_batch_steps", 50
        )

        try:
            # 梯形速度曲线参数
            accel_ratio = 0.3
            decel_ratio = 0.7

            print(f"🔄 执行平滑轨迹（{smooth_steps} 个关键点）...")

            # 生成梯形速度轨迹
            for i in range(1, smooth_steps + 1):
                t = i / smooth_steps

                # 梯形速度曲线的位置映射
                if t < accel_ratio:
                    s = 0.5 * (t / accel_ratio) ** 2 * accel_ratio
                elif t < decel_ratio:
                    s = accel_ratio * 0.5 + (t - accel_ratio)
                else:
                    total_accel = accel_ratio * 0.5
                    total_const = (decel_ratio - accel_ratio)
                    decel_progress = (t - decel_ratio) / (1 - decel_ratio)
                    s = total_accel + total_const + (1 - decel_ratio) * (decel_progress - 0.5 * decel_progress ** 2)

                s = max(0.0, min(1.0, s))

                # 计算当前步的目标角度
                interpolated_angles = [
                    current_angles[j] + s * (target_angles[j] - current_angles[j])
                    for j in range(6)
                ]

                # 设置目标并仿真
                self.simulator.set_joint_angles(arm, interpolated_angles)

                # 每个关键点仿真一定步数，让执行器有时间跟踪
                self.simulator.step(num_steps=sim_steps_per_interval)

                # 通过 sleep 模拟真实执行时间，否则仿真会“瞬移”
                if step_delay > 0:
                    time.sleep(step_delay)

                # 显示进度
                if i % 5 == 0:
                    progress = i / smooth_steps * 100
                    print(f"   进度: {progress:.0f}%")

            # 最终保持目标并等待误差进入容差
            print("   稳定中...")
            self.simulator.set_joint_angles(arm, target_angles)
            start_time = time.time()
            tolerance_rad = math.radians(settle_tolerance_deg)
            max_error_deg = float("inf")

            while time.time() - start_time < settle_timeout:
                self.simulator.step(num_steps=settle_batch_steps)
                state = self.get_state(arm)
                if not state:
                    break

                errors = [
                    abs(state.joint_angles[j] - target_angles[j])
                    for j in range(6)
                ]
                max_error_deg = max(math.degrees(e) for e in errors)

                if max(errors) <= tolerance_rad:
                    break
            else:
                logger.warning(
                    f"Stabilization timed out (max error {max_error_deg:.2f}°)"
                )

            logger.info(f"✓ Smooth motion completed for {arm.value} arm")

            # 验证是否到达目标位置
            time.sleep(0.1)
            return self.verify_position_reached(arm, target_angles, tolerance_deg=3.0)

        except Exception as e:
            logger.error(f"Smooth motion failed: {e}")
            return False

    def enable_torque(self, arm: ArmSide, enable: bool) -> bool:
        """启用/禁用力矩（仿真中无效）"""
        logger.info(f"Torque control not applicable in MuJoCo simulation")
        return True

    def set_home(self, arm: ArmSide) -> bool:
        """回到初始位置（使用平滑运动）"""
        if not self._is_connected or not self.simulator:
            logger.error("Simulator not running!")
            return False

        logger.info(f"Moving {arm.value} arm to home position (smooth)")

        # 允许通过配置覆盖 home 关节角
        target = self.mujoco_config.get("home_joint_angles", [0.0] * 6)
        if isinstance(target, dict):
            target_angles = target.get(arm.value, target.get("default", [0.0] * 6))
        else:
            target_angles = target

        if not self.move_joint_smooth(arm, target_angles):
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
