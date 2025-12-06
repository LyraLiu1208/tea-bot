"""MuJoCo 仿真器实现"""

import numpy as np
import sys
from pathlib import Path
from typing import Dict, List
import logging

# 添加父目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))
from robot.base_controller import ArmSide
from .base_simulator import BaseSimulator

logger = logging.getLogger(__name__)


class MuJoCoSimulator(BaseSimulator):
    """MuJoCo 物理仿真器

    支持加载 MJCF 模型，提供高性能物理仿真
    """

    def __init__(self, model_path: str, config: Dict = None):
        super().__init__(model_path, config)

        # 延迟导入 MuJoCo（避免未安装时报错）
        try:
            import mujoco
            import mujoco.viewer
            self.mujoco = mujoco
            self.viewer_module = mujoco.viewer
        except ImportError:
            raise ImportError(
                "MuJoCo not installed. Please install: pip install mujoco"
            )

        self.model = None
        self.data = None
        self.viewer = None

        # 双臂关节索引映射（基于真实 Alicia-D 双臂 MJCF）
        # 关节顺序：左臂6个关节 + 左臂2个夹爪 + 右臂6个关节 + 右臂2个夹爪
        self.joint_mapping = {
            ArmSide.LEFT: {
                "joints": [0, 1, 2, 3, 4, 5],  # left_Joint1-6
                "gripper_left": 6,   # left_finger
                "gripper_right": 7   # left_right_finger
            },
            ArmSide.RIGHT: {
                "joints": [8, 9, 10, 11, 12, 13],  # right_Joint1-6
                "gripper_left": 14,  # right_left_finger
                "gripper_right": 15  # right_right_finger
            }
        }

        # 末端执行器 body 名称
        self.ee_body_names = {
            ArmSide.LEFT: "left_Link6",
            ArmSide.RIGHT: "right_Link6"
        }

    def start(self, gui: bool = True) -> bool:
        """启动仿真

        Args:
            gui: 是否显示 GUI

        Returns:
            bool: 启动是否成功
        """
        try:
            logger.info(f"Loading MuJoCo model from: {self.model_path}")

            # 加载模型
            self.model = self.mujoco.MjModel.from_xml_path(self.model_path)
            self.data = self.mujoco.MjData(self.model)

            logger.info(f"✓ Model loaded successfully")
            logger.info(f"  Bodies: {self.model.nbody}")
            logger.info(f"  Joints: {self.model.njnt}")
            logger.info(f"  Actuators: {self.model.nu}")

            # 启动可视化
            if gui:
                logger.info("Starting MuJoCo viewer...")
                try:
                    # macOS 上需要使用 mjpython，尝试使用被动查看器
                    self.viewer = self.viewer_module.launch_passive(
                        self.model, self.data
                    )
                    logger.info("✓ Viewer started")
                except RuntimeError as e:
                    if "mjpython" in str(e):
                        logger.warning(
                            "⚠ GUI viewer requires mjpython on macOS. "
                            "Running in headless mode. "
                            "Use 'mjpython tools/run_simulation.py' for GUI."
                        )
                        self.viewer = None
                    else:
                        raise

            self._is_running = True
            return True

        except Exception as e:
            logger.error(f"Failed to start simulator: {e}")
            import traceback
            traceback.print_exc()
            return False

    def stop(self):
        """停止仿真"""
        if self.viewer:
            self.viewer.close()
            self.viewer = None

        self.model = None
        self.data = None
        self._is_running = False
        logger.info("Simulator stopped")

    def step(self, num_steps: int = 1):
        """推进仿真

        Args:
            num_steps: 仿真步数
        """
        if not self._is_running:
            logger.warning("Simulator not running")
            return

        for _ in range(num_steps):
            self.mujoco.mj_step(self.model, self.data)

        # 同步到查看器
        if self.viewer and self.viewer.is_running():
            self.viewer.sync()

    def reset(self):
        """重置仿真到初始状态"""
        if self.model and self.data:
            self.mujoco.mj_resetData(self.model, self.data)
            logger.info("Simulation reset")

    def set_joint_angles(self, arm: ArmSide, joint_angles: List[float]):
        """设置关节角度

        Args:
            arm: 左臂或右臂
            joint_angles: 6 个关节角度（弧度）
        """
        if not self._is_running:
            return

        joint_indices = self.joint_mapping[arm]["joints"]

        for i, angle in enumerate(joint_angles):
            if i < len(joint_indices):
                joint_id = joint_indices[i]
                # 设置关节位置
                self.data.qpos[joint_id] = angle

        # 更新物理状态
        self.mujoco.mj_forward(self.model, self.data)

    def get_joint_angles(self, arm: ArmSide) -> List[float]:
        """获取关节角度

        Args:
            arm: 左臂或右臂

        Returns:
            List[float]: 6 个关节角度（弧度）
        """
        if not self._is_running:
            return [0.0] * 6

        joint_indices = self.joint_mapping[arm]["joints"]
        angles = []

        for joint_id in joint_indices:
            angles.append(float(self.data.qpos[joint_id]))

        return angles

    def set_gripper(self, arm: ArmSide, value: float):
        """设置夹爪

        Args:
            arm: 左臂或右臂
            value: 夹爪开合度 0-100 (0=完全闭合, 100=完全打开)
        """
        if not self._is_running:
            return

        # 获取左右手指的关节 ID
        left_finger_id = self.joint_mapping[arm]["gripper_left"]
        right_finger_id = self.joint_mapping[arm]["gripper_right"]

        # 将 0-100 映射到夹爪的实际范围
        # left_finger: range="-0.05 0" (向外移动为负)
        # right_finger: range="0 0.05" (向外移动为正)
        # value=0 时完全闭合（left=0, right=0）
        # value=100 时完全打开（left=-0.05, right=0.05）

        opening = value / 100.0 * 0.05  # 0 到 0.05 的开合量

        self.data.qpos[left_finger_id] = -opening  # 左指向外为负
        self.data.qpos[right_finger_id] = opening   # 右指向外为正

        self.mujoco.mj_forward(self.model, self.data)

    def get_end_effector_pose(self, arm: ArmSide) -> Dict:
        """获取末端执行器位姿

        Args:
            arm: 左臂或右臂

        Returns:
            Dict: {"position": [x,y,z], "quaternion": [qx,qy,qz,qw]}
        """
        if not self._is_running:
            return {"position": [0, 0, 0], "quaternion": [0, 0, 0, 1]}

        # 获取末端执行器 body ID
        body_name = self.ee_body_names[arm]

        try:
            body_id = self.mujoco.mj_name2id(
                self.model, self.mujoco.mjtObj.mjOBJ_BODY, body_name
            )

            # 获取位置和姿态
            position = self.data.xpos[body_id].copy()
            quaternion = self.data.xquat[body_id].copy()

            return {
                "position": position.tolist(),
                "quaternion": quaternion.tolist()
            }

        except Exception as e:
            logger.warning(f"Failed to get end effector pose: {e}")
            return {"position": [0, 0, 0], "quaternion": [0, 0, 0, 1]}

    def get_contact_info(self) -> List[Dict]:
        """获取碰撞信息

        Returns:
            List[Dict]: 碰撞接触点列表
        """
        if not self._is_running:
            return []

        contacts = []
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            contacts.append({
                "geom1": contact.geom1,
                "geom2": contact.geom2,
                "dist": contact.dist,
                "force": contact.frame[:3].copy()
            })

        return contacts

    def has_collision(self) -> bool:
        """检测是否有碰撞

        Returns:
            bool: 是否有碰撞
        """
        return self.data.ncon > 0 if self._is_running else False

    def update_joint_mapping(self, mapping: Dict):
        """更新关节索引映射

        当你的 MJCF 模型的关节顺序不同时使用

        Args:
            mapping: 新的关节映射字典
        """
        self.joint_mapping.update(mapping)
        logger.info("Joint mapping updated")

    def update_ee_body_names(self, names: Dict):
        """更新末端执行器 body 名称

        Args:
            names: {ArmSide.LEFT: "name1", ArmSide.RIGHT: "name2"}
        """
        self.ee_body_names.update(names)
        logger.info("End effector body names updated")

    def print_model_info(self):
        """打印模型信息（调试用）"""
        if not self.model:
            print("Model not loaded")
            return

        print("\n=== MuJoCo Model Info ===")
        print(f"Bodies: {self.model.nbody}")
        print(f"Joints: {self.model.njnt}")
        print(f"Actuators: {self.model.nu}")
        print(f"Geoms: {self.model.ngeom}")

        print("\n--- Body Names ---")
        for i in range(self.model.nbody):
            name = self.mujoco.mj_id2name(
                self.model, self.mujoco.mjtObj.mjOBJ_BODY, i
            )
            print(f"  [{i}] {name}")

        print("\n--- Joint Names ---")
        for i in range(self.model.njnt):
            name = self.mujoco.mj_id2name(
                self.model, self.mujoco.mjtObj.mjOBJ_JOINT, i
            )
            print(f"  [{i}] {name}")
