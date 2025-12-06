"""MuJoCo 仿真器实现"""

import numpy as np
import sys
from pathlib import Path
from typing import Dict, List, Optional
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

        # 关节映射将在模型加载后自动检测
        self.joint_mapping = {}
        self.ee_body_names = {}
        self.is_single_arm = False
        # 逆运动学默认配置，可通过 config 覆盖
        self.ik_config = {
            "max_iters": 120,
            "pos_tolerance": 1e-3,
            "ori_tolerance": 1e-2,
            "damping": 1e-3,
            "step_scale": 0.8,
            "max_attempts": 3,
            "orientation_weight": 1.0,
            "allow_orientation_relax": True
        }
        if config:
            self.ik_config.update({
                "max_iters": config.get("ik_max_iters", self.ik_config["max_iters"]),
                "pos_tolerance": config.get("ik_pos_tolerance", self.ik_config["pos_tolerance"]),
                "ori_tolerance": config.get("ik_ori_tolerance", self.ik_config["ori_tolerance"]),
                "damping": config.get("ik_damping", self.ik_config["damping"]),
                "step_scale": config.get("ik_step_scale", self.ik_config["step_scale"]),
                "max_attempts": config.get("ik_max_attempts", self.ik_config["max_attempts"]),
                "orientation_weight": config.get("ik_orientation_weight", self.ik_config["orientation_weight"]),
                "allow_orientation_relax": config.get(
                "ik_allow_orientation_relax", self.ik_config["allow_orientation_relax"]
            )
        })
        # 记录夹爪状态用于 get_state
        self._gripper_state = {
            ArmSide.LEFT: 50.0,
            ArmSide.RIGHT: 50.0
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

            # 自动检测单臂/双臂模型并配置关节映射
            self._detect_and_configure_model()

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

        使用执行器控制来平滑地驱动关节到目标位置。
        这比直接设置 qpos 更符合物理仿真的真实行为。

        Args:
            arm: 左臂或右臂
            joint_angles: 6 个关节角度（弧度）
        """
        if not self._is_running:
            return

        joint_indices = self.joint_mapping[arm]["joints"]

        # 使用执行器控制 (ctrl) 而不是直接设置 qpos
        # 这样物理引擎会平滑地驱动关节到目标位置
        for i, angle in enumerate(joint_angles):
            if i < len(joint_indices):
                actuator_id = joint_indices[i]  # 执行器 ID 与关节 ID 相同
                # 设置执行器目标（motor 类型的执行器使用 ctrl）
                self.data.ctrl[actuator_id] = angle

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

        # 通过执行器控制夹爪，保持与关节控制一致
        actuator_left = left_finger_id  # position actuator 顺序与关节一致
        actuator_right = right_finger_id

        # 数值映射调整：value=0 -> 闭合（0偏移），value=100 -> 最大打开
        opening = (100.0 - value) / 100.0 * 0.05
        target_left = -opening
        target_right = opening

        # 直接更新位置，并设置执行器目标保持住
        self.data.qpos[left_finger_id] = target_left
        self.data.qpos[right_finger_id] = target_right
        self.data.ctrl[actuator_left] = target_left
        self.data.ctrl[actuator_right] = target_right

        # 推进几步让夹爪跟随
        for _ in range(20):
            self.mujoco.mj_step(self.model, self.data)
        self._gripper_state[arm] = float(value)

    def get_gripper_value(self, arm: ArmSide) -> float:
        """返回当前夹爪开合度（0-100）"""
        return self._gripper_state.get(arm, 50.0)

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

    # ========== 逆运动学工具 ==========

    @staticmethod
    def _quat_conjugate(quat: np.ndarray) -> np.ndarray:
        q = quat.copy()
        q[1:] *= -1.0
        return q

    @staticmethod
    def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ])

    def _quat_error(self, target_wxyz: np.ndarray, current_wxyz: np.ndarray) -> np.ndarray:
        q_err = self._quat_multiply(target_wxyz, self._quat_conjugate(current_wxyz))
        if q_err[0] < 0:
            q_err *= -1.0
        return 2.0 * q_err[1:]

    def solve_inverse_kinematics(self, arm: ArmSide, target_pose: List[float]) -> Optional[List[float]]:
        """阻尼伪逆 IK，用于 move_pose"""
        if not self._is_running:
            logger.error("Simulator not running, cannot solve IK")
            return None

        joint_ids = self.joint_mapping[arm]["joints"]
        dof_ids = [self.model.jnt_dofadr[jid] for jid in joint_ids]
        joint_limits = [self.model.jnt_range[jid].copy() for jid in joint_ids]

        target_pos = np.array(target_pose[:3], dtype=float)
        target_quat = np.array([
            target_pose[6], target_pose[3], target_pose[4], target_pose[5]
        ], dtype=float)
        norm = np.linalg.norm(target_quat)
        if norm < 1e-6:
            logger.error("Invalid target quaternion")
            return None
        target_quat /= norm

        base_state = np.array(self.get_joint_angles(arm), dtype=float)

        tmp_data = self.mujoco.MjData(self.model)
        tmp_data.qpos[:] = self.data.qpos[:]
        body_name = self.ee_body_names[arm]
        body_id = self.mujoco.mj_name2id(
            self.model, self.mujoco.mjtObj.mjOBJ_BODY, body_name
        )

        max_iters = self.ik_config["max_iters"]
        pos_tol = self.ik_config["pos_tolerance"]
        ori_tol = self.ik_config["ori_tolerance"]
        damping = self.ik_config["damping"]
        step_scale = self.ik_config["step_scale"]
        max_attempts = max(1, int(self.ik_config["max_attempts"]))
        ori_weight = self.ik_config["orientation_weight"]
        relax_orientation = self.ik_config["allow_orientation_relax"]

        def try_solve(initial_q: np.ndarray, weight: float) -> Optional[np.ndarray]:
            q = initial_q.copy()
            for _ in range(max_iters):
                for idx, jid in enumerate(joint_ids):
                    tmp_data.qpos[jid] = q[idx]
                self.mujoco.mj_forward(self.model, tmp_data)

                current_pos = tmp_data.xpos[body_id].copy()
                current_quat = tmp_data.xquat[body_id].copy()

                pos_err = target_pos - current_pos
                ori_err = self._quat_error(target_quat, current_quat)

                if np.linalg.norm(pos_err) < pos_tol and (np.linalg.norm(ori_err) < ori_tol or weight == 0.0):
                    return q

                jacp = np.zeros((3, self.model.nv))
                jacr = np.zeros((3, self.model.nv))
                self.mujoco.mj_jacBody(self.model, tmp_data, jacp, jacr, body_id)

                jac = np.zeros((6, len(joint_ids)))
                for idx, dof in enumerate(dof_ids):
                    jac[0:3, idx] = jacp[:, dof]
                    jac[3:6, idx] = jacr[:, dof]

                err = np.concatenate([pos_err, weight * ori_err])
                jjT = jac @ jac.T + (damping ** 2) * np.eye(6)
                dq = jac.T @ np.linalg.solve(jjT, err)
                q += step_scale * dq

                for idx, lim in enumerate(joint_limits):
                    q[idx] = np.clip(q[idx], lim[0], lim[1])
            return None

        # 多次尝试：第一次使用当前姿态，后续随机采样
        attempts = 0
        while attempts < max_attempts:
            if attempts == 0:
                init_q = base_state
            else:
                noise = np.random.uniform(-0.2, 0.2, size=len(base_state))
                init_q = np.clip(base_state + noise, [lim[0] for lim in joint_limits], [lim[1] for lim in joint_limits])

            result = try_solve(init_q, ori_weight)
            if result is not None:
                return result.tolist()
            attempts += 1

        if relax_orientation and ori_weight > 0.0:
            logger.warning("IK failed with orientation constraint, retrying without orientation")
            result = try_solve(base_state, 0.0)
            if result is not None:
                return result.tolist()

        logger.warning("IK did not converge within max attempts")
        return None

    def _detect_and_configure_model(self):
        """自动检测单臂/双臂模型并配置关节映射"""
        if not self.model:
            return

        # 检查是否有 left_Joint1 或 Joint1
        has_left_prefix = False
        has_right_prefix = False

        for i in range(self.model.njnt):
            joint_name = self.mujoco.mj_id2name(
                self.model, self.mujoco.mjtObj.mjOBJ_JOINT, i
            )
            if joint_name and "left_Joint" in joint_name:
                has_left_prefix = True
            if joint_name and "right_Joint" in joint_name:
                has_right_prefix = True

        # 判断单臂还是双臂
        if has_left_prefix and has_right_prefix:
            # 双臂模型
            self.is_single_arm = False
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
            self.ee_body_names = {
                ArmSide.LEFT: "left_Link6",
                ArmSide.RIGHT: "right_Link6"
            }
            logger.info("✓ Detected dual-arm model")

        else:
            # 单臂模型 (Joint1-6, left_finger, right_finger)
            self.is_single_arm = True
            self.joint_mapping = {
                ArmSide.LEFT: {
                    "joints": [0, 1, 2, 3, 4, 5],  # Joint1-6
                    "gripper_left": 6,   # left_finger
                    "gripper_right": 7   # right_finger
                },
                # 右臂使用相同映射（单臂模式下只有左臂）
                ArmSide.RIGHT: {
                    "joints": [0, 1, 2, 3, 4, 5],
                    "gripper_left": 6,
                    "gripper_right": 7
                }
            }
            self.ee_body_names = {
                ArmSide.LEFT: "Link6",
                ArmSide.RIGHT: "Link6"
            }
            logger.info("✓ Detected single-arm model")

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
