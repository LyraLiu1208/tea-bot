"""机械臂控制模块

统一的控制器接口支持两种模式：
- Real: 真实硬件控制（Alicia-D SDK）
- MuJoCo: 物理仿真
"""

from .base_controller import BaseDualArmController, ArmState, ArmSide
from .mujoco_controller import MuJoCoController

# Alicia 控制器按需导入（避免硬件依赖）
try:
    from .alicia_controller import AliciaDualArmController
    __all__ = [
        "BaseDualArmController",
        "ArmState",
        "ArmSide",
        "MuJoCoController",
        "AliciaDualArmController"
    ]
except ImportError:
    __all__ = [
        "BaseDualArmController",
        "ArmState",
        "ArmSide",
        "MuJoCoController"
    ]
