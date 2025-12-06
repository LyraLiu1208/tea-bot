"""数据模型"""

from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from enum import Enum


class ArmSide(str, Enum):
    """机械臂左右"""
    LEFT = "left"
    RIGHT = "right"


class ActionType(str, Enum):
    """动作类型"""
    MOVE_JOINT = "move_joint"
    MOVE_POSE = "move_pose"
    CONTROL_GRIPPER = "control_gripper"
    SET_HOME = "set_home"
    ENABLE_TORQUE = "enable_torque"


class Action(BaseModel):
    """单个动作"""
    type: ActionType = Field(..., description="动作类型")
    arm: ArmSide = Field(..., description="左臂或右臂")
    params: Dict[str, Any] = Field(default_factory=dict, description="动作参数")
    wait: bool = Field(default=True, description="是否等待完成")


class TaskRequest(BaseModel):
    """任务请求"""
    task_id: str = Field(..., description="任务ID")
    actions: List[Action] = Field(..., description="动作序列")


class TaskResponse(BaseModel):
    """任务响应"""
    task_id: str
    status: str  # pending, running, completed, failed
    message: Optional[str] = None


class StatusResponse(BaseModel):
    """状态响应"""
    connected: bool
    emergency_stopped: bool
    left_arm: Optional[Dict[str, Any]] = None
    right_arm: Optional[Dict[str, Any]] = None


class ErrorResponse(BaseModel):
    """错误响应"""
    error: str
    detail: Optional[str] = None
