"""FastAPI 主应用"""

import logging
import yaml
from pathlib import Path
from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse
from contextlib import asynccontextmanager

from .models import (
    TaskRequest, TaskResponse, StatusResponse,
    ErrorResponse, ArmSide, ActionType
)

# 全局控制器实例
controller = None

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def load_config():
    """加载配置文件"""
    config_path = Path(__file__).parent.parent / "config" / "robot_config.yaml"
    with open(config_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def create_controller(config):
    """根据配置创建控制器

    支持两种模式:
    - real: 真实硬件 (Alicia-D SDK)
    - mujoco: 物理仿真
    """
    mode = config.get("mode", "mujoco")
    logger.info(f"Operating mode: {mode}")

    if mode == "real":
        from robot.alicia_controller import AliciaDualArmController
        logger.info("Creating AliciaDualArmController")
        return AliciaDualArmController(config)

    elif mode == "mujoco":
        from robot import MuJoCoController
        logger.info("Creating MuJoCoController (physics simulation)")
        return MuJoCoController(config)

    else:
        raise ValueError(
            f"Unknown mode: {mode}. Must be 'real' or 'mujoco'"
        )


@asynccontextmanager
async def lifespan(app: FastAPI):
    """应用生命周期管理"""
    global controller

    # 启动时
    logger.info("Starting TeaBot API...")
    config = load_config()
    controller = create_controller(config)

    # 连接机械臂
    if controller.connect():
        logger.info("✓ Robot connected successfully")
    else:
        logger.error("✗ Failed to connect to robot")

    yield

    # 关闭时
    logger.info("Shutting down TeaBot API...")
    if controller:
        controller.disconnect()
    logger.info("✓ Shutdown complete")


# 创建 FastAPI 应用
app = FastAPI(
    title="TeaBot API",
    description="双臂机械臂控制 API",
    version="0.1.0",
    lifespan=lifespan
)


@app.get("/")
async def root():
    """根路径"""
    return {
        "message": "TeaBot API",
        "version": "0.1.0",
        "docs": "/docs"
    }


@app.post("/task", response_model=TaskResponse)
async def create_task(task: TaskRequest):
    """接收并执行任务

    接收来自 AI Agent 的任务 JSON，执行动作序列
    """
    global controller

    if not controller or not controller.is_connected:
        raise HTTPException(status_code=503, detail="Robot not connected")

    if controller.is_emergency_stopped:
        raise HTTPException(status_code=423, detail="Robot is emergency stopped")

    logger.info(f"Received task: {task.task_id}")

    try:
        # 执行动作序列
        for i, action in enumerate(task.actions):
            logger.info(f"Executing action {i+1}/{len(task.actions)}: {action.type.value} on {action.arm.value}")

            success = execute_action(controller, action)

            if not success:
                return TaskResponse(
                    task_id=task.task_id,
                    status="failed",
                    message=f"Action {i+1} failed: {action.type.value}"
                )

        logger.info(f"Task {task.task_id} completed successfully")
        return TaskResponse(
            task_id=task.task_id,
            status="completed",
            message="All actions executed successfully"
        )

    except Exception as e:
        logger.error(f"Task {task.task_id} failed: {e}")
        return TaskResponse(
            task_id=task.task_id,
            status="failed",
            message=str(e)
        )


def execute_action(controller, action) -> bool:
    """执行单个动作"""
    arm = ArmSide(action.arm)
    params = action.params
    wait = action.wait

    if action.type == ActionType.MOVE_JOINT:
        # 期望参数: joints (list of 6 floats in radians)
        joints = params.get("joints")
        if not joints or len(joints) != 6:
            raise ValueError("move_joint requires 'joints' parameter with 6 values")
        return controller.move_joint(arm, joints, wait=wait)

    elif action.type == ActionType.MOVE_JOINT_SMOOTH:
        # 期望参数: joints (list of 6 floats in radians), steps (int), step_delay (float)
        joints = params.get("joints")
        if not joints or len(joints) != 6:
            raise ValueError("move_joint_smooth requires 'joints' parameter with 6 values")
        steps = params.get("steps", 20)
        step_delay = params.get("step_delay", 0.05)
        return controller.move_joint_smooth(arm, joints, steps=steps, step_delay=step_delay)

    elif action.type == ActionType.MOVE_POSE:
        # 期望参数: pose (list of 7 floats: x,y,z,qx,qy,qz,qw)
        pose = params.get("pose")
        if not pose or len(pose) != 7:
            raise ValueError("move_pose requires 'pose' parameter with 7 values [x,y,z,qx,qy,qz,qw]")
        return controller.move_pose(arm, pose, wait=wait)

    elif action.type == ActionType.CONTROL_GRIPPER:
        # 期望参数: value (float 0-100)
        value = params.get("value")
        if value is None:
            raise ValueError("control_gripper requires 'value' parameter (0-100)")
        return controller.control_gripper(arm, float(value), wait=wait)

    elif action.type == ActionType.SET_HOME:
        # 回到初始位置
        return controller.set_home(arm)

    elif action.type == ActionType.ENABLE_TORQUE:
        # 期望参数: enable (bool)
        enable = params.get("enable", True)
        return controller.enable_torque(arm, enable)

    else:
        raise ValueError(f"Unknown action type: {action.type}")


@app.get("/status", response_model=StatusResponse)
async def get_status():
    """查询机械臂状态"""
    global controller

    if not controller:
        raise HTTPException(status_code=503, detail="Controller not initialized")

    left_state = controller.get_state(ArmSide.LEFT)
    right_state = controller.get_state(ArmSide.RIGHT)

    return StatusResponse(
        connected=controller.is_connected,
        emergency_stopped=controller.is_emergency_stopped,
        left_arm={
            "joint_angles": left_state.joint_angles if left_state else None,
            "gripper_value": left_state.gripper_value if left_state else None,
            "is_moving": left_state.is_moving if left_state else None,
        } if left_state else None,
        right_arm={
            "joint_angles": right_state.joint_angles if right_state else None,
            "gripper_value": right_state.gripper_value if right_state else None,
            "is_moving": right_state.is_moving if right_state else None,
        } if right_state else None,
    )


@app.post("/stop")
async def emergency_stop():
    """紧急停止"""
    global controller

    if not controller:
        raise HTTPException(status_code=503, detail="Controller not initialized")

    success = controller.emergency_stop()

    if success:
        return {"message": "Emergency stop activated"}
    else:
        raise HTTPException(status_code=500, detail="Failed to emergency stop")


@app.post("/reset")
async def reset_system():
    """复位系统"""
    global controller

    if not controller:
        raise HTTPException(status_code=503, detail="Controller not initialized")

    success = controller.reset()

    if success:
        return {"message": "System reset successfully"}
    else:
        raise HTTPException(status_code=500, detail="Failed to reset system")


@app.get("/health")
async def health_check():
    """健康检查"""
    return {
        "status": "healthy",
        "connected": controller.is_connected if controller else False
    }
