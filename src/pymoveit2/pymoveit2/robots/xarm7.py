from typing import List

MOVE_GROUP_ARM: str = "xarm7"
MOVE_GROUP_GRIPPER: str = "xarm_gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.5, 0.5]


def joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
        prefix + "joint7",
    ]


def base_link_name(prefix: str = "") -> str:
    return prefix + "link_base"


def end_effector_name(prefix: str = "") -> str:
    return prefix + "link_eef"

def xarm7_traj_controller() -> str:
    return "xarm7_traj_controller/follow_joint_trajectory"

def gripper_joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "drive_joint",
        prefix + "drive_joint"
    ]
