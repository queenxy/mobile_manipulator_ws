#!/usr/bin/env python3
"""
Example of interacting with the gripper.
- ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="toggle"
- ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="open"
- ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="close"
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import GripperInterface
from pymoveit2.robots import xarm7


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_gripper")

    # node.declare_parameter(
    #     "joint_positions", 0.5
    # )
    # Declare parameter for gripper action
    node.declare_parameter(
        "action",
        "close",
    )

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create gripper interface
    gripper_interface = GripperInterface(
        node=node,
        gripper_joint_names=xarm7.gripper_joint_names(),
        open_gripper_joint_positions=xarm7.OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=xarm7.CLOSED_GRIPPER_JOINT_POSITIONS,
        gripper_group_name=xarm7.MOVE_GROUP_GRIPPER,
        callback_group=callback_group,
        follow_joint_trajectory_action_name="xarm_gripper_traj_controller/follow_joint_trajectory",
        gripper_command_action_name="gripper_action_controller/gripper_cmd",
    )
#xarm_gripper_traj_controller/follow_joint_trajectory
#xarm_gripper/gripper_action
    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Sleep a while in order to get the first joint state
    node.create_rate(10.0).sleep()

    # Get parameter
    action = node.get_parameter("action").get_parameter_value().string_value

    # Perform gripper action
    node.get_logger().info(f'Performing gripper action "{action}"')
    if "open" == action:
        gripper_interface.open()
        gripper_interface.wait_until_executed()
    elif "close" == action:
        gripper_interface.close()
        gripper_interface.wait_until_executed()
    else:
        period_s = 1.0
        rate = node.create_rate(1 / period_s)
        while rclpy.ok():
            gripper_interface()
            gripper_interface.wait_until_executed()
            rate.sleep()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()