#!/usr/bin/env python3
"""
Example of moving to a pose goal.
- ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda
from pymoveit2.robots import xarm7


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.5, 0.5, 0.25])
    node.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    # moveit2 = MoveIt2(
    #     node=node,
    #     joint_names=panda.joint_names(),
    #     base_link_name=panda.base_link_name(),
    #     end_effector_name=panda.end_effector_name(),
    #     group_name=panda.MOVE_GROUP_ARM,
    #     callback_group=callback_group,
    # )
    moveit2 = MoveIt2(
        node=node,
        joint_names=xarm7.joint_names(),
        base_link_name=xarm7.base_link_name(),
        end_effector_name=xarm7.end_effector_name(),
        group_name=xarm7.MOVE_GROUP_ARM,
        callback_group=callback_group,
        follow_joint_trajectory_action_name=xarm7.xarm7_traj_controller()
    )
    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 0.1
    moveit2.max_acceleration = 0.1

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
