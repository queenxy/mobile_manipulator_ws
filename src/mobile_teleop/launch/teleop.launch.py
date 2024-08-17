#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type', default='dev')
    dof = LaunchConfiguration('dof', default=7)
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)
    robot_type = LaunchConfiguration('robot_type', default='xarm')

    tracer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('tracer_base'), 'launch', 'tracer_base.launch.py'])),
    )

    xarm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', 'xarm7_driver.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
        }.items(),
    )
    
    # xarm moveit servo
    robot_moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('mobile_teleop'), 'launch', '_robot_moveit_servo_realmove.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'baud_checkset': baud_checkset,
            'default_gripper_baud': default_gripper_baud,
            'dof': dof,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'robot_type': robot_type,
            'ros2_control_plugin': 'uf_robot_hardware/UFRobotSystemHardware',
            # 'prefix': 'xarm_',
            'attach_to': 'tracer',
            'attach_xyz': '0.18 0.00023421 0.02',
            'attach_rpy': '0.0 0.0 -1.5708',
        }.items(),
    )

    joystick_to_msgs_node = Node(
        package='mobile_teleop',
        executable='joystick_to_msgs',
        output='screen',
    )

    joystick_to_msgs_node = Node(
        package='mobile_teleop',
        executable='gripper_send_action',
        output='screen',
    )

    return LaunchDescription([
        # tracer_launch,
        # xarm_launch,
        robot_moveit_servo_launch,
        joystick_to_msgs_node
    ])
