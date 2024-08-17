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


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    
    # robot moveit realmove launch
    # xarm_moveit_config/launch/_robot_moveit_realmove.launch.py
    robot_moveit_realmove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('mobile_bringup'), 'launch', '_robot_moveit_realmove.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'dof': '7',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'false',
            'add_gripper': 'true',
            'add_realsense_d435i': 'true',
            'add_d435i_links': 'true',
            'prefix': 'xarm_',
            'attach_to': 'base_link',
            'attach_xyz': '0.15 0.00023421 0.02',
            'attach_rpy': '0.0 0.0 -1.5708',
        }.items(),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'camera_name': "xarm_camera",
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': '1280x720x15',
            'depth_module.depth_profile': '1280x720x15', 
            'pointcloud.enable': 'true',
        }.items(),
    )
    
    return LaunchDescription([
        robot_moveit_realmove_launch,
        camera_launch
    ])
