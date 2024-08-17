import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
import launch_ros
import os
import launch_ros.descriptions

def generate_launch_description():
    pkg_name = 'mobile_bringup'
    pkg_share = launch_ros.substitutions.FindPackageShare(package=pkg_name).find(pkg_name)
    # default_model_path = os.path.join(pkg_share, model_name)
    default_model_path = PathJoinSubstitution(
                [launch_ros.substitutions.FindPackageShare("mobile_bringup"), "urdf", "mobile_base.urdf.xacro"]
            )
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/view_robot.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str) }]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])