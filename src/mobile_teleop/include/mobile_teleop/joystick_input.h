/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#ifndef __XARM_JOY_STICK_INPUT_H__
#define __XARM_JOY_STICK_INPUT_H__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float32.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>


namespace mobile_teleop
{

class JoyToServoPub : public rclcpp::Node
{
public:
    JoyToServoPub();
private:
    template <typename T>
    void _declare_or_get_param(T& output_value, const std::string& param_name, const T default_value = T{});
    int _convert_xbox360_joy_to_cmd(
        const std::vector<float>& axes, const std::vector<int>& buttons,
        std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
        std::unique_ptr<control_msgs::msg::JointJog>& joint,
        std::unique_ptr<geometry_msgs::msg::Twist>& base_vel,
        std::unique_ptr<std_msgs::msg::Float32>& gripper_cmd);

    void _filter_twist_msg(std::unique_ptr<geometry_msgs::msg::Twist>& twist, double val = 0.05);
    void _filter_twiststamped_msg(std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist, double val = 0.05);
    void _joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

    int dof_;
    int ros_queue_size_;
    int initialized_status_;

    std::string joy_topic_;
    std::string cartesian_command_in_topic_;
    std::string joint_command_in_topic_;
    std::string base_command_in_topic_;
    std::string gripper_command_in_topic_;

    std::string robot_link_command_frame_;
    std::string ee_frame_name_;

    std::string planning_frame_;
};
}


#endif // __XARM_JOY_STICK_INPUT_H__
