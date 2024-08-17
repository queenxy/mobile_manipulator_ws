/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "mobile_teleop/joystick_input.h"
#include <iostream>


namespace mobile_teleop
{

enum XBOX360_WIRELESS_CONTROLLER_AXIS
{
    XBOX360_WIRELESS_LEFT_STICK_LR = 0,
    XBOX360_WIRELESS_LEFT_STICK_FB = 1,
    XBOX360_WIRELESS_RIGHT_STICK_LR = 2,
    XBOX360_WIRELESS_RIGHT_STICK_FB = 3,
};

enum XBOX360_CONTROLLER_BUTTON
{
    XBOX360_BTN_A = 0,
    XBOX360_BTN_B = 1,
    XBOX360_BTN_X = 2,
    XBOX360_BTN_Y = 3,
    XBOX360_BTN_LB = 4,
    XBOX360_BTN_RB = 5,
    XBOX360_BTN_LT = 6,
    XBOX360_BTN_RT = 7,
    XBOX360_BTN_BACK = 8,
    XBOX360_BTN_START = 9,
    XBOX360_BTN_POWER = 10,
    XBOX360_BTN_STICK_LEFT = 11,
    XBOX360_BTN_STICK_RIGHT = 12,
    XBOX360_BTN_CROSS_L = 13,
    XBOX360_BTN_CROSS_R = 14,
    XBOX360_BTN_CROSS_F = 15,
    XBOX360_BTN_CROSS_B = 16,
};

enum PUB_TYPE
{
    BASE = 0,
    ARM_CARTESIAN = 1,
    ARM_JOINT = 2,
    GRIPPER = 3,
};

JoyToServoPub::JoyToServoPub()
  : Node("joy_to_twist_publisher"), 
    dof_(7), ros_queue_size_(10), initialized_status_(10),
    joy_topic_("/joy"),
    cartesian_command_in_topic_("/servo_server/delta_twist_cmds"), 
    joint_command_in_topic_("/servo_server/delta_joint_cmds"), 
    base_command_in_topic_("/cmd_vel"),
    gripper_command_in_topic_("/gripper_servo_cmd"),
    robot_link_command_frame_("link_base"), 
    ee_frame_name_("link_eef"),
    planning_frame_("link_base")
{
    // init parameter from node
    _declare_or_get_param<int>(dof_, "dof", dof_);
    _declare_or_get_param<int>(ros_queue_size_, "ros_queue_size", ros_queue_size_);
    _declare_or_get_param<std::string>(joy_topic_, "joy_topic", joy_topic_);
    _declare_or_get_param<std::string>(base_command_in_topic_, "base_command_in_topic", base_command_in_topic_);
    _declare_or_get_param<std::string>(gripper_command_in_topic_, "gripper_command_in_topic", gripper_command_in_topic_);
    _declare_or_get_param<std::string>(cartesian_command_in_topic_, "moveit_servo.cartesian_command_in_topic", cartesian_command_in_topic_);
    _declare_or_get_param<std::string>(joint_command_in_topic_, "moveit_servo.joint_command_in_topic", joint_command_in_topic_);
    _declare_or_get_param<std::string>(robot_link_command_frame_, "moveit_servo.robot_link_command_frame", robot_link_command_frame_);
    _declare_or_get_param<std::string>(ee_frame_name_, "moveit_servo.ee_frame_name", ee_frame_name_);
    _declare_or_get_param<std::string>(planning_frame_, "moveit_servo.planning_frame", planning_frame_);

    if (cartesian_command_in_topic_.rfind("~/", 0) == 0) {
        cartesian_command_in_topic_ = "/servo_server/" + cartesian_command_in_topic_.substr(2, cartesian_command_in_topic_.length());
    }
    if (joint_command_in_topic_.rfind("~/", 0) == 0) {
        joint_command_in_topic_ = "/servo_server/" + joint_command_in_topic_.substr(2, joint_command_in_topic_.length());
    }

    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic_, ros_queue_size_, std::bind(&JoyToServoPub::_joy_callback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cartesian_command_in_topic_, ros_queue_size_);
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(joint_command_in_topic_, ros_queue_size_);
    base_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(base_command_in_topic_, ros_queue_size_);
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float32>(gripper_command_in_topic_, ros_queue_size_);
    // collision_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    // Create a service client to start the ServoServer
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
}

template <typename T>
void JoyToServoPub::_declare_or_get_param(T& output_value, const std::string& param_name, const T default_value)
{
    try
    {
        if (this->has_parameter(param_name))
        {
            this->get_parameter<T>(param_name, output_value);
        }
        else
        {
            output_value = this->declare_parameter<T>(param_name, default_value);
        }
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "InvalidParameterTypeException(" << param_name << "): " << e.what());
        RCLCPP_ERROR_STREAM(this->get_logger(), "Error getting parameter \'" << param_name << "\', check parameter type in YAML file");
        throw e;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Found parameter - " << param_name << ": " << output_value);
}

void JoyToServoPub::_filter_twiststamped_msg(std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist, double val)
{
    if (abs(twist->twist.linear.x) < val) {
        twist->twist.linear.x = 0;
    }
    if (abs(twist->twist.linear.y) < val) {
        twist->twist.linear.y = 0;
    }
    if (abs(twist->twist.linear.z) < val) {
        twist->twist.linear.z = 0;
    }
    if (abs(twist->twist.angular.x) < val) {
        twist->twist.angular.x = 0;
    }
    if (abs(twist->twist.angular.y) < val) {
        twist->twist.angular.y = 0;
    }
    if (abs(twist->twist.angular.z) < val) {
        twist->twist.angular.z = 0;
    }
}

void JoyToServoPub::_filter_twist_msg(std::unique_ptr<geometry_msgs::msg::Twist>& twist, double val)
{
    if (abs(twist->linear.x) < val) {
        twist->linear.x = 0;
    }
    if (abs(twist->linear.y) < val) {
        twist->linear.y = 0;
    }
    if (abs(twist->linear.z) < val) {
        twist->linear.z = 0;
    }
    if (abs(twist->angular.x) < val) {
        twist->angular.x = 0;
    }
    if (abs(twist->angular.y) < val) {
        twist->angular.y = 0;
    }
    if (abs(twist->angular.z) < val) {
        twist->angular.z = 0;
    }
}

int JoyToServoPub::_convert_xbox360_joy_to_cmd(
    const std::vector<float>& axes, const std::vector<int>& buttons,
    std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
    std::unique_ptr<control_msgs::msg::JointJog>& joint,
    std::unique_ptr<geometry_msgs::msg::Twist>& base_vel,
    std::unique_ptr<std_msgs::msg::Float32>& gripper_cmd)
{   
    int left_stick_lr = XBOX360_WIRELESS_LEFT_STICK_LR;
    int left_stick_fb = XBOX360_WIRELESS_LEFT_STICK_FB;
    int right_stick_lr = XBOX360_WIRELESS_RIGHT_STICK_LR;
    int right_stick_fb = XBOX360_WIRELESS_RIGHT_STICK_FB;

    // if (buttons[XBOX360_BTN_BACK] && planning_frame_ == ee_frame_name_) {
    //     planning_frame_ = robot_link_command_frame_;
    // }
    // else if (buttons[XBOX360_BTN_START] && planning_frame_ == robot_link_command_frame_) {
    //     planning_frame_ = ee_frame_name_;
    // }

    if (buttons[XBOX360_BTN_BACK] || buttons[XBOX360_BTN_START])
    {   
        if(buttons[XBOX360_BTN_BACK])
            gripper_cmd->data = 0.0;
        else gripper_cmd->data = 0.7;
        return GRIPPER;
    }

    if (buttons[XBOX360_BTN_RT] || buttons[XBOX360_BTN_RB] 
        || buttons[XBOX360_BTN_LT] || buttons[XBOX360_BTN_LB])
    {
        base_vel->linear.x = 0.1*(buttons[XBOX360_BTN_LT] - buttons[XBOX360_BTN_RT]);
        base_vel->angular.z = 0.3*(buttons[XBOX360_BTN_LB] - buttons[XBOX360_BTN_RB]);
        return BASE;
    }

    if (buttons[XBOX360_BTN_A] || buttons[XBOX360_BTN_B] 
        || buttons[XBOX360_BTN_X] || buttons[XBOX360_BTN_Y])
    {
        // Map the D_PAD to the proximal joints
        // joint->joint_names.push_back("joint1");
        // joint->velocities.push_back(axes[cross_key_lr] * 1);
        // joint->joint_names.push_back("joint2");
        // joint->velocities.push_back(axes[cross_key_fb] * 1);
        if (buttons[XBOX360_BTN_STICK_LEFT]) {
            joint->joint_names.push_back("joint1");
            joint->velocities.push_back((buttons[XBOX360_BTN_B] - buttons[XBOX360_BTN_X]) * 1);
            joint->joint_names.push_back("joint2");
            joint->velocities.push_back((buttons[XBOX360_BTN_Y] - buttons[XBOX360_BTN_A]) * 1);
            return ARM_JOINT;   
        }

        // Map the diamond to the distal joints
        joint->joint_names.push_back("joint" + std::to_string(dof_));
        joint->velocities.push_back((buttons[XBOX360_BTN_X] - buttons[XBOX360_BTN_B]) * 1);
        joint->joint_names.push_back("joint" + std::to_string(dof_ - 1));
        joint->velocities.push_back((buttons[XBOX360_BTN_Y] - buttons[XBOX360_BTN_A]) * 1);
        return ARM_JOINT;   
    }

    // The bread and butter: map buttons to twist commands
    twist->twist.linear.x = 0.6 * axes[left_stick_fb];
    twist->twist.linear.y = 0.6 * axes[left_stick_lr];
    twist->twist.linear.z = 0.6 * (buttons[XBOX360_BTN_CROSS_F] - buttons[XBOX360_BTN_CROSS_B]);
    twist->twist.angular.y = axes[right_stick_fb];
    twist->twist.angular.x = -axes[right_stick_lr];
    twist->twist.angular.z = 0.6 * (buttons[XBOX360_BTN_CROSS_L] - buttons[XBOX360_BTN_CROSS_R]);

    return ARM_CARTESIAN;
}

void JoyToServoPub::_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{   
    // std::cout << "Enter joy_callback" << std::endl;
    // std::string axes_str = "[ ";
    // for (int i = 0; i < msg->axes.size(); i++) { 
    //     axes_str += std::to_string(msg->axes[i]); 
    //     axes_str += " ";
    // }
    // axes_str += "]";
    // RCLCPP_INFO(this->get_logger(), "axes_str: %s", axes_str.c_str());
    // return;

    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto base_msg = std::make_unique<geometry_msgs::msg::Twist>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
    auto gripper_msg = std::make_unique<std_msgs::msg::Float32>();

    if (dof_ == 7 && initialized_status_) {
        initialized_status_ -= 1;
        joint_msg->joint_names.push_back("joint1");
        joint_msg->velocities.push_back(initialized_status_ > 0 ? 0.01 : 0);

        joint_msg->header.stamp = this->now();
        joint_msg->header.frame_id = "joint1";
        joint_pub_->publish(std::move(joint_msg));

        return;
    }

    int pub_type = -1;

    if (msg->axes.size() != 4 || msg->buttons.size() != 17){
        std::cout << msg->axes.size() << " " << msg->buttons.size() << std::endl;
        std::cout << "Signal size mismatch!" << std::endl;
        return;
    }
    pub_type = _convert_xbox360_joy_to_cmd(msg->axes, msg->buttons, twist_msg, joint_msg, base_msg, gripper_msg);

    switch (pub_type){
        case BASE:
            _filter_twist_msg(base_msg, 0.05);
            base_vel_pub_->publish(std::move(base_msg));
            break;
        case ARM_CARTESIAN:
            _filter_twiststamped_msg(twist_msg, 0.2);
            twist_msg->header.frame_id = planning_frame_;
            twist_msg->header.stamp = this->now();
            twist_pub_->publish(std::move(twist_msg));
            break;
        case ARM_JOINT:
            joint_msg->header.stamp = this->now();
            joint_msg->header.frame_id = "joint";
            joint_pub_->publish(std::move(joint_msg));
            break;
        case GRIPPER:
            gripper_pub_->publish(std::move(gripper_msg));
            break;
        default:
            return;
    }
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mobile_teleop::JoyToServoPub>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
