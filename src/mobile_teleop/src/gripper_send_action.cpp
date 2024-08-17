#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include <std_msgs/msg/float32.hpp>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>


class GripperActionClient : public rclcpp::Node
{
public:
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<control_msgs::action::GripperCommand>;

  GripperActionClient(): Node("gripper_send_action")
  {
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    action_client_= rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      this,
      "/xarm_gripper/gripper_action", client_cb_group_);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = sub_cb_group_;
    subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/gripper_servo_cmd", 1, std::bind(&GripperActionClient::callback, this, std::placeholders::_1), sub_opt
      );
  }

private:
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr action_client_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

  void callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    auto goal_msg = control_msgs::action::GripperCommand::Goal();
    goal_msg.command.position = msg->data;
    goal_msg.command.max_effort = 10.0;
    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&GripperActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&GripperActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&GripperActionClient::result_callback, this, std::placeholders::_1);
    auto future = this->action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const GoalHandleGripper::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

   void feedback_callback(
    GoalHandleGripper::SharedPtr,
    const std::shared_ptr<const control_msgs::action::GripperCommand::Feedback> feedback)
  {
    if (feedback->reached_goal) {
      RCLCPP_INFO(this->get_logger(), "Goal reached");
    }
    return;
  }

  void result_callback(const GoalHandleGripper::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
  }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperActionClient>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}