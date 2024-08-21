#include <functional>
#include <memory>
#include <thread>

#include "custom_action_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "custom_action_cpp/visibility_control.h"

namespace custom_action_cpp
{
class FibonacciActionServer : public rclcpp::Node
{
public:
  // 使用声明
  using Fibonacci = custom_action_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  // 构造函数，初始化节点和动作服务器
  CUSTOM_ACTION_CPP_PUBLIC
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  {
    using namespace std::placeholders;

    // 处理目标请求的回调函数
    auto handle_goal = [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Fibonacci::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    // 处理取消请求的回调函数
    auto handle_cancel = [this](
      const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    // 处理接受目标的回调函数
    auto handle_accepted = [this](
      const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
      // 为了避免阻塞执行器，创建一个新线程来执行目标
      auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
      std::thread{execute_in_thread}.detach();
    };

    // 创建动作服务器
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      handle_goal,
      handle_cancel,
      handle_accepted);
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  // 执行目标的函数
  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // 检查是否有取消请求
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // 更新序列
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // 发布反馈
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // 检查目标是否完成
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  };

};  // class FibonacciActionServer

}  // namespace custom_action_cpp

// 注册节点
RCLCPP_COMPONENTS_REGISTER_NODE(custom_action_cpp::FibonacciActionServer)