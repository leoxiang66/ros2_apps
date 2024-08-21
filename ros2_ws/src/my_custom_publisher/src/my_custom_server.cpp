#include "rclcpp/rclcpp.hpp"
#include "my_custom_msgs/srv/my_custom_srv.hpp"
#include <memory>

void handle_service(
  const std::shared_ptr<my_custom_msgs::srv::MyCustomSrv::Request> request,
  std::shared_ptr<my_custom_msgs::srv::MyCustomSrv::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %d b: %d", request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("my_custom_server");
  auto service = node->create_service<my_custom_msgs::srv::MyCustomSrv>("add_two_ints", handle_service);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
  rclcpp::spin(node);
  rclcpp::shutdown();
}