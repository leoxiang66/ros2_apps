#include "rclcpp/rclcpp.hpp"
#include "my_custom_msgs/srv/my_custom_srv.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: add_two_ints_client X Y");
    return 1;
  }

  auto node = rclcpp::Node::make_shared("my_custom_client");
  auto client = node->create_client<my_custom_msgs::srv::MyCustomSrv>("add_two_ints");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  auto request = std::make_shared<my_custom_msgs::srv::MyCustomSrv::Request>();
  request->a = std::stoi(argv[1]);
  request->b = std::stoi(argv[2]);

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result: %d", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}