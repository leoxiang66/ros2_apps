// ~/ros2_ws/src/my_cpp_pkg/src/hello_world.cpp
#include <rclcpp/rclcpp.hpp>

class HelloWorldNode : public rclcpp::Node {
public:
  HelloWorldNode() : Node("hello_world_node") {
    RCLCPP_INFO(this->get_logger(), "Hello, ROS 2 world!");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HelloWorldNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}