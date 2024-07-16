#include "rclcpp/rclcpp.hpp"
#include "my_custom_msgs/msg/my_custom_msg.hpp"

using namespace std::chrono_literals;

class MyCustomPublisher : public rclcpp::Node
{
public:
  MyCustomPublisher()
  : Node("my_custom_publisher")
  {
    publisher_ = this->create_publisher<my_custom_msgs::msg::MyCustomMsg>("NB", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&MyCustomPublisher::publish_message, this));
  }

private:
  void publish_message()
  {
    auto message = my_custom_msgs::msg::MyCustomMsg();
    message.data = 42;
    message.text = "Hello, ROS2";
    message.value = 3.14;

    RCLCPP_INFO(this->get_logger(), "Publishing: data=%d, text=%s, value=%f",
                message.data, message.text.c_str(), message.value);
    publisher_->publish(message);
  }

  rclcpp::Publisher<my_custom_msgs::msg::MyCustomMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyCustomPublisher>());
  rclcpp::shutdown();
  return 0;
}