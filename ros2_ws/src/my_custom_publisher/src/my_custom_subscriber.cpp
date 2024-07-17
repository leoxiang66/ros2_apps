#include "rclcpp/rclcpp.hpp"
#include "my_custom_msgs/msg/my_custom_msg.hpp"

class MyCustomSubscriber : public rclcpp::Node
{
public:
  MyCustomSubscriber()
  : Node("my_custom_subscriber")
  {
    subscription_ = this->create_subscription<my_custom_msgs::msg::MyCustomMsg>(
      "NB", 10, std::bind(&MyCustomSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const my_custom_msgs::msg::MyCustomMsg::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: data=%d, text=%s, value=%f",
                msg->data, msg->text.c_str(), msg->value);
  }

  rclcpp::Subscription<my_custom_msgs::msg::MyCustomMsg>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyCustomSubscriber>());
  rclcpp::shutdown();
  return 0;
}