#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <perception/CameraDetectorInterface.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber")
  {
    auto topic_callback =
      [this](sensor_msgs::msg::Image::UniquePtr msg) -> void {
        try {
          auto cv_image = cv_bridge::toCvCopy(std::move(msg), "bgr8");
          cv::imshow("Camera Image", cv_image->image);

          auto image = cv_image->image;

          cv::waitKey(10); // Add a delay for the image to be displayed
        } catch (cv_bridge::Exception& e) {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
        }
      };

    subscription_ =
      this->create_subscription<sensor_msgs::msg::Image>("/camera_imgs", 10, topic_callback);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}