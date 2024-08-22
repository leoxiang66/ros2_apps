#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <perception/CameraDetectorInterface.hpp>
#include "detector.hpp"
#include <chrono>

namespace fs = ghc::filesystem;




void print_objects(const std::vector<DetectedBox>& objs)
{
    for (const auto& obj : objs) {
        // Extract bounding box coordinates
        int x1 = static_cast<int>(obj.coord.x_min);
        int y1 = static_cast<int>(obj.coord.y_min);
        int x2 = static_cast<int>(obj.coord.x_max);
        int y2 = static_cast<int>(obj.coord.y_max);

        // Print class, confidence, and bounding box coordinates
        printf("Class: %s, Confidence: %.2f%%, Box: [%d, %d, %d, %d]\n",
               obj.label.c_str(), obj.conf * 100, x1, y1, x2, y2);
    }
}


class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber")
  {
    // Set CUDA device
    cudaSetDevice(0);

    detector_ = std::make_unique<Detector>();

    auto topic_callback =
      [this](sensor_msgs::msg::Image::UniquePtr msg) -> void {
        try {
          auto cv_image = cv_bridge::toCvCopy(std::move(msg), "bgr8");
          auto image = cv_image->image;

          std::vector<cv::Mat> images = {image};
          auto detections = detector_->image_detection(images);

          print_objects(detections[0]);

       
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
  std::unique_ptr<Detector> detector_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}