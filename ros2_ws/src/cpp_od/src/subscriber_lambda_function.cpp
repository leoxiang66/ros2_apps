#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <perception/CameraDetectorInterface.hpp>
#include "yolov8.hpp"
#include <chrono>

namespace fs = ghc::filesystem;

const std::vector<std::string> CLASS_NAMES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant",
    "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie",
    "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
    "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush"};


void print_objects(const cv::Mat& image, const std::vector<Object>& objs, const std::vector<std::string>& CLASS_NAMES)
{
    for (const auto& obj : objs) {
        // Extract bounding box coordinates
        int x1 = static_cast<int>(obj.rect.x);
        int y1 = static_cast<int>(obj.rect.y);
        int x2 = x1 + static_cast<int>(obj.rect.width);
        int y2 = y1 + static_cast<int>(obj.rect.height);

        // Print class, confidence, and bounding box coordinates
        printf("Class: %s, Confidence: %.2f%%, Box: [%d, %d, %d, %d]\n",
               CLASS_NAMES[obj.label].c_str(), obj.prob * 100, x1, y1, x2, y2);
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

    engine_file_path_ = "/home/xiang-tao/git/ros2_apps/ros2_ws/src/cpp_od/yolov8s.engine";
    yolov8_ = std::make_unique<YOLOv8>(engine_file_path_);
    yolov8_->make_pipe(true);

    auto topic_callback =
      [this](sensor_msgs::msg::Image::UniquePtr msg) -> void {
        try {
          auto cv_image = cv_bridge::toCvCopy(std::move(msg), "bgr8");
          // cv::imshow("Camera Image", cv_image->image);

          auto image = cv_image->image;
          cv::Size size = cv::Size{640, 640};
          std::vector<Object> objs; 

          objs.clear();
          yolov8_->copy_from_Mat(image, size);
          yolov8_->infer();
          yolov8_->postprocess(objs);

          print_objects(image, objs, CLASS_NAMES);

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
  std::unique_ptr<YOLOv8> yolov8_;
  std::string engine_file_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}