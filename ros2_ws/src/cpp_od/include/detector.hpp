#ifndef PERCEPTION_DETECTOR_HPP
#define PERCEPTION_DETECTOR_HPP

#include <perception/CameraDetectorInterface.hpp>
#include <memory>
#include "yolov8.hpp"

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


class Detector : public CameraDetectorInterface {
public:
    Detector();
    ~Detector() override = default;
    std::vector<std::vector<DetectedBox>> image_detection(const std::vector<cv::Mat> &images) override;

private:
    std::unique_ptr<YOLOv8> yolov8_;

    // 辅助方法
    std::vector<DetectedBox> process_single_image(const cv::Mat& image);
    DetectedBox convert_to_detected_box(const Object& yolo_object);
};

// 实现构造函数
Detector::Detector() {
    auto engine_file_path_ = "/home/xiang-tao/git/ros2_apps/ros2_ws/src/cpp_od/yolov8s.engine";
    yolov8_ = std::make_unique<YOLOv8>(engine_file_path_);
    yolov8_->make_pipe(true);
}

// 实现 image_detection 方法
std::vector<std::vector<DetectedBox>> Detector::image_detection(const std::vector<cv::Mat>& images) {
    std::vector<std::vector<DetectedBox>> all_detections;
    for (const auto& image : images) {
        all_detections.push_back(process_single_image(image));
    }
    return all_detections;
}

// 处理单张图片
std::vector<DetectedBox> Detector::process_single_image(const cv::Mat& image) {
    cv::Size size = cv::Size{640, 640};
    std::vector<Object> objs; 

    objs.clear();
    yolov8_->copy_from_Mat(image, size);
    yolov8_->infer();
    yolov8_->postprocess(objs);
    
    std::vector<DetectedBox> detections;
    for (const auto& obj : objs) {
        detections.push_back(convert_to_detected_box(obj));
    }
    
    return detections;
}

// 将 YOLOv8 的 Object 转换为 DetectedBox
DetectedBox Detector::convert_to_detected_box(const Object& yolo_object) {
    
    auto x_min = yolo_object.rect.x;
    auto y_min = yolo_object.rect.y;
    auto x_max = x_min + yolo_object.rect.width;
    auto y_max = y_min + yolo_object.rect.height;
    auto conf = yolo_object.prob;
    auto label = yolo_object.label;
    // 注意：这里假设 YOLOv8 的 Object 类有这些成员。可能需要根据实际情况调整。
    return DetectedBox(BoxCoordinate(x_min,y_min,x_max,y_max) ,conf,CLASS_NAMES[label]);
}

#endif // PERCEPTION_DETECTOR_HPP