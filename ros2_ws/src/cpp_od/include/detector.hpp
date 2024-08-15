#ifndef PERCEPTION_DETECTOR_HPP
#define PERCEPTION_DETECTOR_HPP

#include <perception/CameraDetectorInterface.hpp>
#include <memory>

class Detector : public CameraDetectorInterface {
public:
    Detector(const std::string& model_path);
    ~Detector() override = default;

    std::vector<std::vector<DetectedBox>> image_detection(const std::vector<cv::Mat>& images) override;

private:
    std::unique_ptr<YOLOv8> yolo_model;
    
    // 辅助方法
    std::vector<DetectedBox> process_single_image(const cv::Mat& image);
    DetectedBox convert_to_detected_box(const Object& yolo_object);
};

// 实现构造函数
Detector::Detector(const std::string& model_path) {
    yolo_model = std::make_unique<YOLOv8>(model_path);
    yolo_model->make_pipe(true);
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
    cv::Size size{640, 640};  // 假设 YOLOv8 使用 640x640 的输入大小
    yolo_model->copy_from_Mat(image, size);
    yolo_model->infer();
    
    std::vector<Object> yolo_objects;
    yolo_model->postprocess(yolo_objects);
    
    std::vector<DetectedBox> detections;
    for (const auto& obj : yolo_objects) {
        detections.push_back(convert_to_detected_box(obj));
    }
    
    return detections;
}

// 将 YOLOv8 的 Object 转换为 DetectedBox
DetectedBox Detector::convert_to_detected_box(const Object& yolo_object) {
    DetectedBox box;
    box.x = yolo_object.rect.x;
    box.y = yolo_object.rect.y;
    box.width = yolo_object.rect.width;
    box.height = yolo_object.rect.height;
    box.confidence = yolo_object.prob;
    box.class_id = yolo_object.label;
    // 注意：这里假设 YOLOv8 的 Object 类有这些成员。可能需要根据实际情况调整。
    return box;
}

#endif // PERCEPTION_DETECTOR_HPP