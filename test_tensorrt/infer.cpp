#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <unordered_map>

using namespace nvinfer1;

// Logger for TensorRT info/warning/errors
class Logger : public ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << msg << std::endl;
        }
    }
};

// Function to find the index with the highest value
int getMaxIndex(const std::vector<float>& data) {
    return std::distance(data.begin(), std::max_element(data.begin(), data.end()));
}

// Function to draw bounding box
void drawBoundingBox(cv::Mat& image, int class_id, float score, int left, int top, int right, int bottom) {
    cv::rectangle(image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 3);
    std::string label = cv::format("%d: %.2f", class_id, score);
    int baseLine;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = std::max(top, labelSize.height);
    cv::putText(image, label, cv::Point(left, top - 4), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
}

int main() {
    // Load TensorRT engine
    std::ifstream engine_file("/home/xiang-tao/git/model.trt", std::ios::binary);
    if (!engine_file) {
        std::cerr << "Error opening engine file!" << std::endl;
        return -1;
    }
    std::vector<char> engine_data(std::istreambuf_iterator<char>(engine_file), {});
    
    Logger logger;
    IRuntime* runtime = createInferRuntime(logger);
    if (!runtime) {
        std::cerr << "Failed to create runtime!" << std::endl;
        return -1;
    }
    ICudaEngine* engine = runtime->deserializeCudaEngine(engine_data.data(), engine_data.size());
    if (!engine) {
        std::cerr << "Failed to create engine!" << std::endl;
        // runtime->destroy();
        return -1;
    }

    // Create execution context
    IExecutionContext* context = engine->createExecutionContext();
    if (!context) {
        std::cerr << "Failed to create execution context!" << std::endl;
        // engine->destroy();
        // runtime->destroy();
        return -1;
    }

    // Prepare input and output buffers
    std::vector<float> input_data(3 * 640 * 640); // 假设输入shape为 (1, 3, 640, 640)
    std::vector<float> output_data(84 * 8400); // 假设输出shape为 (1, 84, 8400)
    
    void* buffers[2];
    cudaMalloc(&buffers[0], input_data.size() * sizeof(float));
    cudaMalloc(&buffers[1], output_data.size() * sizeof(float));
    
    cudaMemcpy(buffers[0], input_data.data(), input_data.size() * sizeof(float), cudaMemcpyHostToDevice);
    
    // Execute inference
    context->executeV2(buffers);
    
    // Retrieve output result
    cudaMemcpy(output_data.data(), buffers[1], output_data.size() * sizeof(float), cudaMemcpyDeviceToHost);
    
    // Process output
    int rows = 8400;
    
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;
    
    for (int i = 0; i < rows; ++i) {
        std::vector<float> classes_scores(output_data.begin() + i * 84 + 4, output_data.begin() + (i + 1) * 84);
        int max_class_index = getMaxIndex(classes_scores);
        float max_score = classes_scores[max_class_index];
        
        if (max_score >= 0.25) {
            cv::Rect box(
                static_cast<int>(output_data[i * 84] - 0.5f * output_data[i * 84 + 2]),
                static_cast<int>(output_data[i * 84 + 1] - 0.5f * output_data[i * 84 + 3]),
                static_cast<int>(output_data[i * 84 + 2]),
                static_cast<int>(output_data[i * 84 + 3])
            );
            boxes.push_back(box);
            scores.push_back(max_score);
            class_ids.push_back(max_class_index);
        }
    }

    // Apply NMS (Non-maximum suppression)
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, 0.25, 0.45, indices);

    // Load class names
    std::unordered_map<int, std::string> class_names = {
        {0, "person"}, {1, "bicycle"}, {2, "car"}, {3, "motorcycle"}, {4, "airplane"},
        // Add other class mappings here...
        {78, "hair drier"}, {79, "toothbrush"}
    };

    // Load image for visualization
    cv::Mat original_image = cv::imread("/home/xiang-tao/git/bus.jpg");
    float scale = 1.0; // Adjust scale as needed

    // Iterate through NMS results to draw bounding boxes and labels
    for (int i : indices) {
        cv::Rect box = boxes[i];
        drawBoundingBox(
            original_image,
            class_ids[i],
            scores[i],
            static_cast<int>(box.x * scale),
            static_cast<int>(box.y * scale),
            static_cast<int>((box.x + box.width) * scale),
            static_cast<int>((box.y + box.height) * scale)
        );
    }

    // Display the result
    cv::imshow("Detections", original_image);
    cv::waitKey(0);

    // Clean up resources
    cudaFree(buffers[0]);
    cudaFree(buffers[1]);
    
    // context->destroy();
    // engine->destroy();
    // runtime->destroy();
    
    return 0;
}