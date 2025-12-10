#pragma once

#include <string>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>

// 检测结果结构体
struct Detection {
    cv::Rect box;
    float conf;
    int classId;
};

// 检测器配置参数
struct DetectorConfig {
    float confThreshold = 0.4f;
    float iouThreshold = 0.45f;
    std::string enginePath;
};

class TensorRTDetector {
public:
    // 构造函数
    explicit TensorRTDetector(const DetectorConfig& config);
    
    // 析构函数
    ~TensorRTDetector();

    // 禁止拷贝，只允许移动（管理裸指针资源的最佳实践）
    TensorRTDetector(const TensorRTDetector&) = delete;
    TensorRTDetector& operator=(const TensorRTDetector&) = delete;

    // 核心检测函数：输入图片，返回检测结果列表（不包含绘图）
    std::vector<Detection> detect(const cv::Mat& img);

    // 静态辅助函数：将检测结果绘制到图片上
    static void draw(cv::Mat& img, const std::vector<Detection>& objects);

private:
    // 内部实现细节，隐藏在 cpp 中
    void letterbox(const cv::Mat& src, cv::Mat& dst, float& ratio, int& dw, int& dh, int target_w, int target_h);

    // 成员变量
    DetectorConfig config_;
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    void* stream_ = nullptr; // 使用 void* 避免在头文件中引入 cuda_runtime_api.h

    // 资源管理
    std::map<std::string, void*> deviceBufferMap_;
    std::map<std::string, size_t> bufferSizes_;
    std::string inputName_;
    std::string outputName_;
    nvinfer1::Dims inputDims_;
    nvinfer1::Dims outputDims_;
    std::vector<float> hostOutput_;
};