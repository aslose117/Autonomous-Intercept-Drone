#include "yolo_detector.hpp"

#include <fstream>
#include <iostream>
#include <numeric>
#include <algorithm>
#include <opencv2/dnn.hpp>
#include <cuda_runtime_api.h>

// 检查 CUDA 调用的宏
#define CHECK(status) \
    do { \
        auto ret = (status); \
        if (ret != 0) { \
            std::cerr << "Cuda failure: " << ret << " at line " << __LINE__ << std::endl; \
            abort(); \
        } \
    } while (0)

// 内部 Logger 类，对外部隐藏
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << "[TRT] " << msg << std::endl;
        }
    }
} gLogger;

TensorRTDetector::TensorRTDetector(const DetectorConfig& config) : config_(config) {
    // 1. 读取 Engine 文件
    std::ifstream file(config.enginePath, std::ios::binary);
    if (!file.good()) {
        std::cerr << "Error: Could not read engine file: " << config.enginePath << std::endl;
        throw std::runtime_error("Engine file load failed");
    }
    std::string serialized_engine;
    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0, file.beg);
    serialized_engine.resize(size);
    file.read(&serialized_engine[0], size);
    file.close();

    // 2. 初始化 Runtime 和 Engine
    runtime_ = nvinfer1::createInferRuntime(gLogger);
    engine_ = runtime_->deserializeCudaEngine(serialized_engine.data(), size);
    context_ = engine_->createExecutionContext();

    // 3. 分配内存 (Host 和 Device)
    int num_io_tensors = engine_->getNbIOTensors(); 
    
    for (int i = 0; i < num_io_tensors; ++i) {
        const char* name = engine_->getIOTensorName(i);
        nvinfer1::TensorIOMode mode = engine_->getTensorIOMode(name);
        nvinfer1::Dims dims = engine_->getTensorShape(name);
        
        size_t vol = 1;
        int vec_dim = 1; 
        for (int j = 0; j < dims.nbDims; ++j) {
            if (dims.d[j] == -1) vec_dim *= 1; 
            else vec_dim *= dims.d[j];
        }
        vol = vec_dim;

        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            inputName_ = name;
            inputDims_ = dims;
        } else {
            outputName_ = name;
            outputDims_ = dims;
        }

        void* dev_ptr;
        CHECK(cudaMalloc(&dev_ptr, vol * sizeof(float)));
        
        deviceBufferMap_[name] = dev_ptr;
        bufferSizes_[name] = vol * sizeof(float);
    }
    
    cudaStream_t stream;
    CHECK(cudaStreamCreate(&stream));
    stream_ = stream;
    
    size_t outputVol = bufferSizes_[outputName_] / sizeof(float);
    hostOutput_.resize(outputVol);
}

TensorRTDetector::~TensorRTDetector() {
    if (stream_) CHECK(cudaStreamDestroy((cudaStream_t)stream_));
    for (auto& pair : deviceBufferMap_) {
        CHECK(cudaFree(pair.second));
    }
    delete context_;
    delete engine_;
    delete runtime_;
}

void TensorRTDetector::letterbox(const cv::Mat& src, cv::Mat& dst, float& ratio, int& dw, int& dh, int target_w, int target_h) {
    int in_w = src.cols;
    int in_h = src.rows;
    float r = std::min((float)target_w / in_w, (float)target_h / in_h);
    int new_unpad_w = std::round(in_w * r);
    int new_unpad_h = std::round(in_h * r);
    dw = target_w - new_unpad_w;
    dh = target_h - new_unpad_h;
    dw /= 2; dh /= 2;
    if (in_w != new_unpad_w || in_h != new_unpad_h) {
        cv::resize(src, dst, cv::Size(new_unpad_w, new_unpad_h), 0, 0, cv::INTER_LINEAR);
    } else {
        dst = src.clone();
    }
    int top = std::round(dh - 0.1);
    int bottom = std::round(dh + 0.1);
    int left = std::round(dw - 0.1);
    int right = std::round(dw + 0.1);
    cv::copyMakeBorder(dst, dst, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    ratio = r;
}

std::vector<Detection> TensorRTDetector::detect(const cv::Mat& img_raw) {
    int target_h = inputDims_.d[2] == -1 ? 640 : inputDims_.d[2];
    int target_w = inputDims_.d[3] == -1 ? 640 : inputDims_.d[3];
    cudaStream_t stream = (cudaStream_t)stream_;

    // 1. 预处理
    cv::Mat img;
    float ratio;
    int dw, dh;
    letterbox(img_raw, img, ratio, dw, dh, target_w, target_h);

    cv::Mat blob;
    cv::dnn::blobFromImage(img, blob, 1.0/255.0, cv::Size(target_w, target_h), cv::Scalar(0,0,0), true, false);

    // 2. 推理
    void* inputPtr = deviceBufferMap_[inputName_];
    void* outputPtr = deviceBufferMap_[outputName_];

    CHECK(cudaMemcpyAsync(inputPtr, blob.ptr<float>(), bufferSizes_[inputName_], cudaMemcpyHostToDevice, stream));

    context_->setTensorAddress(inputName_.c_str(), inputPtr);
    context_->setTensorAddress(outputName_.c_str(), outputPtr);

    nvinfer1::Dims runtimeInputDims = inputDims_;
    runtimeInputDims.d[0] = 1; 
    runtimeInputDims.d[2] = target_h;
    runtimeInputDims.d[3] = target_w;
    context_->setInputShape(inputName_.c_str(), runtimeInputDims);

    context_->enqueueV3(stream);

    CHECK(cudaMemcpyAsync(hostOutput_.data(), outputPtr, bufferSizes_[outputName_], cudaMemcpyDeviceToHost, stream));
    CHECK(cudaStreamSynchronize(stream));

    // 3. 解析结果
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    int num_anchors = outputDims_.d[1]; 
    int data_len = outputDims_.d[2];
    float* pdata = hostOutput_.data();
    
    for (int i = 0; i < num_anchors; ++i) {
        float* row = pdata + i * data_len;
        float obj_conf = row[4];
        
        float max_cls_score = 0;
        int class_id = 0;
        if (data_len > 5) {
            for (int c = 5; c < data_len; ++c) {
                if (row[c] > max_cls_score) {
                    max_cls_score = row[c];
                    class_id = c - 5;
                }
            }
        } else {
            max_cls_score = 1.0f;
        }

        float score = obj_conf * max_cls_score;

        if (score > config_.confThreshold) {
            float cx = row[0];
            float cy = row[1];
            float w = row[2];
            float h = row[3];

            cx = (cx - dw) / ratio;
            cy = (cy - dh) / ratio;
            w = w / ratio;
            h = h / ratio;

            boxes.push_back(cv::Rect((int)(cx - w/2), (int)(cy - h/2), (int)w, (int)h));
            confidences.push_back(score);
            classIds.push_back(class_id);
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, config_.confThreshold, config_.iouThreshold, indices);

    std::vector<Detection> results;
    for (int idx : indices) {
        results.push_back({boxes[idx], confidences[idx], classIds[idx]});
    }
    return results;
}

void TensorRTDetector::draw(cv::Mat& img, const std::vector<Detection>& objects) {
    for (const auto& obj : objects) {
        cv::rectangle(img, obj.box, cv::Scalar(255, 0, 0), 2);
        std::string label = cv::format("%.2f", obj.conf);
        cv::putText(img, label, cv::Point(obj.box.x, obj.box.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
    }
}