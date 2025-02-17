#ifndef YOLOV5_DET_H
#define YOLOV5_DET_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <NvInfer.h>  // Add TensorRT includes
#include <cuda_runtime_api.h>  
#include "config.h"

// Add namespace declarations
namespace nvinfer1 {
    class IRuntime;
    class ICudaEngine;
    class IExecutionContext;
}


class YoloDetector {
public:
    YoloDetector(const std::string& engine_path);
    ~YoloDetector();
    std::vector<Detection> process(const cv::Mat& frame);
    
private:
    void deserialize_engine(const std::string& engine_path);
    void prepare_buffers();
    void infer(const cv::Mat& frame);
    
    // Add namespace qualification to TensorRT types
    void* mBuffers[2];
    float* mCpuOutputBuffer;
    cudaStream_t mStream;
    nvinfer1::IRuntime* mRuntime;
    nvinfer1::ICudaEngine* mEngine;
    nvinfer1::IExecutionContext* mContext;
};

#endif // YOLOV5_DET_H
