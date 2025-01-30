#include "yolov5_det.h"
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include <chrono>

using namespace nvinfer1;

static Logger gLogger;

// Keep these constants or make them configurable
const static int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

YoloDetector::YoloDetector(const std::string& engine_path) {
    cudaSetDevice(kGpuId);
    deserialize_engine(engine_path);
    CUDA_CHECK(cudaStreamCreate(&mStream));
    cuda_preprocess_init(kMaxInputImageSize);
    prepare_buffers();
}

YoloDetector::~YoloDetector() {
    // Cleanup resources
    cudaStreamDestroy(mStream);
    CUDA_CHECK(cudaFree(mBuffers[0]));
    CUDA_CHECK(cudaFree(mBuffers[1]));
    delete[] mCpuOutputBuffer;
    cuda_preprocess_destroy();
    mContext->destroy();
    mEngine->destroy();
    mRuntime->destroy();
}

void YoloDetector::deserialize_engine(const std::string& engine_path) {
    std::ifstream file(engine_path, std::ios::binary);
    if (!file.good()) {
        throw std::runtime_error("Unable to read engine file");
    }

    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0, file.beg);
    char* serialized_engine = new char[size];
    file.read(serialized_engine, size);
    file.close();

    mRuntime = createInferRuntime(gLogger);
    mEngine = mRuntime->deserializeCudaEngine(serialized_engine, size);
    mContext = mEngine->createExecutionContext();
    
    delete[] serialized_engine;
}

void YoloDetector::prepare_buffers() {
    // Allocate GPU and CPU memory
    const int inputIndex = mEngine->getBindingIndex(kInputTensorName);
    assert(inputIndex == 0);
    const int outputIndex = mEngine->getBindingIndex(kOutputTensorName);
    assert(outputIndex == 1);

    CUDA_CHECK(cudaMalloc(&mBuffers[inputIndex], kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&mBuffers[outputIndex], kBatchSize * kOutputSize * sizeof(float)));
    mCpuOutputBuffer = new float[kBatchSize * kOutputSize];
}

std::vector<Detection> YoloDetector::process(const cv::Mat& frame) {
    // Preprocess
    std::vector<cv::Mat> batch = {frame};
    cuda_batch_preprocess(batch, (float*)mBuffers[0], kInputW, kInputH, mStream);

    // Inference
    mContext->enqueue(kBatchSize, mBuffers, mStream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(mCpuOutputBuffer, mBuffers[1], 
                              kBatchSize * kOutputSize * sizeof(float),
                              cudaMemcpyDeviceToHost, mStream));
    cudaStreamSynchronize(mStream);

    // Postprocess
    std::vector<std::vector<Detection>> batch_res;
    batch_nms(batch_res, mCpuOutputBuffer, 1, kOutputSize, kConfThresh, kNmsThresh);

    return batch_res[0];
}



