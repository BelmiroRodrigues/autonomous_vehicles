#ifndef INFER_HPP
#define INFER_HPP

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

using namespace nvinfer1;

class Logger : public ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << msg << std::endl;
    }
};

struct Buffer {
    void* device;
    float* host;
    size_t size;
};

class TensorRTInference {
public:
    TensorRTInference(const std::string& engine_path);
    ~TensorRTInference();

    std::vector<float> infer(const std::vector<float>& inputData);

private:
    Logger logger;
    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    IExecutionContext* context = nullptr;

    std::vector<Buffer> inputBuffers;
    std::vector<Buffer> outputBuffers;
    std::vector<void*> bindings;

    void allocateBuffers();
};

#endif