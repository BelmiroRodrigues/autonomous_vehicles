#include "infer.hpp"
#include <fstream>
#include <stdexcept>
#include <cuda_runtime.h>

TensorRTInference::TensorRTInference(const std::string& engine_path) {
    std::ifstream engineFile(engine_path, std::ios::binary);
    if (!engineFile) {
        throw std::runtime_error(std::string("Lane engine not found: ") + engine_path);
    }

    engineFile.seekg(0, engineFile.end);
    size_t fsize = static_cast<size_t>(engineFile.tellg());
    engineFile.seekg(0, engineFile.beg);

    std::vector<char> engineData(fsize);
    engineFile.read(engineData.data(), fsize);

    runtime = std::unique_ptr<nvinfer1::IRuntime>(createInferRuntime(logger));
    if (!runtime) {
        throw std::runtime_error("Failed to create IRuntime");
    }
    
    engine = std::unique_ptr<nvinfer1::ICudaEngine>(runtime->deserializeCudaEngine(engineData.data(), fsize));
    if (!engine) {
        throw std::runtime_error("Failed to deserialize ICudaEngine");
    }
    
    context = std::unique_ptr<nvinfer1::IExecutionContext>(engine->createExecutionContext());
    if (!context) {
        throw std::runtime_error("Failed to create IExecutionContext");
    }
    allocateBuffers();
}

TensorRTInference::~TensorRTInference() {
    for (auto& mem : inputBuffers)  { if (mem.device) cudaFree(mem.device);  delete[] mem.host; }
    for (auto& mem : outputBuffers) { if (mem.device) cudaFree(mem.device);  delete[] mem.host; }
}

void TensorRTInference::allocateBuffers() {
    const int nbBindings = engine->getNbBindings();

    inputBuffers.resize(1);
    outputBuffers.resize(1);
    bindings.resize(nbBindings, nullptr);

    for (int i = 0; i < nbBindings; ++i) {
        Dims dims = engine->getBindingDimensions(i);
        size_t vol = 1;
        for (int j = 0; j < dims.nbDims; ++j) {
            vol *= static_cast<size_t>(dims.d[j]);
        }

        const size_t totalSize = vol * sizeof(float);

        void* deviceMem = nullptr;
        auto st = cudaMalloc(&deviceMem, totalSize);
        if (st != cudaSuccess) {
            throw std::runtime_error(std::string("Lane: cudaMalloc failed: ") + cudaGetErrorString(st));
        }

        float* hostMem = new float[vol];
        bindings[i] = deviceMem;

        if (engine->bindingIsInput(i)) {
            inputBuffers[0] = Buffer{deviceMem, hostMem, totalSize};
        } else {
            outputBuffers[0] = Buffer{deviceMem, hostMem, totalSize};
        }
    }
}

std::vector<float> TensorRTInference::infer(const std::vector<float>& inputData) {
    cudaMemcpy(inputBuffers[0].device, inputData.data(), inputBuffers[0].size, cudaMemcpyHostToDevice);

    context->executeV2(bindings.data());
    auto err = cudaPeekAtLastError();
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("Lane CUDA kernel error: ") + cudaGetErrorString(err));
    }
    auto syncErr = cudaDeviceSynchronize();
    if (syncErr != cudaSuccess) {
        throw std::runtime_error(std::string("Lane CUDA sync error: ") + cudaGetErrorString(syncErr));
    }

    cudaMemcpy(outputBuffers[0].host, outputBuffers[0].device, outputBuffers[0].size, cudaMemcpyDeviceToHost);

    const size_t num_floats = outputBuffers[0].size / sizeof(float);
    return std::vector<float>(outputBuffers[0].host, outputBuffers[0].host + num_floats);
}