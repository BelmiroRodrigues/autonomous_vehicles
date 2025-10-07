#ifndef TENSORRT_YOLO_HPP
#define TENSORRT_YOLO_HPP

#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "control_systems/LanesGeometry/infer.hpp"

struct Detection {
  int class_id;
  std::string class_name;
  float confidence;
  cv::Rect bbox;
};

class TensorRTYOLO {
public:
  explicit TensorRTYOLO(const std::string& engine_path, int input_sz = 640);
  ~TensorRTYOLO();

  std::vector<Detection> infer(const cv::Mat& image);
  cv::Scalar getColor(int class_id) const;

private:
  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;

  struct Buffer {
    void* device{nullptr};
    float* host{nullptr};
    size_t size{0};
  };
  
  std::vector<Buffer> inputBuffers_;
  std::vector<Buffer> outputBuffers_;
  std::vector<void*> bindings_;

  int input_size_{640};
  int num_classes_{0};
  size_t output_size_{0};

  std::map<int, std::string> classes_;
  std::map<int, cv::Scalar> colors_;

  void allocateBuffers();
  std::vector<float> preprocess(const cv::Mat& image, float& scale, int& dw, int& dh);
  std::vector<Detection> postprocess(const std::vector<float>& output,
                                     float scale, int dw, int dh,
                                     float conf_threshold = 0.3f,
                                     float nms_threshold = 0.4f);
};

#endif // TENSORRT_YOLO_HPP
