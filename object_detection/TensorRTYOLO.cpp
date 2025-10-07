#include "TensorRTYOLO.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>

using namespace nvinfer1;

TensorRTYOLO::TensorRTYOLO(const std::string& engine_path, int input_sz)
  : input_size_(input_sz) {
  // Class names (ID -> label)
  classes_ = {
    {0, "STOP"}, {1, "YIELD"}, {2, "SPEED_50"}, {3, "SPEED_80"},
    {4, "LIGHT_RED"}, {5, "LIGHT_GREEN"},
    {6, "LIGHT_YELLOW"}, {7, "CROSSWALK"}, {8, "DANGER"}, {9, "DANGER_CURVE"}
  };

  // Colors for drawing
  colors_ = {
    {0, cv::Scalar(255, 0, 0)},   {1, cv::Scalar(0, 255, 0)},
    {2, cv::Scalar(0, 0, 255)},   {3, cv::Scalar(255, 255, 0)},
    {4, cv::Scalar(255, 0, 255)}, {5, cv::Scalar(0, 255, 255)},
    {6, cv::Scalar(128, 0, 128)}, {7, cv::Scalar(0, 128, 255)},
    {8, cv::Scalar(128, 128, 0)}, {9, cv::Scalar(255, 165, 0)}
  };

  num_classes_ = static_cast<int>(classes_.size());

  std::ifstream engineFile(engine_path, std::ios::binary);
  if (!engineFile) {
    throw std::runtime_error("Error loading engine: " + engine_path);
  }

  engineFile.seekg(0, engineFile.end);
  size_t fsize = static_cast<size_t>(engineFile.tellg());
  engineFile.seekg(0, engineFile.beg);

  std::vector<char> engineData(fsize);
  engineFile.read(engineData.data(), fsize);

  runtime_.reset(createInferRuntime(logger_));
  if (!runtime_) {
    throw std::runtime_error("Failed to create nvinfer1::IRuntime");
  }

  engine_.reset(runtime_->deserializeCudaEngine(engineData.data(), fsize));
  if (!engine_) {
    throw std::runtime_error("Failed to deserialize nvinfer1::ICudaEngine");
  }

  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("Failed to create nvinfer1::IExecutionContext");
  }

  allocateBuffers();

  std::cout << "TensorRT Engine loaded: " << engine_path << std::endl;
  std::cout << "Input size: " << input_size_ << "x" << input_size_ << std::endl;
  std::cout << "Num classes: " << num_classes_ << std::endl;
  std::cout << "Output size: " << output_size_ << std::endl;
}

TensorRTYOLO::~TensorRTYOLO() {
  for (auto& mem : inputBuffers_) {
    if (mem.device) cudaFree(mem.device);
    delete[] mem.host;
  }
  for (auto& mem : outputBuffers_) {
    if (mem.device) cudaFree(mem.device);
    delete[] mem.host;
  }
}

void TensorRTYOLO::allocateBuffers() {
  const int nbBindings = engine_->getNbBindings();
  inputBuffers_.resize(1);
  outputBuffers_.resize(1);
  bindings_.resize(nbBindings, nullptr);

  for (int i = 0; i < nbBindings; ++i) {
    Dims dims = engine_->getBindingDimensions(i);
    size_t vol = 1;
    for (int j = 0; j < dims.nbDims; ++j) {
      vol *= static_cast<size_t>(dims.d[j]);
    }
    const size_t totalSize = vol * sizeof(float);

    void* deviceMem = nullptr;
    cudaMalloc(&deviceMem, totalSize);
    float* hostMem = new float[vol];

    bindings_[i] = deviceMem;

    if (engine_->bindingIsInput(i)) {
      inputBuffers_[0] = {deviceMem, hostMem, totalSize};
    } else {
      outputBuffers_[0] = {deviceMem, hostMem, totalSize};
      output_size_ = vol;
    }
  }
}

std::vector<float> TensorRTYOLO::preprocess(const cv::Mat& image, float& scale, int& dw, int& dh) {
  if (image.empty()) {
    throw std::runtime_error("Empty input image in preprocess");
  }
  const int h = image.rows;
  const int w = image.cols;
  if (w <= 0 || h <= 0) {
    throw std::runtime_error("Invalid image dimensions");
  }

  scale = std::min(static_cast<float>(input_size_) / w,
                   static_cast<float>(input_size_) / h);
  const int nw = static_cast<int>(scale * w);
  const int nh = static_cast<int>(scale * h);
  if (nw <= 0 || nh <= 0) {
    throw std::runtime_error("Invalid resized dimensions");
  }

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(nw, nh));
  if (resized.empty()) {
    throw std::runtime_error("Failed to resize: empty result");
  }

  cv::Mat padded = cv::Mat::ones(input_size_, input_size_, CV_8UC3) * 114;
  dw = (input_size_ - nw) / 2;
  dh = (input_size_ - nh) / 2;
  resized.copyTo(padded(cv::Rect(dw, dh, nw, nh)));

  padded.convertTo(padded, CV_32FC3, 1.0 / 255.0);

  std::vector<cv::Mat> channels(3);
  cv::split(padded, channels);

  std::vector<float> inputData;
  inputData.reserve(3 * input_size_ * input_size_);
  for (int i = 0; i < 3; ++i) {
    inputData.insert(inputData.end(), (float*)channels[i].datastart, (float*)channels[i].dataend);
  }
  return inputData;
}

std::vector<Detection> TensorRTYOLO::postprocess(const std::vector<float>& output,
                                                 float scale, int dw, int dh,
                                                 float conf_threshold,
                                                 float nms_threshold) {
  std::vector<Detection> detections;

  try {
    const int total_elements = static_cast<int>(output.size());
    const int stride = 4 + num_classes_;
    if (total_elements % stride != 0) {
      // Fallback: try the layout used by the original code (column-major blocks)
    }
    const int num_detections = total_elements / stride;

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    // Layout assumption: column-blocked as in the provided code:
    // [cx block][cy block][w block][h block][scores for class 0][class 1]...
    for (int i = 0; i < num_detections; ++i) {
      const float cx     = output[i];
      const float cy     = output[num_detections + i];
      const float width  = output[2 * num_detections + i];
      const float height = output[3 * num_detections + i];

      float max_conf = 0.0f;
      int best_id = 0;
      for (int j = 0; j < num_classes_; ++j) {
        const float cls_conf = output[(4 + j) * num_detections + i];
        if (cls_conf > max_conf) {
          max_conf = cls_conf;
          best_id = j;
        }
      }

      if (max_conf > conf_threshold) {
        const float x_center = (cx - static_cast<float>(dw)) / scale;
        const float y_center = (cy - static_cast<float>(dh)) / scale;
        const float w = width / scale;
        const float h = height / scale;

        const float x1 = x_center - w * 0.5f;
        const float y1 = y_center - h * 0.5f;

        if (x1 >= 0 && y1 >= 0 && w > 0 && h > 0) {
          boxes.emplace_back(static_cast<int>(x1), static_cast<int>(y1),
                             static_cast<int>(w), static_cast<int>(h));
          confidences.push_back(max_conf);
          class_ids.push_back(best_id);
        }
      }
    }

    if (!boxes.empty()) {
      std::vector<int> indices;
      cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);
      for (int idx : indices) {
        Detection det;
        det.class_id = class_ids[idx];
        auto it = classes_.find(det.class_id);
        det.class_name = (it != classes_.end()) ? it->second : "unknown";
        det.confidence = confidences[idx];
        det.bbox = boxes[idx];
        detections.push_back(det);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Postprocessing error: " << e.what() << std::endl;
  }

  return detections;
}

std::vector<Detection> TensorRTYOLO::infer(const cv::Mat& image) {
  try {
    float scale = 1.0f;
    int dw = 0, dh = 0;

    const std::vector<float> input_data = preprocess(image, scale, dw, dh);
    std::copy(input_data.begin(), input_data.end(), inputBuffers_[0].host);

    cudaMemcpy(inputBuffers_[0].device, inputBuffers_[0].host,
               inputBuffers_[0].size, cudaMemcpyHostToDevice);

    context_->executeV2(bindings_.data());

    cudaMemcpy(outputBuffers_[0].host, outputBuffers_[0].device,
               outputBuffers_[0].size, cudaMemcpyDeviceToHost);

    std::vector<float> output(outputBuffers_[0].host,
                              outputBuffers_[0].host + output_size_);

    return postprocess(output, scale, dw, dh, 0.6f, 0.4f);
  } catch (const std::exception& e) {
    std::cerr << "Error during inference: " << e.what() << std::endl;
    return {};
  }
}

cv::Scalar TensorRTYOLO::getColor(int class_id) const {
  auto it = colors_.find(class_id);
  return (it != colors_.end()) ? it->second : cv::Scalar(255, 255, 255);
}
