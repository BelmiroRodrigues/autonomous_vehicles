#ifndef LANE_HPP
#define LANE_HPP

#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <thread>
#include <atomic>
#include <vector>
#include <deque>
#include <iostream>
#include <chrono>

struct LineCoef {
    double m, b;
    bool valid;
};

// Estrutura para armazenar os dados persistentes
struct LaneMemory {
    std::vector<cv::Point> pre_left_edge;
    std::vector<cv::Point> pre_right_edge;
    LineCoef pre_left_coeffs;
    LineCoef pre_right_coeffs;

    // Inicialização padrão
    LaneMemory() : pre_left_coeffs{0.0, 0.0, false}, pre_right_coeffs{0.0, 0.0, false} {
        pre_left_edge.clear();
        pre_right_edge.clear();
    }
};

struct LineIntersect {
    float xlt;
    float xlb; 
    float xrt;
    float xrb;
    float slope;        // Inclinação da mediana estimada
    float psi;          // Yaw (ângulo) da mediana estimada
    float offset;    // offset no centro de massa do carro
    float w_real = 0.26; // distancia entre as linhas da pista em metros
    bool valid;
};

struct LineIntersectBuffer {
    std::deque<LineIntersect> buffer;

    void add(const LineIntersect& value) {
        if (buffer.size() >= 10) {
            buffer.pop_front();
        }
        buffer.push_back(value);
    }

    std::deque<LineIntersect> get() const {
        return buffer;
    }
};

static LaneMemory lane_memory;

static constexpr double Asy = -3.39e-06;
static constexpr double Bsy = 1.61e-03;
static constexpr double P2_y_img_frame = 0.24;
static constexpr double P1_y_img_frame = 0.475;

std::vector<float>  preprocess_frame(const cv::Mat& frame);
cv::Mat             postprocess(float* ll_output, cv::Mat& original_frame, std::vector<cv::Point>& medianPoints, LineIntersect& intersect);
LineIntersect       findIntersect(const LineCoef& left_coeffs, const LineCoef& right_coeffs, int height, int width);
void                findOffset(LineIntersect& intersect);
void                drawLanes(LineCoef left_coeffs, LineCoef right_coeffs, cv::Mat& result_frame, std::vector<cv::Point> medianPoints, int roi_start_y,  int roi_end_y);

#endif