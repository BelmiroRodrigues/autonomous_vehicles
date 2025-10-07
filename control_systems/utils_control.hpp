#ifndef UTILS_CONTROL_HPP
#define UTILS_CONTROL_HPP

#include "LanesGeometry/lane.hpp"
#include "MPC/nmpc.hpp"
#include "PID/pid.hpp"
#include "ZMQ/ZmqPublisher.hpp"
#include "ZMQ/ZmqSubscriber.hpp"
#include "Cam/cam.hpp"
#include "LanesGeometry/infer.hpp"

#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <unistd.h>
#include <csignal>
#include <iomanip>
#include <zmq.hpp>
#include <sstream>
#include <queue>
#include <signal.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// ---- Estruturas ----
struct LaneControl {
    TensorRTInference trt;
    CSICamera cam;
    LaneControl(const std::string& model_path, int width, int height, int fps);
};

// ---- Funções auxiliares ----
ZmqPublisher* initZmq(zmq::context_t& context);

void drawHUD(cv::Mat& frame,
             double smoothedFPS,
             double delta,
             double v_actual,
             double motor_pwm,
             double offset,
             double psi,
             int steering_angle);
#endif
