#include "utils_control.hpp"
#include <iostream>
#include <stdexcept>
#include <sstream>

LaneControl::LaneControl(const std::string& model_path, int width, int height, int fps)
    : trt(model_path), cam(width, height, fps) {}


ZmqPublisher* initZmq(zmq::context_t& context) {
    const std::string ZMQ_HOST = "127.0.0.1";
    const int ZMQ_PORT = 5558;

    try {
        auto* zmq_publisher = new ZmqPublisher(context, ZMQ_HOST, ZMQ_PORT);
        if (!zmq_publisher->isConnected()) {
            std::cerr << "[WARNING]: Failed to initialize ZMQ Publisher." << std::endl;
        }
        return zmq_publisher;
    } catch (const std::exception& e) {
        std::cerr << "FATAL ERROR starting ZMQ Publisher: " << e.what() << std::endl;
        return nullptr;
    }
}

// ---- HUD ----
void drawHUD(cv::Mat& frame,
             double smoothedFPS,
             double delta,
             double v_actual,
             double motor_pwm,
             double offset,
             double psi,
             int steering_angle) 
{
    int y = 20; 
    int step = 20;

    auto put = [&](const std::string& text, cv::Scalar color) {
        cv::putText(frame, text, cv::Point(10, y), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        y += step;
    };

    //put("FPS: " + std::to_string(static_cast<int>(smoothedFPS)), cv::Scalar(0, 255, 0));
    put("Delta: " + std::to_string(delta * 180.0 / M_PI) + " deg", cv::Scalar(0, 0, 255));
    put("V_actual: " + std::to_string(v_actual) + " m/s", cv::Scalar(0, 0, 255));
    put("Motor: " + std::to_string(motor_pwm), cv::Scalar(0, 255, 0));
    put("Desv Lat: " + std::to_string(offset), cv::Scalar(255, 100, 0));
    put("Psi(rad): " + std::to_string(psi) + 
        " (deg): " + std::to_string(psi * 180.0 / M_PI), cv::Scalar(0, 255, 0));
    put("Steering: " + std::to_string(steering_angle) + " deg", cv::Scalar(200, 0, 0));
}
