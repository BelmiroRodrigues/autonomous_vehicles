#ifndef UTILS_CONTROL_HPP
#define UTILS_CONTROL_HPP

#include "LanesGeometry/lane.hpp"
#include "MPC/nmpc.hpp"
#include "PID/pid.hpp"
#include "PID/SpeedPIDController.hpp"
#include "PID/SpeedPIDTuner.hpp"
#include "aux/scurve.hpp"
#include "aux/MovingAverage.hpp"
#include "aux/SpeedFilter.hpp"
#include "ZmqPublisher.hpp"
#include "ZmqSubscriber.hpp"
#include "cam.hpp"
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

// Componentes do seu sistema para controlo de motor e CAN Bus
#include "FServo/FServo.hpp"
#include "Control/ControlAssembly.hpp"
#include "BackMotors/BackMotors.hpp"
#include "../MCP2515/CanBusManager.hpp"
#include "../MCP2515/MCP2515Controller.hpp"
#include "../MCP2515/SPIController.hpp"
#include "../MCP2515/MCP2515Configurator.hpp"
#include "../MCP2515/CANMessageProcessor.hpp"


// ---- Estruturas ----
struct LaneControl {
    TensorRTInference trt;
    CSICamera cam;

    LaneControl(const std::string& model_path, int width, int height, int fps);
};

// ---- Funções auxiliares ----
std::unique_ptr<LaneControl> initLaneControl();
bool initMotors(BackMotors& backMotors);
bool initServo(FServo& servo);
std::unique_ptr<CanBusManager> initCanBus(std::shared_ptr<CANMessageProcessor>& messageProcessor);
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
