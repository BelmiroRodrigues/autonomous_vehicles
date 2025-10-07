#ifndef UTILS_HPP
#define UTILS_HPP

#include "control_systems/utils_control.hpp"
#include "object_detection/fps.hpp"
#include "object_detection/frame.hpp"
#include "object_detection/TensorRTYOLO.hpp"
#include <opencv2/opencv.hpp>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>

// Forward declarations
class TensorRTYOLO;
class TensorRTInference;
class NMPCController;
class PID;
class CSICamera;
class FrameSkipper;
class FPSCalculator;
class ZmqPublisher;
class ZmqSubscriber;

// Structs
struct ObjectResults {
    std::vector<Detection> detections;
};

struct LaneResults {
    cv::Mat processed_frame;
    double offset = 0.0;
    double psi = 0.0;
    double delta_rad = 0.0;
    int steering_angle_deg = 0;
    std::vector<cv::Point> medianPoints;
    LineIntersect intersect;
};

extern std::atomic<bool> keep_running;
extern std::atomic<double> current_speed_ms;
extern std::atomic<double> current_speed;
extern std::queue<cv::Mat> frame_queue_objects;
extern std::queue<cv::Mat> frame_queue_lanes;
extern std::mutex mtx_objects, mtx_lanes, mtx_results;
extern std::condition_variable cv_objects, cv_lanes, cv_results;
extern ObjectResults latest_objects;
extern LaneResults latest_lanes;
extern bool results_ready;

void objectInferenceThread(TensorRTYOLO& detector, FrameSkipper& frame_skipper, 
                          FPSCalculator& fps_calculator, ZmqPublisher* lane_pub);

void laneInferenceThread(TensorRTInference& trt, NMPCController& mpc, PID& pid, 
                        double setpoint_velocity, FrameSkipper& frame_skipper, 
                        ZmqPublisher* lane_pub, ZmqPublisher* ctrl_pub);

cv::Mat combineAndDraw(const cv::Mat& original_frame, const ObjectResults& obj_res, 
                      const LaneResults& lane_res, double smooth_fps, 
                      int processed_frames, int skipped_frames);

void cleanup(CSICamera& camera, std::thread& obj_thread, std::thread& lane_thread, 
            ZmqPublisher* lane_pub, ZmqPublisher* ctrl_pub, ZmqPublisher* obj_pub, 
            ZmqSubscriber* speed_sub);

#endif // UTILS_HPP
