#include "utils.hpp"
#include <iostream>
#include <chrono>
#include <sstream>
#include <cmath>
#include <algorithm>

std::atomic<bool> keep_running{true};
std::atomic<double> current_speed_ms{0.0};
std::atomic<double> current_speed{0.0};
std::queue<cv::Mat> frame_queue_objects;
std::queue<cv::Mat> frame_queue_lanes;
std::mutex mtx_objects, mtx_lanes, mtx_results;
std::condition_variable cv_objects, cv_lanes, cv_results;
ObjectResults latest_objects;
LaneResults latest_lanes;
bool results_ready = false;

void objectInferenceThread(TensorRTYOLO& detector, FrameSkipper& frame_skipper, 
                          FPSCalculator& fps_calculator, ZmqPublisher* lane_pub) {
    std::vector<Detection> last_detections;
    while (keep_running.load()) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(mtx_objects);
            cv_objects.wait(lock, [&] { return !frame_queue_objects.empty() || !keep_running.load(); });
            if (!keep_running.load()) break;
            frame = frame_queue_objects.front();
            frame_queue_objects.pop();
        }

        fps_calculator.update();
        std::vector<Detection> detections;
        if (frame_skipper.shouldProcessFrame()) {
            auto start = std::chrono::high_resolution_clock::now();
            if (frame.empty()) {
                std::cerr << "Empty frame in objectInferenceThread! Skipping inference." << std::endl;
                continue;
            }
            detections = detector.infer(frame);
            auto end = std::chrono::high_resolution_clock::now();
            double time = std::chrono::duration<double>(end - start).count();
            frame_skipper.recordProcessingTime(time);
            last_detections = detections;
        } else {
            detections = last_detections;
        }

        if (!detections.empty()) {
            std::cout << "Detected objects (" << detections.size() << "): ";
            for (const auto& det : detections) {
                std::cout << det.class_name << " (" << det.confidence << ") ";
            }
            std::cout << std::endl;

            if (lane_pub && lane_pub->isConnected()) {
                for (const auto& det : detections) {
                    std::stringstream ss;
                    ss << det.class_name;
                    lane_pub->publishMessage(ss.str());
                }
            }
        }
        {
            std::unique_lock<std::mutex> lock(mtx_results);
            latest_objects.detections = detections;
            results_ready = true;
            cv_results.notify_one();
        }
    }
}

void laneInferenceThread(TensorRTInference& trt, NMPCController& mpc, PID& pid, 
                        double setpoint_velocity, FrameSkipper& frame_skipper, 
                        ZmqPublisher* lane_pub, ZmqPublisher* ctrl_pub) {
    auto lastTime = std::chrono::steady_clock::now();
    double last_delta = 0.0;
    auto pid_last_time = std::chrono::steady_clock::now();

    while (keep_running.load()) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(mtx_lanes);
            cv_lanes.wait(lock, [&] { return !frame_queue_lanes.empty() || !keep_running.load(); });
            if (!keep_running.load()) break;
            frame = frame_queue_lanes.front();
            frame_queue_lanes.pop();
        }

        if (!frame_skipper.shouldProcessFrame()) continue;

        auto start = std::chrono::high_resolution_clock::now();
        if (frame.empty()) {
            std::cerr << "Empty frame in laneInferenceThread! Skipping." << std::endl;
            continue;
        }
        
        std::vector<float> input = preprocess_frame(frame);
        auto outputs = trt.infer(input);
        std::vector<cv::Point> medianPoints;
        LineIntersect intersect;
        cv::Mat result = postprocess(outputs.data(), frame, medianPoints, intersect);
        auto end = std::chrono::high_resolution_clock::now();
        double time = std::chrono::duration<double>(end - start).count();
        frame_skipper.recordProcessingTime(time);

        auto currentTime = std::chrono::steady_clock::now();
        lastTime = currentTime;

        double v_actual = current_speed_ms.load();
        std::cout << "Speed: " << v_actual << std::endl;
        auto pid_now = std::chrono::steady_clock::now();
        double pid_dt = std::chrono::duration<double>(pid_now - pid_last_time).count();
        double motor_pwm = 0.0;
        if (pid_dt >= 0.02) {
            motor_pwm = pid.compute(setpoint_velocity, v_actual, pid_dt);
            pid_last_time = pid_now;
        }

        double offset = intersect.offset;
        double psi = intersect.psi;
        double delta = last_delta;
        if (!std::isnan(offset) && !std::isnan(psi)) {
            delta = -mpc.computeControl(offset, psi, 0.7);
        }

        double target_steering_angle = delta * 180.0 / M_PI;
        double smoothed_steering_angle = target_steering_angle;
        int steering_angle = static_cast<int>(smoothed_steering_angle);
        steering_angle = std::max(-40, std::min(40, steering_angle));
        last_delta = delta;

        int lane = (offset < -0.01) ? 2 : ((offset > 0.02) ? 1 : 0);
        if (lane_pub && lane_pub->isConnected()) {
            std::stringstream ss;
            ss << "lane:" << lane;
            lane_pub->publishMessage(ss.str());
        }
        if (ctrl_pub && ctrl_pub->isConnected()) {
            std::stringstream ss2;
            ss2 << "throttle:" << motor_pwm << ";steering:" << steering_angle << ";";
            ctrl_pub->publishMessage(ss2.str());
        }
        {
            std::unique_lock<std::mutex> lock(mtx_results);
            latest_lanes.processed_frame = result;
            latest_lanes.offset = offset;
            latest_lanes.psi = psi;
            latest_lanes.medianPoints = medianPoints;
            latest_lanes.intersect = intersect;
            latest_lanes.delta_rad = delta;
            latest_lanes.steering_angle_deg = steering_angle;
            results_ready = true;
            cv_results.notify_one();
        }
    }
}

cv::Mat combineAndDraw(const cv::Mat& original_frame, const ObjectResults& obj_res, 
                      const LaneResults& lane_res, double smooth_fps, 
                      int processed_frames, int skipped_frames) {
    if (original_frame.empty()) {
        std::cerr << "Empty original frame in combineAndDraw." << std::endl;
        return cv::Mat();
    }
    
    cv::Mat combined = original_frame.clone();
    
    for (const auto& det : obj_res.detections) {
        cv::Scalar color = cv::Scalar(255, 0, 0);
        cv::rectangle(combined, det.bbox, color, 2);
        std::string label = det.class_name + ": " + std::to_string(det.confidence).substr(0, 4);
        cv::putText(combined, label, cv::Point(det.bbox.x, det.bbox.y - 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
    }

    if (!lane_res.processed_frame.empty()) {
        combined = lane_res.processed_frame.clone();
    } else {
        std::cout << "Lane processed frame is empty — using original." << std::endl;
    }

    drawHUD(combined, smooth_fps, lane_res.delta_rad, current_speed_ms.load(), 0.0, 
            lane_res.offset, lane_res.psi, lane_res.steering_angle_deg);

    return combined;
}

void cleanup(CSICamera& camera, std::thread& obj_thread, std::thread& lane_thread, 
             ZmqPublisher* lane_pub, ZmqPublisher* ctrl_pub, ZmqPublisher* obj_pub, 
             ZmqSubscriber* speed_sub) {
    std::cout << "Initializing cleanup..." << std::endl;
    keep_running.store(false);
    cv_objects.notify_all();
    cv_lanes.notify_all();
    cv_results.notify_all();
    
    // Send final "zero" messages before shutting down publishers
    if (ctrl_pub && ctrl_pub->isConnected()) {
        std::string zero_msg = "throttle:0;steering:0;";
        ctrl_pub->publishMessage(zero_msg);
        std::cout << "Sent final control message: " << zero_msg << std::endl;
    }
    if (lane_pub && lane_pub->isConnected()) {
        std::string zero_msg = "lane:0";
        lane_pub->publishMessage(zero_msg);
        std::cout << "Sent final lane message: " << zero_msg << std::endl;
    }
    if (obj_pub && obj_pub->isConnected()) {
        std::string zero_msg = "objects:0";
        obj_pub->publishMessage(zero_msg);
        std::cout << "Sent final object message: " << zero_msg << std::endl;
    }

    // Wait for threads to finish
    if (obj_thread.joinable()) obj_thread.join();
    if (lane_thread.joinable()) lane_thread.join();
    
    // Stop camera and destroy windows
    camera.stop();
    cv::destroyAllWindows();
    
    // Clean up ZeroMQ publishers and subscriber
    if (lane_pub) delete lane_pub;
    if (ctrl_pub) delete ctrl_pub;
    if (obj_pub) delete obj_pub;
    if (speed_sub) {
        speed_sub->stop();
        delete speed_sub;
    }
    std::cout << "Complete cleanup!" << std::endl;
}