#include "utils.hpp"
#include <filesystem>

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received." << std::endl;
    keep_running.store(false);
}

int main() {
    std::signal(SIGINT, signalHandler);

    CSICamera camera(640, 480, 15);
    std::unique_ptr<TensorRTYOLO> obj_detector;
    std::unique_ptr<TensorRTInference> lane_trt;
    FPSCalculator fps_calculator(30);
    FrameSkipper obj_skipper(FrameSkipper::FIXED, 20, 15.0);
    FrameSkipper lane_skipper(FrameSkipper::FIXED, 8, 15.0);
    NMPCController mpc;
    PID pid;
    double setpoint_velocity = 0.4;
    std::thread obj_thread;
    std::thread lane_thread;
    zmq::context_t zmq_context(1);
    ZmqPublisher* lane_pub = nullptr; // Para porta 5558 (object)
    ZmqPublisher* ctrl_pub = nullptr; // Para porta 5560 (throttle e steering)
    ZmqPublisher* obj_pub = nullptr; // Para porta 5559 (object)
    ZmqSubscriber* speed_sub = nullptr; // Novo: Para porta 5555 (speed)

    try {
        std::cout << "Initializing integrated system..." << std::endl;
        std::string obj_engine_path = "../engines/best.engine";
        std::ifstream check_obj(obj_engine_path);
        if (!check_obj.good()) throw std::runtime_error("Engine de objetos não encontrado ou inacessível: " + obj_engine_path);
        obj_detector = std::make_unique<TensorRTYOLO>(obj_engine_path, 640);
        std::cout << "Object engine loaded: " << obj_engine_path << std::endl;

        std::string lane_engine_path = "../engines/model.engine";
        std::ifstream check_lane(lane_engine_path);
        if (!check_lane.good()) throw std::runtime_error("Lane engine not found or inaccessible: " + lane_engine_path);
        lane_trt = std::make_unique<TensorRTInference>(lane_engine_path);
        std::cout << "Lane engine loaded: " << lane_engine_path << std::endl;

        // ZMQ publishers
        lane_pub = new ZmqPublisher(zmq_context, "127.0.0.1", 5558, "tcp");
        if (!lane_pub->isConnected()) throw std::runtime_error("Failed to initialize ZMQ on port 5558");

        ctrl_pub = new ZmqPublisher(zmq_context, "127.0.0.1", 5560, "tcp");
        if (!ctrl_pub->isConnected()) throw std::runtime_error("Failed to initialize ZMQ on port 5560");

        obj_pub = new ZmqPublisher(zmq_context, "127.0.0.1", 5559, "tcp");
        if (!obj_pub->isConnected()) throw std::runtime_error("Failed to initialize ZMQ on port 5559");

        speed_sub = new ZmqSubscriber(zmq_context, "127.0.0.1", 5555, current_speed_ms);
        if (!speed_sub->isConnected()) {
            throw std::runtime_error("Failed to initialize ZMQ Subscriber on port 5555");
        }
        speed_sub->start();
        camera.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        obj_thread = std::thread(objectInferenceThread, std::ref(*obj_detector), std::ref(obj_skipper), std::ref(fps_calculator), obj_pub);
        lane_thread = std::thread(laneInferenceThread, std::ref(*lane_trt), std::ref(mpc), std::ref(pid), setpoint_velocity, std::ref(lane_skipper), lane_pub, ctrl_pub);
        std::cout << "Threads launched. Press 'q' to exit." << std::endl;
        int frame_count = 0;
        int processed_frames = 0;
        int skipped_frames = 0;
        while (keep_running.load()) {
            cv::Mat frame = camera.read();
            if (frame.empty()) continue;
            frame_count++;
            {
                std::unique_lock lock_obj(mtx_objects);
                frame_queue_objects.push(frame.clone());
                cv_objects.notify_one();
            }
            {
                std::unique_lock lock_lane(mtx_lanes);
                frame_queue_lanes.push(frame.clone());
                cv_lanes.notify_one();
            }
            {
                std::unique_lock lock(mtx_results);
                cv_results.wait(lock, [&] { return results_ready || !keep_running.load(); });
                if (!keep_running.load()) break;
                results_ready = false;
            }
            double smooth_fps = fps_calculator.getSmoothFPS();
            cv::Mat displayed = combineAndDraw(frame, latest_objects, latest_lanes, smooth_fps, processed_frames, skipped_frames);
            if (!displayed.empty()) cv::imshow("Integrated Detection", displayed);
            cv::imshow("Integrated Detection", displayed);
            char key = cv::waitKey(1) & 0xFF;
            if (key == 'q') keep_running.store(false);
            processed_frames++;
        }
    } catch (const std::runtime_error& e) {
        std::cerr << "Runtime error (initialization failure): " << e.what() << std::endl;
        keep_running.store(false);
    } catch (const std::exception& e) {
        std::cerr << "General error: " << e.what() << std::endl;
        keep_running.store(false);
    }

    cleanup(camera, obj_thread, lane_thread, lane_pub, ctrl_pub, obj_pub, speed_sub);
    return 0;
}
