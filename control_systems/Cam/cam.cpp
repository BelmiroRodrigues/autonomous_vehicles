#include "cam.hpp"

CSICamera::CSICamera(int width, int height, int fps) {
    std::ostringstream pipeline;
    pipeline << "nvarguscamerasrc sensor-mode=4 ! "
             << "video/x-raw(memory:NVMM), width=" << width << ", height=" << height
             << ", format=NV12, framerate=" << fps << "/1 ! "
             << "nvvidconv flip-method=0 ! video/x-raw, width=" << width
             << ", height=" << height << ", format=BGRx ! "
             << "videoconvert ! video/x-raw, format=BGR ! appsink";
    cap.open(pipeline.str(), cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        throw std::runtime_error("CSICamera: failed to open GStreamer pipeline");
    }
}

void CSICamera::start() {
    running = true;
    thread = std::thread(&CSICamera::update, this);
}

void CSICamera::stop() {
    running = false;
    if (thread.joinable()) thread.join();
    cap.release();
}

cv::Mat CSICamera::read() const {
    return frame.clone();
}

void CSICamera::update() {
    while (running) {
        cv::Mat f;
        cap.read(f);
        if (f.empty()) {
            std::cerr << "Failed to read frame from camera! Continuing..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Evita loop apertado
            continue;
        }
        cv::resize(f, f, cv::Size(640, 360));
        frame = f;
    }
}

