#include "frame.hpp"

FrameSkipper::FrameSkipper(Strategy strategy, int skip, double target)
  : skip_strategy_(strategy),
    skip_frames_(skip),
    target_fps_(target),
    frame_counter_(0),
    last_process_time_(std::chrono::high_resolution_clock::now()) {}

bool FrameSkipper::shouldProcessFrame() {
  bool should_process = false;

  if (skip_strategy_ == FIXED) {
    should_process = (frame_counter_ % (skip_frames_ + 1)) == 0;
  } else if (skip_strategy_ == ADAPTIVE) {
    if (processing_times_.size() > 5) {
      double avg_processing_time = 0.0;
      for (double t : processing_times_) avg_processing_time += t;
      avg_processing_time /= processing_times_.size();

      const double target_interval = 1.0 / target_fps_;
      if (avg_processing_time > target_interval) {
        const int skip_ratio = std::max(1, static_cast<int>(avg_processing_time / target_interval));
        should_process = (frame_counter_ % skip_ratio) == 0;
      } else {
        should_process = true;
      }
    } else {
      should_process = true;
    }
  } else if (skip_strategy_ == TIME_BASED) {
    const auto current_time = std::chrono::high_resolution_clock::now();
    const double elapsed = std::chrono::duration<double>(current_time - last_process_time_).count();
    const double target_interval = 1.0 / target_fps_;
    should_process = (elapsed >= target_interval);
    if (should_process) last_process_time_ = current_time;
  }

  frame_counter_++;
  return should_process;
}

void FrameSkipper::recordProcessingTime(double processing_time) {
  if (processing_time > 0.001 && processing_time < 5.0) {
    processing_times_.push_back(processing_time);
    if (processing_times_.size() > 20) processing_times_.pop_front();
  }
}
