#include "fps.hpp"

FPSCalculator::FPSCalculator(size_t window)
  : frame_times_{},
    last_time_(std::chrono::high_resolution_clock::now()),
    window_size_(window),
    frame_count_(0) {}

void FPSCalculator::update() {
  const auto current_time = std::chrono::high_resolution_clock::now();
  const double dt = std::chrono::duration<double>(current_time - last_time_).count();

  if (dt > 0.001 && dt < 1.0) {
    frame_times_.push_back(dt);
    if (frame_times_.size() > window_size_) frame_times_.pop_front();
  }
  last_time_ = current_time;
  frame_count_++;
}

double FPSCalculator::getFPS() const {
  if (frame_times_.size() < 2) return 0.0;
  double sum = 0.0;
  for (double t : frame_times_) sum += t;
  const double avg = sum / frame_times_.size();
  return (avg > 0.0) ? (1.0 / avg) : 0.0;
}

double FPSCalculator::getSmoothFPS() const {
  if (frame_times_.size() < 5) return getFPS();

  std::vector<double> sorted(frame_times_.begin(), frame_times_.end());
  std::sort(sorted.begin(), sorted.end());
  const size_t trim = std::max<size_t>(1, sorted.size() / 5);

  if (sorted.size() <= 2 * trim) return getFPS();

  double sum = 0.0;
  size_t count = 0;
  for (size_t i = trim; i + trim < sorted.size(); ++i) {
    sum += sorted[i];
    ++count;
  }
  const double avg = (count > 0) ? sum / count : 0.0;
  return (avg > 0.0) ? (1.0 / avg) : 0.0;
}
