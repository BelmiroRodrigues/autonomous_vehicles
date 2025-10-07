#ifndef FPS_HPP
#define FPS_HPP

#include <chrono>
#include <deque>
#include <vector>
#include <algorithm>

class FPSCalculator {
public:
  explicit FPSCalculator(size_t window = 30);

  void update();
  double getFPS() const;
  double getSmoothFPS() const;

private:
  std::deque<double> frame_times_;
  std::chrono::high_resolution_clock::time_point last_time_;
  size_t window_size_;
  int frame_count_;
};

#endif // FPS_HPP
