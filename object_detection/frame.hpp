#ifndef FRAME_HPP
#define FRAME_HPP

#include <chrono>
#include <deque>
#include <algorithm>

class FrameSkipper {
public:
  enum Strategy { FIXED, ADAPTIVE, TIME_BASED };

  explicit FrameSkipper(Strategy strategy = FIXED, int skip = 2, double target = 15.0);

  bool shouldProcessFrame();
  void recordProcessingTime(double processing_time);

  void setStrategy(Strategy strategy) { skip_strategy_ = strategy; }
  Strategy getStrategy() const { return skip_strategy_; }

private:
  Strategy skip_strategy_;
  int skip_frames_;
  double target_fps_;
  int frame_counter_;
  std::chrono::high_resolution_clock::time_point last_process_time_;
  std::deque<double> processing_times_;
};

#endif // FRAME_HPP
