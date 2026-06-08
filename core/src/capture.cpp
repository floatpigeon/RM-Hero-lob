#include "capture.hpp"

#include <opencv2/imgproc.hpp>

namespace hero_lob {

Capture::Capture(const PipelineConfig &config) : config_(config) {}

bool Capture::Open(const std::string &input_video_path) {
  recent_frames_.clear();
  next_frame_index_ = 0;
  capture_.release();

  if (!capture_.open(input_video_path)) {
    return false;
  }

  const double fps = capture_.get(cv::CAP_PROP_FPS);
  frames_per_second_ = fps > 0.0 ? fps : 60.0;
  return true;
}

bool Capture::ReadNext(FrameData &frame) {
  cv::Mat bgr_frame;
  if (!capture_.read(bgr_frame)) {
    return false;
  }

  frame.frame_index = next_frame_index_;
  frame.timestamp_seconds = capture_.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
  if (frame.timestamp_seconds <= 0.0 && frames_per_second_ > 0.0) {
    frame.timestamp_seconds =
        static_cast<double>(next_frame_index_) / frames_per_second_;
  }
  frame.bgr = bgr_frame;
  cv::cvtColor(frame.bgr, frame.hsv, cv::COLOR_BGR2HSV);

  recent_frames_.push_back(frame);
  TrimRecentFrames(frame.timestamp_seconds);
  ++next_frame_index_;
  return true;
}

const std::deque<FrameData> &Capture::RecentFrames() const {
  return recent_frames_;
}

double Capture::FramesPerSecond() const { return frames_per_second_; }

void Capture::TrimRecentFrames(double current_timestamp_seconds) {
  const double earliest_allowed =
      current_timestamp_seconds - config_.stable_window_seconds;
  while (!recent_frames_.empty() &&
         recent_frames_.front().timestamp_seconds < earliest_allowed) {
    recent_frames_.pop_front();
  }
}

} // namespace hero_lob
