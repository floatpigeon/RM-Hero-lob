#include "capture.hpp"

#include <cmath>

#include <opencv2/imgproc.hpp>

namespace hero_lob {

Capture::Capture(const PipelineConfig& config) : config_(config) {}

bool Capture::Open(const std::string& input_video_path) {
    ResetState();

    if (!capture_.open(input_video_path) || !capture_.isOpened()) {
        capture_.release();
        return false;
    }

    const double fps = capture_.get(cv::CAP_PROP_FPS);
    frames_per_second_ = std::isfinite(fps) && fps > 0.0 ? fps : 60.0;
    is_open_ = true;
    return true;
}

bool Capture::ReadNext(FrameData& frame) {
    frame = FrameData{};
    if (!IsCaptureReady()) {
        return false;
    }

    cv::Mat bgr_frame;
    if (!capture_.read(bgr_frame) || bgr_frame.empty()) {
        return false;
    }

    frame.frame_index = next_frame_index_;
    frame.timestamp_seconds =
        ResolveTimestampSeconds(capture_.get(cv::CAP_PROP_POS_MSEC) / 1000.0);
    frame.bgr = bgr_frame;
    cv::cvtColor(frame.bgr, frame.hsv, cv::COLOR_BGR2HSV);

    recent_frames_.push_back(MakeWindowFrameCopy(frame));
    TrimRecentFrames(frame.timestamp_seconds);
    ++next_frame_index_;
    return true;
}

const std::deque<FrameData>& Capture::RecentFrames() const { return recent_frames_; }

double Capture::FramesPerSecond() const { return frames_per_second_; }

FrameData Capture::MakeWindowFrameCopy(const FrameData& frame) const {
    FrameData window_frame = frame;
    window_frame.bgr = frame.bgr.clone();
    window_frame.hsv = frame.hsv.clone();
    return window_frame;
}

void Capture::ResetState() {
    recent_frames_.clear();
    next_frame_index_ = 0;
    frames_per_second_ = 60.0;
    is_open_ = false;
    capture_.release();
}

double Capture::ResolveTimestampSeconds(double raw_timestamp_seconds) const {
    const bool has_non_negative_timestamp =
        std::isfinite(raw_timestamp_seconds) && raw_timestamp_seconds >= 0.0;
    const bool is_first_frame_zero =
        next_frame_index_ == 0 && raw_timestamp_seconds == 0.0;
    const bool is_positive_non_first_timestamp =
        next_frame_index_ > 0 && raw_timestamp_seconds > 0.0;
    const bool is_monotonic =
        recent_frames_.empty() || raw_timestamp_seconds >= recent_frames_.back().timestamp_seconds;

    if (has_non_negative_timestamp &&
        (is_first_frame_zero || is_positive_non_first_timestamp) &&
        is_monotonic) {
        return raw_timestamp_seconds;
    }

    if (frames_per_second_ > 0.0) {
        return static_cast<double>(next_frame_index_) / frames_per_second_;
    }

    return 0.0;
}

void Capture::TrimRecentFrames(double current_timestamp_seconds) {
    const double earliest_allowed = current_timestamp_seconds - config_.stable_window_seconds;
    while (!recent_frames_.empty() && recent_frames_.front().timestamp_seconds < earliest_allowed) {
        recent_frames_.pop_front();
    }
}

bool Capture::IsCaptureReady() const { return is_open_ && capture_.isOpened(); }

}  // namespace hero_lob
