#pragma once

#include <deque>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "types.hpp"

namespace hero_lob {

class Capture {
public:
    explicit Capture(const PipelineConfig& config)
        : config_(config) {}

    bool Open(const std::string& input_video_path) {
        ResetState();
        if (!capture_.open(input_video_path)) {
            return false;
        }
        frames_per_second_ = capture_.get(cv::CAP_PROP_FPS);
        if (frames_per_second_ <= 0.0) {
            frames_per_second_ = 60.0;
        }
        is_open_ = true;
        return true;
    }

    bool ReadNext(FrameData& frame) {
        if (!IsCaptureReady()) {
            return false;
        }
        cv::Mat bgr;
        if (!capture_.read(bgr) || bgr.empty()) {
            return false;
        }
        frame.frame_index = next_frame_index_++;
        frame.timestamp_seconds = ResolveTimestampSeconds(capture_.get(cv::CAP_PROP_POS_MSEC));
        frame.bgr = bgr;
        cv::cvtColor(bgr, frame.hsv, cv::COLOR_BGR2HSV);
        TrimRecentFrames(frame.timestamp_seconds);
        recent_frames_.push_back(MakeWindowFrameCopy(frame));
        return true;
    }

    const std::deque<FrameData>& RecentFrames() const { return recent_frames_; }

    double FramesPerSecond() const { return frames_per_second_; }

private:
    FrameData MakeWindowFrameCopy(const FrameData& frame) const {
        FrameData copy;
        copy.frame_index = frame.frame_index;
        copy.timestamp_seconds = frame.timestamp_seconds;
        copy.bgr = frame.bgr.clone();
        copy.hsv = frame.hsv.clone();
        return copy;
    }

    void ResetState() {
        if (capture_.isOpened()) {
            capture_.release();
        }
        recent_frames_.clear();
        next_frame_index_ = 0;
        frames_per_second_ = 60.0;
        is_open_ = false;
    }

    double ResolveTimestampSeconds(double raw_timestamp_seconds) const {
        return raw_timestamp_seconds / 1000.0;
    }

    void TrimRecentFrames(double current_timestamp_seconds) {
        double window = config_.stable_window_seconds;
        while (!recent_frames_.empty()
               && (current_timestamp_seconds - recent_frames_.front().timestamp_seconds) > window) {
            recent_frames_.pop_front();
        }
    }

    bool IsCaptureReady() const { return is_open_ && capture_.isOpened(); }

    PipelineConfig config_;
    cv::VideoCapture capture_;
    std::deque<FrameData> recent_frames_;
    std::int64_t next_frame_index_ = 0;
    double frames_per_second_ = 60.0;
    bool is_open_ = false;
};

}  // namespace hero_lob
