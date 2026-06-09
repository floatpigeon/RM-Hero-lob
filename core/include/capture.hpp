#pragma once

#include <deque>
#include <string>

#include <opencv2/videoio.hpp>

#include "types.hpp"

namespace hero_lob {

class Capture {
public:
    explicit Capture(const PipelineConfig& config);

    bool Open(const std::string& input_video_path);
    bool ReadNext(FrameData& frame);
    const std::deque<FrameData>& RecentFrames() const;
    double FramesPerSecond() const;

private:
    FrameData MakeWindowFrameCopy(const FrameData& frame) const;
    void ResetState();
    double ResolveTimestampSeconds(double raw_timestamp_seconds) const;
    void TrimRecentFrames(double current_timestamp_seconds);
    bool IsCaptureReady() const;

    PipelineConfig config_;
    cv::VideoCapture capture_;
    std::deque<FrameData> recent_frames_;
    std::int64_t next_frame_index_ = 0;
    double frames_per_second_ = 60.0;
    bool is_open_ = false;
};

}  // namespace hero_lob
