#include "capture.hpp"

namespace hero_lob {

Capture::Capture(const PipelineConfig& config) : config_(config) {}

bool Capture::Open(const std::string& input_video_path) {
    return false;
}

bool Capture::ReadNext(FrameData& frame) {
    return false;
}

const std::deque<FrameData>& Capture::RecentFrames() const {
    static const std::deque<FrameData> empty;
    return empty;
}

double Capture::FramesPerSecond() const {
    return frames_per_second_;
}

FrameData Capture::MakeWindowFrameCopy(const FrameData& frame) const {
    return frame;
}

void Capture::ResetState() {}

double Capture::ResolveTimestampSeconds(double raw_timestamp_seconds) const {
    return raw_timestamp_seconds;
}

void Capture::TrimRecentFrames(double current_timestamp_seconds) {}

bool Capture::IsCaptureReady() const {
    return false;
}

}  // namespace hero_lob
