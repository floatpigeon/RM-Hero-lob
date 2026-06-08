#include "tracker.hpp"

namespace hero_lob {

Tracker::Tracker(const PipelineConfig& config) : config_(config) {}

TrackingResult Tracker::Process(const FrameData& frame, const DetectionResult& detection) {
    TrackingResult result;

    if (detection.valid && detection.anchors.valid) {
        result.anchors = detection.anchors;
        result.state = detection.anchors.placeholder
            ? TrackingState::kPlaceholder
            : TrackingState::kTracking;
        result.lost = false;
        result.valid = true;
        last_result_ = result;
        last_seen_timestamp_seconds_ = frame.timestamp_seconds;
        return result;
    }

    const double time_since_last_seen = last_seen_timestamp_seconds_ >= 0.0
        ? frame.timestamp_seconds - last_seen_timestamp_seconds_
        : config_.lost_timeout_seconds + 1.0;

    if (last_result_.valid && time_since_last_seen <= config_.lost_timeout_seconds) {
        result = last_result_;
        result.lost = false;
        return result;
    }

    if (last_result_.valid) {
        result = last_result_;
        result.state = TrackingState::kLost;
        result.lost = true;
        result.valid = false;
        return result;
    }

    result.state = TrackingState::kLost;
    result.lost = true;
    result.valid = false;
    return result;
}

}  // namespace hero_lob
