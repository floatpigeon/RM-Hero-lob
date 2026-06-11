#include "reference_frame_selector.hpp"

namespace hero_lob {

ReferenceFrameSelector::ReferenceFrameSelector(const PipelineConfig& config) : config_(config) {}

ReferenceFrameResult ReferenceFrameSelector::Process(
    const FrameData& frame,
    const TrackingResult& tracking) {
    (void)tracking;
    if (!frame.IsValid()) {
        return current_reference_;
    }

    anchor_window_.emplace_back(frame, tracking);
    TrimWindow(frame.timestamp_seconds);

    if (!current_reference_.has_reference) {
        current_reference_.reference_frame = frame;
        current_reference_.has_reference = true;
        current_reference_.mode = ReferenceMode::kStable;
    }

    current_reference_.frozen = frame.timestamp_seconds < current_reference_.trigger_end_time_seconds;
    current_reference_.mode = current_reference_.frozen
        ? ReferenceMode::kFrozen
        : ReferenceMode::kStable;

    return current_reference_;
}

void ReferenceFrameSelector::StartTrigger(double trigger_start_time_seconds) {
    current_reference_.trigger_end_time_seconds =
        trigger_start_time_seconds + config_.trigger_window_seconds;
    current_reference_.frozen = true;
    if (current_reference_.has_reference) {
        current_reference_.mode = ReferenceMode::kFrozen;
    }
}

void ReferenceFrameSelector::Reset() {
    anchor_window_.clear();
    current_reference_ = {};
}

void ReferenceFrameSelector::TrimWindow(double current_timestamp_seconds) {
    const double earliest_allowed = current_timestamp_seconds - config_.stable_window_seconds;
    while (!anchor_window_.empty() &&
           anchor_window_.front().first.timestamp_seconds < earliest_allowed) {
        anchor_window_.pop_front();
    }
}

}  // namespace hero_lob
