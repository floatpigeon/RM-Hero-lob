#include "reference_frame_selector.hpp"

namespace hero_lob {

ReferenceFrameSelector::ReferenceFrameSelector(const PipelineConfig& config)
    : config_(config) {}

ReferenceFrameResult ReferenceFrameSelector::Process(
    const FrameData& frame, const TrackingResult& tracking) {
    if (!current_reference_.has_reference) {
        current_reference_.reference_frame = frame;
        current_reference_.has_reference = true;
        current_reference_.mode = ReferenceMode::kStable;
        current_reference_.frozen = true;
    }
    return current_reference_;
}

void ReferenceFrameSelector::StartTrigger(double trigger_start_time_seconds) {}

void ReferenceFrameSelector::Reset() {
    current_reference_ = {};
}

void ReferenceFrameSelector::TrimWindow(double current_timestamp_seconds) {}

}  // namespace hero_lob
