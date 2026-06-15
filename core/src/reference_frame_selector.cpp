#include "reference_frame_selector.hpp"

namespace hero_lob {

ReferenceFrameSelector::ReferenceFrameSelector(const PipelineConfig& config)
    : config_(config) {}

ReferenceFrameResult ReferenceFrameSelector::Process(
    const FrameData& frame, const TrackingResult& tracking) {
    return {};
}

void ReferenceFrameSelector::StartTrigger(double trigger_start_time_seconds) {}

void ReferenceFrameSelector::Reset() {}

void ReferenceFrameSelector::TrimWindow(double current_timestamp_seconds) {}

}  // namespace hero_lob
