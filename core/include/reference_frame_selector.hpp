#pragma once

#include <deque>

#include "types.hpp"

namespace hero_lob {

class ReferenceFrameSelector {
public:
    explicit ReferenceFrameSelector(const PipelineConfig& config)
        : config_(config) {}

    ReferenceFrameResult Process(const FrameData& frame, const TrackingResult& tracking) {
        if (!current_reference_.has_reference) {
            current_reference_.reference_frame = frame;
            current_reference_.has_reference = true;
            current_reference_.mode = ReferenceMode::kStable;
            current_reference_.frozen = true;
        }
        return current_reference_;
    }

    void StartTrigger(double trigger_start_time_seconds) {}

    void Reset() { current_reference_ = {}; }

private:
    void TrimWindow(double current_timestamp_seconds) {}

    PipelineConfig config_;
    std::deque<std::pair<FrameData, TrackingResult>> anchor_window_;
    ReferenceFrameResult current_reference_ = {};
};

}  // namespace hero_lob
