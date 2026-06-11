#pragma once

#include "types.hpp"

namespace hero_lob {

class Tracker {
public:
    explicit Tracker(const PipelineConfig& config);

    TrackingResult Process(const FrameData& frame, const DetectionResult& detection);
    void Reset();

private:
    PipelineConfig config_;
    TrackingResult last_result_ = {};
    double last_seen_timestamp_seconds_ = -1.0;
};

}  // namespace hero_lob
