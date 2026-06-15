#include "tracker.hpp"

namespace hero_lob {

Tracker::Tracker(const PipelineConfig& config) : config_(config) {}

TrackingResult Tracker::Process(const FrameData& frame, const DetectionResult& detection) {
    return {};
}

void Tracker::Reset() {}

}  // namespace hero_lob
