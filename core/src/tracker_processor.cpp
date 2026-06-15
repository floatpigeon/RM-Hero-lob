#include "tracker_processor.hpp"

namespace hero_lob {

TrackerProcessor::TrackerProcessor(const PipelineConfig& config) : config_(config) {}

TrajectoryResult TrackerProcessor::Process(const ForegroundMaskResult& foreground_mask) {
    return {};
}

void TrackerProcessor::Reset() {}

}  // namespace hero_lob
