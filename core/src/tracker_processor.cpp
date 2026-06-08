#include "tracker_processor.hpp"

#include <opencv2/core.hpp>

namespace hero_lob {

TrackerProcessor::TrackerProcessor(const PipelineConfig& config) : config_(config) {}

TrajectoryResult TrackerProcessor::Process(const ForegroundMaskResult& foreground_mask) {
    TrajectoryResult result;
    if (!foreground_mask.valid || foreground_mask.candidate_mask.empty()) {
        return result;
    }

    if (accumulated_mask_.empty() ||
        accumulated_mask_.size() != foreground_mask.candidate_mask.size()) {
        accumulated_mask_ = cv::Mat::zeros(
            foreground_mask.candidate_mask.size(),
            foreground_mask.candidate_mask.type());
        accumulated_frames_ = 0;
    }

    cv::bitwise_or(accumulated_mask_, foreground_mask.candidate_mask, accumulated_mask_);
    ++accumulated_frames_;

    result.valid = true;
    result.trajectory_layer = accumulated_mask_.clone();
    result.accumulated_frames = accumulated_frames_;
    return result;
}

void TrackerProcessor::Reset() {
    accumulated_mask_.release();
    accumulated_frames_ = 0;
}

}  // namespace hero_lob
