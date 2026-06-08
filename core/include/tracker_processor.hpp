#pragma once

#include "types.hpp"

namespace hero_lob {

class TrackerProcessor {
public:
    explicit TrackerProcessor(const PipelineConfig& config);

    TrajectoryResult Process(const ForegroundMaskResult& foreground_mask);
    void Reset();

private:
    PipelineConfig config_;
    cv::Mat accumulated_mask_;
    int accumulated_frames_ = 0;
};

}  // namespace hero_lob
