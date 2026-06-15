#pragma once

#include <deque>
#include <vector>

#include "types.hpp"

namespace hero_lob {

class TrackerProcessor {
public:
    struct ComponentInfo {
        cv::Mat mask;
        cv::Point2f centroid = {};
    };

    explicit TrackerProcessor(const PipelineConfig& config);

    TrajectoryResult Process(const ForegroundMaskResult& foreground_mask);
    void Reset();

private:
    struct TemporalMaskEntry {
        cv::Mat mask;
    };

    struct TrajectoryWindowEntry {
        double timestamp_seconds = 0.0;
        cv::Mat color_frame;
    };

    PipelineConfig config_;
    std::deque<TemporalMaskEntry> temporal_masks_;
    std::deque<TrajectoryWindowEntry> trajectory_window_;
    std::vector<ComponentInfo> previous_components_;
};

}  // namespace hero_lob
