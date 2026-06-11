#pragma once

#include <deque>
#include <vector>

#include "types.hpp"

namespace hero_lob {

class TrackerProcessor {
public:
    explicit TrackerProcessor(const PipelineConfig& config);

    TrajectoryResult Process(const ForegroundMaskResult& foreground_mask);
    void Reset();

private:
    struct TemporalMaskEntry {
        cv::Mat mask;
    };

    struct TrajectoryWindowEntry {
        double timestamp_seconds = 0.0;
        std::vector<cv::Point> points;
    };

    PipelineConfig config_;
    cv::Mat hit_count_map_;
    std::deque<TemporalMaskEntry> temporal_masks_;
    std::deque<TrajectoryWindowEntry> trajectory_window_;
};

}  // namespace hero_lob
