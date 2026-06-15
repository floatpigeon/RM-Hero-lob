#pragma once

#include <vector>

#include "types.hpp"

namespace hero_lob {

class TrackerProcessor {
public:
    struct ComponentInfo {
        cv::Mat mask;
        cv::Point2f centroid = {};
        cv::Point2f smoothed_velocity = {};
        bool velocity_initialized = false;
    };

    explicit TrackerProcessor(const PipelineConfig& config);

    TrajectoryResult Process(const ForegroundMaskResult& foreground_mask);
    void Reset();

private:
    PipelineConfig config_;
    std::vector<ComponentInfo> previous_components_;
    cv::Mat trajectory_layer_;
    int frame_count_ = 0;
    double previous_timestamp_ = 0.0;
};

}  // namespace hero_lob
