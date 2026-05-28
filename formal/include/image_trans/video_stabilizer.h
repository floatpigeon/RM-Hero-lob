#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

namespace image_trans {

struct StabilizationParams {
    int max_corners = 300;
    double quality_level = 0.01;
    double min_distance = 20.0;
    int lk_window = 21;
    int lk_max_level = 3;
    double ransac_reproj_threshold = 3.0;
    int min_inliers = 40;
    int smoothing_radius = 15;
    bool auto_crop = true;
};

struct StabilizationStats {
    int fallback_frame_count = 0;
    std::vector<std::size_t> fallback_indices;
};

struct MotionPlan {
    std::vector<cv::Mat> raw_transforms;
    std::vector<cv::Mat> correction_transforms;
    cv::Rect crop_roi;
    cv::Size output_size;
    StabilizationStats stats;
};

class VideoStabilizer {
public:
    explicit VideoStabilizer(StabilizationParams params = {});

    MotionPlan analyze(const std::vector<cv::Mat>& frames) const;
    cv::Mat stabilizeFrame(const cv::Mat& frame, const MotionPlan& plan, std::size_t index) const;
    std::vector<cv::Mat> stabilize(const std::vector<cv::Mat>& frames, MotionPlan* out_plan = nullptr) const;

private:
    StabilizationParams params_;
};

}  // namespace image_trans
