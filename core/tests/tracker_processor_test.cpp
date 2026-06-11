#include <iostream>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "tracker_processor.hpp"

namespace {

hero_lob::PipelineConfig MakeConfig() {
    hero_lob::PipelineConfig config;
    config.trajectory_window.window_seconds = 3.0;
    config.trajectory_window.temporal_vote_frames = 3;
    config.trajectory_window.temporal_vote_threshold = 2;
    config.trajectory_window.min_component_area_pixels = 1;
    config.trajectory_window.max_component_area_ratio = 1.0F;
    return config;
}

hero_lob::ForegroundMaskResult MakeForegroundMask(
    std::int64_t frame_index,
    double timestamp_seconds,
    const std::vector<cv::Rect>& bright_regions) {
    hero_lob::ForegroundMaskResult result;
    result.valid = true;
    result.frame_index = frame_index;
    result.timestamp_seconds = timestamp_seconds;
    result.roi = cv::Rect(0, 0, 20, 20);
    result.static_exclusion_mask = cv::Mat::zeros(cv::Size(20, 20), CV_8UC1);
    result.candidate_mask = cv::Mat::zeros(cv::Size(20, 20), CV_8UC1);
    for (const cv::Rect& region : bright_regions) {
        cv::rectangle(result.candidate_mask, region, cv::Scalar(255), cv::FILLED, cv::LINE_8);
    }
    return result;
}

bool TestSingleFrameNoiseIsRejected() {
    hero_lob::TrackerProcessor processor(MakeConfig());

    const hero_lob::TrajectoryResult first = processor.Process(
        MakeForegroundMask(0, 0.0, {cv::Rect(5, 5, 1, 1)}));
    const hero_lob::TrajectoryResult second = processor.Process(
        MakeForegroundMask(1, 0.1, {}));
    const hero_lob::TrajectoryResult third = processor.Process(
        MakeForegroundMask(2, 0.2, {}));

    return first.valid && second.valid && third.valid &&
        cv::countNonZero(first.trajectory_layer) == 0 &&
        cv::countNonZero(second.trajectory_layer) == 0 &&
        cv::countNonZero(third.trajectory_layer) == 0;
}

bool TestContinuousMotionAccumulatesAndExpires() {
    hero_lob::TrackerProcessor processor(MakeConfig());

    processor.Process(MakeForegroundMask(0, 0.0, {cv::Rect(4, 4, 2, 2)}));
    const hero_lob::TrajectoryResult second = processor.Process(
        MakeForegroundMask(1, 0.1, {cv::Rect(4, 4, 2, 2)}));
    const hero_lob::TrajectoryResult third = processor.Process(
        MakeForegroundMask(2, 0.2, {cv::Rect(4, 4, 2, 2)}));
    processor.Process(MakeForegroundMask(3, 3.3, {}));
    processor.Process(MakeForegroundMask(4, 3.4, {}));
    const hero_lob::TrajectoryResult expired = processor.Process(
        MakeForegroundMask(5, 3.5, {}));

    return cv::countNonZero(second.trajectory_layer) > 0 &&
        cv::countNonZero(third.trajectory_layer) > 0 &&
        cv::countNonZero(expired.trajectory_layer) == 0;
}

}  // namespace

int main() {
    int failed = 0;

    if (!TestSingleFrameNoiseIsRejected()) {
        std::cerr << "TestSingleFrameNoiseIsRejected failed\n";
        ++failed;
    }
    if (!TestContinuousMotionAccumulatesAndExpires()) {
        std::cerr << "TestContinuousMotionAccumulatesAndExpires failed\n";
        ++failed;
    }

    if (failed > 0) {
        return 1;
    }

    std::cout << "tracker_processor_test passed\n";
    return 0;
}
