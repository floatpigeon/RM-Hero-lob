#pragma once

#include <cstdint>

#include <opencv2/core.hpp>

namespace hero_lob {

enum class TargetColor {
    kUnknown,
    kRed,
    kBlue,
};

enum class TrackingState {
    kUninitialized,
    kTracking,
    kLost,
    kPlaceholder,
};

enum class ReferenceMode {
    kUninitialized,
    kStable,
    kDynamic,
    kFrozen,
};

struct PipelineConfig {
    double stable_window_seconds = 0.5;
    double lost_timeout_seconds = 0.2;
    double trigger_window_seconds = 3.0;
    float placeholder_light_length_ratio = 0.16F;
    float placeholder_light_gap_ratio = 0.10F;
};

struct FrameData {
    std::int64_t frame_index = -1;
    double timestamp_seconds = 0.0;
    cv::Mat bgr;
    cv::Mat hsv;

    bool IsValid() const {
        return !bgr.empty();
    }
};

struct AnchorSet {
    cv::Point2f guide_center = {};
    cv::Point2f left_top = {};
    cv::Point2f left_bottom = {};
    cv::Point2f right_top = {};
    cv::Point2f right_bottom = {};
    cv::Point2f direction = {1.0F, 0.0F};
    bool valid = false;
    bool placeholder = false;
};

struct DetectionResult {
    AnchorSet anchors = {};
    TargetColor color = TargetColor::kUnknown;
    bool valid = false;
};

struct TrackingResult {
    AnchorSet anchors = {};
    TrackingState state = TrackingState::kUninitialized;
    bool lost = true;
    bool valid = false;
};

struct ReferenceFrameResult {
    ReferenceMode mode = ReferenceMode::kUninitialized;
    FrameData reference_frame = {};
    bool has_reference = false;
    bool frozen = false;
    double trigger_end_time_seconds = 0.0;
};

struct RegistrationResult {
    bool valid = false;
    cv::Matx23f transform = cv::Matx23f(1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F);
    cv::Mat registered_bgr;
    cv::Mat registered_hsv;
};

struct ForegroundMaskResult {
    bool valid = false;
    cv::Rect roi = {};
    cv::Mat static_exclusion_mask;
    cv::Mat candidate_mask;
};

struct TrajectoryResult {
    bool valid = false;
    cv::Mat trajectory_layer;
    int accumulated_frames = 0;
};

struct SynthesisResult {
    bool valid = false;
    cv::Mat output_image;
};

}  // namespace hero_lob
