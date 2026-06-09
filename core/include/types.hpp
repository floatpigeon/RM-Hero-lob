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

struct HsvRangeConfig {
    int hue_min = 0;
    int hue_max = 0;
};

struct ColorThresholdConfig {
    HsvRangeConfig green = {45, 90};
    HsvRangeConfig red_low = {0, 12};
    HsvRangeConfig red_high = {166, 179};
    HsvRangeConfig blue = {95, 130};
    int min_saturation = 10;
    int min_value = 100;
};

struct MorphologyConfig {
    int blur_kernel_size = 5;
    int open_kernel_size = 3;
    int close_kernel_size = 5;
};

struct GuideConstraintConfig {
    float min_area_ratio = 0.00005F;
    float max_area_ratio = 0.02F;
    float max_aspect_ratio_deviation = 0.45F;
    float min_circularity = 0.65F;
    float min_color_advantage = 20.0F;
};

struct LightConstraintConfig {
    float min_length_ratio = 0.02F;
    float max_length_ratio = 0.30F;
    float min_width_ratio = 0.003F;
    float max_width_ratio = 0.08F;
    float min_aspect_ratio = 2.0F;
    float max_aspect_ratio = 18.0F;
    float min_fill_ratio = 0.45F;
    float min_color_advantage = 20.0F;
};

struct PairConstraintConfig {
    float max_angle_difference_degrees = 12.0F;
    float max_length_delta_ratio = 0.35F;
    float max_center_y_delta_ratio = 0.45F;
    float min_center_distance_ratio = 0.40F;
    float max_center_distance_ratio = 4.50F;
    float max_overlap_ratio = 0.15F;
};

struct TripletConstraintConfig {
    float max_guide_midpoint_x_offset_ratio = 0.90F;
    float min_guide_midpoint_y_offset_ratio = 0.15F;
    float max_guide_midpoint_y_offset_ratio = 2.20F;
    float min_guide_radius_to_light_length_ratio = 0.15F;
    float max_guide_radius_to_light_length_ratio = 1.10F;
};

struct IdentifierConfig {
    ColorThresholdConfig color = {};
    MorphologyConfig morphology = {};
    GuideConstraintConfig guide = {};
    LightConstraintConfig light = {};
    PairConstraintConfig pair = {};
    TripletConstraintConfig triplet = {};
};

struct PipelineConfig {
    double stable_window_seconds = 0.5;
    double lost_timeout_seconds = 0.2;
    double trigger_window_seconds = 3.0;
    IdentifierConfig identifier = {};
};

struct FrameData {
    std::int64_t frame_index = -1;
    double timestamp_seconds = 0.0;
    cv::Mat bgr;
    cv::Mat hsv;

    bool IsValid() const { return !bgr.empty(); }
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
