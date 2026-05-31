#ifndef IMAGE_TRANS_COMMON_TYPES_HPP_
#define IMAGE_TRANS_COMMON_TYPES_HPP_

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace image_trans {

enum class TriggerMode {
    kFrameIndex,
    kTimestampMs,
};

enum class RejectReason {
    kNone,
    kRegistrationFailed,
    kLowRegistrationScore,
    kInvalidMotionMask,
};

struct TriggerSpec {
    TriggerMode mode = TriggerMode::kFrameIndex;
    std::int64_t value = 0;
};

struct FramePacket {
    std::int64_t frame_index = 0;
    std::int64_t timestamp_ms = 0;
    cv::Mat bgr;
};

struct WindowCapture {
    std::vector<FramePacket> pretrigger_frames;
    std::vector<FramePacket> posttrigger_frames;
    double source_fps = 0.0;
    cv::Size source_frame_size;
};

struct ReferenceFrameSet {
    cv::Mat background_bgr;
    cv::Mat background_gray;
    std::int64_t anchor_frame_index = -1;
};

struct RegistrationResult {
    bool accepted = false;
    bool used_translation_fallback = false;
    double score = 0.0;
    RejectReason reject_reason = RejectReason::kNone;
    cv::Mat warp_2x3;
};

struct MotionMaskResult {
    cv::Mat binary_mask;
    int motion_pixel_count = 0;
};

struct FrameDebug {
    std::int64_t frame_index = 0;
    std::int64_t timestamp_ms = 0;
    bool accepted = false;
    bool used_translation_fallback = false;
    double registration_score = 0.0;
    int motion_pixel_count = 0;
    RejectReason reject_reason = RejectReason::kNone;
};

struct CompositeResult {
    cv::Mat final_bgr;
    cv::Mat reference_background_bgr;
    cv::Mat trail_layer_bgr;
    int accepted_frame_count = 0;
    int dropped_frame_count = 0;
    std::vector<FrameDebug> frame_debug_list;
};

}  // namespace image_trans

#endif  // IMAGE_TRANS_COMMON_TYPES_HPP_
