#ifndef IMAGE_TRANS_CONFIG_TYPES_HPP_
#define IMAGE_TRANS_CONFIG_TYPES_HPP_

#include <opencv2/core.hpp>

namespace image_trans {

struct CropConfig {
    bool enabled = false;
    cv::Rect crop_rect;
};

struct RegistrationConfig {
    double downsample_ratio = 0.5;
    int auto_tile_count = 6;
    int ecc_iterations = 50;
    double registration_score_threshold = 0.90;
    double translation_fallback_threshold = 0.60;
    int saturation_threshold = 245;
    int min_tile_gradient = 20;
};

struct MotionConfig {
    double sigma_multiplier = 3.0;
    int floor_threshold = 18;
    int min_visible_value = 32;
    int min_blob_area = 6;
};

struct DebugConfig {
    bool enabled = false;
    bool export_key_masks = true;
    int max_key_masks = 12;
};

struct CompositeConfig {
    CropConfig crop;
    RegistrationConfig registration;
    MotionConfig motion;
    DebugConfig debug;
    int pretrigger_frame_count = 10;
    int capture_duration_ms = 3000;
};

}  // namespace image_trans

#endif  // IMAGE_TRANS_CONFIG_TYPES_HPP_
