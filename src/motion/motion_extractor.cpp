#include "image_trans/motion_extractor.hpp"

#include <stdexcept>

namespace image_trans {

MotionExtractor::MotionExtractor(CompositeConfig config)
    : config_(std::move(config)) {}

MotionMaskResult
    MotionExtractor::extract(const ReferenceFrameSet& reference, const cv::Mat& aligned_bgr) const {
    static_cast<void>(config_);
    if (reference.background_bgr.empty() || reference.background_gray.empty()) {
        throw std::runtime_error("reference background is empty");
    }
    if (aligned_bgr.empty()) {
        throw std::runtime_error("aligned frame is empty");
    }

    MotionMaskResult result;
    result.binary_mask = make_empty_mask(aligned_bgr.size());
    result.motion_pixel_count = 0;
    return result;
}

cv::Mat MotionExtractor::make_empty_mask(const cv::Size& size) const {
    return cv::Mat::zeros(size, CV_8UC1);
}

} // namespace image_trans
