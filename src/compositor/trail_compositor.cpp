#include "image_trans/trail_compositor.hpp"

#include <stdexcept>

namespace image_trans {

TrailCompositor::TrailCompositor(cv::Size canvas_size)
    : canvas_size_(canvas_size) {
    reset();
}

void TrailCompositor::reset() { trail_layer_bgr_ = cv::Mat::zeros(canvas_size_, CV_8UC3); }

void TrailCompositor::accumulate(const cv::Mat& aligned_bgr, const cv::Mat& motion_mask) {
    validate_inputs(aligned_bgr, motion_mask);

    for (int row = 0; row < aligned_bgr.rows; ++row) {
        const cv::Vec3b* input_ptr = aligned_bgr.ptr<cv::Vec3b>(row);
        const std::uint8_t* mask_ptr = motion_mask.ptr<std::uint8_t>(row);
        cv::Vec3b* trail_ptr = trail_layer_bgr_.ptr<cv::Vec3b>(row);

        for (int col = 0; col < aligned_bgr.cols; ++col) {
            if (mask_ptr[col] == 0) {
                continue;
            }

            trail_ptr[col][0] = std::max(trail_ptr[col][0], input_ptr[col][0]);
            trail_ptr[col][1] = std::max(trail_ptr[col][1], input_ptr[col][1]);
            trail_ptr[col][2] = std::max(trail_ptr[col][2], input_ptr[col][2]);
        }
    }
}

cv::Mat TrailCompositor::trail_layer() const { return trail_layer_bgr_.clone(); }

cv::Mat TrailCompositor::compose_with_background(const cv::Mat& reference_bgr) const {
    if (reference_bgr.empty()) {
        throw std::runtime_error("reference background is empty");
    }
    if (reference_bgr.size() != trail_layer_bgr_.size()) {
        throw std::runtime_error("reference background size does not match compositor canvas");
    }

    cv::Mat composed = reference_bgr.clone();
    for (int row = 0; row < composed.rows; ++row) {
        cv::Vec3b* output_ptr = composed.ptr<cv::Vec3b>(row);
        const cv::Vec3b* trail_ptr = trail_layer_bgr_.ptr<cv::Vec3b>(row);
        for (int col = 0; col < composed.cols; ++col) {
            output_ptr[col][0] = std::max(output_ptr[col][0], trail_ptr[col][0]);
            output_ptr[col][1] = std::max(output_ptr[col][1], trail_ptr[col][1]);
            output_ptr[col][2] = std::max(output_ptr[col][2], trail_ptr[col][2]);
        }
    }
    return composed;
}

void TrailCompositor::validate_inputs(
    const cv::Mat& aligned_bgr, const cv::Mat& motion_mask) const {
    if (aligned_bgr.empty()) {
        throw std::runtime_error("aligned frame is empty");
    }
    if (motion_mask.empty()) {
        throw std::runtime_error("motion mask is empty");
    }
    if (aligned_bgr.size() != canvas_size_) {
        throw std::runtime_error("aligned frame size does not match compositor canvas");
    }
    if (motion_mask.size() != canvas_size_) {
        throw std::runtime_error("motion mask size does not match compositor canvas");
    }
    if (aligned_bgr.type() != CV_8UC3) {
        throw std::runtime_error("aligned frame must be CV_8UC3");
    }
    if (motion_mask.type() != CV_8UC1) {
        throw std::runtime_error("motion mask must be CV_8UC1");
    }
}

} // namespace image_trans
