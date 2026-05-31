#ifndef IMAGE_TRANS_TRAIL_COMPOSITOR_HPP_
#define IMAGE_TRANS_TRAIL_COMPOSITOR_HPP_

#include "image_trans/common_types.hpp"

namespace image_trans {

class TrailCompositor {
public:
    explicit TrailCompositor(cv::Size canvas_size);

    void reset();
    void accumulate(const cv::Mat &aligned_bgr, const cv::Mat &motion_mask);
    cv::Mat trail_layer() const;
    cv::Mat compose_with_background(const cv::Mat &reference_bgr) const;

private:
    cv::Size canvas_size_;
    cv::Mat trail_layer_bgr_;

    void validate_inputs(const cv::Mat &aligned_bgr, const cv::Mat &motion_mask) const;
};

}  // namespace image_trans

#endif  // IMAGE_TRANS_TRAIL_COMPOSITOR_HPP_
