#ifndef IMAGE_TRANS_MOTION_EXTRACTOR_HPP_
#define IMAGE_TRANS_MOTION_EXTRACTOR_HPP_

#include "image_trans/common_types.hpp"
#include "image_trans/config_types.hpp"

namespace image_trans {

class MotionExtractor {
public:
    explicit MotionExtractor(CompositeConfig config);

    MotionMaskResult extract(const ReferenceFrameSet &reference, const cv::Mat &aligned_bgr) const;

private:
    CompositeConfig config_;

    cv::Mat make_empty_mask(const cv::Size &size) const;
};

}  // namespace image_trans

#endif  // IMAGE_TRANS_MOTION_EXTRACTOR_HPP_
