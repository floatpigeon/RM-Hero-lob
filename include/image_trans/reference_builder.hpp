#ifndef IMAGE_TRANS_REFERENCE_BUILDER_HPP_
#define IMAGE_TRANS_REFERENCE_BUILDER_HPP_

#include <span>

#include "image_trans/common_types.hpp"
#include "image_trans/config_types.hpp"

namespace image_trans {

class ReferenceBuilder {
public:
    explicit ReferenceBuilder(CompositeConfig config);

    ReferenceFrameSet build(const WindowCapture &window) const;

private:
    CompositeConfig config_;

    int select_anchor_index(std::span<const FramePacket> frames) const;
    cv::Mat to_gray(const cv::Mat &bgr) const;
};

}  // namespace image_trans

#endif  // IMAGE_TRANS_REFERENCE_BUILDER_HPP_
