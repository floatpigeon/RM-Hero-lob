#ifndef IMAGE_TRANS_REGISTRATION_ENGINE_HPP_
#define IMAGE_TRANS_REGISTRATION_ENGINE_HPP_

#include "image_trans/common_types.hpp"
#include "image_trans/config_types.hpp"

namespace image_trans {

struct RegistrationContext {
    cv::Mat reference_gray;
    cv::Mat reference_bgr;
};

class RegistrationEngine {
public:
    explicit RegistrationEngine(CompositeConfig config);

    RegistrationResult estimate(const RegistrationContext &context, const FramePacket &frame) const;
    cv::Mat warp_bgr(const cv::Mat &input_bgr, const cv::Mat &warp_2x3, const cv::Size &output_size) const;

private:
    CompositeConfig config_;

    cv::Mat make_identity_warp() const;
};

}  // namespace image_trans

#endif  // IMAGE_TRANS_REGISTRATION_ENGINE_HPP_
