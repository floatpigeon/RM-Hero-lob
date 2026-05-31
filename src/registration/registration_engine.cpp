#include "image_trans/registration_engine.hpp"

#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace image_trans {

RegistrationEngine::RegistrationEngine(CompositeConfig config)
    : config_(std::move(config)) {}

RegistrationResult RegistrationEngine::estimate(
    const RegistrationContext& context, const FramePacket& frame) const {
    static_cast<void>(config_);
    if (context.reference_gray.empty() || context.reference_bgr.empty()) {
        throw std::runtime_error("registration context is missing reference images");
    }
    if (frame.bgr.empty()) {
        throw std::runtime_error("frame for registration is empty");
    }

    RegistrationResult result;
    result.accepted = true;
    result.used_translation_fallback = false;
    result.score = 1.0;
    result.reject_reason = RejectReason::kNone;
    result.warp_2x3 = make_identity_warp();
    return result;
}

cv::Mat RegistrationEngine::warp_bgr(
    const cv::Mat& input_bgr, const cv::Mat& warp_2x3, const cv::Size& output_size) const {
    if (input_bgr.empty()) {
        throw std::runtime_error("input image for warp is empty");
    }
    if (warp_2x3.empty()) {
        throw std::runtime_error("warp matrix is empty");
    }

    cv::Mat output;
    cv::warpAffine(
        input_bgr, output, warp_2x3, output_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
        cv::Scalar::all(0));
    return output;
}

cv::Mat RegistrationEngine::make_identity_warp() const {
    return (cv::Mat_<double>(2, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

} // namespace image_trans
