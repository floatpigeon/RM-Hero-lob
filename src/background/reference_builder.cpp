#include "image_trans/reference_builder.hpp"

#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace image_trans {

ReferenceBuilder::ReferenceBuilder(CompositeConfig config)
    : config_(std::move(config)) {}

ReferenceFrameSet ReferenceBuilder::build(const WindowCapture& window) const {
    if (window.pretrigger_frames.empty()) {
        throw std::runtime_error("pretrigger frame buffer is empty");
    }

    const int anchor_index = select_anchor_index(window.pretrigger_frames);
    const FramePacket& anchor_frame =
        window.pretrigger_frames.at(static_cast<std::size_t>(anchor_index));

    ReferenceFrameSet reference;
    reference.background_bgr = anchor_frame.bgr.clone();
    reference.background_gray = to_gray(anchor_frame.bgr);
    reference.anchor_frame_index = anchor_frame.frame_index;
    return reference;
}

int ReferenceBuilder::select_anchor_index(std::span<const FramePacket> frames) const {
    static_cast<void>(config_);
    return static_cast<int>(frames.size() / 2);
}

cv::Mat ReferenceBuilder::to_gray(const cv::Mat& bgr) const {
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

} // namespace image_trans
