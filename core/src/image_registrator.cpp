#include "image_registrator.hpp"

namespace hero_lob {

ImageRegistrator::ImageRegistrator(const PipelineConfig& config) : config_(config) {}

RegistrationResult ImageRegistrator::Process(
    const ReferenceFrameResult& reference,
    const FrameData& frame,
    const TrackingResult& tracking) const {
    RegistrationResult result;
    if (!reference.has_reference || !frame.IsValid() || !tracking.anchors.valid) {
        return result;
    }

    result.valid = true;
    result.registered_bgr = frame.bgr.clone();
    result.registered_hsv = frame.hsv.clone();
    return result;
}

}  // namespace hero_lob
