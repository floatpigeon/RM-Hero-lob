#include "image_registrator.hpp"

namespace hero_lob {

ImageRegistrator::ImageRegistrator(const PipelineConfig& config) : config_(config) {}

RegistrationResult ImageRegistrator::Process(
    const ReferenceFrameResult& reference,
    const FrameData& frame,
    const TrackingResult& tracking) const {
    (void)tracking;
    RegistrationResult result;
    if (!reference.has_reference || !frame.IsValid()) {
        return result;
    }

    result.valid = true;
    result.frame_index = frame.frame_index;
    result.timestamp_seconds = frame.timestamp_seconds;
    result.registered_bgr = frame.bgr.clone();
    result.registered_hsv = frame.hsv.clone();
    return result;
}

}  // namespace hero_lob
