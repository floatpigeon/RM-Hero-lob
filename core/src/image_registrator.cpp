#include "image_registrator.hpp"

namespace hero_lob {

ImageRegistrator::ImageRegistrator(const PipelineConfig& config) : config_(config) {}

RegistrationResult ImageRegistrator::Process(
    const ReferenceFrameResult& reference, const FrameData& frame,
    const TrackingResult& tracking) const {
    return {};
}

}  // namespace hero_lob
