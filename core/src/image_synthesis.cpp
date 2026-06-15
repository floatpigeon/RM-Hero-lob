#include "image_synthesis.hpp"

namespace hero_lob {

ImageSynthesis::ImageSynthesis(const PipelineConfig& config) : config_(config) {}

SynthesisResult ImageSynthesis::Process(
    const ReferenceFrameResult& reference,
    const TrajectoryResult& trajectory) const {
    return {};
}

}  // namespace hero_lob
