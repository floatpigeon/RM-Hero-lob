#include "background_remover.hpp"

namespace hero_lob {

BackgroundRemover::BackgroundRemover(const PipelineConfig& config) : config_(config) {}

ForegroundMaskResult BackgroundRemover::Process(
    const ReferenceFrameResult& reference,
    const RegistrationResult& registration) {
    return {};
}

void BackgroundRemover::Reset() {}

void BackgroundRemover::ResetStateForFrameSize(const cv::Size& frame_size) {}

}  // namespace hero_lob
