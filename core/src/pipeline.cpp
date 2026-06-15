#include "pipeline.hpp"

namespace hero_lob {

Pipeline::Pipeline(const PipelineConfig& config)
    : config_(config),
      capture_(config),
      identifier_(config),
      tracker_(config),
      reference_frame_selector_(config),
      image_registrator_(config),
      background_remover_(config),
      tracker_processor_(config),
      image_synthesis_(config) {}

bool Pipeline::Run(const std::string& input_video, const std::string& output_image) {
    return false;
}

}  // namespace hero_lob
