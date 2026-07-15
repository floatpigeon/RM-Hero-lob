#pragma once

#include <string>

#include "background_remover.hpp"
#include "capture.hpp"
#include "compression.hpp"
#include "image_registrator.hpp"
#include "image_registrator_orb.hpp"
#include "image_synthesis.hpp"
#include "reference_frame_selector.hpp"
#include "tracker_processor.hpp"
#include "tracker_processor_fast.hpp"
#include "types.hpp"

namespace hero_lob {

class Pipeline {
public:
    explicit Pipeline(const PipelineConfig& config = {});

    bool Run(const std::string& input_video, const std::string& output_image);

private:
    PipelineConfig config_;
    Capture capture_;
    ReferenceFrameSelector reference_frame_selector_;
    ImageRegistratorOrb image_registrator_;
    BackgroundRemover background_remover_;
    TrackerProcessorFast tracker_processor_fast_;
    ImageSynthesis image_synthesis_;
    Compression compression_;
};

}  // namespace hero_lob
