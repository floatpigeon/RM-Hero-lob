#pragma once

#include <string>

#include "background_remover.hpp"
#include "capture.hpp"
#include "identifier.hpp"
#include "image_registrator.hpp"
#include "image_synthesis.hpp"
#include "reference_frame_selector.hpp"
#include "tracker.hpp"
#include "tracker_processor.hpp"
#include "types.hpp"

namespace hero_lob {

class Pipeline {
public:
    explicit Pipeline(const PipelineConfig& config = {});

    bool Run(const std::string& input_video, const std::string& output_image);

private:
    PipelineConfig config_;
    Capture capture_;
    Identifier identifier_;
    Tracker tracker_;
    ReferenceFrameSelector reference_frame_selector_;
    ImageRegistrator image_registrator_;
    BackgroundRemover background_remover_;
    TrackerProcessor tracker_processor_;
    ImageSynthesis image_synthesis_;
};

}  // namespace hero_lob
