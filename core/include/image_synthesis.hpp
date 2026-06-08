#pragma once

#include "types.hpp"

namespace hero_lob {

class ImageSynthesis {
public:
    explicit ImageSynthesis(const PipelineConfig& config);

    SynthesisResult Process(
        const ReferenceFrameResult& reference,
        const TrajectoryResult& trajectory) const;

private:
    PipelineConfig config_;
};

}  // namespace hero_lob
