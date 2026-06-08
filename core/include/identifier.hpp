#pragma once

#include "types.hpp"

namespace hero_lob {

class Identifier {
public:
    explicit Identifier(const PipelineConfig& config);

    DetectionResult Process(const FrameData& frame) const;

private:
    PipelineConfig config_;
};

}  // namespace hero_lob
