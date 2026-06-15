#pragma once

#include <opencv2/core.hpp>

#include "types.hpp"

namespace hero_lob {

class BackgroundRemover {
public:
    explicit BackgroundRemover(const PipelineConfig& config);

    ForegroundMaskResult Process(
        const ReferenceFrameResult& reference,
        const RegistrationResult& registration);
    void Reset();

private:
    PipelineConfig config_;
};

}  // namespace hero_lob
