#pragma once

#include "types.hpp"

namespace hero_lob {

class BackgroundRemover {
public:
    explicit BackgroundRemover(const PipelineConfig& config);

    ForegroundMaskResult Process(
        const ReferenceFrameResult& reference,
        const RegistrationResult& registration) const;

private:
    PipelineConfig config_;
};

}  // namespace hero_lob
