#pragma once

#include "types.hpp"

namespace hero_lob {

class ImageRegistrator {
public:
    explicit ImageRegistrator(const PipelineConfig& config);

    RegistrationResult Process(const ReferenceFrameResult& reference, const FrameData& frame);

private:
    PipelineConfig config_;
};

}  // namespace hero_lob
