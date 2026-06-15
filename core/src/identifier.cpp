#include "identifier.hpp"

namespace hero_lob {

Identifier::Identifier(const PipelineConfig& config) : config_(config) {}

IdentifierAnalysisResult Identifier::Analyze(const FrameData& frame) const {
    return {};
}

DetectionResult Identifier::Process(const FrameData& frame) const {
    return {};
}

}  // namespace hero_lob
