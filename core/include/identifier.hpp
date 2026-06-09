#pragma once

#include "types.hpp"

namespace hero_lob {

struct IdentifierDebugArtifacts {
    cv::Mat raw_guide_mask;
    cv::Mat raw_red_mask;
    cv::Mat raw_blue_mask;
    cv::Mat guide_mask;
    cv::Mat red_mask;
    cv::Mat blue_mask;
    cv::Mat candidate_overlay;
    cv::Mat result_overlay;
};

struct IdentifierAnalysisResult {
    DetectionResult detection = {};
    IdentifierDebugArtifacts debug = {};
};

class Identifier {
public:
    explicit Identifier(const PipelineConfig& config);

    IdentifierAnalysisResult Analyze(const FrameData& frame) const;
    DetectionResult Process(const FrameData& frame) const;

private:
    PipelineConfig config_;
};

}  // namespace hero_lob
