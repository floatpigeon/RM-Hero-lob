#pragma once

#include "types.hpp"

namespace hero_lob {

struct IdentifierDebugArtifacts {
    cv::Mat raw_brightness_mask;
    cv::Mat brightness_mask;
    cv::Mat guide_candidate_mask;
    cv::Mat light_candidate_mask;
    cv::Mat stable_pair_roi;
    cv::Mat edge_red_mask;
    cv::Mat edge_blue_mask;
    cv::Mat candidate_overlay;
    cv::Mat stable_pair_overlay;
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
