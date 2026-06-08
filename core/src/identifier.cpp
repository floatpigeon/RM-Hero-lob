#include "identifier.hpp"

namespace hero_lob {

Identifier::Identifier(const PipelineConfig& config) : config_(config) {}

DetectionResult Identifier::Process(const FrameData& frame) const {
    DetectionResult result;
    if (!frame.IsValid()) {
        return result;
    }

    const float width = static_cast<float>(frame.bgr.cols);
    const float height = static_cast<float>(frame.bgr.rows);
    const float light_length = height * config_.placeholder_light_length_ratio;
    const float light_gap = width * config_.placeholder_light_gap_ratio;

    result.anchors.guide_center = {width * 0.5F, height * 0.5F};
    result.anchors.left_top = {result.anchors.guide_center.x - light_gap, result.anchors.guide_center.y - light_length * 0.5F};
    result.anchors.left_bottom = {result.anchors.guide_center.x - light_gap, result.anchors.guide_center.y + light_length * 0.5F};
    result.anchors.right_top = {result.anchors.guide_center.x + light_gap, result.anchors.guide_center.y - light_length * 0.5F};
    result.anchors.right_bottom = {result.anchors.guide_center.x + light_gap, result.anchors.guide_center.y + light_length * 0.5F};
    result.anchors.direction = {1.0F, 0.0F};
    result.anchors.valid = true;
    result.anchors.placeholder = true;
    result.color = TargetColor::kUnknown;
    result.valid = true;
    return result;
}

}  // namespace hero_lob
