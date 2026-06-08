#include "image_synthesis.hpp"

#include <opencv2/imgproc.hpp>

namespace hero_lob {

ImageSynthesis::ImageSynthesis(const PipelineConfig& config) : config_(config) {}

SynthesisResult ImageSynthesis::Process(
    const ReferenceFrameResult& reference,
    const TrajectoryResult& trajectory) const {
    SynthesisResult result;
    if (!reference.has_reference || reference.reference_frame.bgr.empty()) {
        return result;
    }

    result.output_image = reference.reference_frame.bgr.clone();
    if (trajectory.valid && !trajectory.trajectory_layer.empty()) {
        cv::Mat trajectory_bgr;
        cv::cvtColor(trajectory.trajectory_layer, trajectory_bgr, cv::COLOR_GRAY2BGR);
        cv::max(result.output_image, trajectory_bgr, result.output_image);
    }
    result.valid = true;
    return result;
}

}  // namespace hero_lob
