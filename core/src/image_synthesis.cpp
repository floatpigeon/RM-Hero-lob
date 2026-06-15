#include "image_synthesis.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace hero_lob {

ImageSynthesis::ImageSynthesis(const PipelineConfig& config)
    : config_(config) {}

SynthesisResult ImageSynthesis::Process(
    const ReferenceFrameResult& reference,
    const TrajectoryResult& trajectory) const {
    SynthesisResult result;
    if (!reference.has_reference || !trajectory.valid ||
        trajectory.trajectory_layer.empty()) {
        if (reference.has_reference) {
            std::cerr << "[ImageSynthesis] No trajectory data, outputting "
                         "reference frame only\n";
            result.valid = true;
            result.output_image = reference.reference_frame.bgr.clone();
        }
        return result;
    }
    const auto& tw = config_.trajectory_window;
    cv::Mat layer = trajectory.trajectory_layer;
    std::vector<float> flat;
    layer.reshape(1, 1).copyTo(flat);
    std::sort(flat.begin(), flat.end());
    auto nonzero_it = std::lower_bound(flat.begin(), flat.end(), 1e-6F);
    int nonzero_count = static_cast<int>(std::distance(nonzero_it, flat.end()));
    int p99_index = static_cast<int>(nonzero_count * tw.normalization_percentile);
    if (p99_index >= nonzero_count) {
        p99_index = nonzero_count - 1;
    }
    float max_val = (nonzero_count > 0) ? *(nonzero_it + p99_index) : 0.0F;
    std::cerr << "[ImageSynthesis] p99_percentile=" << tw.normalization_percentile
              << " p99_value=" << max_val
              << " accumulated_frames=" << trajectory.accumulated_frames << '\n';
    if (max_val < 1e-6F) {
        std::cerr << "[ImageSynthesis] Trajectory layer is all zero, "
                     "outputting reference frame\n";
        result.valid = true;
        result.output_image = reference.reference_frame.bgr.clone();
        return result;
    }
    cv::Mat normalized;
    layer.convertTo(normalized, CV_32F, 255.0 / max_val);
    cv::Mat clamped;
    normalized.convertTo(clamped, CV_8UC3);
    cv::Mat ref = reference.reference_frame.bgr;
    cv::Mat output;
    cv::add(ref, clamped, output);
    std::cerr << "[ImageSynthesis] output=" << output.cols << "x"
              << output.rows << '\n';
    result.valid = true;
    result.output_image = output;
    return result;
}

}  // namespace hero_lob
