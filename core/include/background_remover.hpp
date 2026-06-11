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
    void ResetStateForFrameSize(const cv::Size& frame_size);

    PipelineConfig config_;
    cv::Mat background_gray_;
    cv::Mat warmup_gray_sum_;
    cv::Mat static_bright_counts_;
    cv::Mat static_exclusion_mask_;
    int observed_warmup_frames_ = 0;
    bool background_initialized_ = false;
};

}  // namespace hero_lob
