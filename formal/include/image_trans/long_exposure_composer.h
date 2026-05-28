#pragma once

#include <opencv2/core.hpp>

namespace image_trans {

struct LongExposureParams {
    double brightness_threshold = 180.0;
};

struct LongExposureStats {
    int accumulated_frames = 0;
};

class LongExposureComposer {
public:
    explicit LongExposureComposer(LongExposureParams params = {});

    void reset(cv::Size frame_size);
    void accumulate(const cv::Mat& stabilized_frame);
    cv::Mat finalize() const;
    const LongExposureStats& stats() const;

private:
    LongExposureParams params_;
    LongExposureStats stats_;
    cv::Size frame_size_;
    cv::Mat average_accumulator_;
    cv::Mat trail_buffer_;
    cv::Mat trail_mask_;
};

}  // namespace image_trans
