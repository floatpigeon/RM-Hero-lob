#include "background_remover.hpp"

#include <algorithm>

#include <opencv2/imgproc.hpp>

namespace hero_lob {

namespace {

int NormalizeKernelSize(int requested_size) {
    return requested_size > 1 ? requested_size : 1;
}

cv::Mat MakeKernel(int kernel_size) {
    const int normalized_size = NormalizeKernelSize(kernel_size);
    return cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(normalized_size, normalized_size));
}

}  // namespace

BackgroundRemover::BackgroundRemover(const PipelineConfig& config) : config_(config) {}

ForegroundMaskResult BackgroundRemover::Process(
    const ReferenceFrameResult& reference,
    const RegistrationResult& registration) {
    ForegroundMaskResult result;
    if (!reference.has_reference || !registration.valid ||
        registration.registered_bgr.empty() || registration.registered_hsv.empty()) {
        return result;
    }

    const cv::Size frame_size = registration.registered_bgr.size();
    ResetStateForFrameSize(frame_size);

    result.frame_index = registration.frame_index;
    result.timestamp_seconds = registration.timestamp_seconds;
    result.roi = cv::Rect(0, 0, frame_size.width, frame_size.height);

    cv::Mat gray_frame;
    cv::cvtColor(registration.registered_bgr, gray_frame, cv::COLOR_BGR2GRAY);

    cv::Mat value_channel;
    cv::extractChannel(registration.registered_hsv, value_channel, 2);

    const int warmup_frames = std::max(1, config_.motion_foreground.warmup_frames);
    if (!background_initialized_) {
        cv::Mat gray_float;
        gray_frame.convertTo(gray_float, CV_32F);
        warmup_gray_sum_ += gray_float;

        cv::Mat bright_mask;
        cv::threshold(
            value_channel,
            bright_mask,
            config_.motion_foreground.static_bright_value_threshold - 1,
            1,
            cv::THRESH_BINARY);

        cv::Mat bright_mask_u16;
        bright_mask.convertTo(bright_mask_u16, CV_16UC1);
        static_bright_counts_ += bright_mask_u16;

        ++observed_warmup_frames_;
        if (observed_warmup_frames_ >= warmup_frames) {
            warmup_gray_sum_.convertTo(
                background_gray_,
                CV_32F,
                1.0 / static_cast<double>(observed_warmup_frames_));

            cv::compare(
                static_bright_counts_,
                cv::Scalar::all(observed_warmup_frames_),
                static_exclusion_mask_,
                cv::CMP_GE);

            background_initialized_ = true;
        }

        result.static_exclusion_mask = static_exclusion_mask_.clone();
        return result;
    }

    cv::Mat background_gray_u8;
    background_gray_.convertTo(background_gray_u8, CV_8UC1);

    cv::Mat diff;
    cv::absdiff(gray_frame, background_gray_u8, diff);

    cv::Mat diff_mask;
    cv::threshold(
        diff,
        diff_mask,
        config_.motion_foreground.min_diff_value - 1,
        255,
        cv::THRESH_BINARY);

    cv::Mat brightness_mask;
    cv::threshold(
        value_channel,
        brightness_mask,
        config_.motion_foreground.min_brightness_value - 1,
        255,
        cv::THRESH_BINARY);

    cv::Mat candidate_mask;
    cv::bitwise_and(diff_mask, brightness_mask, candidate_mask);
    if (!static_exclusion_mask_.empty()) {
        cv::Mat inverse_static_mask;
        cv::bitwise_not(static_exclusion_mask_, inverse_static_mask);
        cv::bitwise_and(candidate_mask, inverse_static_mask, candidate_mask);
    }

    const cv::Mat open_kernel = MakeKernel(config_.motion_foreground.open_kernel_size);
    const cv::Mat close_kernel = MakeKernel(config_.motion_foreground.close_kernel_size);
    if (open_kernel.rows > 1 || open_kernel.cols > 1) {
        cv::morphologyEx(candidate_mask, candidate_mask, cv::MORPH_OPEN, open_kernel);
    }
    if (close_kernel.rows > 1 || close_kernel.cols > 1) {
        cv::morphologyEx(candidate_mask, candidate_mask, cv::MORPH_CLOSE, close_kernel);
    }

    const float alpha = std::clamp(config_.motion_foreground.background_alpha, 0.0F, 1.0F);
    cv::Mat inverse_candidate_mask;
    cv::bitwise_not(candidate_mask, inverse_candidate_mask);
    cv::Mat gray_float;
    gray_frame.convertTo(gray_float, CV_32F);
    cv::Mat updated_background =
        background_gray_ * (1.0F - alpha) + gray_float * alpha;
    updated_background.copyTo(background_gray_, inverse_candidate_mask);

    result.valid = true;
    result.static_exclusion_mask = static_exclusion_mask_.clone();
    result.candidate_mask = candidate_mask;
    return result;
}

void BackgroundRemover::Reset() {
    background_gray_.release();
    warmup_gray_sum_.release();
    static_bright_counts_.release();
    static_exclusion_mask_.release();
    observed_warmup_frames_ = 0;
    background_initialized_ = false;
}

void BackgroundRemover::ResetStateForFrameSize(const cv::Size& frame_size) {
    if (!warmup_gray_sum_.empty() &&
        warmup_gray_sum_.size() == frame_size &&
        static_bright_counts_.size() == frame_size &&
        static_exclusion_mask_.size() == frame_size &&
        (background_gray_.empty() || background_gray_.size() == frame_size)) {
        return;
    }

    background_gray_ = cv::Mat();
    warmup_gray_sum_ = cv::Mat::zeros(frame_size, CV_32FC1);
    static_bright_counts_ = cv::Mat::zeros(frame_size, CV_16UC1);
    static_exclusion_mask_ = cv::Mat::zeros(frame_size, CV_8UC1);
    observed_warmup_frames_ = 0;
    background_initialized_ = false;
}

}  // namespace hero_lob
