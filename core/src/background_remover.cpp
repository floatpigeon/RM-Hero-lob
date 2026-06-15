#include "background_remover.hpp"

#include <iostream>

#include <opencv2/imgproc.hpp>

namespace hero_lob {

BackgroundRemover::BackgroundRemover(const PipelineConfig& config)
    : config_(config) {}

ForegroundMaskResult BackgroundRemover::Process(
    const ReferenceFrameResult& reference,
    const RegistrationResult& registration) {
    ForegroundMaskResult result;
    if (!reference.has_reference || registration.registered_bgr.empty()) {
        return result;
    }
    const auto& fg = config_.motion_foreground;
    cv::Mat ref_gray, cur_gray;
    cv::cvtColor(reference.reference_frame.bgr, ref_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(registration.registered_bgr, cur_gray, cv::COLOR_BGR2GRAY);
    cv::Mat diff;
    cv::absdiff(ref_gray, cur_gray, diff);
    cv::Mat bright_mask;
    cv::threshold(cur_gray, bright_mask, fg.min_brightness_value, 255,
                  cv::THRESH_BINARY);
    cv::Mat diff_mask;
    cv::threshold(diff, diff_mask, fg.min_diff_value, 255, cv::THRESH_BINARY);
    cv::Mat candidate_mask;
    cv::bitwise_and(diff_mask, bright_mask, candidate_mask);
    if (fg.open_kernel_size > 0) {
        cv::Mat open_kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(fg.open_kernel_size, fg.open_kernel_size));
        cv::morphologyEx(candidate_mask, candidate_mask, cv::MORPH_OPEN,
                         open_kernel);
    }
    if (fg.close_kernel_size > 0) {
        cv::Mat close_kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(fg.close_kernel_size, fg.close_kernel_size));
        cv::morphologyEx(candidate_mask, candidate_mask, cv::MORPH_CLOSE,
                         close_kernel);
    }
    int fg_pixels = cv::countNonZero(candidate_mask);
    int total_pixels = candidate_mask.cols * candidate_mask.rows;
    double fg_ratio = (total_pixels > 0)
        ? static_cast<double>(fg_pixels) / total_pixels * 100.0
        : 0.0;
    std::cerr << "[BackgroundRemover] frame=" << registration.frame_index
              << " t=" << registration.timestamp_seconds
              << "s fg_pixels=" << fg_pixels
              << " (" << fg_ratio << "%)\n";
    result.valid = true;
    result.frame_index = registration.frame_index;
    result.timestamp_seconds = registration.timestamp_seconds;
    result.roi = cv::Rect(0, 0, candidate_mask.cols, candidate_mask.rows);
    result.candidate_mask = candidate_mask;
    result.candidate_bgr = registration.registered_bgr;
    return result;
}

void BackgroundRemover::Reset() {}

}  // namespace hero_lob
