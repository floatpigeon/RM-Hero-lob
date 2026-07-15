#pragma once

#include <cmath>

#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

#include "types.hpp"

namespace hero_lob {

class ImageRegistrator {
public:
    explicit ImageRegistrator(const PipelineConfig& config)
        : config_(config) {}

    RegistrationResult Process(const ReferenceFrameResult& reference, const FrameData& frame) {
        RegistrationResult result;
        result.frame_index = frame.frame_index;
        result.timestamp_seconds = frame.timestamp_seconds;

        if (!reference.has_reference || frame.bgr.empty()) {
            result.valid = false;
            return result;
        }

        cv::Mat ref_gray, cur_gray;
        cv::cvtColor(reference.reference_frame.bgr, ref_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(frame.bgr, cur_gray, cv::COLOR_BGR2GRAY);

        const float scale = config_.image_registrator.downscale_factor;
        if (scale < 1.0F) {
            cv::resize(ref_gray, ref_gray, cv::Size(), scale, scale, cv::INTER_AREA);
            cv::resize(cur_gray, cur_gray, cv::Size(), scale, scale, cv::INTER_AREA);
        }

        cv::Mat ref_f, cur_f;
        ref_gray.convertTo(ref_f, CV_32F);
        cur_gray.convertTo(cur_f, CV_32F);
        cv::Point2d shift = cv::phaseCorrelate(ref_f, cur_f);

        double dx = shift.x;
        double dy = shift.y;
        if (scale < 1.0F) {
            dx /= scale;
            dy /= scale;
        }

        double shift_magnitude = std::sqrt(dx * dx + dy * dy);
        if (shift_magnitude > config_.image_registrator.max_shift_pixels) {
            result.valid = true;
            result.transform = cv::Matx23f(1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F);
            result.registered_bgr = frame.bgr.clone();
            cv::cvtColor(result.registered_bgr, result.registered_hsv, cv::COLOR_BGR2HSV);
            return result;
        }

        cv::Mat warp_mat = cv::Mat::eye(2, 3, CV_32F);
        warp_mat.at<float>(0, 2) = static_cast<float>(dx);
        warp_mat.at<float>(1, 2) = static_cast<float>(dy);

        cv::Mat registered_bgr;
        cv::warpAffine(frame.bgr, registered_bgr, warp_mat,
                       reference.reference_frame.bgr.size(), cv::INTER_LINEAR,
                       cv::BORDER_REFLECT);

        result.valid = true;
        result.transform = cv::Matx23f(
            1.0F, 0.0F, static_cast<float>(dx),
            0.0F, 1.0F, static_cast<float>(dy));
        result.registered_bgr = registered_bgr;
        cv::cvtColor(registered_bgr, result.registered_hsv, cv::COLOR_BGR2HSV);
        return result;
    }

private:
    PipelineConfig config_;
};

}  // namespace hero_lob
