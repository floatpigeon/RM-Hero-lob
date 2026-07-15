#include "image_registrator_orb.hpp"

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace hero_lob {

ImageRegistratorOrb::ImageRegistratorOrb(const PipelineConfig& config)
    : config_(config)
    , orb_(cv::ORB::create(config.image_registrator_orb.max_features)) {}

RegistrationResult ImageRegistratorOrb::Process(
    const ReferenceFrameResult& reference, const FrameData& frame) {
    RegistrationResult result;
    result.frame_index = frame.frame_index;
    result.timestamp_seconds = frame.timestamp_seconds;

    auto t0 = std::chrono::steady_clock::now();

    if (!reference.has_reference || frame.bgr.empty()) {
        result.valid = false;
        auto t1 = std::chrono::steady_clock::now();
        timing_stats_.input_validation += std::chrono::duration<double>(t1 - t0).count();
        timing_stats_.frame_count++;
        return result;
    }

    auto t1 = std::chrono::steady_clock::now();
    timing_stats_.input_validation += std::chrono::duration<double>(t1 - t0).count();

    const float scale = config_.image_registrator_orb.downscale_factor;
    const int ref_idx = reference.reference_frame.frame_index;

    t0 = std::chrono::steady_clock::now();
    bool ref_changed = (ref_idx != cached_ref_frame_index_);
    t1 = std::chrono::steady_clock::now();
    timing_stats_.ref_frame_cache_check += std::chrono::duration<double>(t1 - t0).count();

    if (ref_changed) {
        t0 = std::chrono::steady_clock::now();
        cv::Mat ref_gray;
        cv::cvtColor(reference.reference_frame.bgr, ref_gray, cv::COLOR_BGR2GRAY);

        const int ew = config_.image_registrator_orb.exclude_top_left_width;
        const int eh = config_.image_registrator_orb.exclude_top_left_height;
        if (feature_mask_.empty() || feature_mask_.rows != ref_gray.rows ||
            feature_mask_.cols != ref_gray.cols) {
            feature_mask_ = cv::Mat::ones(ref_gray.size(), CV_8UC1) * 255;
            if (ew > 0 && eh > 0) {
                int mw = (scale < 1.0F) ? static_cast<int>(ew * scale) : ew;
                int mh = (scale < 1.0F) ? static_cast<int>(eh * scale) : eh;
                feature_mask_(cv::Rect(0, 0, mw, mh)).setTo(0);
            }
        }

        if (scale < 1.0F) {
            cv::Mat ref_small;
            cv::resize(ref_gray, ref_small, cv::Size(), scale, scale, cv::INTER_AREA);
            cv::Mat mask_small;
            cv::resize(feature_mask_, mask_small, cv::Size(), scale, scale, cv::INTER_NEAREST);
            orb_->detectAndCompute(ref_small, mask_small, ref_kp_, ref_desc_);
            for (auto& kp : ref_kp_) {
                kp.pt.x /= scale;
                kp.pt.y /= scale;
            }
        } else {
            orb_->detectAndCompute(ref_gray, feature_mask_, ref_kp_, ref_desc_);
        }
        cached_ref_frame_index_ = ref_idx;
        t1 = std::chrono::steady_clock::now();
        timing_stats_.ref_frame_preprocess += std::chrono::duration<double>(t1 - t0).count();
    }

    t0 = std::chrono::steady_clock::now();
    cv::Mat cur_gray;
    cv::cvtColor(frame.bgr, cur_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> cur_kp;
    cv::Mat cur_desc;
    if (scale < 1.0F) {
        cv::Mat cur_small;
        cv::resize(cur_gray, cur_small, cv::Size(), scale, scale, cv::INTER_AREA);
        cv::Mat mask_small;
        cv::resize(feature_mask_, mask_small, cv::Size(), scale, scale, cv::INTER_NEAREST);
        orb_->detectAndCompute(cur_small, mask_small, cur_kp, cur_desc);
        for (auto& kp : cur_kp) {
            kp.pt.x /= scale;
            kp.pt.y /= scale;
        }
    } else {
        orb_->detectAndCompute(cur_gray, feature_mask_, cur_kp, cur_desc);
    }
    t1 = std::chrono::steady_clock::now();
    timing_stats_.cur_frame_preprocess += std::chrono::duration<double>(t1 - t0).count();

    t0 = std::chrono::steady_clock::now();
    if (ref_desc_.empty() || cur_desc.empty() || ref_kp_.size() < 4 || cur_kp.size() < 4) {
        result.valid = true;
        result.transform = cv::Matx23f(1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F);
        result.registered_bgr = frame.bgr.clone();
        cv::cvtColor(result.registered_bgr, result.registered_hsv, cv::COLOR_BGR2HSV);
        t1 = std::chrono::steady_clock::now();
        timing_stats_.match_verification += std::chrono::duration<double>(t1 - t0).count();
        timing_stats_.frame_count++;
        return result;
    }

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(cur_desc, ref_desc_, knn_matches, 2);

    const float ratio = config_.image_registrator_orb.match_ratio_threshold;
    std::vector<cv::DMatch> good_matches;
    for (const auto& m : knn_matches) {
        if (m.size() == 2 && m[0].distance < ratio * m[1].distance) {
            good_matches.push_back(m[0]);
        }
    }
    t1 = std::chrono::steady_clock::now();
    timing_stats_.feature_matching += std::chrono::duration<double>(t1 - t0).count();

    t0 = std::chrono::steady_clock::now();
    if (static_cast<int>(good_matches.size()) < config_.image_registrator_orb.min_matches) {
        result.valid = true;
        result.transform = cv::Matx23f(1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F);
        result.registered_bgr = frame.bgr.clone();
        cv::cvtColor(result.registered_bgr, result.registered_hsv, cv::COLOR_BGR2HSV);
        t1 = std::chrono::steady_clock::now();
        timing_stats_.match_verification += std::chrono::duration<double>(t1 - t0).count();
        timing_stats_.frame_count++;
        return result;
    }

    std::vector<cv::Point2f> pts_cur(good_matches.size());
    std::vector<cv::Point2f> pts_ref(good_matches.size());
    for (size_t i = 0; i < good_matches.size(); ++i) {
        pts_cur[i] = cur_kp[good_matches[i].queryIdx].pt;
        pts_ref[i] = ref_kp_[good_matches[i].trainIdx].pt;
    }

    cv::Mat inlier_mask;
    cv::Mat H = cv::findHomography(pts_cur, pts_ref, cv::RANSAC,
                                   config_.image_registrator_orb.ransac_reproj_threshold,
                                   inlier_mask);
    t1 = std::chrono::steady_clock::now();
    timing_stats_.homography_estimation += std::chrono::duration<double>(t1 - t0).count();

    if (H.empty()) {
        result.valid = true;
        result.transform = cv::Matx23f(1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F);
        result.registered_bgr = frame.bgr.clone();
        cv::cvtColor(result.registered_bgr, result.registered_hsv, cv::COLOR_BGR2HSV);
        timing_stats_.frame_count++;
        return result;
    }

    t0 = std::chrono::steady_clock::now();
    const float out_scale = 0.5F;
    cv::Mat H_scaled = H.clone();
    H_scaled.row(0) *= out_scale;
    H_scaled.row(1) *= out_scale;
    cv::Size out_size(
        static_cast<int>(reference.reference_frame.bgr.cols * out_scale),
        static_cast<int>(reference.reference_frame.bgr.rows * out_scale));
    cv::Mat registered_bgr;
    cv::warpPerspective(frame.bgr, registered_bgr, H_scaled, out_size);
    t1 = std::chrono::steady_clock::now();
    timing_stats_.image_transform += std::chrono::duration<double>(t1 - t0).count();

    t0 = std::chrono::steady_clock::now();
    result.valid = true;
    result.transform = cv::Matx23f(
        static_cast<float>(H_scaled.at<double>(0, 0)), static_cast<float>(H_scaled.at<double>(0, 1)),
        static_cast<float>(H_scaled.at<double>(0, 2)), static_cast<float>(H_scaled.at<double>(1, 0)),
        static_cast<float>(H_scaled.at<double>(1, 1)), static_cast<float>(H_scaled.at<double>(1, 2)));
    result.registered_bgr = registered_bgr;
    cv::cvtColor(registered_bgr, result.registered_hsv, cv::COLOR_BGR2HSV);
    t1 = std::chrono::steady_clock::now();
    timing_stats_.result_output += std::chrono::duration<double>(t1 - t0).count();

    timing_stats_.frame_count++;
    return result;
}

void ImageRegistratorOrb::PrintTimingStats() const {
    if (timing_stats_.frame_count == 0) {
        std::cerr << "[ImageRegistratorOrb] No frames processed.\n";
        return;
    }

    const double total_time = timing_stats_.input_validation +
                              timing_stats_.ref_frame_cache_check +
                              timing_stats_.ref_frame_preprocess +
                              timing_stats_.cur_frame_preprocess +
                              timing_stats_.feature_matching +
                              timing_stats_.match_verification +
                              timing_stats_.homography_estimation +
                              timing_stats_.image_transform +
                              timing_stats_.result_output;

    std::cerr << "\n[ImageRegistratorOrb] Timing Statistics:\n";
    std::cerr << "  Frames processed: " << timing_stats_.frame_count << "\n";
    std::cerr << "  Total time: " << total_time << "s\n";
    std::cerr << "  Average per frame: " << (total_time / timing_stats_.frame_count) * 1000.0 << "ms\n\n";

    auto print_stat = [](const char* name, double time, int frame_count, double total_time) {
        double avg_ms = (time / frame_count) * 1000.0;
        double pct = (time / total_time) * 100.0;
        std::cerr << "  " << name << ":\n";
        std::cerr << "    Total: " << time << "s\n";
        std::cerr << "    Average: " << avg_ms << "ms/frame\n";
        std::cerr << "    Percentage: " << pct << "%\n";
    };

    print_stat("Input Validation", timing_stats_.input_validation, timing_stats_.frame_count, total_time);
    print_stat("Ref Frame Cache Check", timing_stats_.ref_frame_cache_check, timing_stats_.frame_count, total_time);
    print_stat("Ref Frame Preprocess", timing_stats_.ref_frame_preprocess, timing_stats_.frame_count, total_time);
    print_stat("Cur Frame Preprocess", timing_stats_.cur_frame_preprocess, timing_stats_.frame_count, total_time);
    print_stat("Feature Matching", timing_stats_.feature_matching, timing_stats_.frame_count, total_time);
    print_stat("Match Verification", timing_stats_.match_verification, timing_stats_.frame_count, total_time);
    print_stat("Homography Estimation", timing_stats_.homography_estimation, timing_stats_.frame_count, total_time);
    print_stat("Image Transform", timing_stats_.image_transform, timing_stats_.frame_count, total_time);
    print_stat("Result Output", timing_stats_.result_output, timing_stats_.frame_count, total_time);
    std::cerr << "\n";
}

}  // namespace hero_lob
