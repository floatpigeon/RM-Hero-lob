#include <iostream>

#include <opencv2/imgproc.hpp>

#include "background_remover.hpp"

namespace {

hero_lob::PipelineConfig MakeConfig() {
    hero_lob::PipelineConfig config;
    config.motion_foreground.warmup_frames = 3;
    config.motion_foreground.min_brightness_value = 150;
    config.motion_foreground.min_diff_value = 20;
    config.motion_foreground.background_alpha = 0.0F;
    config.motion_foreground.open_kernel_size = 1;
    config.motion_foreground.close_kernel_size = 1;
    config.motion_foreground.static_bright_value_threshold = 220;
    return config;
}

cv::Mat MakeBaseScene() {
    cv::Mat image = cv::Mat::zeros(cv::Size(40, 40), CV_8UC3);
    cv::rectangle(
        image,
        cv::Point(2, 2),
        cv::Point(7, 7),
        cv::Scalar(255, 255, 255),
        cv::FILLED,
        cv::LINE_8);
    return image;
}

hero_lob::ReferenceFrameResult MakeReference(const cv::Mat& bgr) {
    hero_lob::ReferenceFrameResult reference;
    reference.has_reference = true;
    reference.mode = hero_lob::ReferenceMode::kStable;
    reference.reference_frame.bgr = bgr.clone();
    cv::cvtColor(reference.reference_frame.bgr, reference.reference_frame.hsv, cv::COLOR_BGR2HSV);
    return reference;
}

hero_lob::RegistrationResult MakeRegistration(
    const cv::Mat& bgr,
    std::int64_t frame_index,
    double timestamp_seconds) {
    hero_lob::RegistrationResult registration;
    registration.valid = true;
    registration.frame_index = frame_index;
    registration.timestamp_seconds = timestamp_seconds;
    registration.registered_bgr = bgr.clone();
    cv::cvtColor(registration.registered_bgr, registration.registered_hsv, cv::COLOR_BGR2HSV);
    return registration;
}

bool RegionHasSignal(const cv::Mat& mask, const cv::Rect& region) {
    if (mask.empty()) {
        return false;
    }

    return cv::countNonZero(mask(region)) > 0;
}

bool TestWarmupFramesDoNotProduceForeground() {
    const cv::Mat base_scene = MakeBaseScene();
    const hero_lob::ReferenceFrameResult reference = MakeReference(base_scene);
    hero_lob::BackgroundRemover remover(MakeConfig());

    for (int frame_index = 0; frame_index < 3; ++frame_index) {
        const hero_lob::ForegroundMaskResult result = remover.Process(
            reference,
            MakeRegistration(base_scene, frame_index, frame_index * 0.1));
        if (result.valid) {
            return false;
        }
    }

    return true;
}

bool TestStaticBrightRegionIsExcludedAndMotionSurvives() {
    const cv::Mat base_scene = MakeBaseScene();
    const hero_lob::ReferenceFrameResult reference = MakeReference(base_scene);
    hero_lob::BackgroundRemover remover(MakeConfig());

    for (int frame_index = 0; frame_index < 3; ++frame_index) {
        remover.Process(reference, MakeRegistration(base_scene, frame_index, frame_index * 0.1));
    }

    cv::Mat motion_scene = base_scene.clone();
    cv::rectangle(
        motion_scene,
        cv::Point(22, 18),
        cv::Point(25, 21),
        cv::Scalar(255, 255, 255),
        cv::FILLED,
        cv::LINE_8);

    const hero_lob::ForegroundMaskResult result = remover.Process(
        reference,
        MakeRegistration(motion_scene, 3, 0.3));

    return result.valid &&
        RegionHasSignal(result.static_exclusion_mask, cv::Rect(2, 2, 6, 6)) &&
        !RegionHasSignal(result.candidate_mask, cv::Rect(2, 2, 6, 6)) &&
        RegionHasSignal(result.candidate_mask, cv::Rect(22, 18, 4, 4));
}

}  // namespace

int main() {
    int failed = 0;

    if (!TestWarmupFramesDoNotProduceForeground()) {
        std::cerr << "TestWarmupFramesDoNotProduceForeground failed\n";
        ++failed;
    }
    if (!TestStaticBrightRegionIsExcludedAndMotionSurvives()) {
        std::cerr << "TestStaticBrightRegionIsExcludedAndMotionSurvives failed\n";
        ++failed;
    }

    if (failed > 0) {
        return 1;
    }

    std::cout << "background_remover_test passed\n";
    return 0;
}
