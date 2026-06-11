#include <cmath>
#include <iostream>
#include <optional>
#include <string>

#include <opencv2/imgproc.hpp>

#include "identifier.hpp"

namespace {

bool NearlyEqual(float first, float second, float epsilon = 1.5F) {
    return std::fabs(first - second) <= epsilon;
}

hero_lob::PipelineConfig MakeSyntheticConfig() {
    hero_lob::PipelineConfig config;
    config.identifier.morphology.blur_kernel_size = 1;
    config.identifier.morphology.open_kernel_size = 1;
    config.identifier.morphology.close_kernel_size = 1;
    config.identifier.stable_pair_roi.half_width_radius_scale = 3.0F;
    config.identifier.stable_pair_roi.top_offset_radius_scale = 1.0F;
    config.identifier.stable_pair_roi.bottom_offset_radius_scale = 8.0F;
    config.identifier.stable_pair.min_midpoint_y_offset_radius_scale = 1.5F;
    config.identifier.stable_pair.max_midpoint_y_offset_radius_scale = 8.0F;
    config.identifier.stable_pair.min_center_distance_ratio = 0.20F;
    config.identifier.stable_pair.max_center_distance_ratio = 6.0F;
    config.identifier.stable_pair.max_center_distance_radius_scale = 8.0F;
    config.identifier.stable_light.max_center_y_offset_radius_scale = 8.0F;
    config.identifier.stable_light.max_center_x_offset_radius_scale = 6.0F;
    config.identifier.stable_light.min_center_x_offset_radius_scale = 0.6F;
    config.identifier.stable_light.center_exclusion_half_width_radius_scale = 0.1F;
    config.identifier.stable_pair_fallback.min_peak_distance_pixels = 4;
    return config;
}

hero_lob::FrameData MakeFrameData(const cv::Mat& bgr) {
    hero_lob::FrameData frame;
    frame.bgr = bgr.clone();
    cv::cvtColor(frame.bgr, frame.hsv, cv::COLOR_BGR2HSV);
    return frame;
}

cv::Mat MakeScene() {
    return cv::Mat::zeros(cv::Size(260, 240), CV_8UC3);
}

void DrawGuide(cv::Mat& image, const cv::Point& center, int radius, const cv::Scalar& color) {
    cv::circle(image, center, radius, color, cv::FILLED, cv::LINE_8);
}

void DrawBar(
    cv::Mat& image,
    int center_x,
    int top_y,
    int bottom_y,
    int outer_half_width,
    const cv::Scalar& outer_color,
    std::optional<cv::Scalar> inner_color = cv::Scalar(255, 255, 255),
    int inner_half_width = 2) {
    cv::rectangle(
        image,
        cv::Point(center_x - outer_half_width, top_y),
        cv::Point(center_x + outer_half_width, bottom_y),
        outer_color,
        cv::FILLED,
        cv::LINE_8);

    if (inner_color.has_value() && inner_half_width > 0) {
        cv::rectangle(
            image,
            cv::Point(center_x - inner_half_width, top_y),
            cv::Point(center_x + inner_half_width, bottom_y),
            *inner_color,
            cv::FILLED,
            cv::LINE_8);
    }
}

bool EndpointNear(
    const cv::Point2f& point, float expected_x, float expected_y, float epsilon = 3.0F) {
    return NearlyEqual(point.x, expected_x, epsilon) && NearlyEqual(point.y, expected_y, epsilon);
}

bool TestStablePairWinsOverLongBars() {
    cv::Mat image = MakeScene();
    DrawGuide(image, cv::Point(130, 55), 10, cv::Scalar(0, 255, 0));

    DrawBar(image, 92, 88, 196, 7, cv::Scalar(255, 0, 0));
    DrawBar(image, 168, 88, 196, 7, cv::Scalar(255, 0, 0));

    DrawBar(image, 118, 86, 124, 3, cv::Scalar(255, 0, 0), cv::Scalar(255, 255, 255), 1);
    DrawBar(image, 142, 86, 124, 3, cv::Scalar(255, 0, 0), cv::Scalar(255, 255, 255), 1);

    hero_lob::Identifier identifier(MakeSyntheticConfig());
    const hero_lob::DetectionResult detection = identifier.Process(MakeFrameData(image));

    return detection.valid &&
        detection.color == hero_lob::TargetColor::kBlue &&
        EndpointNear(detection.anchors.guide_center, 130.0F, 55.0F, 4.0F) &&
        EndpointNear(detection.anchors.left_top, 118.0F, 86.0F, 4.0F) &&
        EndpointNear(detection.anchors.left_bottom, 118.0F, 124.0F, 4.0F) &&
        EndpointNear(detection.anchors.right_top, 142.0F, 86.0F, 4.0F) &&
        EndpointNear(detection.anchors.right_bottom, 142.0F, 124.0F, 4.0F);
}

bool TestStablePairColorFromEdges() {
    cv::Mat image = MakeScene();
    DrawGuide(image, cv::Point(130, 55), 10, cv::Scalar(0, 255, 255));

    DrawBar(image, 118, 86, 124, 3, cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 255), 1);
    DrawBar(image, 142, 86, 124, 3, cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 255), 1);

    hero_lob::Identifier identifier(MakeSyntheticConfig());
    const hero_lob::DetectionResult detection = identifier.Process(MakeFrameData(image));

    return detection.valid &&
        detection.color == hero_lob::TargetColor::kRed;
}

bool TestUnknownColorStillValid() {
    cv::Mat image = MakeScene();
    DrawGuide(image, cv::Point(130, 55), 10, cv::Scalar(0, 255, 255));

    DrawBar(image, 118, 86, 124, 3, cv::Scalar(255, 255, 255), std::nullopt);
    DrawBar(image, 142, 86, 124, 3, cv::Scalar(255, 255, 255), std::nullopt);

    hero_lob::Identifier identifier(MakeSyntheticConfig());
    const hero_lob::DetectionResult detection = identifier.Process(MakeFrameData(image));

    return detection.valid &&
        detection.color == hero_lob::TargetColor::kUnknown &&
        detection.anchors.valid;
}

bool TestStablePairDetectedWithoutLongBars() {
    cv::Mat image = MakeScene();
    DrawGuide(image, cv::Point(130, 55), 10, cv::Scalar(0, 255, 255));

    DrawBar(image, 118, 86, 124, 3, cv::Scalar(255, 0, 0), cv::Scalar(255, 255, 255), 1);
    DrawBar(image, 142, 86, 124, 3, cv::Scalar(255, 0, 0), cv::Scalar(255, 255, 255), 1);

    hero_lob::Identifier identifier(MakeSyntheticConfig());
    const hero_lob::DetectionResult detection = identifier.Process(MakeFrameData(image));

    return detection.valid &&
        EndpointNear(detection.anchors.left_top, 118.0F, 86.0F, 6.0F) &&
        EndpointNear(detection.anchors.left_bottom, 118.0F, 124.0F, 6.0F) &&
        EndpointNear(detection.anchors.right_top, 142.0F, 86.0F, 6.0F) &&
        EndpointNear(detection.anchors.right_bottom, 142.0F, 124.0F, 6.0F);
}

}  // namespace

int main() {
    int failed = 0;

    if (!TestStablePairWinsOverLongBars()) {
        std::cerr << "TestStablePairWinsOverLongBars failed\n";
        ++failed;
    }
    if (!TestStablePairColorFromEdges()) {
        std::cerr << "TestStablePairColorFromEdges failed\n";
        ++failed;
    }
    if (!TestUnknownColorStillValid()) {
        std::cerr << "TestUnknownColorStillValid failed\n";
        ++failed;
    }
    if (!TestStablePairDetectedWithoutLongBars()) {
        std::cerr << "TestStablePairDetectedWithoutLongBars failed\n";
        ++failed;
    }

    if (failed > 0) {
        return 1;
    }

    std::cout << "identifier_synthetic_test passed\n";
    return 0;
}
