#include "identifier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "identifier_geometry.hpp"

namespace hero_lob {
namespace {

using identifier_geometry::GuideCandidate;
using identifier_geometry::LightBarCandidate;
using identifier_geometry::LightPairCandidate;
using identifier_geometry::NormalizeVector;
using identifier_geometry::TripletCandidate;

constexpr float kPi = 3.14159265358979323846F;

cv::Scalar MaskColorFor(TargetColor color) {
    switch (color) {
        case TargetColor::kRed: return {0, 0, 255};
        case TargetColor::kBlue: return {255, 0, 0};
        case TargetColor::kUnknown: break;
    }
    return {255, 255, 255};
}

cv::Mat MakeBinaryMask(
    const cv::Mat& hsv, const HsvRangeConfig& range, const ColorThresholdConfig& config) {
    cv::Mat mask;
    cv::inRange(
        hsv, cv::Scalar(range.hue_min, config.min_saturation, config.min_value),
        cv::Scalar(range.hue_max, 255, 255), mask);
    return mask;
}

cv::Mat ApplyMorphology(const cv::Mat& input, const MorphologyConfig& config) {
    cv::Mat result = input.clone();

    const int blur_kernel = std::max(1, config.blur_kernel_size | 1);
    if (blur_kernel > 1) {
        cv::GaussianBlur(result, result, cv::Size(blur_kernel, blur_kernel), 0.0);
        cv::threshold(result, result, 127, 255, cv::THRESH_BINARY);
    }

    const int open_kernel = std::max(1, config.open_kernel_size);
    const int close_kernel = std::max(1, config.close_kernel_size);
    const cv::Mat open_element =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(open_kernel, open_kernel));
    const cv::Mat close_element =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(close_kernel, close_kernel));

    cv::morphologyEx(result, result, cv::MORPH_OPEN, open_element);
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, close_element);
    return result;
}

std::vector<std::vector<cv::Point>> FindExternalContours(const cv::Mat& mask) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    return contours;
}

float ComputeCircularity(const std::vector<cv::Point>& contour, const double area) {
    const double perimeter = cv::arcLength(contour, true);
    if (perimeter <= 1e-4) {
        return 0.0F;
    }
    return static_cast<float>(4.0 * kPi * area / (perimeter * perimeter));
}

float MeanColorAdvantage(
    const cv::Mat& bgr, const std::vector<cv::Point>& contour, TargetColor target) {
    cv::Mat contour_mask = cv::Mat::zeros(bgr.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours = {contour};
    cv::drawContours(contour_mask, contours, 0, cv::Scalar(255), cv::FILLED);
    const cv::Scalar mean_color = cv::mean(bgr, contour_mask);

    if (target == TargetColor::kRed) {
        return static_cast<float>(mean_color[2] - std::max(mean_color[1], mean_color[0]));
    }
    if (target == TargetColor::kBlue) {
        return static_cast<float>(mean_color[0] - std::max(mean_color[1], mean_color[2]));
    }
    return static_cast<float>(mean_color[1] - std::max(mean_color[0], mean_color[2]));
}

std::vector<GuideCandidate> ExtractGuideCandidates(
    const FrameData& frame, const cv::Mat& guide_mask, const GuideConstraintConfig& config) {
    std::vector<GuideCandidate> guides;
    if (frame.bgr.empty()) {
        return guides;
    }

    const float frame_area = static_cast<float>(frame.bgr.cols * frame.bgr.rows);
    for (const auto& contour : FindExternalContours(guide_mask)) {
        const double area = cv::contourArea(contour);
        if (area <= 0.0) {
            continue;
        }

        const float area_ratio = static_cast<float>(area / frame_area);
        if (area_ratio < config.min_area_ratio || area_ratio > config.max_area_ratio) {
            continue;
        }

        const cv::Rect bbox = cv::boundingRect(contour);
        const float aspect_ratio =
            bbox.height > 0 ? static_cast<float>(bbox.width) / static_cast<float>(bbox.height)
                            : 0.0F;
        if (std::fabs(aspect_ratio - 1.0F) > config.max_aspect_ratio_deviation) {
            continue;
        }

        const float circularity = ComputeCircularity(contour, area);
        if (circularity < config.min_circularity) {
            continue;
        }

        const float color_advantage = MeanColorAdvantage(frame.bgr, contour, TargetColor::kUnknown);
        if (color_advantage < config.min_color_advantage) {
            continue;
        }

        cv::Point2f center;
        float radius = 0.0F;
        cv::minEnclosingCircle(contour, center, radius);

        GuideCandidate candidate;
        candidate.center = center;
        candidate.radius = radius;
        candidate.circularity = circularity;
        candidate.color_advantage = color_advantage;
        candidate.mask_support = static_cast<float>(area);
        guides.push_back(candidate);
    }

    return guides;
}

float AngleDegreesOf(const cv::Point2f& axis) { return std::atan2(axis.y, axis.x) * 180.0F / kPi; }

std::vector<LightBarCandidate> ExtractLightCandidates(
    const FrameData& frame, const cv::Mat& mask, TargetColor color,
    const LightConstraintConfig& config) {
    std::vector<LightBarCandidate> lights;
    if (frame.bgr.empty()) {
        return lights;
    }

    const float max_dimension = static_cast<float>(std::max(frame.bgr.cols, frame.bgr.rows));
    for (const auto& contour : FindExternalContours(mask)) {
        const double area = cv::contourArea(contour);
        if (area <= 0.0) {
            continue;
        }

        const cv::RotatedRect box = cv::minAreaRect(contour);
        const float rect_width = std::max(box.size.width, box.size.height);
        const float rect_height = std::min(box.size.width, box.size.height);
        if (rect_height <= 0.0F || rect_width <= 0.0F) {
            continue;
        }

        const float length_ratio = rect_width / max_dimension;
        const float width_ratio = rect_height / max_dimension;
        if (length_ratio < config.min_length_ratio || length_ratio > config.max_length_ratio) {
            continue;
        }
        if (width_ratio < config.min_width_ratio || width_ratio > config.max_width_ratio) {
            continue;
        }

        const float aspect_ratio = rect_width / rect_height;
        if (aspect_ratio < config.min_aspect_ratio || aspect_ratio > config.max_aspect_ratio) {
            continue;
        }

        const float fill_ratio = static_cast<float>(area / (rect_width * rect_height));
        if (fill_ratio < config.min_fill_ratio) {
            continue;
        }

        const float color_advantage = MeanColorAdvantage(frame.bgr, contour, color);
        if (color_advantage < config.min_color_advantage) {
            continue;
        }

        float angle_radians = box.angle * kPi / 180.0F;
        if (box.size.height > box.size.width) {
            angle_radians += kPi * 0.5F;
        }
        cv::Point2f axis(std::cos(angle_radians), std::sin(angle_radians));
        axis = NormalizeVector(axis);
        const cv::Point2f endpoint_offset = axis * (rect_width * 0.5F);
        const auto endpoints = identifier_geometry::SortEndpointsTopBottom(
            box.center - endpoint_offset, box.center + endpoint_offset);

        LightBarCandidate candidate;
        candidate.color = color;
        candidate.center = box.center;
        candidate.top = endpoints[0];
        candidate.bottom = endpoints[1];
        candidate.axis = NormalizeVector(candidate.bottom - candidate.top);
        candidate.box = box;
        candidate.length = rect_width;
        candidate.width = rect_height;
        candidate.angle_degrees = AngleDegreesOf(candidate.axis);
        candidate.fill_ratio = fill_ratio;
        candidate.color_advantage = color_advantage;
        candidate.mask_support = static_cast<float>(area);
        lights.push_back(identifier_geometry::CanonicalizeLightBar(candidate));
    }

    return lights;
}

std::vector<LightPairCandidate> BuildPairs(
    const std::vector<LightBarCandidate>& red_candidates,
    const std::vector<LightBarCandidate>& blue_candidates, const PairConstraintConfig& config) {
    std::vector<LightPairCandidate> pairs;

    auto append_pairs_for_color = [&](const std::vector<LightBarCandidate>& candidates) {
        for (std::size_t first_index = 0; first_index < candidates.size(); ++first_index) {
            for (std::size_t second_index = first_index + 1; second_index < candidates.size();
                 ++second_index) {
                std::optional<LightPairCandidate> pair = identifier_geometry::TryBuildLightPair(
                    candidates[first_index], candidates[second_index], config);
                if (pair.has_value()) {
                    pairs.push_back(*pair);
                }
            }
        }
    };

    append_pairs_for_color(red_candidates);
    append_pairs_for_color(blue_candidates);
    return pairs;
}

std::vector<TripletCandidate> BuildTriplets(
    const std::vector<GuideCandidate>& guides, const std::vector<LightPairCandidate>& pairs,
    const TripletConstraintConfig& config) {
    std::vector<TripletCandidate> triplets;
    for (const GuideCandidate& guide : guides) {
        for (const LightPairCandidate& pair : pairs) {
            std::optional<TripletCandidate> triplet =
                identifier_geometry::TryBuildTriplet(guide, pair, config);
            if (triplet.has_value()) {
                triplets.push_back(*triplet);
            }
        }
    }
    return triplets;
}

std::optional<TripletCandidate> SelectBestTriplet(std::vector<TripletCandidate> triplets) {
    if (triplets.empty()) {
        return std::nullopt;
    }

    std::sort(
        triplets.begin(), triplets.end(),
        [](const TripletCandidate& first, const TripletCandidate& second) {
            const float penalty_delta = std::fabs(first.penalty - second.penalty);
            if (penalty_delta > 1e-4F) {
                return first.penalty < second.penalty;
            }
            return first.support > second.support;
        });
    return triplets.front();
}

DetectionResult ToDetection(const TripletCandidate& triplet) {
    DetectionResult result;
    result.anchors.guide_center = triplet.guide.center;
    result.anchors.left_top = triplet.pair.left.top;
    result.anchors.left_bottom = triplet.pair.left.bottom;
    result.anchors.right_top = triplet.pair.right.top;
    result.anchors.right_bottom = triplet.pair.right.bottom;
    result.anchors.direction = triplet.pair.direction;
    result.anchors.valid = true;
    result.anchors.placeholder = false;
    result.color = triplet.color;
    result.valid = true;
    return result;
}

void DrawGuideCandidate(cv::Mat& image, const GuideCandidate& guide, const cv::Scalar& color) {
    cv::circle(
        image, guide.center, std::max(1, static_cast<int>(std::lround(guide.radius))), color, 2,
        cv::LINE_AA);
}

void DrawLightBarCandidate(
    cv::Mat& image, const LightBarCandidate& light, const cv::Scalar& color, int thickness) {
    cv::line(image, light.top, light.bottom, color, thickness, cv::LINE_AA);
    cv::circle(image, light.top, 3, color, cv::FILLED, cv::LINE_AA);
    cv::circle(image, light.bottom, 3, color, cv::FILLED, cv::LINE_AA);
}

cv::Mat BuildCandidateOverlay(
    const cv::Mat& base, const std::vector<GuideCandidate>& guides,
    const std::vector<LightBarCandidate>& red_lights,
    const std::vector<LightBarCandidate>& blue_lights,
    const std::vector<LightPairCandidate>& pairs) {
    cv::Mat overlay = base.clone();
    for (const GuideCandidate& guide : guides) {
        DrawGuideCandidate(overlay, guide, cv::Scalar(0, 255, 0));
    }
    for (const LightBarCandidate& light : red_lights) {
        DrawLightBarCandidate(overlay, light, cv::Scalar(0, 0, 200), 2);
    }
    for (const LightBarCandidate& light : blue_lights) {
        DrawLightBarCandidate(overlay, light, cv::Scalar(200, 0, 0), 2);
    }
    for (const LightPairCandidate& pair : pairs) {
        cv::line(
            overlay, pair.left.center, pair.right.center, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        cv::circle(overlay, pair.midpoint, 4, cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_AA);
    }
    return overlay;
}

cv::Mat BuildResultOverlay(
    const cv::Mat& base, const std::optional<TripletCandidate>& best_triplet) {
    cv::Mat overlay = base.clone();
    if (!best_triplet.has_value()) {
        return overlay;
    }

    const TripletCandidate& triplet = *best_triplet;
    DrawGuideCandidate(overlay, triplet.guide, cv::Scalar(0, 255, 0));
    const cv::Scalar light_color = MaskColorFor(triplet.color);
    DrawLightBarCandidate(overlay, triplet.pair.left, light_color, 3);
    DrawLightBarCandidate(overlay, triplet.pair.right, light_color, 3);
    cv::line(
        overlay, triplet.pair.left.center, triplet.pair.right.center, cv::Scalar(0, 255, 255), 2,
        cv::LINE_AA);
    cv::arrowedLine(
        overlay, triplet.pair.left.center, triplet.pair.right.center, cv::Scalar(255, 255, 255), 2,
        cv::LINE_AA);
    return overlay;
}

}  // namespace

Identifier::Identifier(const PipelineConfig& config) : config_(config) {}

IdentifierAnalysisResult Identifier::Analyze(const FrameData& frame) const {
    IdentifierAnalysisResult analysis;
    if (!frame.IsValid() || frame.hsv.empty()) {
        return analysis;
    }

    const IdentifierConfig& config = config_.identifier;
    cv::Mat green_mask = MakeBinaryMask(frame.hsv, config.color.green, config.color);
    cv::Mat red_low_mask = MakeBinaryMask(frame.hsv, config.color.red_low, config.color);
    cv::Mat red_high_mask = MakeBinaryMask(frame.hsv, config.color.red_high, config.color);
    cv::Mat blue_mask = MakeBinaryMask(frame.hsv, config.color.blue, config.color);
    cv::Mat red_mask;
    cv::bitwise_or(red_low_mask, red_high_mask, red_mask);

    analysis.debug.raw_guide_mask = green_mask.clone();
    analysis.debug.raw_red_mask = red_mask.clone();
    analysis.debug.raw_blue_mask = blue_mask.clone();

    green_mask = ApplyMorphology(green_mask, config.morphology);
    red_mask = ApplyMorphology(red_mask, config.morphology);
    blue_mask = ApplyMorphology(blue_mask, config.morphology);

    const std::vector<GuideCandidate> guides =
        ExtractGuideCandidates(frame, green_mask, config.guide);
    const std::vector<LightBarCandidate> red_candidates =
        ExtractLightCandidates(frame, red_mask, TargetColor::kRed, config.light);
    const std::vector<LightBarCandidate> blue_candidates =
        ExtractLightCandidates(frame, blue_mask, TargetColor::kBlue, config.light);
    const std::vector<LightPairCandidate> pairs =
        BuildPairs(red_candidates, blue_candidates, config.pair);
    const std::vector<TripletCandidate> triplets = BuildTriplets(guides, pairs, config.triplet);
    const std::optional<TripletCandidate> best_triplet = SelectBestTriplet(triplets);

    if (best_triplet.has_value()) {
        analysis.detection = ToDetection(*best_triplet);
    }

    analysis.debug.guide_mask = green_mask;
    analysis.debug.red_mask = red_mask;
    analysis.debug.blue_mask = blue_mask;
    analysis.debug.candidate_overlay =
        BuildCandidateOverlay(frame.bgr, guides, red_candidates, blue_candidates, pairs);
    analysis.debug.result_overlay = BuildResultOverlay(frame.bgr, best_triplet);
    return analysis;
}

DetectionResult Identifier::Process(const FrameData& frame) const {
    return Analyze(frame).detection;
}

}  // namespace hero_lob
