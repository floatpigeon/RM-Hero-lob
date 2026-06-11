#include "identifier_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace hero_lob::identifier_geometry {
namespace {

constexpr float kEpsilon = 1e-4F;
constexpr float kPi = 3.14159265358979323846F;

float Clamp01(const float value) { return std::clamp(value, 0.0F, 1.0F); }

float LengthOf(const cv::Point2f& point) {
    return std::sqrt(point.x * point.x + point.y * point.y);
}

float DegreesBetweenAxes(const cv::Point2f& first, const cv::Point2f& second) {
    const cv::Point2f normalized_first = NormalizeVector(first);
    const cv::Point2f normalized_second = NormalizeVector(second);
    float dot = normalized_first.dot(normalized_second);
    dot = std::clamp(dot, -1.0F, 1.0F);
    const float degrees = std::acos(std::fabs(dot)) * 180.0F / kPi;
    return degrees;
}

cv::Point2f PerpendicularOf(const cv::Point2f& axis) { return {-axis.y, axis.x}; }

cv::Point2f OrientLateralAxis(const cv::Point2f& axis) {
    cv::Point2f oriented = NormalizeVector(axis);
    if (oriented.x < 0.0F || (std::fabs(oriented.x) <= kEpsilon && oriented.y < 0.0F)) {
        oriented *= -1.0F;
    }
    return oriented;
}

float MaxValue(const float first, const float second) { return std::max(first, second); }

float MinValue(const float first, const float second) { return std::min(first, second); }

float SafeRatio(const float numerator, const float denominator) {
    if (std::fabs(denominator) <= kEpsilon) {
        return 0.0F;
    }
    return numerator / denominator;
}

float PointDistance(const cv::Point2f& first, const cv::Point2f& second) {
    return LengthOf(first - second);
}

float ProjectionDistance(const cv::Point2f& point, const cv::Point2f& axis) {
    const cv::Point2f normalized_axis = NormalizeVector(axis);
    return point.dot(normalized_axis);
}

float OverlapRatioAlongAxis(
    const LightBarCandidate& first,
    const LightBarCandidate& second,
    const cv::Point2f& axis) {
    const cv::Point2f normalized_axis = NormalizeVector(axis);
    const float first_center = ProjectionDistance(first.center, normalized_axis);
    const float second_center = ProjectionDistance(second.center, normalized_axis);
    const float half_span = (first.width + second.width) * 0.5F;
    const float overlap = std::max(0.0F, half_span - std::fabs(second_center - first_center));
    const float min_width = std::max(kEpsilon, std::min(first.width, second.width));
    return overlap / min_width;
}

float NormalizedPenaltyComponent(const float value, const float max_value) {
    if (max_value <= kEpsilon) {
        return 0.0F;
    }
    return Clamp01(value / max_value);
}

}  // namespace

cv::Point2f NormalizeVector(const cv::Point2f& vector) {
    const float length = LengthOf(vector);
    if (length <= kEpsilon) {
        return {0.0F, 0.0F};
    }
    return vector * (1.0F / length);
}

std::array<cv::Point2f, 2> SortEndpointsTopBottom(
    const cv::Point2f& first, const cv::Point2f& second) {
    if (first.y < second.y) {
        return {first, second};
    }
    if (first.y > second.y) {
        return {second, first};
    }
    return first.x <= second.x ? std::array<cv::Point2f, 2>{first, second}
                               : std::array<cv::Point2f, 2>{second, first};
}

LightBarCandidate CanonicalizeLightBar(const LightBarCandidate& candidate) {
    LightBarCandidate normalized = candidate;
    const cv::Point2f axis = NormalizeVector(candidate.bottom - candidate.top);
    const cv::Point2f resolved_axis = LengthOf(axis) > kEpsilon ? axis : NormalizeVector(candidate.axis);
    normalized.axis = resolved_axis.y < 0.0F ? resolved_axis * -1.0F : resolved_axis;
    const auto sorted = SortEndpointsTopBottom(candidate.top, candidate.bottom);
    normalized.top = sorted[0];
    normalized.bottom = sorted[1];
    normalized.center = (normalized.top + normalized.bottom) * 0.5F;
    normalized.length = std::max(candidate.length, PointDistance(normalized.top, normalized.bottom));
    normalized.width = candidate.width;
    normalized.angle_degrees = std::atan2(normalized.axis.y, normalized.axis.x) * 180.0F / kPi;
    return normalized;
}

std::optional<LightPairCandidate> TryBuildLightPair(
    const LightBarCandidate& first,
    const LightBarCandidate& second,
    const PairConstraintConfig& config) {
    const LightBarCandidate normalized_first = CanonicalizeLightBar(first);
    const LightBarCandidate normalized_second = CanonicalizeLightBar(second);

    const float average_length =
        std::max(kEpsilon, (normalized_first.length + normalized_second.length) * 0.5F);
    const float angle_delta =
        DegreesBetweenAxes(normalized_first.axis, normalized_second.axis);
    if (angle_delta > config.max_angle_difference_degrees) {
        return std::nullopt;
    }

    const float length_delta_ratio =
        std::fabs(normalized_first.length - normalized_second.length) / average_length;
    if (length_delta_ratio > config.max_length_delta_ratio) {
        return std::nullopt;
    }

    const float center_y_delta_ratio =
        std::fabs(normalized_first.center.y - normalized_second.center.y) / average_length;
    if (center_y_delta_ratio > config.max_center_y_delta_ratio) {
        return std::nullopt;
    }

    const cv::Point2f average_axis =
        NormalizeVector(normalized_first.axis + normalized_second.axis);
    const cv::Point2f lateral_axis = OrientLateralAxis(PerpendicularOf(average_axis));
    if (LengthOf(lateral_axis) <= kEpsilon) {
        return std::nullopt;
    }

    const float signed_distance =
        ProjectionDistance(normalized_second.center - normalized_first.center, lateral_axis);
    const float center_distance = std::fabs(signed_distance);
    const float center_distance_ratio = center_distance / average_length;
    if (center_distance_ratio < config.min_center_distance_ratio ||
        center_distance_ratio > config.max_center_distance_ratio) {
        return std::nullopt;
    }

    const float overlap_ratio =
        OverlapRatioAlongAxis(normalized_first, normalized_second, lateral_axis);
    if (overlap_ratio > config.max_overlap_ratio) {
        return std::nullopt;
    }

    LightPairCandidate pair;
    const bool second_is_right = signed_distance >= 0.0F;
    pair.left = second_is_right ? normalized_first : normalized_second;
    pair.right = second_is_right ? normalized_second : normalized_first;
    pair.midpoint = (pair.left.center + pair.right.center) * 0.5F;
    pair.direction = NormalizeVector(pair.right.center - pair.left.center);
    pair.average_axis = average_axis.y < 0.0F ? average_axis * -1.0F : average_axis;
    pair.angle_delta_degrees = angle_delta;
    pair.length_delta_ratio = length_delta_ratio;
    pair.center_y_delta_ratio = center_y_delta_ratio;
    pair.center_distance_ratio = center_distance_ratio;
    pair.overlap_ratio = overlap_ratio;
    return pair;
}

std::optional<TripletCandidate> TryBuildTriplet(
    const GuideCandidate& guide,
    const LightPairCandidate& pair,
    const TripletConstraintConfig& config) {
    const float average_length =
        std::max(kEpsilon, (pair.left.length + pair.right.length) * 0.5F);
    const cv::Point2f lateral_axis = pair.direction;
    const cv::Point2f vertical_axis = pair.average_axis;

    const cv::Point2f offset = pair.midpoint - guide.center;
    const float horizontal_offset_ratio =
        std::fabs(ProjectionDistance(offset, lateral_axis)) / average_length;
    if (horizontal_offset_ratio > config.max_guide_midpoint_x_offset_ratio) {
        return std::nullopt;
    }

    const float signed_vertical_offset =
        ProjectionDistance(offset, vertical_axis);
    const float vertical_offset_ratio = signed_vertical_offset / average_length;
    if (vertical_offset_ratio < config.min_guide_midpoint_y_offset_ratio ||
        vertical_offset_ratio > config.max_guide_midpoint_y_offset_ratio) {
        return std::nullopt;
    }

    const float guide_radius_ratio = guide.radius / average_length;
    if (guide_radius_ratio < config.min_guide_radius_to_light_length_ratio ||
        guide_radius_ratio > config.max_guide_radius_to_light_length_ratio) {
        return std::nullopt;
    }

    TripletCandidate triplet;
    triplet.guide = guide;
    triplet.pair = pair;
    triplet.color = pair.left.color;
    triplet.guide_horizontal_offset_ratio = horizontal_offset_ratio;
    triplet.guide_vertical_offset_ratio = vertical_offset_ratio;
    triplet.guide_radius_ratio = guide_radius_ratio;
    triplet.penalty = ComputeTripletPenalty(guide, pair, config);
    triplet.support =
        guide.mask_support + pair.left.mask_support + pair.right.mask_support;
    return triplet;
}

float ComputeTripletPenalty(
    const GuideCandidate& guide,
    const LightPairCandidate& pair,
    const TripletConstraintConfig& config) {
    const float average_length =
        std::max(kEpsilon, (pair.left.length + pair.right.length) * 0.5F);
    const cv::Point2f offset = pair.midpoint - guide.center;
    const float horizontal_offset_ratio =
        std::fabs(ProjectionDistance(offset, pair.direction)) / average_length;
    const float vertical_offset_ratio =
        ProjectionDistance(offset, pair.average_axis) / average_length;
    const float guide_radius_ratio = guide.radius / average_length;

    const float circularity_penalty = Clamp01(1.0F - guide.circularity);
    const float length_penalty = Clamp01(pair.length_delta_ratio);
    const float angle_penalty = NormalizedPenaltyComponent(
        pair.angle_delta_degrees, config.max_guide_midpoint_y_offset_ratio * 10.0F);
    const float center_y_penalty = Clamp01(pair.center_y_delta_ratio);

    const float desired_vertical_center =
        (config.min_guide_midpoint_y_offset_ratio + config.max_guide_midpoint_y_offset_ratio) * 0.5F;
    const float vertical_penalty = NormalizedPenaltyComponent(
        std::fabs(vertical_offset_ratio - desired_vertical_center),
        std::max(
            std::fabs(desired_vertical_center - config.min_guide_midpoint_y_offset_ratio),
            std::fabs(config.max_guide_midpoint_y_offset_ratio - desired_vertical_center)));
    const float horizontal_penalty = NormalizedPenaltyComponent(
        horizontal_offset_ratio, std::max(config.max_guide_midpoint_x_offset_ratio, kEpsilon));

    const float desired_radius =
        (config.min_guide_radius_to_light_length_ratio +
         config.max_guide_radius_to_light_length_ratio) * 0.5F;
    const float radius_penalty = NormalizedPenaltyComponent(
        std::fabs(guide_radius_ratio - desired_radius),
        std::max(
            std::fabs(desired_radius - config.min_guide_radius_to_light_length_ratio),
            std::fabs(config.max_guide_radius_to_light_length_ratio - desired_radius)));

    return (circularity_penalty + length_penalty + angle_penalty + center_y_penalty +
            vertical_penalty + horizontal_penalty + radius_penalty) /
        7.0F;
}

}  // namespace hero_lob::identifier_geometry
