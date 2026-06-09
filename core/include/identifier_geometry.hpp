#pragma once

#include <array>
#include <optional>

#include <opencv2/core.hpp>

#include "types.hpp"

namespace hero_lob::identifier_geometry {

struct GuideCandidate {
    cv::Point2f center = {};
    float radius = 0.0F;
    float circularity = 0.0F;
    float color_advantage = 0.0F;
    float mask_support = 0.0F;
};

struct LightBarCandidate {
    TargetColor color = TargetColor::kUnknown;
    cv::Point2f center = {};
    cv::Point2f top = {};
    cv::Point2f bottom = {};
    cv::Point2f axis = {0.0F, 1.0F};
    cv::RotatedRect box = {};
    float length = 0.0F;
    float width = 0.0F;
    float angle_degrees = 0.0F;
    float fill_ratio = 0.0F;
    float color_advantage = 0.0F;
    float mask_support = 0.0F;
};

struct LightPairCandidate {
    LightBarCandidate left = {};
    LightBarCandidate right = {};
    cv::Point2f midpoint = {};
    cv::Point2f direction = {1.0F, 0.0F};
    cv::Point2f average_axis = {0.0F, 1.0F};
    float angle_delta_degrees = 0.0F;
    float length_delta_ratio = 0.0F;
    float center_y_delta_ratio = 0.0F;
    float center_distance_ratio = 0.0F;
    float overlap_ratio = 0.0F;
};

struct TripletCandidate {
    GuideCandidate guide = {};
    LightPairCandidate pair = {};
    TargetColor color = TargetColor::kUnknown;
    float guide_horizontal_offset_ratio = 0.0F;
    float guide_vertical_offset_ratio = 0.0F;
    float guide_radius_ratio = 0.0F;
    float penalty = 0.0F;
    float support = 0.0F;
};

cv::Point2f NormalizeVector(const cv::Point2f& vector);
std::array<cv::Point2f, 2> SortEndpointsTopBottom(
    const cv::Point2f& first, const cv::Point2f& second);
LightBarCandidate CanonicalizeLightBar(const LightBarCandidate& candidate);

std::optional<LightPairCandidate> TryBuildLightPair(
    const LightBarCandidate& first,
    const LightBarCandidate& second,
    const PairConstraintConfig& config);

std::optional<TripletCandidate> TryBuildTriplet(
    const GuideCandidate& guide,
    const LightPairCandidate& pair,
    const TripletConstraintConfig& config);

float ComputeTripletPenalty(
    const GuideCandidate& guide,
    const LightPairCandidate& pair,
    const TripletConstraintConfig& config);

}  // namespace hero_lob::identifier_geometry
