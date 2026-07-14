#include "identifier_geometry.hpp"

namespace hero_lob::identifier_geometry {

cv::Point2f NormalizeVector(const cv::Point2f& vector) { return vector; }

std::array<cv::Point2f, 2> SortEndpointsTopBottom(
    const cv::Point2f& first, const cv::Point2f& second) {
    return {first, second};
}

LightBarCandidate CanonicalizeLightBar(const LightBarCandidate& candidate) { return candidate; }

std::optional<LightPairCandidate> TryBuildLightPair(
    const LightBarCandidate& first, const LightBarCandidate& second,
    const PairConstraintConfig& config) {
    return std::nullopt;
}

std::optional<TripletCandidate> TryBuildTriplet(
    const GuideCandidate& guide, const LightPairCandidate& pair,
    const TripletConstraintConfig& config) {
    return std::nullopt;
}

float ComputeTripletPenalty(
    const GuideCandidate& guide, const LightPairCandidate& pair,
    const TripletConstraintConfig& config) {
    return 0.0F;
}

}  // namespace hero_lob::identifier_geometry
