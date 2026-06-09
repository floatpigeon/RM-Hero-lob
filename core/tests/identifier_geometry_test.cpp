#include <cmath>
#include <iostream>
#include <optional>

#include "identifier_geometry.hpp"

namespace {

using hero_lob::PairConstraintConfig;
using hero_lob::TargetColor;
using hero_lob::TripletConstraintConfig;
using hero_lob::identifier_geometry::GuideCandidate;
using hero_lob::identifier_geometry::LightBarCandidate;
using hero_lob::identifier_geometry::TryBuildLightPair;
using hero_lob::identifier_geometry::TryBuildTriplet;

bool NearlyEqual(float first, float second, float epsilon = 1e-3F) {
    return std::fabs(first - second) <= epsilon;
}

hero_lob::identifier_geometry::LightBarCandidate MakeLight(
    const cv::Point2f& center,
    const cv::Point2f& top,
    const cv::Point2f& bottom,
    TargetColor color,
    float length = 10.0F) {
    LightBarCandidate light;
    light.center = center;
    light.top = top;
    light.bottom = bottom;
    light.axis = hero_lob::identifier_geometry::NormalizeVector(bottom - top);
    light.length = length;
    light.width = 2.0F;
    light.color = color;
    light.mask_support = 80.0F;
    return light;
}

bool TestEndpointSorting() {
    const auto sorted = hero_lob::identifier_geometry::SortEndpointsTopBottom(
        cv::Point2f(4.0F, 12.0F), cv::Point2f(8.0F, 2.0F));
    return NearlyEqual(sorted[0].x, 8.0F) && NearlyEqual(sorted[0].y, 2.0F) &&
        NearlyEqual(sorted[1].x, 4.0F) && NearlyEqual(sorted[1].y, 12.0F);
}

bool TestPairSortingAndDirection() {
    PairConstraintConfig config;
    const LightBarCandidate right_candidate = MakeLight(
        cv::Point2f(16.0F, 20.0F),
        cv::Point2f(16.0F, 15.0F),
        cv::Point2f(16.0F, 25.0F),
        TargetColor::kRed);
    const LightBarCandidate left_candidate = MakeLight(
        cv::Point2f(6.0F, 20.0F),
        cv::Point2f(6.0F, 15.0F),
        cv::Point2f(6.0F, 25.0F),
        TargetColor::kRed);

    const std::optional<hero_lob::identifier_geometry::LightPairCandidate> pair =
        TryBuildLightPair(right_candidate, left_candidate, config);
    if (!pair.has_value()) {
        return false;
    }

    return pair->left.center.x < pair->right.center.x &&
        NearlyEqual(pair->direction.x, 1.0F) &&
        NearlyEqual(pair->direction.y, 0.0F);
}

bool TestIllegalPairRejected() {
    PairConstraintConfig config;
    config.max_angle_difference_degrees = 5.0F;

    LightBarCandidate first = MakeLight(
        cv::Point2f(10.0F, 20.0F),
        cv::Point2f(10.0F, 12.0F),
        cv::Point2f(10.0F, 28.0F),
        TargetColor::kBlue,
        16.0F);
    LightBarCandidate second = MakeLight(
        cv::Point2f(24.0F, 20.0F),
        cv::Point2f(20.0F, 12.0F),
        cv::Point2f(28.0F, 28.0F),
        TargetColor::kBlue,
        17.9F);

    return !TryBuildLightPair(first, second, config).has_value();
}

bool TestPenaltyOrderingStable() {
    PairConstraintConfig pair_config;
    TripletConstraintConfig triplet_config;

    const LightBarCandidate left = MakeLight(
        cv::Point2f(10.0F, 30.0F),
        cv::Point2f(10.0F, 20.0F),
        cv::Point2f(10.0F, 40.0F),
        TargetColor::kRed,
        20.0F);
    const LightBarCandidate right = MakeLight(
        cv::Point2f(22.0F, 30.0F),
        cv::Point2f(22.0F, 20.0F),
        cv::Point2f(22.0F, 40.0F),
        TargetColor::kRed,
        20.0F);
    const auto pair = TryBuildLightPair(left, right, pair_config);
    if (!pair.has_value()) {
        return false;
    }

    GuideCandidate better_guide;
    better_guide.center = cv::Point2f(16.0F, 16.0F);
    better_guide.radius = 5.0F;
    better_guide.circularity = 0.95F;
    better_guide.mask_support = 60.0F;

    GuideCandidate worse_guide = better_guide;
    worse_guide.center = cv::Point2f(19.0F, 10.0F);
    worse_guide.circularity = 0.72F;

    const auto better_triplet = TryBuildTriplet(better_guide, *pair, triplet_config);
    const auto worse_triplet = TryBuildTriplet(worse_guide, *pair, triplet_config);
    if (!better_triplet.has_value() || !worse_triplet.has_value()) {
        return false;
    }

    return better_triplet->penalty < worse_triplet->penalty;
}

}  // namespace

int main() {
    int failed = 0;

    if (!TestEndpointSorting()) {
        std::cerr << "TestEndpointSorting failed\n";
        ++failed;
    }
    if (!TestPairSortingAndDirection()) {
        std::cerr << "TestPairSortingAndDirection failed\n";
        ++failed;
    }
    if (!TestIllegalPairRejected()) {
        std::cerr << "TestIllegalPairRejected failed\n";
        ++failed;
    }
    if (!TestPenaltyOrderingStable()) {
        std::cerr << "TestPenaltyOrderingStable failed\n";
        ++failed;
    }

    if (failed > 0) {
        return 1;
    }

    std::cout << "identifier_geometry_test passed\n";
    return 0;
}
