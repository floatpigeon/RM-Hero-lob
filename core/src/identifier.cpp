#include "identifier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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
using identifier_geometry::NormalizeVector;

constexpr float kEpsilon = 1e-4F;
constexpr float kPi = 3.14159265358979323846F;

struct StablePairCandidate {
    LightBarCandidate left = {};
    LightBarCandidate right = {};
    cv::Point2f midpoint = {};
    cv::Point2f direction = {1.0F, 0.0F};
    float angle_delta_degrees = 0.0F;
    float length_delta_ratio = 0.0F;
    float center_y_delta_ratio = 0.0F;
    float distance_symmetry_ratio = 0.0F;
    float midpoint_x_offset_ratio = 0.0F;
    float midpoint_y_offset_ratio = 0.0F;
    float center_distance_ratio = 0.0F;
    float score = 0.0F;
};

struct StablePairDetection {
    StablePairCandidate pair = {};
    cv::Rect roi = {};
    bool valid = false;
};

cv::Scalar MaskColorFor(TargetColor color) {
    switch (color) {
        case TargetColor::kRed: return {0, 0, 255};
        case TargetColor::kBlue: return {255, 0, 0};
        case TargetColor::kUnknown: break;
    }
    return {200, 200, 200};
}

float LengthOf(const cv::Point2f& point) {
    return std::sqrt(point.x * point.x + point.y * point.y);
}

float AngleDegreesBetweenAxes(const cv::Point2f& first, const cv::Point2f& second) {
    const cv::Point2f normalized_first = NormalizeVector(first);
    const cv::Point2f normalized_second = NormalizeVector(second);
    float dot = normalized_first.dot(normalized_second);
    dot = std::clamp(dot, -1.0F, 1.0F);
    return std::acos(std::fabs(dot)) * 180.0F / kPi;
}

float AngleDegreesOf(const cv::Point2f& axis) { return std::atan2(axis.y, axis.x) * 180.0F / kPi; }

cv::Mat MakeBrightnessMask(const cv::Mat& hsv, const BrightnessThresholdConfig& config) {
    cv::Mat mask;
    cv::inRange(
        hsv, cv::Scalar(0, 0, std::clamp(config.min_value, 0, 255)),
        cv::Scalar(179, 255, 255), mask);
    return mask;
}

cv::Mat MakeColorMask(
    const cv::Mat& hsv, const HsvRangeConfig& range, const EdgeColorThresholdConfig& config) {
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

cv::Mat BuildContourMask(const cv::Size& size, const std::vector<cv::Point>& contour) {
    cv::Mat contour_mask = cv::Mat::zeros(size, CV_8UC1);
    std::vector<std::vector<cv::Point>> contours = {contour};
    cv::drawContours(contour_mask, contours, 0, cv::Scalar(255), cv::FILLED);
    return contour_mask;
}

TargetColor ResolveEdgeColor(
    const cv::Size& size,
    const std::vector<cv::Point>& contour,
    const cv::Mat& red_mask,
    const cv::Mat& blue_mask,
    const EdgeColorThresholdConfig& config) {
    const cv::Mat contour_mask = BuildContourMask(size, contour);
    const int kernel_size = std::max(1, config.edge_band_kernel_size);
    const cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));

    cv::Mat dilated;
    cv::Mat eroded;
    cv::dilate(contour_mask, dilated, kernel);
    cv::erode(contour_mask, eroded, kernel);

    cv::Mat edge_band;
    cv::subtract(dilated, eroded, edge_band);

    cv::Mat red_samples;
    cv::Mat blue_samples;
    cv::bitwise_and(edge_band, red_mask, red_samples);
    cv::bitwise_and(edge_band, blue_mask, blue_samples);

    const int red_pixels = cv::countNonZero(red_samples);
    const int blue_pixels = cv::countNonZero(blue_samples);
    const bool red_valid = red_pixels >= config.min_edge_pixels &&
        static_cast<float>(red_pixels) >= static_cast<float>(std::max(1, blue_pixels)) *
            config.min_color_ratio;
    const bool blue_valid = blue_pixels >= config.min_edge_pixels &&
        static_cast<float>(blue_pixels) >= static_cast<float>(std::max(1, red_pixels)) *
            config.min_color_ratio;

    if (red_valid == blue_valid) {
        return TargetColor::kUnknown;
    }
    return red_valid ? TargetColor::kRed : TargetColor::kBlue;
}

std::optional<GuideCandidate> BuildGuideCandidate(
    const cv::Size& frame_size,
    const std::vector<cv::Point>& contour,
    const GuideConstraintConfig& config) {
    const double area = cv::contourArea(contour);
    if (area <= 0.0) {
        return std::nullopt;
    }

    const float frame_area = static_cast<float>(frame_size.area());
    const float area_ratio = static_cast<float>(area / frame_area);
    if (area_ratio < config.min_area_ratio || area_ratio > config.max_area_ratio) {
        return std::nullopt;
    }

    const cv::Rect bbox = cv::boundingRect(contour);
    const float aspect_ratio =
        bbox.height > 0 ? static_cast<float>(bbox.width) / static_cast<float>(bbox.height) : 0.0F;
    if (std::fabs(aspect_ratio - 1.0F) > config.max_aspect_ratio_deviation) {
        return std::nullopt;
    }

    const float circularity = ComputeCircularity(contour, area);
    if (circularity < config.min_circularity) {
        return std::nullopt;
    }

    cv::Point2f center;
    float radius = 0.0F;
    cv::minEnclosingCircle(contour, center, radius);

    GuideCandidate candidate;
    candidate.center = center;
    candidate.radius = radius;
    candidate.circularity = circularity;
    candidate.mask_support = static_cast<float>(area);
    return candidate;
}

std::optional<LightBarCandidate> BuildGeneralLightCandidate(
    const cv::Size& frame_size,
    const std::vector<cv::Point>& contour,
    const cv::Mat& red_mask,
    const cv::Mat& blue_mask,
    const IdentifierConfig& config) {
    const double area = cv::contourArea(contour);
    if (area <= 0.0) {
        return std::nullopt;
    }

    const cv::RotatedRect box = cv::minAreaRect(contour);
    const float rect_width = std::max(box.size.width, box.size.height);
    const float rect_height = std::min(box.size.width, box.size.height);
    if (rect_height <= 0.0F || rect_width <= 0.0F) {
        return std::nullopt;
    }

    const float max_dimension = static_cast<float>(std::max(frame_size.width, frame_size.height));
    const float length_ratio = rect_width / max_dimension;
    const float width_ratio = rect_height / max_dimension;
    if (length_ratio < config.light.min_length_ratio ||
        length_ratio > config.light.max_length_ratio) {
        return std::nullopt;
    }
    if (width_ratio < config.light.min_width_ratio ||
        width_ratio > config.light.max_width_ratio) {
        return std::nullopt;
    }

    const float aspect_ratio = rect_width / rect_height;
    if (aspect_ratio < config.light.min_aspect_ratio ||
        aspect_ratio > config.light.max_aspect_ratio) {
        return std::nullopt;
    }

    const float fill_ratio = static_cast<float>(area / (rect_width * rect_height));
    if (fill_ratio < config.light.min_fill_ratio) {
        return std::nullopt;
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
    candidate.color = ResolveEdgeColor(frame_size, contour, red_mask, blue_mask, config.edge_color);
    candidate.center = box.center;
    candidate.top = endpoints[0];
    candidate.bottom = endpoints[1];
    candidate.axis = NormalizeVector(candidate.bottom - candidate.top);
    candidate.box = box;
    candidate.length = rect_width;
    candidate.width = rect_height;
    candidate.angle_degrees = AngleDegreesOf(candidate.axis);
    candidate.fill_ratio = fill_ratio;
    candidate.mask_support = static_cast<float>(area);
    return identifier_geometry::CanonicalizeLightBar(candidate);
}

cv::Rect BuildStablePairRoi(
    const cv::Size& frame_size,
    const GuideCandidate& guide,
    const StablePairRoiConfig& config) {
    const float half_width = std::max(10.0F, guide.radius * config.half_width_radius_scale);
    const float top = guide.center.y + guide.radius * config.top_offset_radius_scale;
    const float bottom = guide.center.y + guide.radius * config.bottom_offset_radius_scale;

    const int x0 = std::max(0, static_cast<int>(std::floor(guide.center.x - half_width)));
    const int y0 = std::max(0, static_cast<int>(std::floor(top)));
    const int x1 = std::min(frame_size.width, static_cast<int>(std::ceil(guide.center.x + half_width)));
    const int y1 = std::min(frame_size.height, static_cast<int>(std::ceil(bottom)));
    if (x1 <= x0 || y1 <= y0) {
        return {};
    }
    return cv::Rect(x0, y0, x1 - x0, y1 - y0);
}

std::optional<LightBarCandidate> BuildStableLightCandidate(
    const cv::Size& full_frame_size,
    const std::vector<cv::Point>& contour,
    const cv::Mat& red_mask,
    const cv::Mat& blue_mask,
    const cv::Rect& roi,
    const GuideCandidate& guide,
    const StableLightConstraintConfig& config,
    const EdgeColorThresholdConfig& edge_color) {
    const double area = cv::contourArea(contour);
    if (area <= 0.0) {
        return std::nullopt;
    }

    const cv::Rect local_bbox = cv::boundingRect(contour);
    const float rect_length = static_cast<float>(local_bbox.height);
    const float rect_width = static_cast<float>(local_bbox.width);
    if (rect_length <= 0.0F || rect_width <= 0.0F) {
        return std::nullopt;
    }

    const float roi_height = static_cast<float>(std::max(roi.height, 1));
    const float roi_width = static_cast<float>(std::max(roi.width, 1));
    const float length_ratio = rect_length / roi_height;
    const float width_ratio = rect_width / roi_width;
    if (length_ratio < config.min_length_ratio_to_roi_height ||
        length_ratio > config.max_length_ratio_to_roi_height) {
        return std::nullopt;
    }
    if (width_ratio < config.min_width_ratio_to_roi_width ||
        width_ratio > config.max_width_ratio_to_roi_width) {
        return std::nullopt;
    }

    const float aspect_ratio = rect_length / rect_width;
    if (aspect_ratio < config.min_aspect_ratio || aspect_ratio > config.max_aspect_ratio) {
        return std::nullopt;
    }

    const float fill_ratio = static_cast<float>(area / (rect_length * rect_width));
    if (fill_ratio < config.min_fill_ratio) {
        return std::nullopt;
    }

    std::vector<cv::Point> full_contour;
    full_contour.reserve(contour.size());
    for (const cv::Point& point : contour) {
        full_contour.emplace_back(point.x + roi.x, point.y + roi.y);
    }

    const float center_x = roi.x + static_cast<float>(local_bbox.x) + rect_width * 0.5F;
    const float center_y = roi.y + static_cast<float>(local_bbox.y) + rect_length * 0.5F;
    const cv::Point2f global_center(center_x, center_y);
    const cv::Point2f axis(0.0F, 1.0F);
    const cv::Point2f top_endpoint(
        center_x,
        roi.y + static_cast<float>(local_bbox.y));
    const cv::Point2f bottom_endpoint(
        center_x,
        roi.y + static_cast<float>(local_bbox.y + local_bbox.height - 1));

    LightBarCandidate candidate;
    candidate.color = ResolveEdgeColor(full_frame_size, full_contour, red_mask, blue_mask, edge_color);
    candidate.center = global_center;
    candidate.top = top_endpoint;
    candidate.bottom = bottom_endpoint;
    candidate.axis = axis;
    candidate.box = cv::RotatedRect(global_center, cv::Size2f(rect_width, rect_length), 90.0F);
    candidate.length = rect_length;
    candidate.width = rect_width;
    candidate.angle_degrees = 90.0F;
    candidate.fill_ratio = fill_ratio;
    candidate.mask_support = static_cast<float>(area);
    candidate.center = (candidate.top + candidate.bottom) * 0.5F;
    candidate.length = std::max(candidate.length, LengthOf(candidate.bottom - candidate.top));

    const float center_y_offset = candidate.center.y - guide.center.y;
    if (center_y_offset < guide.radius * config.min_center_y_offset_radius_scale ||
        center_y_offset > guide.radius * config.max_center_y_offset_radius_scale) {
        return std::nullopt;
    }

    const float center_x_offset = std::fabs(candidate.center.x - guide.center.x);
    if (center_x_offset < guide.radius * config.min_center_x_offset_radius_scale ||
        center_x_offset > guide.radius * config.max_center_x_offset_radius_scale) {
        return std::nullopt;
    }

    const float normalized_area = static_cast<float>(area) / std::max(kEpsilon, guide.radius * guide.radius);
    if (normalized_area < config.min_area_radius_scale_squared ||
        normalized_area > config.max_area_radius_scale_squared) {
        return std::nullopt;
    }

    return candidate;
}

cv::Mat BuildGuideExclusionMask(
    const cv::Size& size,
    const GuideCandidate& guide,
    const StableLightConstraintConfig& config) {
    cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
    const int radius = std::max(
        1, static_cast<int>(std::lround(guide.radius * config.guide_exclusion_radius_scale)));
    cv::circle(mask, guide.center, radius, cv::Scalar(255), cv::FILLED, cv::LINE_8);
    return mask;
}

std::vector<int> FindSplitIndices(
    const std::vector<int>& column_sums, const StablePairFallbackConfig& config) {
    std::vector<int> peaks;
    for (int index = 1; index + 1 < static_cast<int>(column_sums.size()); ++index) {
        if (column_sums[index] >= column_sums[index - 1] &&
            column_sums[index] >= column_sums[index + 1] &&
            column_sums[index] > 0) {
            peaks.push_back(index);
        }
    }
    if (peaks.size() < 2) {
        return {};
    }

    int best_left = -1;
    int best_right = -1;
    int best_sum = -1;
    for (std::size_t left_index = 0; left_index < peaks.size(); ++left_index) {
        for (std::size_t right_index = left_index + 1; right_index < peaks.size(); ++right_index) {
            const int left_peak = peaks[left_index];
            const int right_peak = peaks[right_index];
            if (right_peak - left_peak < config.min_peak_distance_pixels) {
                continue;
            }
            const int peak_sum = column_sums[left_peak] + column_sums[right_peak];
            if (peak_sum > best_sum) {
                best_sum = peak_sum;
                best_left = left_peak;
                best_right = right_peak;
            }
        }
    }
    if (best_left < 0 || best_right < 0) {
        return {};
    }

    int valley_index = best_left;
    int valley_value = column_sums[best_left];
    for (int index = best_left + 1; index < best_right; ++index) {
        if (column_sums[index] < valley_value) {
            valley_value = column_sums[index];
            valley_index = index;
        }
    }

    const int left_peak_value = column_sums[best_left];
    const int right_peak_value = column_sums[best_right];
    const float valley_ratio = static_cast<float>(valley_value) /
        static_cast<float>(std::max(1, std::min(left_peak_value, right_peak_value)));
    if (valley_ratio > config.min_valley_ratio) {
        return {};
    }
    return {valley_index};
}

std::vector<std::vector<cv::Point>> SplitWideContour(
    const std::vector<cv::Point>& contour,
    const cv::Rect& roi,
    const StablePairFallbackConfig& config) {
    const cv::Rect local_bbox = cv::boundingRect(contour);
    const float width_ratio = static_cast<float>(local_bbox.width) /
        static_cast<float>(std::max(roi.width, 1));
    if (width_ratio < config.split_width_ratio_to_roi_width) {
        return {contour};
    }

    cv::Mat contour_mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contour_group = {contour};
    cv::drawContours(contour_mask, contour_group, 0, cv::Scalar(255), cv::FILLED, cv::LINE_8);

    std::vector<int> column_sums(local_bbox.width, 0);
    for (int dx = 0; dx < local_bbox.width; ++dx) {
        const cv::Rect column_rect(local_bbox.x + dx, local_bbox.y, 1, local_bbox.height);
        column_sums[dx] = cv::countNonZero(contour_mask(column_rect));
    }

    const std::vector<int> split_indices = FindSplitIndices(column_sums, config);
    if (split_indices.empty()) {
        return {contour};
    }

    const int split_x = local_bbox.x + split_indices.front();
    cv::Mat left_mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    cv::Mat right_mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    for (int y = local_bbox.y; y < local_bbox.y + local_bbox.height; ++y) {
        for (int x = local_bbox.x; x < local_bbox.x + local_bbox.width; ++x) {
            if (contour_mask.at<std::uint8_t>(y, x) == 0) {
                continue;
            }
            if (x <= split_x) {
                left_mask.at<std::uint8_t>(y, x) = 255;
            } else {
                right_mask.at<std::uint8_t>(y, x) = 255;
            }
        }
    }

    std::vector<std::vector<cv::Point>> left_contours = FindExternalContours(left_mask);
    std::vector<std::vector<cv::Point>> right_contours = FindExternalContours(right_mask);
    if (left_contours.empty() || right_contours.empty()) {
        return {contour};
    }

    auto contour_area_desc = [](const std::vector<cv::Point>& first, const std::vector<cv::Point>& second) {
        return cv::contourArea(first) > cv::contourArea(second);
    };
    std::sort(left_contours.begin(), left_contours.end(), contour_area_desc);
    std::sort(right_contours.begin(), right_contours.end(), contour_area_desc);
    return {left_contours.front(), right_contours.front()};
}

std::vector<LightBarCandidate> ExtractStableLightCandidates(
    const FrameData& frame,
    const GuideCandidate& guide,
    const cv::Mat& red_mask,
    const cv::Mat& blue_mask,
    const IdentifierConfig& config,
    cv::Mat& stable_pair_roi_mask,
    cv::Mat& light_candidate_mask) {
    const cv::Rect roi = BuildStablePairRoi(frame.bgr.size(), guide, config.stable_pair_roi);
    stable_pair_roi_mask = cv::Mat::zeros(frame.bgr.size(), CV_8UC1);
    if (roi.width <= 0 || roi.height <= 0) {
        return {};
    }

    stable_pair_roi_mask(roi).setTo(255);

    cv::Mat stable_mask;
    cv::inRange(
        frame.hsv(roi),
        cv::Scalar(0, 0, std::clamp(config.stable_light.local_min_value, 0, 255)),
        cv::Scalar(179, 255, 255),
        stable_mask);
    const cv::Mat guide_exclusion_full = BuildGuideExclusionMask(frame.bgr.size(), guide, config.stable_light);
    const cv::Mat guide_exclusion = guide_exclusion_full(roi);
    stable_mask.setTo(0, guide_exclusion);

    const int center_band_half_width = std::max(
        1, static_cast<int>(std::lround(guide.radius * config.stable_light.center_exclusion_half_width_radius_scale)));
    const int center_band_x0 =
        std::max(0, static_cast<int>(std::floor(guide.center.x)) - roi.x - center_band_half_width);
    const int center_band_x1 =
        std::min(roi.width, static_cast<int>(std::ceil(guide.center.x)) - roi.x + center_band_half_width);
    if (center_band_x1 > center_band_x0) {
        stable_mask(cv::Rect(center_band_x0, 0, center_band_x1 - center_band_x0, roi.height)).setTo(0);
    }

    const cv::Mat vertical_open_kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(
            std::max(1, config.stable_light.vertical_open_kernel_width),
            std::max(1, config.stable_light.vertical_open_kernel_height)));
    const cv::Mat vertical_close_kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(
            std::max(1, config.stable_light.vertical_close_kernel_width),
            std::max(1, config.stable_light.vertical_close_kernel_height)));
    cv::morphologyEx(stable_mask, stable_mask, cv::MORPH_OPEN, vertical_open_kernel);
    cv::morphologyEx(stable_mask, stable_mask, cv::MORPH_CLOSE, vertical_close_kernel);

    std::vector<LightBarCandidate> candidates;
    for (const auto& contour : FindExternalContours(stable_mask)) {
        const std::vector<std::vector<cv::Point>> split_contours =
            SplitWideContour(contour, roi, config.stable_pair_fallback);
        for (const auto& split_contour : split_contours) {
        const std::optional<LightBarCandidate> light = BuildStableLightCandidate(
            frame.bgr.size(), split_contour, red_mask, blue_mask, roi, guide, config.stable_light,
            config.edge_color);
            if (!light.has_value()) {
                continue;
            }
            candidates.push_back(*light);

            std::vector<cv::Point> global_contour;
            global_contour.reserve(split_contour.size());
            for (const cv::Point& point : split_contour) {
                global_contour.emplace_back(point.x + roi.x, point.y + roi.y);
            }
            std::vector<std::vector<cv::Point>> contour_group = {global_contour};
            cv::drawContours(
                light_candidate_mask, contour_group, 0, cv::Scalar(255), cv::FILLED, cv::LINE_8);
        }
    }
    return candidates;
}

TargetColor ResolvePairColor(const StablePairCandidate& pair) {
    if (pair.left.color == pair.right.color) {
        return pair.left.color;
    }
    if (pair.left.color == TargetColor::kUnknown) {
        return pair.right.color;
    }
    if (pair.right.color == TargetColor::kUnknown) {
        return pair.left.color;
    }
    return TargetColor::kUnknown;
}

std::optional<StablePairCandidate> SelectStablePair(
    const GuideCandidate& guide,
    const std::vector<LightBarCandidate>& candidates,
    const StablePairConstraintConfig& config) {
    const float radius_scale = std::max(kEpsilon, guide.radius);

    auto side_score = [&](const LightBarCandidate& candidate) {
        const float x_offset_ratio = std::fabs(candidate.center.x - guide.center.x) / radius_scale;
        const float y_offset_ratio = std::fabs(candidate.center.y - guide.center.y) / radius_scale;
        const float desired_x_offset_ratio = 2.0F;
        const float desired_y_offset_ratio = 2.3F;
        return 50.0F
            - std::fabs(x_offset_ratio - desired_x_offset_ratio) * 8.0F
            - std::fabs(y_offset_ratio - desired_y_offset_ratio) * 10.0F
            + candidate.mask_support * 0.1F;
    };

    std::optional<LightBarCandidate> best_left;
    std::optional<LightBarCandidate> best_right;
    float best_left_score = std::numeric_limits<float>::lowest();
    float best_right_score = std::numeric_limits<float>::lowest();

    for (const LightBarCandidate& candidate : candidates) {
        const float score = side_score(candidate);
        if (candidate.center.x < guide.center.x) {
            if (!best_left.has_value() || score > best_left_score) {
                best_left = candidate;
                best_left_score = score;
            }
            continue;
        }
        if (candidate.center.x > guide.center.x) {
            if (!best_right.has_value() || score > best_right_score) {
                best_right = candidate;
                best_right_score = score;
            }
        }
    }

    if (!best_left.has_value() || !best_right.has_value()) {
        return std::nullopt;
    }

    const LightBarCandidate& left = *best_left;
    const LightBarCandidate& right = *best_right;
    const float average_length = std::max(kEpsilon, (left.length + right.length) * 0.5F);
    const float angle_delta = AngleDegreesBetweenAxes(left.axis, right.axis);
    if (angle_delta > config.max_angle_difference_degrees) {
        return std::nullopt;
    }

    const float length_delta_ratio = std::fabs(left.length - right.length) / average_length;
    if (length_delta_ratio > config.max_length_delta_ratio) {
        return std::nullopt;
    }

    const float center_y_delta = std::fabs(left.center.y - right.center.y);
    if (center_y_delta > guide.radius * config.max_center_y_delta_radius_scale) {
        return std::nullopt;
    }

    const float left_distance = std::fabs(guide.center.x - left.center.x);
    const float right_distance = std::fabs(right.center.x - guide.center.x);
    const float max_distance = std::max(left_distance, right_distance);
    const float distance_symmetry_ratio =
        max_distance <= kEpsilon ? 0.0F : std::fabs(left_distance - right_distance) / max_distance;
    if (distance_symmetry_ratio > config.max_distance_symmetry_ratio) {
        return std::nullopt;
    }

    const cv::Point2f midpoint = (left.center + right.center) * 0.5F;
    const float midpoint_x_offset = std::fabs(midpoint.x - guide.center.x);
    if (midpoint_x_offset > guide.radius * config.max_midpoint_x_offset_radius_scale) {
        return std::nullopt;
    }

    const float midpoint_y_offset = std::fabs(midpoint.y - guide.center.y);
    if (midpoint_y_offset < guide.radius * config.min_midpoint_y_offset_radius_scale ||
        midpoint_y_offset > guide.radius * config.max_midpoint_y_offset_radius_scale) {
        return std::nullopt;
    }

    const float center_distance = LengthOf(right.center - left.center);
    if (center_distance < guide.radius * config.min_center_distance_radius_scale ||
        center_distance > guide.radius * config.max_center_distance_radius_scale) {
        return std::nullopt;
    }

    StablePairCandidate pair;
    pair.left = left;
    pair.right = right;
    pair.midpoint = midpoint;
    pair.direction = NormalizeVector(right.center - left.center);
    pair.angle_delta_degrees = angle_delta;
    pair.length_delta_ratio = length_delta_ratio;
    pair.center_y_delta_ratio = center_y_delta / radius_scale;
    pair.distance_symmetry_ratio = distance_symmetry_ratio;
    pair.midpoint_x_offset_ratio = midpoint_x_offset / radius_scale;
    pair.midpoint_y_offset_ratio = midpoint_y_offset / radius_scale;
    pair.center_distance_ratio = center_distance / radius_scale;
    pair.score =
        100.0F
        - pair.midpoint_x_offset_ratio * 20.0F
        - pair.midpoint_y_offset_ratio * 10.0F
        - pair.center_distance_ratio * 5.0F
        - pair.distance_symmetry_ratio * 20.0F
        - pair.center_y_delta_ratio * 20.0F
        - pair.length_delta_ratio * 20.0F
        - angle_delta * 0.5F
        + (left.mask_support + right.mask_support) * 0.02F;
    return pair;
}

DetectionResult ToDetection(const GuideCandidate& guide, const StablePairCandidate& pair) {
    DetectionResult result;
    result.anchors.guide_center = guide.center;
    result.anchors.left_top = pair.left.top;
    result.anchors.left_bottom = pair.left.bottom;
    result.anchors.right_top = pair.right.top;
    result.anchors.right_bottom = pair.right.bottom;
    result.anchors.direction = pair.direction;
    result.anchors.valid = true;
    result.anchors.placeholder = false;
    result.color = ResolvePairColor(pair);
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
    const cv::Mat& base,
    const std::vector<GuideCandidate>& guides,
    const std::vector<LightBarCandidate>& general_lights,
    const std::vector<LightBarCandidate>& stable_lights) {
    cv::Mat overlay = base.clone();
    for (const GuideCandidate& guide : guides) {
        DrawGuideCandidate(overlay, guide, cv::Scalar(0, 255, 0));
    }
    for (const LightBarCandidate& light : general_lights) {
        DrawLightBarCandidate(overlay, light, cv::Scalar(80, 80, 80), 1);
    }
    for (const LightBarCandidate& light : stable_lights) {
        DrawLightBarCandidate(overlay, light, MaskColorFor(light.color), 2);
    }
    return overlay;
}

cv::Mat BuildStablePairOverlay(
    const cv::Mat& base,
    const GuideCandidate& guide,
    const cv::Rect& roi,
    const std::vector<LightBarCandidate>& stable_lights,
    const std::optional<StablePairCandidate>& best_pair) {
    cv::Mat overlay = base.clone();
    DrawGuideCandidate(overlay, guide, cv::Scalar(0, 255, 0));
    if (roi.width > 0 && roi.height > 0) {
        cv::rectangle(overlay, roi, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
    }
    for (const LightBarCandidate& light : stable_lights) {
        DrawLightBarCandidate(overlay, light, cv::Scalar(255, 255, 0), 2);
    }
    if (best_pair.has_value()) {
        const cv::Scalar color = MaskColorFor(ResolvePairColor(*best_pair));
        DrawLightBarCandidate(overlay, best_pair->left, color, 3);
        DrawLightBarCandidate(overlay, best_pair->right, color, 3);
        cv::line(
            overlay, best_pair->left.center, best_pair->right.center, cv::Scalar(0, 255, 255), 2,
            cv::LINE_AA);
    }
    return overlay;
}

cv::Mat BuildResultOverlay(
    const cv::Mat& base,
    const GuideCandidate& guide,
    const std::optional<StablePairCandidate>& best_pair) {
    cv::Mat overlay = base.clone();
    if (!best_pair.has_value()) {
        return overlay;
    }

    DrawGuideCandidate(overlay, guide, cv::Scalar(0, 255, 0));
    const cv::Scalar color = MaskColorFor(ResolvePairColor(*best_pair));
    DrawLightBarCandidate(overlay, best_pair->left, color, 3);
    DrawLightBarCandidate(overlay, best_pair->right, color, 3);
    cv::line(
        overlay, best_pair->left.center, best_pair->right.center, cv::Scalar(0, 255, 255), 2,
        cv::LINE_AA);
    cv::arrowedLine(
        overlay, best_pair->left.center, best_pair->right.center, cv::Scalar(255, 255, 255), 2,
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
    cv::Mat brightness_mask = MakeBrightnessMask(frame.hsv, config.brightness);
    cv::Mat red_low_mask = MakeColorMask(frame.hsv, config.edge_color.red_low, config.edge_color);
    cv::Mat red_high_mask = MakeColorMask(frame.hsv, config.edge_color.red_high, config.edge_color);
    cv::Mat blue_mask = MakeColorMask(frame.hsv, config.edge_color.blue, config.edge_color);
    cv::Mat red_mask;
    cv::bitwise_or(red_low_mask, red_high_mask, red_mask);

    analysis.debug.raw_brightness_mask = brightness_mask.clone();
    analysis.debug.edge_red_mask = red_mask.clone();
    analysis.debug.edge_blue_mask = blue_mask.clone();

    brightness_mask = ApplyMorphology(brightness_mask, config.morphology);

    std::vector<GuideCandidate> guides;
    std::vector<LightBarCandidate> general_lights;
    cv::Mat guide_candidate_mask = cv::Mat::zeros(brightness_mask.size(), CV_8UC1);
    for (const auto& contour : FindExternalContours(brightness_mask)) {
        const std::optional<GuideCandidate> guide =
            BuildGuideCandidate(frame.bgr.size(), contour, config.guide);
        if (guide.has_value()) {
            guides.push_back(*guide);
            std::vector<std::vector<cv::Point>> contours = {contour};
            cv::drawContours(
                guide_candidate_mask, contours, 0, cv::Scalar(255), cv::FILLED, cv::LINE_8);
            continue;
        }

        const std::optional<LightBarCandidate> light =
            BuildGeneralLightCandidate(frame.bgr.size(), contour, red_mask, blue_mask, config);
        if (light.has_value()) {
            general_lights.push_back(*light);
        }
    }

    cv::Mat light_candidate_mask = cv::Mat::zeros(brightness_mask.size(), CV_8UC1);
    cv::Mat stable_pair_roi_mask = cv::Mat::zeros(brightness_mask.size(), CV_8UC1);
    std::vector<LightBarCandidate> best_stable_lights;
    std::optional<StablePairCandidate> best_pair;
    std::optional<GuideCandidate> best_guide;
    cv::Rect best_roi;
    std::optional<GuideCandidate> fallback_guide;
    cv::Rect fallback_roi;
    std::vector<LightBarCandidate> fallback_stable_lights;
    cv::Mat fallback_light_mask = cv::Mat::zeros(brightness_mask.size(), CV_8UC1);
    cv::Mat fallback_roi_mask = cv::Mat::zeros(brightness_mask.size(), CV_8UC1);

    float best_pair_score = std::numeric_limits<float>::lowest();
    for (const GuideCandidate& guide : guides) {
        cv::Mat roi_mask;
        cv::Mat light_mask = cv::Mat::zeros(brightness_mask.size(), CV_8UC1);
        const std::vector<LightBarCandidate> stable_lights = ExtractStableLightCandidates(
            frame, guide, red_mask, blue_mask, config, roi_mask, light_mask);
        if (!fallback_guide.has_value() || stable_lights.size() > fallback_stable_lights.size()) {
            fallback_guide = guide;
            fallback_stable_lights = stable_lights;
            fallback_light_mask = light_mask.clone();
            fallback_roi_mask = roi_mask.clone();
            fallback_roi = BuildStablePairRoi(frame.bgr.size(), guide, config.stable_pair_roi);
        }
        const std::optional<StablePairCandidate> pair =
            SelectStablePair(guide, stable_lights, config.stable_pair);
        if (!pair.has_value()) {
            continue;
        }

        if (!best_pair.has_value() || pair->score > best_pair_score) {
            best_pair = pair;
            best_guide = guide;
            best_stable_lights = stable_lights;
            best_pair_score = pair->score;
            light_candidate_mask = light_mask;
            stable_pair_roi_mask = roi_mask;
            best_roi = BuildStablePairRoi(frame.bgr.size(), guide, config.stable_pair_roi);
        }
    }

    if (best_pair.has_value() && best_guide.has_value()) {
        analysis.detection = ToDetection(*best_guide, *best_pair);
    }

    analysis.debug.brightness_mask = brightness_mask;
    analysis.debug.guide_candidate_mask = guide_candidate_mask;
    analysis.debug.light_candidate_mask =
        best_pair.has_value() ? light_candidate_mask : fallback_light_mask;
    analysis.debug.stable_pair_roi =
        best_pair.has_value() ? stable_pair_roi_mask : fallback_roi_mask;
    analysis.debug.candidate_overlay = BuildCandidateOverlay(
        frame.bgr, guides, general_lights,
        best_pair.has_value() ? best_stable_lights : fallback_stable_lights);
    if (best_guide.has_value()) {
        analysis.debug.stable_pair_overlay = BuildStablePairOverlay(
            frame.bgr, *best_guide, best_roi, best_stable_lights, best_pair);
        analysis.debug.result_overlay = BuildResultOverlay(frame.bgr, *best_guide, best_pair);
    } else if (fallback_guide.has_value()) {
        analysis.debug.stable_pair_overlay = BuildStablePairOverlay(
            frame.bgr, *fallback_guide, fallback_roi, fallback_stable_lights, std::nullopt);
        analysis.debug.result_overlay = frame.bgr.clone();
    } else {
        analysis.debug.stable_pair_overlay = frame.bgr.clone();
        analysis.debug.result_overlay = frame.bgr.clone();
    }
    return analysis;
}

DetectionResult Identifier::Process(const FrameData& frame) const {
    return Analyze(frame).detection;
}

}  // namespace hero_lob
