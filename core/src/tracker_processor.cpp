#include "tracker_processor.hpp"

#include <algorithm>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace hero_lob {

namespace {

cv::Mat FilterConnectedComponents(
    const cv::Mat& mask,
    int min_component_area_pixels,
    float max_component_area_ratio) {
    if (mask.empty()) {
        return cv::Mat();
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int component_count = cv::connectedComponentsWithStats(
        mask,
        labels,
        stats,
        centroids,
        8,
        CV_32S);

    cv::Mat filtered = cv::Mat::zeros(mask.size(), CV_8UC1);
    const int max_component_area_pixels = max_component_area_ratio > 0.0F
        ? std::max(
              min_component_area_pixels,
              static_cast<int>(mask.total() * static_cast<double>(max_component_area_ratio)))
        : std::numeric_limits<int>::max();

    for (int label = 1; label < component_count; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < min_component_area_pixels || area > max_component_area_pixels) {
            continue;
        }

        cv::Mat component_mask = labels == label;
        filtered.setTo(255, component_mask);
    }

    return filtered;
}

std::vector<cv::Point> ExtractPoints(const cv::Mat& mask) {
    std::vector<cv::Point> points;
    if (mask.empty()) {
        return points;
    }

    cv::Mat non_zero_points;
    cv::findNonZero(mask, non_zero_points);
    points.reserve(non_zero_points.total());
    for (int row = 0; row < non_zero_points.rows; ++row) {
        points.push_back(non_zero_points.at<cv::Point>(row));
    }

    return points;
}

}  // namespace

TrackerProcessor::TrackerProcessor(const PipelineConfig& config) : config_(config) {}

TrajectoryResult TrackerProcessor::Process(const ForegroundMaskResult& foreground_mask) {
    TrajectoryResult result;
    if (!foreground_mask.valid || foreground_mask.candidate_mask.empty()) {
        return result;
    }

    if (hit_count_map_.empty() || hit_count_map_.size() != foreground_mask.candidate_mask.size()) {
        Reset();
        hit_count_map_ = cv::Mat::zeros(foreground_mask.candidate_mask.size(), CV_32SC1);
    }

    if (!trajectory_window_.empty() &&
        foreground_mask.timestamp_seconds - trajectory_window_.back().timestamp_seconds >
            config_.trajectory_window.window_seconds) {
        temporal_masks_.clear();
        trajectory_window_.clear();
        hit_count_map_.setTo(0);
    }

    TemporalMaskEntry temporal_entry;
    temporal_entry.mask = foreground_mask.candidate_mask.clone();
    temporal_masks_.push_back(std::move(temporal_entry));

    const int max_temporal_frames = std::max(1, config_.trajectory_window.temporal_vote_frames);
    while (static_cast<int>(temporal_masks_.size()) > max_temporal_frames) {
        temporal_masks_.pop_front();
    }

    const int temporal_vote_threshold =
        std::max(1, config_.trajectory_window.temporal_vote_threshold);
    cv::Mat voted_mask = cv::Mat::zeros(foreground_mask.candidate_mask.size(), CV_8UC1);
    if (static_cast<int>(temporal_masks_.size()) >= temporal_vote_threshold) {
        const int required_votes = std::min(temporal_vote_threshold, max_temporal_frames);
        cv::Mat vote_counts = cv::Mat::zeros(foreground_mask.candidate_mask.size(), CV_16UC1);
        for (const TemporalMaskEntry& entry : temporal_masks_) {
            cv::Mat binary_mask;
            entry.mask.convertTo(binary_mask, CV_16UC1, 1.0 / 255.0);
            vote_counts += binary_mask;
        }

        cv::threshold(vote_counts, vote_counts, required_votes - 1, 255, cv::THRESH_BINARY);
        vote_counts.convertTo(voted_mask, CV_8UC1);
    }

    cv::Mat filtered_mask = FilterConnectedComponents(
        voted_mask,
        std::max(1, config_.trajectory_window.min_component_area_pixels),
        config_.trajectory_window.max_component_area_ratio);

    TrajectoryWindowEntry window_entry;
    window_entry.timestamp_seconds = foreground_mask.timestamp_seconds;
    window_entry.points = ExtractPoints(filtered_mask);
    trajectory_window_.push_back(window_entry);

    for (const cv::Point& point : window_entry.points) {
        ++hit_count_map_.at<int>(point);
    }

    const double earliest_timestamp =
        foreground_mask.timestamp_seconds - config_.trajectory_window.window_seconds;
    while (!trajectory_window_.empty() &&
           trajectory_window_.front().timestamp_seconds < earliest_timestamp) {
        for (const cv::Point& point : trajectory_window_.front().points) {
            --hit_count_map_.at<int>(point);
        }
        trajectory_window_.pop_front();
    }

    result.trajectory_layer = cv::Mat::zeros(filtered_mask.size(), CV_8UC1);
    double max_value = 0.0;
    cv::minMaxLoc(hit_count_map_, nullptr, &max_value);
    if (max_value > 0.0) {
        hit_count_map_.convertTo(
            result.trajectory_layer,
            CV_8UC1,
            255.0 / max_value);
    }

    result.valid = true;
    result.accumulated_frames = static_cast<int>(trajectory_window_.size());
    return result;
}

void TrackerProcessor::Reset() {
    hit_count_map_.release();
    temporal_masks_.clear();
    trajectory_window_.clear();
}

}  // namespace hero_lob
