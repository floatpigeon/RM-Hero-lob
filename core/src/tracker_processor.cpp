#include "tracker_processor.hpp"

#include <cmath>
#include <iostream>

#include <opencv2/imgproc.hpp>

namespace hero_lob {

TrackerProcessor::TrackerProcessor(const PipelineConfig& config)
    : config_(config) {}

TrajectoryResult TrackerProcessor::Process(
    const ForegroundMaskResult& foreground_mask) {
    TrajectoryResult result;
    if (!foreground_mask.valid || foreground_mask.candidate_mask.empty()) {
        if (frame_count_ > 0) {
            result.valid = true;
            result.trajectory_layer = trajectory_layer_;
            result.accumulated_frames = frame_count_;
        }
        return result;
    }
    const auto& tw = config_.trajectory_window;
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(
        foreground_mask.candidate_mask, labels, stats, centroids, 8,
        CV_32S);
    int raw_components = num_labels - 1;
    std::vector<ComponentInfo> current_components;
    for (int i = 1; i < num_labels; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < tw.min_component_area_pixels) {
            continue;
        }
        ComponentInfo comp;
        comp.centroid =
            cv::Point2f(static_cast<float>(centroids.at<double>(i, 0)),
                        static_cast<float>(centroids.at<double>(i, 1)));
        comp.mask = (labels == i);
        current_components.push_back(comp);
    }
    int filtered_components = raw_components - static_cast<int>(current_components.size());
    if (trajectory_layer_.empty()) {
        trajectory_layer_ = cv::Mat::zeros(
            foreground_mask.candidate_mask.size(), CV_32FC3);
    }
    int matched_count = 0;
    int new_count = 0;
    int direction_filtered = 0;
    int accumulated_pixels = 0;
    for (auto& comp : current_components) {
        const ComponentInfo* best_match = nullptr;
        float best_dist = std::numeric_limits<float>::max();
        for (const auto& prev : previous_components_) {
            float dx = comp.centroid.x - prev.centroid.x;
            float dy = comp.centroid.y - prev.centroid.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < best_dist) {
                best_dist = dist;
                best_match = &prev;
            }
        }
        bool should_accumulate = false;
        if (best_match != nullptr &&
            best_dist < tw.component_match_max_distance_pixels) {
            ++matched_count;
            comp.velocity_initialized = best_match->velocity_initialized;
            double dt = foreground_mask.timestamp_seconds - previous_timestamp_;
            if (dt > 1e-6) {
                cv::Point2f raw_velocity(
                    static_cast<float>((comp.centroid.x - best_match->centroid.x) / dt),
                    static_cast<float>((comp.centroid.y - best_match->centroid.y) / dt));
                if (!comp.velocity_initialized) {
                    comp.smoothed_velocity = raw_velocity;
                    comp.velocity_initialized = true;
                } else {
                    float alpha = tw.velocity_smoothing_alpha;
                    comp.smoothed_velocity =
                        alpha * raw_velocity +
                        (1.0F - alpha) * best_match->smoothed_velocity;
                }
            }
            float angle = std::abs(
                std::atan2(comp.smoothed_velocity.x,
                           comp.smoothed_velocity.y) *
                180.0F / static_cast<float>(M_PI));
            if (angle < tw.vertical_motion_half_angle_degrees) {
                should_accumulate = true;
            } else {
                ++direction_filtered;
            }
        } else {
            ++new_count;
            should_accumulate = true;
        }
        if (should_accumulate) {
            accumulated_pixels += cv::countNonZero(comp.mask);
            cv::Rect bbox = cv::boundingRect(comp.mask);
            cv::Mat mask_roi = comp.mask(bbox);
            cv::Mat bgr_roi = foreground_mask.candidate_bgr(bbox);
            cv::Mat traj_roi = trajectory_layer_(bbox);
            cv::Mat mask_f;
            mask_roi.convertTo(mask_f, CV_32F, 1.0 / 255.0);
            for (int r = 0; r < bbox.height; ++r) {
                const float* mask_row = mask_f.ptr<float>(r);
                const uchar* bgr_row = bgr_roi.ptr<uchar>(r);
                float* traj_row = traj_roi.ptr<float>(r);
                for (int col = 0; col < bbox.width; ++col) {
                    float m = mask_row[col];
                    if (m < 1e-6F) {
                        continue;
                    }
                    int idx = col * 3;
                    traj_row[idx + 0] += bgr_row[idx + 0] * m;
                    traj_row[idx + 1] += bgr_row[idx + 1] * m;
                    traj_row[idx + 2] += bgr_row[idx + 2] * m;
                }
            }
        }
    }
    std::cerr << "[TrackerProcessor] frame=" << foreground_mask.frame_index
              << " t=" << foreground_mask.timestamp_seconds
              << "s raw_comp=" << raw_components
              << " filtered=" << filtered_components
              << " matched=" << matched_count
              << " new=" << new_count
              << " dir_rejected=" << direction_filtered
              << " acc_pixels=" << accumulated_pixels << '\n';
    previous_components_ = current_components;
    previous_timestamp_ = foreground_mask.timestamp_seconds;
    frame_count_++;
    result.valid = true;
    result.trajectory_layer = trajectory_layer_;
    result.accumulated_frames = frame_count_;
    return result;
}

void TrackerProcessor::Reset() {
    previous_components_.clear();
    trajectory_layer_ = cv::Mat();
    frame_count_ = 0;
    previous_timestamp_ = 0.0;
}

}  // namespace hero_lob
