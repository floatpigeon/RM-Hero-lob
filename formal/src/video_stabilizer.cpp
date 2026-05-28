#include "image_trans/video_stabilizer.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace image_trans {
namespace {

constexpr int kMinimumModelPoints = 8;

struct Pose2D {
    double dx = 0.0;
    double dy = 0.0;
    double da = 0.0;
};

cv::Mat makeTransform(double dx, double dy, double da) {
    const double c = std::cos(da);
    const double s = std::sin(da);
    cv::Mat transform = (cv::Mat_<double>(2, 3) << c, -s, dx, s, c, dy);
    return transform;
}

Pose2D poseFromTransform(const cv::Mat& transform) {
    Pose2D pose;
    pose.dx = transform.at<double>(0, 2);
    pose.dy = transform.at<double>(1, 2);
    pose.da = std::atan2(transform.at<double>(1, 0), transform.at<double>(0, 0));
    return pose;
}

Pose2D averageRange(const std::vector<Pose2D>& trajectory, int begin, int end) {
    Pose2D mean;
    const int count = std::max(1, end - begin);
    for (int index = begin; index < end; ++index) {
        mean.dx += trajectory[index].dx;
        mean.dy += trajectory[index].dy;
        mean.da += trajectory[index].da;
    }
    mean.dx /= count;
    mean.dy /= count;
    mean.da /= count;
    return mean;
}

cv::Rect computeAutoCropRoi(const std::vector<cv::Mat>& transforms, const cv::Size& frame_size) {
    cv::Mat valid_mask(frame_size, CV_8UC1, cv::Scalar(255));
    cv::Mat intersection = valid_mask.clone();
    cv::Mat warped_mask;

    for (const cv::Mat& transform : transforms) {
        cv::warpAffine(valid_mask,
                       warped_mask,
                       transform,
                       frame_size,
                       cv::INTER_NEAREST,
                       cv::BORDER_CONSTANT,
                       cv::Scalar::all(0));
        cv::bitwise_and(intersection, warped_mask, intersection);
    }

    std::vector<cv::Point> non_zero_points;
    cv::findNonZero(intersection, non_zero_points);
    if (non_zero_points.empty()) {
        return {0, 0, frame_size.width, frame_size.height};
    }

    return cv::boundingRect(non_zero_points);
}

cv::Mat ensureAffine(const cv::Mat& transform) {
    if (transform.empty()) {
        return makeTransform(0.0, 0.0, 0.0);
    }

    cv::Mat affine;
    transform.convertTo(affine, CV_64F);
    return affine;
}

}  // namespace

VideoStabilizer::VideoStabilizer(StabilizationParams params)
    : params_(std::move(params)) {}

MotionPlan VideoStabilizer::analyze(const std::vector<cv::Mat>& frames) const {
    if (frames.empty()) {
        throw std::invalid_argument("VideoStabilizer::analyze requires at least one frame");
    }

    MotionPlan plan;
    plan.output_size = frames.front().size();
    plan.raw_transforms.reserve(frames.size());
    plan.correction_transforms.reserve(frames.size());
    plan.raw_transforms.push_back(makeTransform(0.0, 0.0, 0.0));

    cv::Mat previous_gray;
    cv::cvtColor(frames.front(), previous_gray, cv::COLOR_BGR2GRAY);
    cv::Mat previous_transform = makeTransform(0.0, 0.0, 0.0);

    for (std::size_t index = 1; index < frames.size(); ++index) {
        cv::Mat current_gray;
        cv::cvtColor(frames[index], current_gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> previous_points;
        cv::goodFeaturesToTrack(previous_gray,
                                previous_points,
                                params_.max_corners,
                                params_.quality_level,
                                params_.min_distance);

        cv::Mat transform = previous_transform.clone();
        bool used_fallback = true;

        if (!previous_points.empty()) {
            std::vector<cv::Point2f> current_points;
            std::vector<unsigned char> status;
            std::vector<float> errors;
            cv::calcOpticalFlowPyrLK(previous_gray,
                                     current_gray,
                                     previous_points,
                                     current_points,
                                     status,
                                     errors,
                                     cv::Size(params_.lk_window, params_.lk_window),
                                     params_.lk_max_level);

            std::vector<cv::Point2f> filtered_previous;
            std::vector<cv::Point2f> filtered_current;
            for (std::size_t point_index = 0; point_index < status.size(); ++point_index) {
                if (status[point_index]) {
                    filtered_previous.push_back(previous_points[point_index]);
                    filtered_current.push_back(current_points[point_index]);
                }
            }

            if (static_cast<int>(filtered_previous.size()) >= kMinimumModelPoints) {
                cv::Mat inlier_mask;
                cv::Mat estimated = cv::estimateAffinePartial2D(filtered_previous,
                                                                filtered_current,
                                                                inlier_mask,
                                                                cv::RANSAC,
                                                                params_.ransac_reproj_threshold);
                if (!estimated.empty()) {
                    const int inlier_count = cv::countNonZero(inlier_mask);
                    const int adaptive_inlier_target = std::max(
                        kMinimumModelPoints,
                        std::min(params_.min_inliers,
                                 static_cast<int>(std::ceil(filtered_previous.size() * 0.6))));
                    if (inlier_count >= adaptive_inlier_target) {
                        transform = ensureAffine(estimated);
                        previous_transform = transform.clone();
                        used_fallback = false;
                    }
                }
            }
        }

        if (used_fallback) {
            ++plan.stats.fallback_frame_count;
            plan.stats.fallback_indices.push_back(index);
        }

        plan.raw_transforms.push_back(transform);
        previous_gray = current_gray;
    }

    std::vector<Pose2D> trajectory(frames.size());
    for (std::size_t index = 1; index < plan.raw_transforms.size(); ++index) {
        const Pose2D delta = poseFromTransform(plan.raw_transforms[index]);
        trajectory[index].dx = trajectory[index - 1].dx + delta.dx;
        trajectory[index].dy = trajectory[index - 1].dy + delta.dy;
        trajectory[index].da = trajectory[index - 1].da + delta.da;
    }

    std::vector<Pose2D> smoothed(trajectory.size());
    for (std::size_t index = 0; index < trajectory.size(); ++index) {
        const int begin = std::max<int>(0, static_cast<int>(index) - params_.smoothing_radius);
        const int end = std::min<int>(static_cast<int>(trajectory.size()), static_cast<int>(index) + params_.smoothing_radius + 1);
        smoothed[index] = averageRange(trajectory, begin, end);
    }

    plan.correction_transforms.reserve(plan.raw_transforms.size());
    for (std::size_t index = 0; index < plan.raw_transforms.size(); ++index) {
        const Pose2D correction = {
            smoothed[index].dx - trajectory[index].dx,
            smoothed[index].dy - trajectory[index].dy,
            smoothed[index].da - trajectory[index].da
        };
        plan.correction_transforms.push_back(makeTransform(correction.dx, correction.dy, correction.da));
    }

    plan.crop_roi = params_.auto_crop
        ? computeAutoCropRoi(plan.correction_transforms, plan.output_size)
        : cv::Rect(0, 0, plan.output_size.width, plan.output_size.height);
    return plan;
}

cv::Mat VideoStabilizer::stabilizeFrame(const cv::Mat& frame, const MotionPlan& plan, std::size_t index) const {
    if (index >= plan.correction_transforms.size()) {
        throw std::out_of_range("VideoStabilizer::stabilizeFrame index is out of range");
    }

    cv::Mat warped;
    cv::warpAffine(frame,
                   warped,
                   plan.correction_transforms[index],
                   plan.output_size,
                   cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT,
                   cv::Scalar::all(0));

    if (!params_.auto_crop) {
        return warped;
    }

    const cv::Rect bounded_roi = plan.crop_roi & cv::Rect(0, 0, warped.cols, warped.rows);
    cv::Mat cropped = warped(bounded_roi).clone();
    cv::Mat resized;
    cv::resize(cropped, resized, plan.output_size, 0.0, 0.0, cv::INTER_LINEAR);
    return resized;
}

std::vector<cv::Mat> VideoStabilizer::stabilize(const std::vector<cv::Mat>& frames, MotionPlan* out_plan) const {
    MotionPlan plan = analyze(frames);
    std::vector<cv::Mat> stabilized;
    stabilized.reserve(frames.size());
    for (std::size_t index = 0; index < frames.size(); ++index) {
        stabilized.push_back(stabilizeFrame(frames[index], plan, index));
    }

    if (out_plan != nullptr) {
        *out_plan = std::move(plan);
    }

    return stabilized;
}

}  // namespace image_trans
