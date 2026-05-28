#include "image_trans/long_exposure_composer.h"

#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace image_trans {

LongExposureComposer::LongExposureComposer(LongExposureParams params)
    : params_(std::move(params)) {}

void LongExposureComposer::reset(cv::Size frame_size) {
  if (frame_size.empty()) {
    throw std::invalid_argument(
        "LongExposureComposer::reset requires a non-empty frame size");
  }

  frame_size_ = frame_size;
  stats_ = {};
  average_accumulator_ = cv::Mat::zeros(frame_size_, CV_32FC3);
  trail_buffer_ = cv::Mat::zeros(frame_size_, CV_8UC3);
  trail_mask_ = cv::Mat::zeros(frame_size_, CV_8UC1);
}

void LongExposureComposer::accumulate(const cv::Mat &stabilized_frame) {
  if (frame_size_.empty()) {
    throw std::logic_error(
        "LongExposureComposer::reset must be called before accumulate");
  }
  if (stabilized_frame.size() != frame_size_) {
    throw std::invalid_argument("LongExposureComposer::accumulate received a "
                                "frame with unexpected size");
  }
  if (stabilized_frame.type() != CV_8UC3) {
    throw std::invalid_argument(
        "LongExposureComposer::accumulate expects CV_8UC3 frames");
  }

  cv::Mat frame_float;
  stabilized_frame.convertTo(frame_float, CV_32FC3);
  const double alpha = 1.0 / static_cast<double>(stats_.accumulated_frames + 1);
  if (stats_.accumulated_frames == 0) {
    average_accumulator_ = frame_float;
  } else {
    average_accumulator_ =
        average_accumulator_ * static_cast<float>(1.0 - alpha) +
        frame_float * static_cast<float>(alpha);
  }

  cv::Mat grayscale;
  cv::cvtColor(stabilized_frame, grayscale, cv::COLOR_BGR2GRAY);
  cv::Mat bright_mask;
  cv::threshold(grayscale, bright_mask, params_.brightness_threshold, 255,
                cv::THRESH_BINARY);

  for (int row = 0; row < stabilized_frame.rows; ++row) {
    const cv::Vec3b *input_ptr = stabilized_frame.ptr<cv::Vec3b>(row);
    cv::Vec3b *trail_ptr = trail_buffer_.ptr<cv::Vec3b>(row);
    const unsigned char *mask_ptr = bright_mask.ptr<unsigned char>(row);
    unsigned char *trail_mask_ptr = trail_mask_.ptr<unsigned char>(row);
    for (int col = 0; col < stabilized_frame.cols; ++col) {
      if (!mask_ptr[col]) {
        continue;
      }

      trail_mask_ptr[col] = 255;
      for (int channel = 0; channel < 3; ++channel) {
        trail_ptr[col][channel] =
            std::max(trail_ptr[col][channel], input_ptr[col][channel]);
      }
    }
  }

  ++stats_.accumulated_frames;
}

cv::Mat LongExposureComposer::finalize() const {
  if (stats_.accumulated_frames == 0) {
    throw std::logic_error("LongExposureComposer::finalize requires at least "
                           "one accumulated frame");
  }

  cv::Mat average_u8;
  average_accumulator_.convertTo(average_u8, CV_8UC3);

  cv::Mat result = average_u8.clone();
  for (int row = 0; row < result.rows; ++row) {
    cv::Vec3b *result_ptr = result.ptr<cv::Vec3b>(row);
    const cv::Vec3b *trail_ptr = trail_buffer_.ptr<cv::Vec3b>(row);
    const unsigned char *mask_ptr = trail_mask_.ptr<unsigned char>(row);
    for (int col = 0; col < result.cols; ++col) {
      if (!mask_ptr[col]) {
        continue;
      }

      for (int channel = 0; channel < 3; ++channel) {
        result_ptr[col][channel] =
            std::max(result_ptr[col][channel], trail_ptr[col][channel]);
      }
    }
  }

  return result;
}

const LongExposureStats &LongExposureComposer::stats() const { return stats_; }

} // namespace image_trans
