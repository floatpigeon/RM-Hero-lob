#include "background_remover.hpp"

namespace hero_lob {

BackgroundRemover::BackgroundRemover(const PipelineConfig &config)
    : config_(config) {}

ForegroundMaskResult
BackgroundRemover::Process(const ReferenceFrameResult &reference,
                           const RegistrationResult &registration) const {
  ForegroundMaskResult result;
  if (!reference.has_reference || !registration.valid ||
      registration.registered_bgr.empty()) {
    return result;
  }

  result.valid = true;
  result.roi = cv::Rect(0, 0, registration.registered_bgr.cols,
                        registration.registered_bgr.rows);
  result.static_exclusion_mask =
      cv::Mat::zeros(registration.registered_bgr.size(), CV_8UC1);
  result.candidate_mask =
      cv::Mat::zeros(registration.registered_bgr.size(), CV_8UC1);
  return result;
}

} // namespace hero_lob
