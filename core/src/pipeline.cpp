#include "pipeline.hpp"

#include <iostream>

#include <opencv2/imgcodecs.hpp>

namespace hero_lob {

Pipeline::Pipeline(const PipelineConfig& config)
    : config_(config),
      capture_(config),
      identifier_(config),
      tracker_(config),
      reference_frame_selector_(config),
      image_registrator_(config),
      background_remover_(config),
      tracker_processor_(config),
      image_synthesis_(config) {}

bool Pipeline::Run(const std::string& input_video,
                   const std::string& output_image) {
    std::cerr << "[Pipeline] Opening video: " << input_video << '\n';
    if (!capture_.Open(input_video)) {
        std::cerr << "[Pipeline] Failed to open video: " << input_video << '\n';
        return false;
    }
    std::cerr << "[Pipeline] FPS: " << capture_.FramesPerSecond() << '\n';
    FrameData first_frame;
    if (!capture_.ReadNext(first_frame)) {
        std::cerr << "[Pipeline] Failed to read first frame\n";
        return false;
    }
    std::cerr << "[Pipeline] Resolution: " << first_frame.bgr.cols << "x"
              << first_frame.bgr.rows << '\n';
    std::cerr << "[Pipeline] First frame: index=" << first_frame.frame_index
              << " timestamp=" << first_frame.timestamp_seconds << "s\n";
    TrackingResult empty_tracking;
    ReferenceFrameResult reference_result =
        reference_frame_selector_.Process(first_frame, empty_tracking);
    std::cerr << "[Pipeline] Reference frame set (first frame)\n";
    std::cerr << "[Pipeline] --- Processing frames ---\n";
    TrajectoryResult last_trajectory;
    FrameData frame;
    int total_frames = 0;
    while (capture_.ReadNext(frame)) {
        RegistrationResult registration;
        registration.valid = true;
        registration.frame_index = frame.frame_index;
        registration.timestamp_seconds = frame.timestamp_seconds;
        registration.registered_bgr = frame.bgr;
        registration.registered_hsv = frame.hsv;
        ForegroundMaskResult foreground =
            background_remover_.Process(reference_result, registration);
        last_trajectory =
            tracker_processor_.Process(foreground);
        ++total_frames;
    }
    std::cerr << "[Pipeline] --- Processing complete ---\n";
    std::cerr << "[Pipeline] Total frames processed: " << total_frames << '\n';
    SynthesisResult synthesis =
        image_synthesis_.Process(reference_result, last_trajectory);
    if (!synthesis.valid || synthesis.output_image.empty()) {
        std::cerr << "[Pipeline] Synthesis failed\n";
        return false;
    }
    std::cerr << "[Pipeline] Output: " << output_image << " ("
              << synthesis.output_image.cols << "x"
              << synthesis.output_image.rows << ")\n";
    bool ok = cv::imwrite(output_image, synthesis.output_image);
    if (ok) {
        std::cerr << "[Pipeline] Done.\n";
    } else {
        std::cerr << "[Pipeline] Failed to write output: " << output_image << '\n';
    }
    return ok;
}

}  // namespace hero_lob
