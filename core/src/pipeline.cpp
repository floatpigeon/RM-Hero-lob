#include "pipeline.hpp"

#include <iostream>

#include <opencv2/imgcodecs.hpp>

namespace hero_lob {

Pipeline::Pipeline(const PipelineConfig& config)
    : config_(config)
    , capture_(config_)
    , identifier_(config_)
    , tracker_(config_)
    , reference_frame_selector_(config_)
    , image_registrator_(config_)
    , background_remover_(config_)
    , tracker_processor_(config_)
    , image_synthesis_(config_) {}

bool Pipeline::Run(const std::string& input_video, const std::string& output_image) {
    std::cout << "Opening input video: " << input_video << '\n';
    if (!capture_.Open(input_video)) {
        std::cerr << "Failed to open input video: " << input_video << '\n';
        return false;
    }
    std::cout << "Capture FPS: " << capture_.FramesPerSecond() << '\n';

    tracker_processor_.Reset();

    FrameData frame;
    SynthesisResult last_synthesis;
    bool processed_any_frame = false;

    while (capture_.ReadNext(frame)) {
        processed_any_frame = true;

        const DetectionResult detection = identifier_.Process(frame);
        const TrackingResult tracking = tracker_.Process(frame, detection);
        const ReferenceFrameResult reference = reference_frame_selector_.Process(frame, tracking);
        const RegistrationResult registration =
            image_registrator_.Process(reference, frame, tracking);
        const ForegroundMaskResult foreground =
            background_remover_.Process(reference, registration);
        const TrajectoryResult trajectory = tracker_processor_.Process(foreground);
        const SynthesisResult synthesis = image_synthesis_.Process(reference, trajectory);

        if (synthesis.valid) {
            last_synthesis = synthesis;
        }

        if (frame.frame_index % 30 == 0) {
            std::cout << "Processed frame " << frame.frame_index << " at "
                      << frame.timestamp_seconds << "s\n";
        }
    }

    if (!processed_any_frame) {
        std::cerr << "Input video contains no decodable frames: " << input_video << '\n';
        return false;
    }

    if (!last_synthesis.valid || last_synthesis.output_image.empty()) {
        std::cerr << "Pipeline completed without producing an output image.\n";
        return false;
    }

    if (!cv::imwrite(output_image, last_synthesis.output_image)) {
        std::cerr << "Failed to write output image: " << output_image << '\n';
        return false;
    }

    std::cout << "Wrote output image: " << output_image << '\n';
    return true;
}

}  // namespace hero_lob
