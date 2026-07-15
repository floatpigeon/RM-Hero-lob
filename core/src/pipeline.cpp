#include "pipeline.hpp"

#include <fstream>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace hero_lob {

Pipeline::Pipeline(const PipelineConfig& config)
    : config_(config)
    , capture_(config)
    , reference_frame_selector_(config)
    , image_registrator_(config)
    , background_remover_(config)
    , tracker_processor_fast_(config)
    , image_synthesis_(config)
    , compression_(config) {}

bool Pipeline::Run(const std::string& input_video, const std::string& output_image) {
    if (!capture_.Open(input_video)) {
        std::cerr << "[Pipeline] Failed to open video: " << input_video << '\n';
        return false;
    }
    FrameData first_frame;
    if (!capture_.ReadNext(first_frame)) {
        std::cerr << "[Pipeline] Failed to read first frame\n";
        return false;
    }

    int total_frames = 0;

    TrackingResult empty_tracking;
    ReferenceFrameResult reference_result =
        reference_frame_selector_.Process(first_frame, empty_tracking);

    TrajectoryResult last_trajectory;
    FrameData frame;

    while (capture_.ReadNext(frame)) {
        RegistrationResult registration = image_registrator_.Process(reference_result, frame);

        ForegroundMaskResult foreground =
            background_remover_.Process(reference_result, registration);

        last_trajectory = tracker_processor_fast_.Process(foreground);

        ++total_frames;
    }

    SynthesisResult synthesis = image_synthesis_.Process(reference_result, last_trajectory);

    if (!synthesis.valid || synthesis.output_image.empty()) {
        std::cerr << "[Pipeline] Synthesis failed\n";
        return false;
    }

    CompressionResult compressed = compression_.Process(synthesis);
    if (!compressed.valid || compressed.output_image.empty()) {
        std::cerr << "[Pipeline] Compression failed\n";
        return false;
    }

    std::cerr << "[Pipeline] Output size: " << compressed.output_image.cols << "x"
              << compressed.output_image.rows << '\n';

    bool ok = cv::imwrite(output_image, compressed.output_image);
    if (!ok) {
        std::cerr << "[Pipeline] Failed to write output: " << output_image << '\n';
    } else {
        std::ifstream file(output_image, std::ios::binary | std::ios::ate);
        if (file.is_open()) {
            auto size = file.tellg();
            std::cerr << "[Pipeline] Output file size: " << size << " bytes\n";
        }
    }

    std::cerr << "[Pipeline] Frames: " << total_frames << '\n';

    return ok;
}

}  // namespace hero_lob
