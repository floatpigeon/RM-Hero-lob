#include "pipeline.hpp"

#include <chrono>
#include <iostream>

#include <opencv2/imgcodecs.hpp>

namespace hero_lob {

Pipeline::Pipeline(const PipelineConfig& config)
    : config_(config)
    , capture_(config)
    , identifier_(config)
    , tracker_(config)
    , reference_frame_selector_(config)
    , image_registrator_(config)
    , background_remover_(config)
    , tracker_processor_(config)
    , tracker_processor_fast_(config)
    , image_synthesis_(config) {}

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

    double time_reference_frame = 0.0;
    double time_background_remover = 0.0;
    double time_tracker_processor = 0.0;
    double time_tracker_processor_fast = 0.0;
    double time_synthesis = 0.0;
    int total_frames = 0;

    TrackingResult empty_tracking;
    auto t0 = std::chrono::steady_clock::now();
    ReferenceFrameResult reference_result =
        reference_frame_selector_.Process(first_frame, empty_tracking);
    auto t1 = std::chrono::steady_clock::now();
    time_reference_frame += std::chrono::duration<double>(t1 - t0).count();

    TrajectoryResult last_trajectory;
    FrameData frame;
    while (capture_.ReadNext(frame)) {
        RegistrationResult registration;
        registration.valid = true;
        registration.frame_index = frame.frame_index;
        registration.timestamp_seconds = frame.timestamp_seconds;
        registration.registered_bgr = frame.bgr;
        registration.registered_hsv = frame.hsv;

        t0 = std::chrono::steady_clock::now();
        ForegroundMaskResult foreground =
            background_remover_.Process(reference_result, registration);
        t1 = std::chrono::steady_clock::now();
        time_background_remover += std::chrono::duration<double>(t1 - t0).count();

        t0 = std::chrono::steady_clock::now();
        // last_trajectory = //
        // tracker_processor_.Process(foreground);
        t1 = std::chrono::steady_clock::now();
        time_tracker_processor += std::chrono::duration<double>(t1 - t0).count();

        t0 = std::chrono::steady_clock::now();
        last_trajectory =  //
            tracker_processor_fast_.Process(foreground);
        t1 = std::chrono::steady_clock::now();
        time_tracker_processor_fast += std::chrono::duration<double>(t1 - t0).count();

        ++total_frames;
    }

    t0 = std::chrono::steady_clock::now();
    SynthesisResult synthesis = image_synthesis_.Process(reference_result, last_trajectory);
    t1 = std::chrono::steady_clock::now();
    time_synthesis += std::chrono::duration<double>(t1 - t0).count();

    if (!synthesis.valid || synthesis.output_image.empty()) {
        std::cerr << "[Pipeline] Synthesis failed\n";
        return false;
    }
    bool ok = cv::imwrite(output_image, synthesis.output_image);
    if (!ok) {
        std::cerr << "[Pipeline] Failed to write output: " << output_image << '\n';
    }

    double total =
        time_reference_frame + time_background_remover + time_tracker_processor + time_synthesis;
    double fps = total_frames / total;
    std::cerr << "[Pipeline] Frames: " << total_frames << '\n'
              << "[Pipeline] Time breakdown (total / per-frame):\n"
              << "  reference_frame_selector: " << time_reference_frame << "s / "
              << (total_frames > 0 ? time_reference_frame / total_frames * 1000.0 : 0.0) << "ms ("
              << (total > 0 ? time_reference_frame / total * 100.0 : 0.0) << "%)\n"
              << "  background_remover:       " << time_background_remover << "s / "
              << (total_frames > 0 ? time_background_remover / total_frames * 1000.0 : 0.0)
              << "ms (" << (total > 0 ? time_background_remover / total * 100.0 : 0.0) << "%)\n"
              << "  tracker_processor:        " << time_tracker_processor << "s / "
              << (total_frames > 0 ? time_tracker_processor / total_frames * 1000.0 : 0.0) << "ms ("
              << (total > 0 ? time_tracker_processor / total * 100.0 : 0.0) << "%)\n"
              << "  tracker_processor_fast:   " << time_tracker_processor_fast << "s / "
              << (total_frames > 0 ? time_tracker_processor_fast / total_frames * 1000.0 : 0.0)
              << "ms\n"
              << "  image_synthesis:          " << time_synthesis << "s / "
              << (total_frames > 0 ? time_synthesis / total_frames * 1000.0 : 0.0) << "ms ("
              << (total > 0 ? time_synthesis / total * 100.0 : 0.0) << "%)\n"
              << "[Pipeline] Total: " << total << "s, " << fps << " fps\n"
              << "[Pipeline] tracker_processor speedup: "
              << (time_tracker_processor_fast > 0
                      ? time_tracker_processor / time_tracker_processor_fast
                      : 0.0)
              << "x\n";

    return ok;
}

}  // namespace hero_lob
