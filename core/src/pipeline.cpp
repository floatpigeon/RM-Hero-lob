#include "pipeline.hpp"

#include <chrono>
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
    double time_image_registrator = 0.0;
    double time_background_remover = 0.0;
    double time_tracker_processor_fast = 0.0;
    double time_synthesis = 0.0;
    double time_compression = 0.0;
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
        t0 = std::chrono::steady_clock::now();
        RegistrationResult registration = image_registrator_.Process(reference_result, frame);
        t1 = std::chrono::steady_clock::now();
        time_image_registrator += std::chrono::duration<double>(t1 - t0).count();

        t0 = std::chrono::steady_clock::now();
        ForegroundMaskResult foreground =
            background_remover_.Process(reference_result, registration);
        t1 = std::chrono::steady_clock::now();
        time_background_remover += std::chrono::duration<double>(t1 - t0).count();

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

    t0 = std::chrono::steady_clock::now();
    cv::Mat final_output;
    if (config_.output_width > 0 && config_.output_height > 0) {
        cv::resize(
            synthesis.output_image, final_output,
            cv::Size(config_.output_width, config_.output_height), 0, 0, cv::INTER_AREA);
    } else {
        final_output = synthesis.output_image;
    }

    std::cerr << "[Pipeline] Output size: " << final_output.cols << "x" << final_output.rows
              << '\n';

    bool ok = cv::imwrite(output_image, final_output);
    if (!ok) {
        std::cerr << "[Pipeline] Failed to write output: " << output_image << '\n';
    } else {
        std::ifstream file(output_image, std::ios::binary | std::ios::ate);
        if (file.is_open()) {
            auto size = file.tellg();
            std::cerr << "[Pipeline] Output file size: " << size << " bytes\n";
        }
    }
    t1 = std::chrono::steady_clock::now();
    time_compression += std::chrono::duration<double>(t1 - t0).count();

    image_registrator_.PrintTimingStats();

    double total = time_reference_frame + time_image_registrator + time_background_remover
                 + time_tracker_processor_fast + time_synthesis + time_compression;
    double fps = total_frames / total;
    std::cerr << "[Pipeline] Frames: " << total_frames << '\n'
              << "[Pipeline] Time breakdown (total / per-frame / percent):\n"
              << "  reference_frame_selector: " << time_reference_frame << "s / "
              << (total_frames > 0 ? time_reference_frame / total_frames * 1000.0 : 0.0) << "ms / "
              << (total > 0 ? time_reference_frame / total * 100.0 : 0.0) << "%\n"
              << "  image_registrator:        " << time_image_registrator << "s / "
              << (total_frames > 0 ? time_image_registrator / total_frames * 1000.0 : 0.0)
              << "ms / " << (total > 0 ? time_image_registrator / total * 100.0 : 0.0) << "%\n"
              << "  background_remover:       " << time_background_remover << "s / "
              << (total_frames > 0 ? time_background_remover / total_frames * 1000.0 : 0.0)
              << "ms / " << (total > 0 ? time_background_remover / total * 100.0 : 0.0) << "%\n"
              << "  tracker_processor_fast:   " << time_tracker_processor_fast << "s / "
              << (total_frames > 0 ? time_tracker_processor_fast / total_frames * 1000.0 : 0.0)
              << "ms / " << (total > 0 ? time_tracker_processor_fast / total * 100.0 : 0.0) << "%\n"
              << "  image_synthesis:          " << time_synthesis << "s / "
              << (total_frames > 0 ? time_synthesis / total_frames * 1000.0 : 0.0)
              << "ms / " << (total > 0 ? time_synthesis / total * 100.0 : 0.0) << "%\n"
              << "  compression:              " << time_compression << "s / "
              << (total_frames > 0 ? time_compression / total_frames * 1000.0 : 0.0)
              << "ms / " << (total > 0 ? time_compression / total * 100.0 : 0.0) << "%\n"
              << "[Pipeline] Total: " << total << "s, " << fps << " fps\n";

    return ok;
}

}  // namespace hero_lob
