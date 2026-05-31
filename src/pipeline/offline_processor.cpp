#include "image_trans/offline_processor.hpp"

#include "image_trans/debug_recorder.hpp"
#include "image_trans/motion_extractor.hpp"
#include "image_trans/reference_builder.hpp"
#include "image_trans/registration_engine.hpp"
#include "image_trans/trail_compositor.hpp"
#include "image_trans/video_file_source.hpp"
#include "image_trans/window_assembler.hpp"

namespace image_trans {

OfflineProcessor::OfflineProcessor(CompositeConfig config)
    : config_(std::move(config)) {}

CompositeResult OfflineProcessor::run(const ReplayRequest& request) const {
    VideoFileSource source(request.input_path);
    WindowAssembler assembler(config_);
    const WindowCapture window = assembler.build_window(source, request.trigger);
    return process_window(window, request.output_dir);
}

CompositeResult OfflineProcessor::process_window(
    const WindowCapture& window, const std::filesystem::path& output_dir) const {
    ReferenceBuilder reference_builder(config_);
    RegistrationEngine registration_engine(config_);
    MotionExtractor motion_extractor(config_);
    DebugRecorder debug_recorder(config_);

    const ReferenceFrameSet reference = reference_builder.build(window);
    RegistrationContext registration_context{reference.background_gray, reference.background_bgr};
    TrailCompositor compositor(reference.background_bgr.size());

    CompositeResult result;
    result.reference_background_bgr = reference.background_bgr.clone();

    for (const FramePacket& frame : window.posttrigger_frames) {
        const RegistrationResult registration =
            registration_engine.estimate(registration_context, frame);
        if (!registration.accepted) {
            ++result.dropped_frame_count;
            result.frame_debug_list.push_back(make_rejected_debug(frame, registration));
            continue;
        }

        const cv::Mat aligned_bgr = registration_engine.warp_bgr(
            frame.bgr, registration.warp_2x3, reference.background_bgr.size());
        const MotionMaskResult motion = motion_extractor.extract(reference, aligned_bgr);

        compositor.accumulate(aligned_bgr, motion.binary_mask);

        FrameDebug frame_debug;
        frame_debug.frame_index = frame.frame_index;
        frame_debug.timestamp_ms = frame.timestamp_ms;
        frame_debug.accepted = true;
        frame_debug.used_translation_fallback = registration.used_translation_fallback;
        frame_debug.registration_score = registration.score;
        frame_debug.motion_pixel_count = motion.motion_pixel_count;
        frame_debug.reject_reason = RejectReason::kNone;
        result.frame_debug_list.push_back(frame_debug);
        ++result.accepted_frame_count;
    }

    result.trail_layer_bgr = compositor.trail_layer();
    result.final_bgr = compositor.compose_with_background(reference.background_bgr);

    DebugArtifacts artifacts;
    artifacts.final_bgr = result.final_bgr;
    artifacts.background_bgr = result.reference_background_bgr;
    artifacts.trail_layer_bgr = result.trail_layer_bgr;
    artifacts.frame_debug_list = result.frame_debug_list;
    debug_recorder.write(output_dir, artifacts);

    return result;
}

FrameDebug OfflineProcessor::make_rejected_debug(
    const FramePacket& frame, const RegistrationResult& reg) const {
    static_cast<void>(config_);
    FrameDebug debug;
    debug.frame_index = frame.frame_index;
    debug.timestamp_ms = frame.timestamp_ms;
    debug.accepted = false;
    debug.used_translation_fallback = reg.used_translation_fallback;
    debug.registration_score = reg.score;
    debug.motion_pixel_count = 0;
    debug.reject_reason = reg.reject_reason;
    return debug;
}

} // namespace image_trans
