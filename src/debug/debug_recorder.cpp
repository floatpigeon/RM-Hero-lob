#include "image_trans/debug_recorder.hpp"

#include <fstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>

namespace image_trans {

DebugRecorder::DebugRecorder(CompositeConfig config)
    : config_(std::move(config)) {}

void DebugRecorder::write(
    const std::filesystem::path& output_dir, const DebugArtifacts& artifacts) const {
    ensure_output_dir(output_dir);
    write_image_if_present(output_dir, "final.png", artifacts.final_bgr);
    write_image_if_present(output_dir, "background.png", artifacts.background_bgr);
    write_image_if_present(output_dir, "trail_layer.png", artifacts.trail_layer_bgr);
    write_metrics_json(output_dir, artifacts);
}

void DebugRecorder::ensure_output_dir(const std::filesystem::path& output_dir) const {
    static_cast<void>(config_);
    std::error_code error_code;
    std::filesystem::create_directories(output_dir, error_code);
    if (error_code) {
        throw std::runtime_error("failed to create output directory: " + output_dir.string());
    }
}

void DebugRecorder::write_image_if_present(
    const std::filesystem::path& output_dir, const std::string& file_name,
    const cv::Mat& image) const {
    if (image.empty()) {
        return;
    }

    const std::filesystem::path output_path = output_dir / file_name;
    if (!cv::imwrite(output_path.string(), image)) {
        throw std::runtime_error("failed to write image: " + output_path.string());
    }
}

void DebugRecorder::write_metrics_json(
    const std::filesystem::path& output_dir, const DebugArtifacts& artifacts) const {
    const std::filesystem::path output_path = output_dir / "metrics.json";
    std::ofstream stream(output_path);
    if (!stream.is_open()) {
        throw std::runtime_error("failed to write metrics file: " + output_path.string());
    }

    stream << "{\n";
    stream << "  \"frame_debug\": [\n";
    for (std::size_t index = 0; index < artifacts.frame_debug_list.size(); ++index) {
        const FrameDebug& frame_debug = artifacts.frame_debug_list[index];
        stream << "    {\n";
        stream << "      \"frame_index\": " << frame_debug.frame_index << ",\n";
        stream << "      \"timestamp_ms\": " << frame_debug.timestamp_ms << ",\n";
        stream << "      \"accepted\": " << (frame_debug.accepted ? "true" : "false") << ",\n";
        stream << "      \"used_translation_fallback\": "
               << (frame_debug.used_translation_fallback ? "true" : "false") << ",\n";
        stream << "      \"registration_score\": " << frame_debug.registration_score << ",\n";
        stream << "      \"motion_pixel_count\": " << frame_debug.motion_pixel_count << ",\n";
        stream << "      \"reject_reason\": \""
               << reject_reason_to_string(frame_debug.reject_reason) << "\"\n";
        stream << "    }";
        if (index + 1 < artifacts.frame_debug_list.size()) {
            stream << ",";
        }
        stream << "\n";
    }
    stream << "  ]\n";
    stream << "}\n";
}

std::string DebugRecorder::reject_reason_to_string(RejectReason reject_reason) const {
    switch (reject_reason) {
    case RejectReason::kNone: return "none";
    case RejectReason::kRegistrationFailed: return "registration_failed";
    case RejectReason::kLowRegistrationScore: return "low_registration_score";
    case RejectReason::kInvalidMotionMask: return "invalid_motion_mask";
    }

    return "unknown";
}

} // namespace image_trans
