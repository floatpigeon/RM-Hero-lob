#include "image_trans/window_assembler.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>

namespace image_trans {

WindowAssembler::WindowAssembler(CompositeConfig config)
    : config_(std::move(config)) {}

WindowCapture
    WindowAssembler::build_window(ReplaySource& source, const TriggerSpec& trigger) const {
    source.reset();

    WindowCapture window;
    window.source_fps = source.fps();
    window.source_frame_size = source.frame_size();

    const std::int64_t trigger_frame_index = resolve_trigger_frame_index(source, trigger);
    const int posttrigger_frame_count = compute_posttrigger_frame_count(window.source_fps);

    std::deque<FramePacket> rolling_pretrigger;
    FramePacket frame;
    bool found_trigger = false;

    while (source.read(frame)) {
        frame.bgr = apply_crop_if_needed(frame.bgr);
        if (window.source_frame_size.width > 0 && window.source_frame_size.height > 0
            && config_.crop.enabled) {
            window.source_frame_size = frame.bgr.size();
        }

        if (frame.frame_index < trigger_frame_index) {
            rolling_pretrigger.push_back(frame);
            while (static_cast<int>(rolling_pretrigger.size()) > config_.pretrigger_frame_count) {
                rolling_pretrigger.pop_front();
            }
            continue;
        }

        if (!found_trigger) {
            window.pretrigger_frames.assign(rolling_pretrigger.begin(), rolling_pretrigger.end());
            found_trigger = true;
        }

        if (static_cast<int>(window.posttrigger_frames.size()) < posttrigger_frame_count) {
            window.posttrigger_frames.push_back(frame);
        }

        if (static_cast<int>(window.posttrigger_frames.size()) >= posttrigger_frame_count) {
            break;
        }
    }

    if (!found_trigger) {
        throw std::runtime_error("trigger frame was not found in input source");
    }

    return window;
}

cv::Mat WindowAssembler::apply_crop_if_needed(const cv::Mat& input) const {
    if (!config_.crop.enabled) {
        return input;
    }

    const cv::Rect image_rect(0, 0, input.cols, input.rows);
    const cv::Rect crop_rect = config_.crop.crop_rect & image_rect;
    if (crop_rect.width <= 0 || crop_rect.height <= 0) {
        throw std::runtime_error("crop rectangle is outside the input frame");
    }

    return input(crop_rect).clone();
}

std::int64_t WindowAssembler::resolve_trigger_frame_index(
    ReplaySource& source, const TriggerSpec& trigger) const {
    if (trigger.mode == TriggerMode::kFrameIndex) {
        if (trigger.value < 0) {
            throw std::runtime_error("trigger frame index must be non-negative");
        }
        return trigger.value;
    }

    const double source_fps = source.fps();
    if (source_fps <= 0.0) {
        throw std::runtime_error("input source reported invalid fps for timestamp trigger");
    }

    const double trigger_frame = (static_cast<double>(trigger.value) * source_fps) / 1000.0;
    return static_cast<std::int64_t>(std::llround(trigger_frame));
}

int WindowAssembler::compute_posttrigger_frame_count(double fps) const {
    if (fps <= 0.0) {
        return 1;
    }

    const double frame_count = (static_cast<double>(config_.capture_duration_ms) * fps) / 1000.0;
    return std::max(1, static_cast<int>(std::ceil(frame_count)));
}

} // namespace image_trans
