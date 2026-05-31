#include "image_trans/video_file_source.hpp"

#include <stdexcept>

namespace image_trans {

VideoFileSource::VideoFileSource(const std::filesystem::path& input_path)
    : input_path_(input_path) {
    open_capture();
}

double VideoFileSource::fps() const { return capture_.get(cv::CAP_PROP_FPS); }

cv::Size VideoFileSource::frame_size() const {
    return cv::Size(
        static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH)),
        static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT)));
}

bool VideoFileSource::read(FramePacket& out_frame) {
    cv::Mat frame;
    if (!capture_.read(frame)) {
        return false;
    }

    out_frame.frame_index = next_frame_index_;
    out_frame.timestamp_ms = compute_timestamp_ms(next_frame_index_);
    out_frame.bgr = frame;
    ++next_frame_index_;
    return true;
}

void VideoFileSource::reset() { open_capture(); }

void VideoFileSource::open_capture() {
    next_frame_index_ = 0;
    capture_.release();
    capture_.open(path_string());
    if (!capture_.isOpened()) {
        throw std::runtime_error("failed to open video input: " + path_string());
    }
}

std::string VideoFileSource::path_string() const { return input_path_.string(); }

std::int64_t VideoFileSource::compute_timestamp_ms(std::int64_t frame_index) const {
    const double source_fps = fps();
    if (source_fps <= 0.0) {
        return 0;
    }
    return static_cast<std::int64_t>((1000.0 * static_cast<double>(frame_index)) / source_fps);
}

} // namespace image_trans
