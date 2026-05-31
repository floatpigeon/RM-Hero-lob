#ifndef IMAGE_TRANS_VIDEO_FILE_SOURCE_HPP_
#define IMAGE_TRANS_VIDEO_FILE_SOURCE_HPP_

#include <filesystem>
#include <string>

#include <opencv2/videoio.hpp>

#include "image_trans/replay_source.hpp"

namespace image_trans {

class VideoFileSource final : public ReplaySource {
public:
    explicit VideoFileSource(const std::filesystem::path &input_path);

    double fps() const override;
    cv::Size frame_size() const override;
    bool read(FramePacket &out_frame) override;
    void reset() override;

private:
    std::filesystem::path input_path_;
    cv::VideoCapture capture_;
    std::int64_t next_frame_index_ = 0;

    void open_capture();
    std::string path_string() const;
    std::int64_t compute_timestamp_ms(std::int64_t frame_index) const;
};

}  // namespace image_trans

#endif  // IMAGE_TRANS_VIDEO_FILE_SOURCE_HPP_
