#ifndef IMAGE_TRANS_DEBUG_RECORDER_HPP_
#define IMAGE_TRANS_DEBUG_RECORDER_HPP_

#include <filesystem>
#include <utility>
#include <vector>

#include "image_trans/common_types.hpp"
#include "image_trans/config_types.hpp"

namespace image_trans {

struct DebugArtifacts {
    cv::Mat final_bgr;
    cv::Mat background_bgr;
    cv::Mat trail_layer_bgr;
    std::vector<FrameDebug> frame_debug_list;
    std::vector<std::pair<std::int64_t, cv::Mat>> key_masks;
};

class DebugRecorder {
public:
    explicit DebugRecorder(CompositeConfig config);

    void write(const std::filesystem::path &output_dir, const DebugArtifacts &artifacts) const;

private:
    CompositeConfig config_;

    void ensure_output_dir(const std::filesystem::path &output_dir) const;
    void write_image_if_present(
        const std::filesystem::path &output_dir,
        const std::string &file_name,
        const cv::Mat &image) const;
    void write_metrics_json(
        const std::filesystem::path &output_dir,
        const DebugArtifacts &artifacts) const;
    std::string reject_reason_to_string(RejectReason reject_reason) const;
};

}  // namespace image_trans

#endif  // IMAGE_TRANS_DEBUG_RECORDER_HPP_
