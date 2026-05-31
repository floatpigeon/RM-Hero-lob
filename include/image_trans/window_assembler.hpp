#ifndef IMAGE_TRANS_WINDOW_ASSEMBLER_HPP_
#define IMAGE_TRANS_WINDOW_ASSEMBLER_HPP_

#include "image_trans/config_types.hpp"
#include "image_trans/replay_source.hpp"

namespace image_trans {

class WindowAssembler {
public:
    explicit WindowAssembler(CompositeConfig config);

    WindowCapture build_window(ReplaySource &source, const TriggerSpec &trigger) const;

private:
    CompositeConfig config_;

    cv::Mat apply_crop_if_needed(const cv::Mat &input) const;
    std::int64_t resolve_trigger_frame_index(ReplaySource &source, const TriggerSpec &trigger) const;
    int compute_posttrigger_frame_count(double fps) const;
};

}  // namespace image_trans

#endif  // IMAGE_TRANS_WINDOW_ASSEMBLER_HPP_
