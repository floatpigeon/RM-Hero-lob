#ifndef IMAGE_TRANS_OFFLINE_PROCESSOR_HPP_
#define IMAGE_TRANS_OFFLINE_PROCESSOR_HPP_

#include <filesystem>

#include "image_trans/common_types.hpp"
#include "image_trans/config_types.hpp"

namespace image_trans {

struct ReplayRequest {
    std::filesystem::path input_path;
    TriggerSpec trigger;
    std::filesystem::path output_dir;
};

class OfflineProcessor {
public:
    explicit OfflineProcessor(CompositeConfig config);

    CompositeResult run(const ReplayRequest &request) const;

private:
    CompositeConfig config_;

    CompositeResult process_window(const WindowCapture &window, const std::filesystem::path &output_dir) const;
    FrameDebug make_rejected_debug(const FramePacket &frame, const RegistrationResult &reg) const;
};

}  // namespace image_trans

#endif  // IMAGE_TRANS_OFFLINE_PROCESSOR_HPP_
