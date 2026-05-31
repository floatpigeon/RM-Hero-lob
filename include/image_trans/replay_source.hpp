#ifndef IMAGE_TRANS_REPLAY_SOURCE_HPP_
#define IMAGE_TRANS_REPLAY_SOURCE_HPP_

#include <opencv2/core.hpp>

#include "image_trans/common_types.hpp"

namespace image_trans {

class ReplaySource {
public:
    virtual ~ReplaySource() = default;

    virtual double fps() const = 0;
    virtual cv::Size frame_size() const = 0;
    virtual bool read(FramePacket &out_frame) = 0;
    virtual void reset() = 0;
};

}  // namespace image_trans

#endif  // IMAGE_TRANS_REPLAY_SOURCE_HPP_
