#ifndef IMAGE_TRANS_CLI_OPTIONS_HPP_
#define IMAGE_TRANS_CLI_OPTIONS_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "image_trans/common_types.hpp"
#include "image_trans/config_types.hpp"

namespace image_trans {

struct CliOptions {
    std::filesystem::path input_path;
    std::filesystem::path output_dir;
    TriggerSpec trigger;
    std::optional<cv::Rect> crop_rect;
    bool debug_enabled = false;
    std::optional<std::filesystem::path> config_path;
};

class CliOptionsParser {
public:
    static CliOptions parse(int argc, char **argv);
    static CompositeConfig make_config_from_options(const CliOptions &options);

private:
    CliOptionsParser() = delete;

    static std::vector<std::string> collect_args(int argc, char **argv);
    static bool has_value(const std::vector<std::string> &args, std::size_t index);
    static cv::Rect parse_crop_rect(const std::string &value);
    static std::int64_t parse_int64(const std::string &value, const std::string &option_name);
    static int parse_int(const std::string &value, const std::string &option_name);
};

}  // namespace image_trans

#endif  // IMAGE_TRANS_CLI_OPTIONS_HPP_
