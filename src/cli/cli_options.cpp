#include "image_trans/cli_options.hpp"

#include <charconv>
#include <stdexcept>

namespace image_trans {

CliOptions CliOptionsParser::parse(int argc, char** argv) {
    const std::vector<std::string> args = collect_args(argc, argv);

    CliOptions options;
    bool has_trigger = false;

    for (std::size_t index = 0; index < args.size(); ++index) {
        const std::string& arg = args[index];
        if (arg == "--input") {
            if (!has_value(args, index + 1)) {
                throw std::runtime_error("--input requires a value");
            }
            options.input_path = args[++index];
            continue;
        }
        if (arg == "--output-dir") {
            if (!has_value(args, index + 1)) {
                throw std::runtime_error("--output-dir requires a value");
            }
            options.output_dir = args[++index];
            continue;
        }
        if (arg == "--trigger-frame") {
            if (!has_value(args, index + 1)) {
                throw std::runtime_error("--trigger-frame requires a value");
            }
            options.trigger.mode = TriggerMode::kFrameIndex;
            options.trigger.value = parse_int64(args[++index], "--trigger-frame");
            has_trigger = true;
            continue;
        }
        if (arg == "--trigger-time-ms") {
            if (!has_value(args, index + 1)) {
                throw std::runtime_error("--trigger-time-ms requires a value");
            }
            options.trigger.mode = TriggerMode::kTimestampMs;
            options.trigger.value = parse_int64(args[++index], "--trigger-time-ms");
            has_trigger = true;
            continue;
        }
        if (arg == "--crop") {
            if (!has_value(args, index + 1)) {
                throw std::runtime_error("--crop requires a value");
            }
            options.crop_rect = parse_crop_rect(args[++index]);
            continue;
        }
        if (arg == "--config") {
            if (!has_value(args, index + 1)) {
                throw std::runtime_error("--config requires a value");
            }
            options.config_path = args[++index];
            continue;
        }
        if (arg == "--debug") {
            options.debug_enabled = true;
            continue;
        }

        throw std::runtime_error("unknown argument: " + arg);
    }

    if (options.input_path.empty()) {
        throw std::runtime_error("--input is required");
    }
    if (options.output_dir.empty()) {
        throw std::runtime_error("--output-dir is required");
    }
    if (!has_trigger) {
        throw std::runtime_error("--trigger-frame or --trigger-time-ms is required");
    }

    return options;
}

CompositeConfig CliOptionsParser::make_config_from_options(const CliOptions& options) {
    CompositeConfig config;
    if (options.crop_rect.has_value()) {
        config.crop.enabled = true;
        config.crop.crop_rect = options.crop_rect.value();
    }
    config.debug.enabled = options.debug_enabled;
    return config;
}

std::vector<std::string> CliOptionsParser::collect_args(int argc, char** argv) {
    std::vector<std::string> args;
    args.reserve(static_cast<std::size_t>(argc > 1 ? argc - 1 : 0));
    for (int index = 1; index < argc; ++index) {
        args.emplace_back(argv[index]);
    }
    return args;
}

bool CliOptionsParser::has_value(const std::vector<std::string>& args, std::size_t index) {
    return index < args.size();
}

cv::Rect CliOptionsParser::parse_crop_rect(const std::string& value) {
    std::vector<int> numbers;
    numbers.reserve(4);

    std::size_t start = 0;
    while (start <= value.size()) {
        const std::size_t comma = value.find(',', start);
        const std::string token =
            value.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
        numbers.push_back(parse_int(token, "--crop"));
        if (comma == std::string::npos) {
            break;
        }
        start = comma + 1;
    }

    if (numbers.size() != 4) {
        throw std::runtime_error("--crop expects x,y,width,height");
    }
    if (numbers[2] <= 0 || numbers[3] <= 0) {
        throw std::runtime_error("--crop width and height must be positive");
    }

    return cv::Rect(numbers[0], numbers[1], numbers[2], numbers[3]);
}

std::int64_t
    CliOptionsParser::parse_int64(const std::string& value, const std::string& option_name) {
    std::int64_t parsed = 0;
    const char* begin = value.data();
    const char* end = value.data() + value.size();
    const auto result = std::from_chars(begin, end, parsed);
    if (result.ec != std::errc() || result.ptr != end) {
        throw std::runtime_error(option_name + " expects an integer value");
    }
    return parsed;
}

int CliOptionsParser::parse_int(const std::string& value, const std::string& option_name) {
    const std::int64_t parsed = parse_int64(value, option_name);
    if (parsed < static_cast<std::int64_t>(std::numeric_limits<int>::min())
        || parsed > static_cast<std::int64_t>(std::numeric_limits<int>::max())) {
        throw std::runtime_error(option_name + " integer value is out of range");
    }
    return static_cast<int>(parsed);
}

} // namespace image_trans
