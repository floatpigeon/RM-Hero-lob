#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

void PrintUsage() {
    std::cout << "Usage: hero_lob_compress <input_image> <output_image> [options]\n"
              << "\nOptions:\n"
              << "  --size WxH       Resize to WxH (e.g., --size 55x55)\n"
              << "  --width W        Resize to width W, keep aspect ratio\n"
              << "  --height H       Resize to height H, keep aspect ratio\n"
              << "  --gray           Convert to grayscale\n"
              << "  --quality Q      JPEG quality 1-100 (default: 95)\n"
              << "  --max-bytes N    Limit output to N bytes (e.g., --max-bytes 300)\n"
              << "\nExamples:\n"
              << "  hero_lob_compress input.png output.jpg --size 55x55 --gray\n"
              << "  hero_lob_compress input.png output.jpg --max-bytes 300 --gray\n";
}

bool IsHelpFlag(const std::string& arg) { return arg == "--help" || arg == "-h"; }

struct Options {
    std::string input_path;
    std::string output_path;
    int target_width = 0;
    int target_height = 0;
    bool use_size = false;
    bool use_width = false;
    bool use_height = false;
    bool grayscale = false;
    int quality = 95;
    int max_bytes = 0;
};

bool ParseArguments(int argc, char** argv, Options& opts) {
    if (argc == 2 && IsHelpFlag(argv[1])) {
        PrintUsage();
        return false;
    }

    if (argc < 3) {
        PrintUsage();
        return false;
    }

    opts.input_path = argv[1];
    opts.output_path = argv[2];

    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--size" && i + 1 < argc) {
            ++i;
            std::string size_str = argv[i];
            size_t x_pos = size_str.find('x');
            if (x_pos == std::string::npos) {
                std::cerr << "Invalid size format: " << size_str << " (expected WxH)\n";
                return false;
            }
            opts.target_width = std::stoi(size_str.substr(0, x_pos));
            opts.target_height = std::stoi(size_str.substr(x_pos + 1));
            opts.use_size = true;
        } else if (arg == "--width" && i + 1 < argc) {
            opts.target_width = std::stoi(argv[++i]);
            opts.use_width = true;
        } else if (arg == "--height" && i + 1 < argc) {
            opts.target_height = std::stoi(argv[++i]);
            opts.use_height = true;
        } else if (arg == "--gray") {
            opts.grayscale = true;
        } else if (arg == "--quality" && i + 1 < argc) {
            opts.quality = std::stoi(argv[++i]);
        } else if (arg == "--max-bytes" && i + 1 < argc) {
            opts.max_bytes = std::stoi(argv[++i]);
        } else {
            std::cerr << "Unknown argument: " << arg << '\n';
            PrintUsage();
            return false;
        }
    }

    return true;
}

cv::Mat ResizeImage(const cv::Mat& input, const Options& opts) {
    cv::Mat result = input;

    if (opts.use_size) {
        cv::resize(input, result, cv::Size(opts.target_width, opts.target_height), 0, 0, cv::INTER_AREA);
    } else if (opts.use_width) {
        double scale = static_cast<double>(opts.target_width) / input.cols;
        int new_height = static_cast<int>(input.rows * scale);
        cv::resize(input, result, cv::Size(opts.target_width, new_height), 0, 0, cv::INTER_AREA);
    } else if (opts.use_height) {
        double scale = static_cast<double>(opts.target_height) / input.rows;
        int new_width = static_cast<int>(input.cols * scale);
        cv::resize(input, result, cv::Size(new_width, opts.target_height), 0, 0, cv::INTER_AREA);
    }

    if (opts.grayscale && result.channels() == 3) {
        cv::cvtColor(result, result, cv::COLOR_BGR2GRAY);
    }

    return result;
}

cv::Mat CompressToSize(const cv::Mat& input, int max_bytes, int quality) {
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
    cv::imencode(".jpg", input, buffer, params);

    int current_quality = quality;
    while (static_cast<int>(buffer.size()) > max_bytes && current_quality > 1) {
        current_quality -= 5;
        if (current_quality < 1) current_quality = 1;
        params[1] = current_quality;
        cv::imencode(".jpg", input, buffer, params);
    }

    cv::Mat result = cv::imdecode(buffer, cv::IMREAD_COLOR);
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    const bool help_requested = argc == 2 && IsHelpFlag(argv[1]);

    Options opts;
    if (!ParseArguments(argc, argv, opts)) {
        return help_requested ? 0 : 1;
    }

    const cv::Mat input = cv::imread(opts.input_path, cv::IMREAD_COLOR);
    if (input.empty()) {
        std::cerr << "Failed to read input image: " << opts.input_path << '\n';
        return 1;
    }

    cv::Mat output = ResizeImage(input, opts);

    if (opts.max_bytes > 0) {
        output = CompressToSize(output, opts.max_bytes, opts.quality);
    }

    if (output.empty()) {
        std::cerr << "Failed to process image\n";
        return 1;
    }

    if (!cv::imwrite(opts.output_path, output, {cv::IMWRITE_JPEG_QUALITY, opts.quality})) {
        std::cerr << "Failed to write output image: " << opts.output_path << '\n';
        return 1;
    }

    std::vector<uchar> final_buffer;
    cv::imencode(".jpg", output, final_buffer, {cv::IMWRITE_JPEG_QUALITY, opts.quality});

    std::cout << "Input: " << opts.input_path << " (" << input.cols << "x" << input.rows << ")\n"
              << "Output: " << opts.output_path << " (" << output.cols << "x" << output.rows << ")\n"
              << "Grayscale: " << (opts.grayscale ? "yes" : "no") << '\n'
              << "File size: " << final_buffer.size() << " bytes\n";
    return 0;
}
