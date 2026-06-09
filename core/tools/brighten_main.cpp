#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

enum class BrightenMode {
    kFixed,
    kAuto,
};

void PrintUsage() {
    std::cout << "Usage: hero_lob_brighten <input_image> <output_image> [--auto]\n"
              << "Default mode applies fixed V-channel enhancement.\n"
              << "Use --auto to apply CLAHE-based automatic enhancement.\n";
}

const char* ModeName(BrightenMode mode) {
    switch (mode) {
        case BrightenMode::kFixed: return "fixed";
        case BrightenMode::kAuto: return "auto";
    }

    return "unknown";
}

bool IsHelpFlag(const std::string& argument) { return argument == "--help" || argument == "-h"; }

bool ParseArguments(
    int argc, char** argv, std::string& input_path, std::string& output_path, BrightenMode& mode) {
    if (argc == 2 && IsHelpFlag(argv[1])) {
        PrintUsage();
        return false;
    }

    if (argc != 3 && argc != 4) {
        PrintUsage();
        return false;
    }

    input_path = argv[1];
    output_path = argv[2];
    mode = BrightenMode::kFixed;

    if (argc == 4) {
        if (std::string(argv[3]) != "--auto") {
            PrintUsage();
            return false;
        }
        mode = BrightenMode::kAuto;
    }

    return true;
}

cv::Mat ApplyFixedBrighten(const cv::Mat& input_bgr) {
    cv::Mat hsv_image;
    cv::cvtColor(input_bgr, hsv_image, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv_image, channels);
    channels[2].convertTo(channels[2], channels[2].type(), 1.35, 18.0);
    cv::merge(channels, hsv_image);

    cv::Mat output_bgr;
    cv::cvtColor(hsv_image, output_bgr, cv::COLOR_HSV2BGR);
    return output_bgr;
}

cv::Mat ApplyAutoBrighten(const cv::Mat& input_bgr) {
    cv::Mat lab_image;
    cv::cvtColor(input_bgr, lab_image, cv::COLOR_BGR2Lab);

    std::vector<cv::Mat> channels;
    cv::split(lab_image, channels);

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.5, cv::Size(8, 8));
    clahe->apply(channels[0], channels[0]);

    cv::merge(channels, lab_image);

    cv::Mat output_bgr;
    cv::cvtColor(lab_image, output_bgr, cv::COLOR_Lab2BGR);
    return output_bgr;
}

}  // namespace

int main(int argc, char** argv) {
    const bool help_requested = argc == 2 && IsHelpFlag(argv[1]);

    std::string input_path;
    std::string output_path;
    BrightenMode mode = BrightenMode::kFixed;
    if (!ParseArguments(argc, argv, input_path, output_path, mode)) {
        return help_requested ? 0 : 1;
    }

    const cv::Mat input_bgr = cv::imread(input_path, cv::IMREAD_COLOR);
    if (input_bgr.empty()) {
        std::cerr << "Failed to read input image: " << input_path << '\n';
        return 1;
    }

    const cv::Mat output_bgr =
        mode == BrightenMode::kAuto ? ApplyAutoBrighten(input_bgr) : ApplyFixedBrighten(input_bgr);

    if (output_bgr.empty()) {
        std::cerr << "Failed to enhance input image: " << input_path << '\n';
        return 1;
    }

    if (!cv::imwrite(output_path, output_bgr)) {
        std::cerr << "Failed to write output image: " << output_path << '\n';
        return 1;
    }

    std::cout << "Mode: " << ModeName(mode) << '\n'
              << "Input: " << input_path << '\n'
              << "Output: " << output_path << '\n'
              << "Size: " << output_bgr.cols << "x" << output_bgr.rows << '\n';
    return 0;
}
