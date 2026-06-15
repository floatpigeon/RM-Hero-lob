#include <iostream>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

void PrintUsage() {
    std::cout << "Usage: hero_lob_brightness_filter <input_image> <threshold> [output_image]\n"
              << "Output a grayscale image where pixels with brightness >= threshold are kept,\n"
              << "others are set to black.\n";
}

bool IsHelpFlag(const std::string& arg) { return arg == "--help" || arg == "-h"; }

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && IsHelpFlag(argv[1])) {
        PrintUsage();
        return 0;
    }

    if (argc != 3 && argc != 4) {
        PrintUsage();
        return 1;
    }

    const std::string input_path = argv[1];

    int threshold = 0;
    try {
        threshold = std::stoi(argv[2]);
    } catch (...) {
        std::cerr << "Invalid threshold value: " << argv[2] << '\n';
        return 1;
    }

    if (threshold < 0 || threshold > 255) {
        std::cerr << "Threshold must be in [0, 255], got: " << threshold << '\n';
        return 1;
    }

    const std::string output_path = (argc == 4) ? argv[3] : "filtered.png";

    const cv::Mat input = cv::imread(input_path, cv::IMREAD_COLOR);
    if (input.empty()) {
        std::cerr << "Failed to read input image: " << input_path << '\n';
        return 1;
    }

    cv::Mat hsv;
    cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);

    cv::Mat v_channel = channels[2];

    cv::Mat mask;
    cv::threshold(v_channel, mask, threshold, 255, cv::THRESH_BINARY);

    cv::Mat gray;
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);

    cv::Mat output;
    cv::bitwise_and(gray, mask, output);

    if (!cv::imwrite(output_path, output)) {
        std::cerr << "Failed to write output image: " << output_path << '\n';
        return 1;
    }

    std::cout << "Input: " << input_path << " (" << input.cols << "x" << input.rows << ")\n"
              << "Threshold: " << threshold << "\n"
              << "Output: " << output_path << "\n";

    return 0;
}
