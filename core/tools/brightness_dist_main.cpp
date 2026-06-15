#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

constexpr int kNumBins = 16;
constexpr int kBinWidth = 256 / kNumBins;
constexpr int kGrayStep = 255 / (kNumBins - 1);

struct HistOptions {
    std::string input_path;
    std::string output_path;
    bool show_text = false;
};

void PrintUsage() {
    std::cout
        << "Usage: hero_lob_brightness_dist <input_image> [output_image] [--text]\n"
        << "Quantize image brightness into " << kNumBins << " grayscale levels.\n"
        << "Output is a spatial brightness map (default) or printed to stdout (--text).\n";
}

bool ParseArguments(int argc, char** argv, HistOptions& options) {
    if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
        PrintUsage();
        return false;
    }

    if (argc < 2 || argc > 4) {
        PrintUsage();
        return false;
    }

    options.input_path = argv[1];
    options.output_path = "brightness_hist.png";

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--text") {
            options.show_text = true;
        } else {
            options.output_path = arg;
        }
    }

    return true;
}

cv::Mat QuantizeBrightness(const cv::Mat& bgr_image) {
    cv::Mat hsv_image;
    cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv_image, channels);

    cv::Mat v_channel = channels[2];

    cv::Mat quantized(v_channel.size(), CV_8UC1);

    for (int y = 0; y < v_channel.rows; ++y) {
        const uchar* src_row = v_channel.ptr<uchar>(y);
        uchar* dst_row = quantized.ptr<uchar>(y);
        for (int x = 0; x < v_channel.cols; ++x) {
            const int bin = std::min(src_row[x] / kBinWidth, kNumBins - 1);
            dst_row[x] = static_cast<uchar>(bin * kGrayStep);
        }
    }

    return quantized;
}

std::array<int, kNumBins> ComputeHistogram(const cv::Mat& quantized) {
    std::array<int, kNumBins> histogram{};
    for (int y = 0; y < quantized.rows; ++y) {
        const uchar* row = quantized.ptr<uchar>(y);
        for (int x = 0; x < quantized.cols; ++x) {
            const int bin = row[x] / kGrayStep;
            ++histogram[std::min(bin, kNumBins - 1)];
        }
    }
    return histogram;
}

void PrintHistogramText(const std::array<int, kNumBins>& histogram) {
    const int max_count = *std::max_element(histogram.begin(), histogram.end());
    const int kBarMaxWidth = 50;

    std::cout << "Brightness Distribution (16 bins, V channel)\n";
    std::cout << "============================================\n";

    for (int i = 0; i < kNumBins; ++i) {
        const int lo = i * kBinWidth;
        const int hi = lo + kBinWidth - 1;

        const int bar_len = max_count > 0
            ? static_cast<int>(static_cast<double>(histogram[i]) / static_cast<double>(max_count) * kBarMaxWidth)
            : 0;

        std::string bar(static_cast<size_t>(bar_len), '#');

        std::cout << "[" << (lo < 10 ? "  " : (lo < 100 ? " " : "")) << lo
                  << "-" << (hi < 10 ? " " : (hi < 100 ? " " : "")) << hi
                  << "] " << bar << " " << histogram[i] << "\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    const bool help_requested = argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h");

    HistOptions options;
    if (!ParseArguments(argc, argv, options)) {
        return help_requested ? 0 : 1;
    }

    const cv::Mat input = cv::imread(options.input_path, cv::IMREAD_COLOR);
    if (input.empty()) {
        std::cerr << "Failed to read input image: " << options.input_path << '\n';
        return 1;
    }

    const cv::Mat quantized = QuantizeBrightness(input);

    const std::array<int, kNumBins> histogram = ComputeHistogram(quantized);

    if (options.show_text) {
        PrintHistogramText(histogram);
        return 0;
    }

    if (!cv::imwrite(options.output_path, quantized)) {
        std::cerr << "Failed to write output image: " << options.output_path << '\n';
        return 1;
    }

    std::cout << "Input: " << options.input_path << " (" << input.cols << "x" << input.rows << ")\n"
              << "Output: " << options.output_path << "\n"
              << "Total pixels: " << (input.cols * input.rows) << "\n";

    PrintHistogramText(histogram);

    return 0;
}
