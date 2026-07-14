#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {

void PrintUsage() {
    std::cout
        << "Usage: hero_lob_median_background <input_video> <output_image>\n"
        << "       [--max-frames N] [--step N]\n"
        << "Extract background using median across frames.\n";
}

cv::Mat ComputeMedianBackground(const std::vector<cv::Mat>& frames) {
    if (frames.empty()) {
        return {};
    }
    const int height = frames[0].rows;
    const int width = frames[0].cols;
    const int channels = frames[0].channels();
    const int count = static_cast<int>(frames.size());
    const int pixels = height * width;

    cv::Mat result(height, width, frames[0].type(), cv::Scalar::all(0));
    std::vector<unsigned char> buffer(pixels * channels * count);

    for (int f = 0; f < count; ++f) {
        const unsigned char* src = frames[f].ptr<unsigned char>(0);
        unsigned char* dst = buffer.data() + f * pixels * channels;
        std::memcpy(dst, src, pixels * channels);
    }

    unsigned char* result_data = result.ptr<unsigned char>(0);
    std::vector<unsigned char> channel_values(count);

    for (int i = 0; i < pixels; ++i) {
        for (int c = 0; c < channels; ++c) {
            for (int f = 0; f < count; ++f) {
                channel_values[f] = buffer[f * pixels * channels + i * channels + c];
            }
            std::nth_element(
                channel_values.begin(),
                channel_values.begin() + count / 2,
                channel_values.end());
            result_data[i * channels + c] = channel_values[count / 2];
        }
    }

    return result;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 &&
        (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
        PrintUsage();
        return 0;
    }
    if (argc < 3) {
        PrintUsage();
        return 1;
    }

    const std::string input_path = argv[1];
    const std::string output_path = argv[2];
    int max_frames = 0;
    int step = 1;
    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--max-frames" && i + 1 < argc) {
            max_frames = std::stoi(argv[++i]);
        } else if (arg == "--step" && i + 1 < argc) {
            step = std::stoi(argv[++i]);
        }
    }

    auto start_time = std::chrono::steady_clock::now();

    cv::VideoCapture cap(input_path);
    if (!cap.isOpened()) {
        std::cerr << "[MedianBackground] Failed to open video: " << input_path << '\n';
        return 1;
    }

    const double fps = cap.get(cv::CAP_PROP_FPS);
    const int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    std::cerr << "[MedianBackground] Video: " << input_path << '\n'
              << "[MedianBackground] FPS: " << fps << '\n'
              << "[MedianBackground] Total frames: " << total_frames << '\n';

    auto read_start = std::chrono::steady_clock::now();
    std::vector<cv::Mat> frames;
    cv::Mat frame;
    int frame_index = 0;

    while (cap.read(frame)) {
        if (frame_index % step == 0) {
            frames.push_back(frame.clone());
            if (max_frames > 0 && static_cast<int>(frames.size()) >= max_frames) {
                break;
            }
        }
        ++frame_index;
    }
    cap.release();

    auto read_end = std::chrono::steady_clock::now();
    double read_elapsed = std::chrono::duration<double>(read_end - read_start).count();
    std::cerr << "[MedianBackground] Collected " << frames.size() << " frames (step=" << step
              << ") in " << read_elapsed << "s\n";

    if (frames.empty()) {
        std::cerr << "[MedianBackground] No frames collected\n";
        return 1;
    }

    auto compute_start = std::chrono::steady_clock::now();
    std::cerr << "[MedianBackground] Computing median background...\n";
    cv::Mat background = ComputeMedianBackground(frames);
    auto compute_end = std::chrono::steady_clock::now();
    double compute_elapsed = std::chrono::duration<double>(compute_end - compute_start).count();
    std::cerr << "[MedianBackground] Median computation completed in " << compute_elapsed << "s\n";

    if (background.empty()) {
        std::cerr << "[MedianBackground] Failed to compute median background\n";
        return 1;
    }

    auto write_start = std::chrono::steady_clock::now();
    if (!cv::imwrite(output_path, background)) {
        std::cerr << "[MedianBackground] Failed to write output: " << output_path << '\n';
        return 1;
    }
    auto write_end = std::chrono::steady_clock::now();
    double write_elapsed = std::chrono::duration<double>(write_end - write_start).count();

    auto end_time = std::chrono::steady_clock::now();
    double total_elapsed = std::chrono::duration<double>(end_time - start_time).count();

    std::cerr << "[MedianBackground] Output: " << output_path << " (" << background.cols << "x"
              << background.rows << ")\n"
              << "[MedianBackground] Write completed in " << write_elapsed << "s\n"
              << "[MedianBackground] Total elapsed: " << total_elapsed << "s\n";
    return 0;
}
