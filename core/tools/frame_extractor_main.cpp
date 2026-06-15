#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

namespace {

void PrintUsage() {
    std::cout << "Usage: hero_lob_frame_extractor <input_video> <output_folder>\n"
              << "Extract every frame from the video and save as PNG images.\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
        PrintUsage();
        return 0;
    }

    if (argc != 3) {
        PrintUsage();
        return 1;
    }

    const std::string input_path = argv[1];
    const std::string output_dir = argv[2];

    cv::VideoCapture cap(input_path);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open video: " << input_path << '\n';
        return 1;
    }

    std::filesystem::create_directories(output_dir);

    const int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    const double fps = cap.get(cv::CAP_PROP_FPS);
    const int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    std::cout << "Video: " << input_path << '\n'
              << "Resolution: " << width << "x" << height << '\n'
              << "FPS: " << fps << '\n'
              << "Total frames: " << total_frames << '\n'
              << "Output: " << output_dir << '\n';

    cv::Mat frame;
    int frame_idx = 0;

    while (cap.read(frame)) {
        std::ostringstream oss;
        oss << output_dir << "/frame_" << std::setw(6) << std::setfill('0') << frame_idx
            << ".png";
        const std::string output_path = oss.str();

        if (!cv::imwrite(output_path, frame)) {
            std::cerr << "Failed to write frame " << frame_idx << " to " << output_path << '\n';
            continue;
        }

        if (frame_idx % 100 == 0) {
            std::cout << "\rExtracted frame " << frame_idx << " / " << total_frames << std::flush;
        }

        ++frame_idx;
    }

    std::cout << "\nDone. Extracted " << frame_idx << " frames to " << output_dir << '\n';
    return 0;
}
