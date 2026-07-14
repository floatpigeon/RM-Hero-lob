#include <iostream>
#include <string>

#include <opencv2/videoio.hpp>

namespace {

void PrintUsage() {
    std::cout << "Usage: hero_lob_video_trimmer <input_avi> <output_avi> --begin <start_frame> --end <end_frame>\n"
              << "Trim an AVI video by frame range, preserving original frame rate and resolution.\n"
              << "\nOptions:\n"
              << "  --begin N    Start frame number (0-based, inclusive)\n"
              << "  --end N      End frame number (0-based, inclusive)\n";
}

bool IsHelpFlag(const std::string& arg) { return arg == "--help" || arg == "-h"; }

struct Options {
    std::string input_path;
    std::string output_path;
    int begin_frame = 0;
    int end_frame = -1;
    bool has_begin = false;
    bool has_end = false;
};

bool ParseArguments(int argc, char** argv, Options& opts) {
    if (argc == 2 && IsHelpFlag(argv[1])) {
        PrintUsage();
        return false;
    }

    if (argc < 2) {
        PrintUsage();
        return false;
    }

    opts.input_path = argv[1];

    if (argc >= 3 && !IsHelpFlag(argv[2])) {
        opts.output_path = argv[2];
    }

    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--begin" && i + 1 < argc) {
            opts.begin_frame = std::stoi(argv[++i]);
            opts.has_begin = true;
        } else if (arg == "--end" && i + 1 < argc) {
            opts.end_frame = std::stoi(argv[++i]);
            opts.has_end = true;
        } else {
            std::cerr << "Unknown argument: " << arg << '\n';
            PrintUsage();
            return false;
        }
    }

    if (opts.output_path.empty()) {
        std::cerr << "Output path is required\n";
        PrintUsage();
        return false;
    }

    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const bool help_requested = argc == 2 && IsHelpFlag(argv[1]);

    Options opts;
    if (!ParseArguments(argc, argv, opts)) {
        return help_requested ? 0 : 1;
    }

    cv::VideoCapture cap(opts.input_path);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open video: " << opts.input_path << '\n';
        return 1;
    }

    const int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    const double fps = cap.get(cv::CAP_PROP_FPS);
    const int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    if (!opts.has_begin) {
        opts.begin_frame = 0;
    }
    if (!opts.has_end) {
        opts.end_frame = total_frames - 1;
    }

    if (opts.begin_frame < 0 || opts.begin_frame >= total_frames) {
        std::cerr << "Begin frame " << opts.begin_frame << " is out of range [0, " << total_frames - 1 << "]\n";
        return 1;
    }

    if (opts.end_frame < opts.begin_frame || opts.end_frame >= total_frames) {
        std::cerr << "End frame " << opts.end_frame << " is out of range [" << opts.begin_frame << ", " << total_frames - 1 << "]\n";
        return 1;
    }

    const int frame_count = opts.end_frame - opts.begin_frame + 1;

    std::cout << "Input: " << opts.input_path << '\n'
              << "Resolution: " << width << "x" << height << '\n'
              << "FPS: " << fps << '\n'
              << "Total frames: " << total_frames << '\n'
              << "Trim range: [" << opts.begin_frame << ", " << opts.end_frame << "]\n"
              << "Output frames: " << frame_count << '\n';

    cv::VideoWriter writer(
        opts.output_path,
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        fps,
        cv::Size(width, height));

    if (!writer.isOpened()) {
        std::cerr << "Failed to open output video writer: " << opts.output_path << '\n';
        return 1;
    }

    cv::Mat frame;
    int frame_idx = 0;
    int written = 0;

    while (cap.read(frame)) {
        if (frame_idx >= opts.begin_frame && frame_idx <= opts.end_frame) {
            writer.write(frame);
            ++written;
        }

        if (frame_idx > opts.end_frame) {
            break;
        }

        if (written % 100 == 0 && written > 0) {
            std::cout << "\rWritten " << written << " / " << frame_count << " frames" << std::flush;
        }

        ++frame_idx;
    }

    writer.release();
    cap.release();

    std::cout << "\nDone. Trimmed " << written << " frames to " << opts.output_path << '\n';
    return 0;
}
