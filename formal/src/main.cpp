#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include "image_trans/long_exposure_composer.h"
#include "image_trans/video_stabilizer.h"

namespace {

struct CliOptions {
    std::string input_path;
    std::string output_dir = "outputs";
    std::string output_prefix;
    std::string fourcc = "mp4v";
    image_trans::StabilizationParams stabilization_params;
    image_trans::LongExposureParams exposure_params;
};

CliOptions parseArgs(int argc, char** argv) {
    CliOptions options;
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        auto requireValue = [&](const std::string& flag) -> std::string {
            if (index + 1 >= argc) {
                throw std::invalid_argument("Missing value for " + flag);
            }
            ++index;
            return argv[index];
        };

        if (arg == "--input") {
            options.input_path = requireValue(arg);
        } else if (arg == "--output-dir") {
            options.output_dir = requireValue(arg);
        } else if (arg == "--output-prefix") {
            options.output_prefix = requireValue(arg);
        } else if (arg == "--fourcc") {
            options.fourcc = requireValue(arg);
        } else if (arg == "--max-corners") {
            options.stabilization_params.max_corners = std::stoi(requireValue(arg));
        } else if (arg == "--quality-level") {
            options.stabilization_params.quality_level = std::stod(requireValue(arg));
        } else if (arg == "--min-distance") {
            options.stabilization_params.min_distance = std::stod(requireValue(arg));
        } else if (arg == "--lk-window") {
            options.stabilization_params.lk_window = std::stoi(requireValue(arg));
        } else if (arg == "--lk-max-level") {
            options.stabilization_params.lk_max_level = std::stoi(requireValue(arg));
        } else if (arg == "--ransac-threshold") {
            options.stabilization_params.ransac_reproj_threshold = std::stod(requireValue(arg));
        } else if (arg == "--min-inliers") {
            options.stabilization_params.min_inliers = std::stoi(requireValue(arg));
        } else if (arg == "--smoothing-radius") {
            options.stabilization_params.smoothing_radius = std::stoi(requireValue(arg));
        } else if (arg == "--disable-auto-crop") {
            options.stabilization_params.auto_crop = false;
        } else if (arg == "--brightness-threshold") {
            options.exposure_params.brightness_threshold = std::stod(requireValue(arg));
        } else {
            throw std::invalid_argument("Unknown argument: " + arg);
        }
    }

    if (options.input_path.empty()) {
        throw std::invalid_argument("Usage: image_trans_cli --input <video_path> [options]");
    }

    if (options.output_prefix.empty()) {
        options.output_prefix = std::filesystem::path(options.input_path).stem().string();
    }

    if (options.fourcc.size() != 4U) {
        throw std::invalid_argument("--fourcc must be exactly four characters");
    }

    return options;
}

std::vector<cv::Mat> readFrames(const std::string& input_path, double* fps_out) {
    cv::VideoCapture capture(input_path);
    if (!capture.isOpened()) {
        throw std::runtime_error("Failed to open input video: " + input_path);
    }

    std::vector<cv::Mat> frames;
    cv::Mat frame;
    while (capture.read(frame)) {
        if (frame.empty()) {
            continue;
        }
        frames.push_back(frame.clone());
    }

    if (frames.empty()) {
        throw std::runtime_error("Input video did not yield any frames");
    }

    double fps = capture.get(cv::CAP_PROP_FPS);
    if (fps <= 1.0) {
        fps = 30.0;
    }
    *fps_out = fps;
    return frames;
}

void writeVideo(const std::filesystem::path& output_path,
                const std::vector<cv::Mat>& frames,
                double fps,
                const std::string& fourcc) {
    const int codec = cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
    cv::VideoWriter writer(output_path.string(), codec, fps, frames.front().size(), true);
    if (!writer.isOpened()) {
        throw std::runtime_error("Failed to open output video: " + output_path.string());
    }

    for (const cv::Mat& frame : frames) {
        writer.write(frame);
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const CliOptions options = parseArgs(argc, argv);
        std::filesystem::create_directories(options.output_dir);

        double fps = 0.0;
        const std::vector<cv::Mat> frames = readFrames(options.input_path, &fps);

        image_trans::VideoStabilizer stabilizer(options.stabilization_params);
        image_trans::MotionPlan plan;
        const std::vector<cv::Mat> stabilized = stabilizer.stabilize(frames, &plan);

        image_trans::LongExposureComposer composer(options.exposure_params);
        composer.reset(stabilized.front().size());
        for (const cv::Mat& frame : stabilized) {
            composer.accumulate(frame);
        }

        const cv::Mat exposure = composer.finalize();

        const std::filesystem::path output_dir(options.output_dir);
        const std::filesystem::path video_path = output_dir / (options.output_prefix + "_stabilized.mp4");
        const std::filesystem::path image_path = output_dir / (options.output_prefix + "_exposure.png");

        writeVideo(video_path, stabilized, fps, options.fourcc);
        if (!cv::imwrite(image_path.string(), exposure)) {
            throw std::runtime_error("Failed to write output image: " + image_path.string());
        }

        std::cout << "Frames: " << stabilized.size() << '\n';
        std::cout << "Fallback frames: " << plan.stats.fallback_frame_count << '\n';
        std::cout << "Output video: " << video_path << '\n';
        std::cout << "Output image: " << image_path << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
