#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "background_remover.hpp"
#include "capture.hpp"
#include "image_registrator.hpp"
#include "image_synthesis.hpp"
#include "reference_frame_selector.hpp"
#include "tracker_processor.hpp"

namespace {

namespace fs = std::filesystem;

constexpr char kWindowName[] = "Hero Lob Trajectory Debug";
constexpr int kDisplayWidth = 1440;
constexpr int kDisplayHeight = 1080;

enum class ViewMode {
    kOriginal,
    kCandidateMask,
    kTrajectoryExposure,
    kTrajectoryHeatmap,
    kTrajectoryOverlay,
};

struct ParsedArguments {
    std::string input_video;
    std::string output_dir;
    bool gui = false;
    int max_frames = -1;
};

struct FrameSummary {
    std::string frame_name;
    std::int64_t frame_index = -1;
    double timestamp_seconds = 0.0;
    int candidate_pixels = 0;
    int trajectory_pixels = 0;
    int accumulated_frames = 0;
};

struct FrameArtifacts {
    cv::Mat original;
    cv::Mat candidate_mask;
    cv::Mat trajectory_exposure;
    cv::Mat trajectory_heatmap;
    cv::Mat trajectory_overlay;
    FrameSummary summary;
};

bool IsHelpFlag(const std::string& argument) { return argument == "--help" || argument == "-h"; }

void PrintUsage() {
    std::cout
        << "Usage:\n"
        << "  hero_lob_trajectory_debug <input_video> <output_dir> [--gui] [--max-frames N]\n";
}

bool ParseArguments(int argc, char** argv, ParsedArguments& parsed) {
    if (argc == 2 && IsHelpFlag(argv[1])) {
        PrintUsage();
        return false;
    }

    if (argc < 3) {
        PrintUsage();
        return false;
    }

    parsed.input_video = argv[1];
    parsed.output_dir = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--gui") {
            parsed.gui = true;
            continue;
        }
        if (argument == "--max-frames" && index + 1 < argc) {
            parsed.max_frames = std::stoi(argv[++index]);
            continue;
        }

        PrintUsage();
        return false;
    }

    return true;
}

bool EnsureDirectory(const fs::path& directory) {
    std::error_code error;
    if (fs::exists(directory, error)) {
        return fs::is_directory(directory, error);
    }
    return fs::create_directories(directory, error);
}

double ComputeDisplayScale(const cv::Size& size) {
    const double width_scale =
        static_cast<double>(kDisplayWidth) / static_cast<double>(size.width);
    const double height_scale =
        static_cast<double>(kDisplayHeight) / static_cast<double>(size.height);
    return std::min(1.0, std::min(width_scale, height_scale));
}

cv::Mat ResizeForDisplay(const cv::Mat& image) {
    if (image.empty()) {
        return image;
    }

    const double scale = ComputeDisplayScale(image.size());
    if (scale >= 1.0) {
        return image.clone();
    }

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(), scale, scale, cv::INTER_AREA);
    return resized;
}

cv::Mat ToBgrMask(const cv::Mat& mask) {
    if (mask.empty()) {
        return mask;
    }

    if (mask.channels() == 1) {
        cv::Mat bgr;
        cv::cvtColor(mask, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }

    return mask.clone();
}

cv::Mat MakeHeatmap(const cv::Mat& exposure) {
    if (exposure.empty()) {
        return exposure;
    }

    cv::Mat heatmap;
    cv::applyColorMap(exposure, heatmap, cv::COLORMAP_TURBO);
    return heatmap;
}

std::string ViewName(ViewMode mode) {
    switch (mode) {
        case ViewMode::kOriginal: return "original";
        case ViewMode::kCandidateMask: return "candidate_mask";
        case ViewMode::kTrajectoryExposure: return "trajectory_exposure";
        case ViewMode::kTrajectoryHeatmap: return "trajectory_heatmap";
        case ViewMode::kTrajectoryOverlay: return "trajectory_overlay";
    }
    return "unknown";
}

cv::Mat ViewImage(const FrameArtifacts& artifacts, ViewMode mode) {
    switch (mode) {
        case ViewMode::kOriginal: return artifacts.original;
        case ViewMode::kCandidateMask: return ToBgrMask(artifacts.candidate_mask);
        case ViewMode::kTrajectoryExposure: return ToBgrMask(artifacts.trajectory_exposure);
        case ViewMode::kTrajectoryHeatmap: return artifacts.trajectory_heatmap;
        case ViewMode::kTrajectoryOverlay: return artifacts.trajectory_overlay;
    }
    return artifacts.original;
}

void PutHud(
    cv::Mat& image,
    const FrameArtifacts& artifacts,
    ViewMode mode,
    std::size_t frame_offset,
    std::size_t total_frames,
    bool playing) {
    const std::string line =
        artifacts.summary.frame_name + " | " + ViewName(mode) + " | " +
        "candidate=" + std::to_string(artifacts.summary.candidate_pixels) + " | " +
        "trajectory=" + std::to_string(artifacts.summary.trajectory_pixels) + " | " +
        "window=" + std::to_string(artifacts.summary.accumulated_frames) + " | " +
        (playing ? "playing" : "paused") + " | " +
        std::to_string(frame_offset + 1) + "/" + std::to_string(total_frames);

    cv::rectangle(
        image,
        cv::Rect(10, 10, std::min(image.cols - 20, 1100), 40),
        cv::Scalar(0, 0, 0),
        cv::FILLED);
    cv::putText(
        image,
        line,
        cv::Point(20, 38),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);
}

void RunVideoGui(const std::vector<FrameArtifacts>& frames) {
    if (frames.empty()) {
        return;
    }

    std::vector<ViewMode> modes = {
        ViewMode::kTrajectoryOverlay,
        ViewMode::kTrajectoryHeatmap,
        ViewMode::kTrajectoryExposure,
        ViewMode::kCandidateMask,
        ViewMode::kOriginal,
    };

    std::size_t mode_index = 0;
    std::size_t frame_index = 0;
    bool playing = false;

    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
    while (true) {
        cv::Mat image = ResizeForDisplay(ViewImage(frames[frame_index], modes[mode_index]));
        PutHud(image, frames[frame_index], modes[mode_index], frame_index, frames.size(), playing);
        cv::imshow(kWindowName, image);

        const int delay_ms = playing ? 33 : 0;
        const int key = cv::waitKey(delay_ms);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }
        if (key == ' ') {
            playing = !playing;
        } else if (key == 'n' || key == 'N' || key == 'd' || key == 'D') {
            frame_index = std::min(frame_index + 1, frames.size() - 1);
            playing = false;
        } else if (key == 'p' || key == 'P' || key == 'a' || key == 'A') {
            frame_index = frame_index == 0 ? 0 : frame_index - 1;
            playing = false;
        } else if (key == 'v' || key == 'V') {
            mode_index = (mode_index + 1) % modes.size();
        } else if (playing) {
            frame_index = (frame_index + 1) % frames.size();
        }

        const double visible = cv::getWindowProperty(kWindowName, cv::WND_PROP_VISIBLE);
        if (visible < 1.0) {
            break;
        }
    }

    cv::destroyWindow(kWindowName);
}

bool WriteSummary(
    const fs::path& output_dir,
    const std::vector<FrameSummary>& summaries,
    const FrameArtifacts& last_artifacts) {
    std::ofstream summary_txt(output_dir / "summary.txt");
    if (!summary_txt.is_open()) {
        return false;
    }

    double max_value = 0.0;
    if (!last_artifacts.trajectory_exposure.empty()) {
        cv::minMaxLoc(last_artifacts.trajectory_exposure, nullptr, &max_value);
    }

    summary_txt
        << "processed_frames," << summaries.size() << '\n'
        << "last_frame_name," << last_artifacts.summary.frame_name << '\n'
        << "last_frame_index," << last_artifacts.summary.frame_index << '\n'
        << "last_timestamp_seconds," << last_artifacts.summary.timestamp_seconds << '\n'
        << "last_candidate_pixels," << last_artifacts.summary.candidate_pixels << '\n'
        << "last_trajectory_pixels," << last_artifacts.summary.trajectory_pixels << '\n'
        << "last_accumulated_frames," << last_artifacts.summary.accumulated_frames << '\n'
        << "exposure_max_intensity," << max_value << '\n';

    std::ofstream summary_csv(output_dir / "summary.csv");
    if (!summary_csv.is_open()) {
        return false;
    }

    summary_csv
        << "frame_name,frame_index,timestamp_seconds,candidate_pixels,trajectory_pixels,"
        << "accumulated_frames\n";
    for (const FrameSummary& summary : summaries) {
        summary_csv << summary.frame_name << ','
                    << summary.frame_index << ','
                    << summary.timestamp_seconds << ','
                    << summary.candidate_pixels << ','
                    << summary.trajectory_pixels << ','
                    << summary.accumulated_frames << '\n';
    }

    return true;
}

bool WriteArtifacts(
    const fs::path& output_dir,
    const cv::Mat& first_frame,
    const std::vector<FrameSummary>& summaries,
    const FrameArtifacts& last_artifacts) {
    if (!cv::imwrite((output_dir / "first_frame.png").string(), first_frame)) {
        return false;
    }
    if (!cv::imwrite(
            (output_dir / "candidate_mask_last.png").string(),
            last_artifacts.candidate_mask)) {
        return false;
    }
    if (!cv::imwrite(
            (output_dir / "trajectory_exposure.png").string(),
            last_artifacts.trajectory_exposure)) {
        return false;
    }
    if (!cv::imwrite(
            (output_dir / "trajectory_heatmap.png").string(),
            last_artifacts.trajectory_heatmap)) {
        return false;
    }
    if (!cv::imwrite(
            (output_dir / "trajectory_overlay.png").string(),
            last_artifacts.trajectory_overlay)) {
        return false;
    }

    return WriteSummary(output_dir, summaries, last_artifacts);
}

FrameArtifacts MakeFrameArtifacts(
    const hero_lob::FrameData& frame,
    const hero_lob::ForegroundMaskResult& foreground,
    const hero_lob::TrajectoryResult& trajectory,
    const hero_lob::SynthesisResult& synthesis) {
    FrameArtifacts artifacts;
    artifacts.original = frame.bgr.clone();
    artifacts.summary.frame_index = frame.frame_index;
    artifacts.summary.timestamp_seconds = frame.timestamp_seconds;

    std::ostringstream frame_name;
    frame_name << "frame_" << std::setw(6) << std::setfill('0') << frame.frame_index;
    artifacts.summary.frame_name = frame_name.str();

    const cv::Size frame_size = frame.bgr.size();
    artifacts.candidate_mask = cv::Mat::zeros(frame_size, CV_8UC1);
    artifacts.trajectory_exposure = cv::Mat::zeros(frame_size, CV_8UC1);
    artifacts.trajectory_overlay = frame.bgr.clone();

    if (foreground.valid && !foreground.candidate_mask.empty()) {
        artifacts.candidate_mask = foreground.candidate_mask.clone();
        artifacts.summary.candidate_pixels = cv::countNonZero(artifacts.candidate_mask);
    }

    if (trajectory.valid && !trajectory.trajectory_layer.empty()) {
        artifacts.trajectory_exposure = trajectory.trajectory_layer.clone();
        artifacts.summary.trajectory_pixels = cv::countNonZero(artifacts.trajectory_exposure);
        artifacts.summary.accumulated_frames = trajectory.accumulated_frames;
    }

    if (synthesis.valid && !synthesis.output_image.empty()) {
        artifacts.trajectory_overlay = synthesis.output_image.clone();
    }

    artifacts.trajectory_heatmap = MakeHeatmap(artifacts.trajectory_exposure);
    return artifacts;
}

}  // namespace

int main(int argc, char** argv) {
    const bool help_requested = argc == 2 && IsHelpFlag(argv[1]);

    ParsedArguments arguments;
    if (!ParseArguments(argc, argv, arguments)) {
        return help_requested ? 0 : 1;
    }

    const fs::path output_dir(arguments.output_dir);
    if (!EnsureDirectory(output_dir)) {
        std::cerr << "Failed to create output directory: " << output_dir << '\n';
        return 1;
    }

    hero_lob::PipelineConfig config;
    hero_lob::Capture capture(config);
    hero_lob::ReferenceFrameSelector reference_frame_selector(config);
    hero_lob::ImageRegistrator image_registrator(config);
    hero_lob::BackgroundRemover background_remover(config);
    hero_lob::TrackerProcessor tracker_processor(config);
    hero_lob::ImageSynthesis image_synthesis(config);

    if (!capture.Open(arguments.input_video)) {
        std::cerr << "Failed to open input video: " << arguments.input_video << '\n';
        return 1;
    }

    reference_frame_selector.Reset();
    background_remover.Reset();
    tracker_processor.Reset();

    hero_lob::FrameData frame;
    cv::Mat first_frame;
    std::vector<FrameSummary> summaries;
    std::vector<FrameArtifacts> gui_frames;
    FrameArtifacts last_artifacts;
    int processed = 0;

    while (capture.ReadNext(frame)) {
        if (arguments.max_frames > 0 && processed >= arguments.max_frames) {
            break;
        }

        if (first_frame.empty()) {
            first_frame = frame.bgr.clone();
        }

        const hero_lob::TrackingResult tracking;
        const hero_lob::ReferenceFrameResult reference =
            reference_frame_selector.Process(frame, tracking);
        const hero_lob::RegistrationResult registration =
            image_registrator.Process(reference, frame, tracking);
        const hero_lob::ForegroundMaskResult foreground =
            background_remover.Process(reference, registration);
        const hero_lob::TrajectoryResult trajectory =
            tracker_processor.Process(foreground);
        const hero_lob::SynthesisResult synthesis =
            image_synthesis.Process(reference, trajectory);

        FrameArtifacts artifacts =
            MakeFrameArtifacts(frame, foreground, trajectory, synthesis);
        summaries.push_back(artifacts.summary);
        if (arguments.gui) {
            gui_frames.push_back(artifacts);
        }
        last_artifacts = std::move(artifacts);
        ++processed;
    }

    if (processed == 0 || first_frame.empty()) {
        std::cerr << "Input video contains no decodable frames: " << arguments.input_video << '\n';
        return 1;
    }

    if (!WriteArtifacts(output_dir, first_frame, summaries, last_artifacts)) {
        std::cerr << "Failed to export trajectory artifacts to: " << output_dir << '\n';
        return 1;
    }

    std::cout << "Exported trajectory debug artifacts to: " << output_dir << '\n';
    if (arguments.gui) {
        RunVideoGui(gui_frames);
    }

    return 0;
}
