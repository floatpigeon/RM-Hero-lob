#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "identifier.hpp"

namespace {

namespace fs = std::filesystem;

constexpr char kWindowName[] = "Hero Lob Identifier Debug";
constexpr int kDisplayWidth = 1440;
constexpr int kDisplayHeight = 1080;

enum class ViewMode {
    kOriginal,
    kRawBrightnessMask,
    kBrightnessMask,
    kGuideCandidateMask,
    kLightCandidateMask,
    kStablePairRoi,
    kEdgeRedMask,
    kEdgeBlueMask,
    kCandidateOverlay,
    kStablePairOverlay,
    kResultOverlay,
};

struct ParsedArguments {
    std::string mode;
    std::string input_path;
    std::string output_dir;
    bool gui = false;
    int max_frames = -1;
};

struct FrameArtifacts {
    cv::Mat original;
    hero_lob::IdentifierAnalysisResult analysis;
    std::string frame_name;
    std::int64_t frame_index = -1;
    double timestamp_seconds = 0.0;
};

bool IsHelpFlag(const std::string& argument) { return argument == "--help" || argument == "-h"; }

void PrintUsage() {
    std::cout
        << "Usage:\n"
        << "  hero_lob_identifier_debug image <input_image> <output_dir> [--gui]\n"
        << "  hero_lob_identifier_debug video <input_video> <output_dir> [--gui] [--max-frames N]\n";
}

std::string ColorName(hero_lob::TargetColor color) {
    switch (color) {
        case hero_lob::TargetColor::kRed: return "red";
        case hero_lob::TargetColor::kBlue: return "blue";
        case hero_lob::TargetColor::kUnknown: break;
    }
    return "unknown";
}

bool ParseArguments(int argc, char** argv, ParsedArguments& parsed) {
    if (argc == 2 && IsHelpFlag(argv[1])) {
        PrintUsage();
        return false;
    }

    if (argc < 4) {
        PrintUsage();
        return false;
    }

    parsed.mode = argv[1];
    parsed.input_path = argv[2];
    parsed.output_dir = argv[3];
    if (parsed.mode != "image" && parsed.mode != "video") {
        PrintUsage();
        return false;
    }

    for (int index = 4; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--gui") {
            parsed.gui = true;
            continue;
        }
        if (argument == "--max-frames" && parsed.mode == "video" && index + 1 < argc) {
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

cv::Mat ToBgrMask(const cv::Mat& mask) {
    cv::Mat bgr;
    if (mask.channels() == 1) {
        cv::cvtColor(mask, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }
    return mask.clone();
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

cv::Mat ViewImage(const FrameArtifacts& artifacts, ViewMode mode) {
    switch (mode) {
        case ViewMode::kOriginal: return artifacts.original;
        case ViewMode::kRawBrightnessMask:
            return ToBgrMask(artifacts.analysis.debug.raw_brightness_mask);
        case ViewMode::kBrightnessMask: return ToBgrMask(artifacts.analysis.debug.brightness_mask);
        case ViewMode::kGuideCandidateMask:
            return ToBgrMask(artifacts.analysis.debug.guide_candidate_mask);
        case ViewMode::kLightCandidateMask:
            return ToBgrMask(artifacts.analysis.debug.light_candidate_mask);
        case ViewMode::kStablePairRoi: return ToBgrMask(artifacts.analysis.debug.stable_pair_roi);
        case ViewMode::kEdgeRedMask: return ToBgrMask(artifacts.analysis.debug.edge_red_mask);
        case ViewMode::kEdgeBlueMask: return ToBgrMask(artifacts.analysis.debug.edge_blue_mask);
        case ViewMode::kCandidateOverlay: return artifacts.analysis.debug.candidate_overlay;
        case ViewMode::kStablePairOverlay: return artifacts.analysis.debug.stable_pair_overlay;
        case ViewMode::kResultOverlay: return artifacts.analysis.debug.result_overlay;
    }
    return artifacts.original;
}

std::string ViewName(ViewMode mode) {
    switch (mode) {
        case ViewMode::kOriginal: return "original";
        case ViewMode::kRawBrightnessMask: return "raw_brightness_mask";
        case ViewMode::kBrightnessMask: return "brightness_mask";
        case ViewMode::kGuideCandidateMask: return "guide_candidate_mask";
        case ViewMode::kLightCandidateMask: return "light_candidate_mask";
        case ViewMode::kStablePairRoi: return "stable_pair_roi";
        case ViewMode::kEdgeRedMask: return "edge_red_mask";
        case ViewMode::kEdgeBlueMask: return "edge_blue_mask";
        case ViewMode::kCandidateOverlay: return "candidate_overlay";
        case ViewMode::kStablePairOverlay: return "stable_pair_overlay";
        case ViewMode::kResultOverlay: return "result_overlay";
    }
    return "unknown";
}

std::string FormatPoint(const cv::Point2f& point) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << point.x << "," << point.y;
    return stream.str();
}

bool WriteSummary(
    const fs::path& output_path,
    const std::string& frame_name,
    const hero_lob::DetectionResult& detection) {
    std::ofstream output(output_path);
    if (!output.is_open()) {
        return false;
    }

    output << "frame," << frame_name << '\n'
           << "valid," << (detection.valid ? 1 : 0) << '\n'
           << "color," << ColorName(detection.color) << '\n'
           << "guide_center," << FormatPoint(detection.anchors.guide_center) << '\n'
           << "left_top," << FormatPoint(detection.anchors.left_top) << '\n'
           << "left_bottom," << FormatPoint(detection.anchors.left_bottom) << '\n'
           << "right_top," << FormatPoint(detection.anchors.right_top) << '\n'
           << "right_bottom," << FormatPoint(detection.anchors.right_bottom) << '\n'
           << "direction," << FormatPoint(detection.anchors.direction) << '\n';
    return true;
}

bool WriteImageArtifacts(const FrameArtifacts& artifacts, const fs::path& output_dir) {
    const std::vector<std::pair<std::string, cv::Mat>> images = {
        {"original.png", artifacts.original},
        {"raw_brightness_mask.png", artifacts.analysis.debug.raw_brightness_mask},
        {"brightness_mask.png", artifacts.analysis.debug.brightness_mask},
        {"guide_candidate_mask.png", artifacts.analysis.debug.guide_candidate_mask},
        {"light_candidate_mask.png", artifacts.analysis.debug.light_candidate_mask},
        {"stable_pair_roi.png", artifacts.analysis.debug.stable_pair_roi},
        {"edge_red_mask.png", artifacts.analysis.debug.edge_red_mask},
        {"edge_blue_mask.png", artifacts.analysis.debug.edge_blue_mask},
        {"candidate_overlay.png", artifacts.analysis.debug.candidate_overlay},
        {"stable_pair_overlay.png", artifacts.analysis.debug.stable_pair_overlay},
        {"result_overlay.png", artifacts.analysis.debug.result_overlay},
    };

    for (const auto& [name, image] : images) {
        if (!cv::imwrite((output_dir / name).string(), image)) {
            return false;
        }
    }

    return WriteSummary(
        output_dir / "summary.txt",
        artifacts.frame_name,
        artifacts.analysis.detection);
}

bool WriteVideoArtifacts(
    const std::vector<FrameArtifacts>& frames, const fs::path& output_dir) {
    const std::vector<std::pair<std::string, ViewMode>> artifact_dirs = {
        {"original", ViewMode::kOriginal},
        {"raw_brightness_mask", ViewMode::kRawBrightnessMask},
        {"brightness_mask", ViewMode::kBrightnessMask},
        {"guide_candidate_mask", ViewMode::kGuideCandidateMask},
        {"light_candidate_mask", ViewMode::kLightCandidateMask},
        {"stable_pair_roi", ViewMode::kStablePairRoi},
        {"edge_red_mask", ViewMode::kEdgeRedMask},
        {"edge_blue_mask", ViewMode::kEdgeBlueMask},
        {"candidate_overlay", ViewMode::kCandidateOverlay},
        {"stable_pair_overlay", ViewMode::kStablePairOverlay},
        {"result_overlay", ViewMode::kResultOverlay},
    };

    for (const auto& [directory_name, _] : artifact_dirs) {
        if (!EnsureDirectory(output_dir / directory_name)) {
            return false;
        }
    }

    std::ofstream summary(output_dir / "summary.csv");
    if (!summary.is_open()) {
        return false;
    }

    summary
        << "frame_name,frame_index,timestamp_seconds,valid,color,guide_center_x,guide_center_y,"
        << "left_top_x,left_top_y,left_bottom_x,left_bottom_y,right_top_x,right_top_y,"
        << "right_bottom_x,right_bottom_y,direction_x,direction_y\n";

    for (const FrameArtifacts& frame : frames) {
        const auto& detection = frame.analysis.detection;
        for (const auto& [directory_name, mode] : artifact_dirs) {
            const cv::Mat image = ViewImage(frame, mode);
            if (!cv::imwrite((output_dir / directory_name / (frame.frame_name + ".png")).string(), image)) {
                return false;
            }
        }

        summary << frame.frame_name << ','
                << frame.frame_index << ','
                << frame.timestamp_seconds << ','
                << (detection.valid ? 1 : 0) << ','
                << ColorName(detection.color) << ','
                << detection.anchors.guide_center.x << ','
                << detection.anchors.guide_center.y << ','
                << detection.anchors.left_top.x << ','
                << detection.anchors.left_top.y << ','
                << detection.anchors.left_bottom.x << ','
                << detection.anchors.left_bottom.y << ','
                << detection.anchors.right_top.x << ','
                << detection.anchors.right_top.y << ','
                << detection.anchors.right_bottom.x << ','
                << detection.anchors.right_bottom.y << ','
                << detection.anchors.direction.x << ','
                << detection.anchors.direction.y << '\n';
    }

    return true;
}

void PutHud(
    cv::Mat& image,
    const FrameArtifacts& artifacts,
    ViewMode mode,
    std::size_t frame_index,
    std::size_t total_frames,
    bool playing) {
    const auto& detection = artifacts.analysis.detection;
    const std::string line =
        artifacts.frame_name + " | " + ViewName(mode) + " | " +
        (detection.valid ? ColorName(detection.color) : "invalid") + " | " +
        (playing ? "playing" : "paused") + " | " +
        std::to_string(frame_index + 1) + "/" + std::to_string(total_frames);
    cv::rectangle(image, cv::Rect(10, 10, std::min(image.cols - 20, 900), 40), cv::Scalar(0, 0, 0), cv::FILLED);
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

void RunImageGui(const FrameArtifacts& artifacts) {
    std::vector<ViewMode> modes = {
        ViewMode::kOriginal,
        ViewMode::kRawBrightnessMask,
        ViewMode::kBrightnessMask,
        ViewMode::kGuideCandidateMask,
        ViewMode::kLightCandidateMask,
        ViewMode::kStablePairRoi,
        ViewMode::kEdgeRedMask,
        ViewMode::kEdgeBlueMask,
        ViewMode::kCandidateOverlay,
        ViewMode::kStablePairOverlay,
        ViewMode::kResultOverlay,
    };
    std::size_t mode_index = 0;
    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);

    while (true) {
        cv::Mat image = ResizeForDisplay(ViewImage(artifacts, modes[mode_index]));
        PutHud(image, artifacts, modes[mode_index], 0, 1, false);
        cv::imshow(kWindowName, image);

        const int key = cv::waitKey(0);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }
        if (key == ' ' || key == 'n' || key == 'N' || key == 'd' || key == 'D') {
            mode_index = (mode_index + 1) % modes.size();
            continue;
        }
        if (key == 'p' || key == 'P' || key == 'a' || key == 'A') {
            mode_index = (mode_index + modes.size() - 1) % modes.size();
            continue;
        }
    }

    cv::destroyWindow(kWindowName);
}

void RunVideoGui(const std::vector<FrameArtifacts>& frames) {
    if (frames.empty()) {
        return;
    }

    std::vector<ViewMode> modes = {
        ViewMode::kResultOverlay,
        ViewMode::kStablePairOverlay,
        ViewMode::kCandidateOverlay,
        ViewMode::kOriginal,
        ViewMode::kRawBrightnessMask,
        ViewMode::kBrightnessMask,
        ViewMode::kGuideCandidateMask,
        ViewMode::kLightCandidateMask,
        ViewMode::kStablePairRoi,
        ViewMode::kEdgeRedMask,
        ViewMode::kEdgeBlueMask,
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

hero_lob::FrameData MakeFrameData(
    const cv::Mat& bgr, std::int64_t frame_index, double timestamp_seconds) {
    hero_lob::FrameData frame;
    frame.frame_index = frame_index;
    frame.timestamp_seconds = timestamp_seconds;
    frame.bgr = bgr.clone();
    cv::cvtColor(frame.bgr, frame.hsv, cv::COLOR_BGR2HSV);
    return frame;
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

    hero_lob::Identifier identifier(hero_lob::PipelineConfig{});

    if (arguments.mode == "image") {
        const cv::Mat image = cv::imread(arguments.input_path, cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << "Failed to read input image: " << arguments.input_path << '\n';
            return 1;
        }

        FrameArtifacts artifacts;
        artifacts.original = image;
        artifacts.frame_name = "frame_000000";
        artifacts.frame_index = 0;
        artifacts.timestamp_seconds = 0.0;
        artifacts.analysis = identifier.Analyze(MakeFrameData(image, 0, 0.0));

        if (!WriteImageArtifacts(artifacts, output_dir)) {
            std::cerr << "Failed to export image artifacts to: " << output_dir << '\n';
            return 1;
        }

        std::cout << "Exported debug artifacts to: " << output_dir << '\n';
        if (arguments.gui) {
            RunImageGui(artifacts);
        }
        return 0;
    }

    cv::VideoCapture capture(arguments.input_path);
    if (!capture.isOpened()) {
        std::cerr << "Failed to open input video: " << arguments.input_path << '\n';
        return 1;
    }

    const double fps = capture.get(cv::CAP_PROP_FPS);
    const double resolved_fps = fps > 0.0 ? fps : 30.0;

    std::vector<FrameArtifacts> frames;
    cv::Mat image;
    int processed = 0;
    while (capture.read(image) && !image.empty()) {
        if (arguments.max_frames > 0 && processed >= arguments.max_frames) {
            break;
        }

        FrameArtifacts artifacts;
        artifacts.original = image.clone();
        std::ostringstream name;
        name << "frame_" << std::setw(6) << std::setfill('0') << processed;
        artifacts.frame_name = name.str();
        artifacts.frame_index = processed;
        artifacts.timestamp_seconds = static_cast<double>(processed) / resolved_fps;
        artifacts.analysis = identifier.Analyze(
            MakeFrameData(image, processed, artifacts.timestamp_seconds));
        frames.push_back(std::move(artifacts));
        ++processed;
    }

    if (frames.empty()) {
        std::cerr << "Input video contains no decodable frames: " << arguments.input_path << '\n';
        return 1;
    }

    if (!WriteVideoArtifacts(frames, output_dir)) {
        std::cerr << "Failed to export video artifacts to: " << output_dir << '\n';
        return 1;
    }

    std::cout << "Exported " << frames.size() << " frames to: " << output_dir << '\n';
    if (arguments.gui) {
        RunVideoGui(frames);
    }

    return 0;
}
