#include <filesystem>
#include <iostream>
#include <string>

#include <opencv2/imgcodecs.hpp>

#include "pipeline.hpp"

namespace {

std::filesystem::path RepoRoot() {
    return std::filesystem::path(HERO_LOB_SOURCE_DIR);
}

bool TestPipelineWritesOutputImage() {
    hero_lob::PipelineConfig config;
    config.motion_foreground.warmup_frames = 3;
    config.motion_foreground.open_kernel_size = 1;
    config.motion_foreground.close_kernel_size = 3;
    config.trajectory_window.window_seconds = 3.0;

    hero_lob::Pipeline pipeline(config);

    const std::filesystem::path input_video =
        RepoRoot() / "data" / "test-video" / "test-video-000.mp4";
    const std::filesystem::path output_image =
        std::filesystem::temp_directory_path() / "hero_lob_pipeline_smoke.png";

    std::error_code error;
    std::filesystem::remove(output_image, error);

    const bool ok = pipeline.Run(input_video.string(), output_image.string());
    if (!ok) {
        return false;
    }

    const cv::Mat image = cv::imread(output_image.string(), cv::IMREAD_COLOR);
    return !image.empty();
}

}  // namespace

int main() {
    if (!TestPipelineWritesOutputImage()) {
        std::cerr << "TestPipelineWritesOutputImage failed\n";
        return 1;
    }

    std::cout << "pipeline_smoke_test passed\n";
    return 0;
}
