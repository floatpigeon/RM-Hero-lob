#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "image_trans/cli_options.hpp"
#include "image_trans/trail_compositor.hpp"
#include "image_trans/window_assembler.hpp"

namespace image_trans {

class FakeReplaySource final : public ReplaySource {
public:
    FakeReplaySource(double fps_value, std::vector<FramePacket> frames)
        : fps_value_(fps_value)
        , frames_(std::move(frames)) {}

    double fps() const override { return fps_value_; }

    cv::Size frame_size() const override {
        if (frames_.empty()) {
            return cv::Size();
        }
        return frames_.front().bgr.size();
    }

    bool read(FramePacket& out_frame) override {
        if (next_index_ >= frames_.size()) {
            return false;
        }
        out_frame = frames_.at(next_index_);
        ++next_index_;
        return true;
    }

    void reset() override { next_index_ = 0; }

private:
    double fps_value_ = 0.0;
    std::vector<FramePacket> frames_;
    std::size_t next_index_ = 0;
};

class TestRunner {
public:
    static void assert_true(bool condition, const std::string& message) {
        if (!condition) {
            throw std::runtime_error(message);
        }
    }

    static void run_all() {
        test_trail_compositor_max_accumulation();
        test_window_assembler_builds_expected_counts();
        test_cli_parser_trigger_and_crop();
    }

private:
    static void test_trail_compositor_max_accumulation() {
        TrailCompositor compositor(cv::Size(2, 1));

        cv::Mat first = cv::Mat::zeros(cv::Size(2, 1), CV_8UC3);
        first.at<cv::Vec3b>(0, 0) = cv::Vec3b(10, 20, 30);
        first.at<cv::Vec3b>(0, 1) = cv::Vec3b(5, 5, 5);

        cv::Mat second = cv::Mat::zeros(cv::Size(2, 1), CV_8UC3);
        second.at<cv::Vec3b>(0, 0) = cv::Vec3b(8, 40, 25);
        second.at<cv::Vec3b>(0, 1) = cv::Vec3b(100, 100, 100);

        cv::Mat first_mask = cv::Mat::zeros(cv::Size(2, 1), CV_8UC1);
        first_mask.at<std::uint8_t>(0, 0) = 255;
        cv::Mat second_mask = cv::Mat::zeros(cv::Size(2, 1), CV_8UC1);
        second_mask.at<std::uint8_t>(0, 0) = 255;

        compositor.accumulate(first, first_mask);
        compositor.accumulate(second, second_mask);

        const cv::Mat trail = compositor.trail_layer();
        const cv::Vec3b pixel = trail.at<cv::Vec3b>(0, 0);
        assert_true(pixel == cv::Vec3b(10, 40, 30), "trail compositor should keep per-channel max");
        assert_true(
            trail.at<cv::Vec3b>(0, 1) == cv::Vec3b(0, 0, 0), "masked-out pixels must stay zero");
    }

    static void test_window_assembler_builds_expected_counts() {
        CompositeConfig config;
        config.pretrigger_frame_count = 2;
        config.capture_duration_ms = 1000;

        std::vector<FramePacket> frames;
        for (int index = 0; index < 6; ++index) {
            FramePacket frame;
            frame.frame_index = index;
            frame.timestamp_ms = index * 500;
            frame.bgr = cv::Mat::zeros(cv::Size(2, 2), CV_8UC3);
            frames.push_back(frame);
        }

        FakeReplaySource source(2.0, frames);
        WindowAssembler assembler(config);

        TriggerSpec trigger;
        trigger.mode = TriggerMode::kFrameIndex;
        trigger.value = 3;

        const WindowCapture window = assembler.build_window(source, trigger);
        assert_true(
            window.pretrigger_frames.size() == 2,
            "window should keep requested pretrigger frame count");
        assert_true(
            window.posttrigger_frames.size() == 2,
            "window should keep duration-based posttrigger frame count");
        assert_true(
            window.pretrigger_frames.front().frame_index == 1,
            "pretrigger buffer should roll forward");
        assert_true(
            window.posttrigger_frames.front().frame_index == 3,
            "posttrigger buffer should start at trigger");
    }

    static void test_cli_parser_trigger_and_crop() {
        const char* argv[] = {
            "image_trans_cli",   "--input", "sample.mp4", "--output-dir", "out/run",
            "--trigger-time-ms", "1500",    "--crop",     "10,20,30,40",  "--debug",
        };

        const CliOptions options =
            CliOptionsParser::parse(static_cast<int>(std::size(argv)), const_cast<char**>(argv));
        assert_true(options.input_path == "sample.mp4", "cli parser should read input path");
        assert_true(options.output_dir == "out/run", "cli parser should read output dir");
        assert_true(
            options.trigger.mode == TriggerMode::kTimestampMs,
            "cli parser should select timestamp trigger");
        assert_true(options.trigger.value == 1500, "cli parser should parse trigger timestamp");
        assert_true(options.crop_rect.has_value(), "cli parser should parse crop rect");
        assert_true(
            options.crop_rect->x == 10 && options.crop_rect->y == 20,
            "cli parser should parse crop origin");
        assert_true(
            options.crop_rect->width == 30 && options.crop_rect->height == 40,
            "cli parser should parse crop size");
        assert_true(options.debug_enabled, "cli parser should parse debug flag");
    }
};

} // namespace image_trans

int main() {
    try {
        image_trans::TestRunner::run_all();
        std::cout << "All tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Test failure: " << error.what() << "\n";
        return EXIT_FAILURE;
    }
}
