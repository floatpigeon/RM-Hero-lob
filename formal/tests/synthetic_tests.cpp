#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "image_trans/long_exposure_composer.h"
#include "image_trans/video_stabilizer.h"

namespace {

cv::Mat makeBaseFrame(cv::Size size) {
    cv::Mat frame(size, CV_8UC3, cv::Scalar(25, 25, 25));
    cv::rectangle(frame, {20, 20}, {size.width - 20, size.height - 20}, cv::Scalar(180, 80, 80), 2);
    cv::line(frame, {0, size.height / 2}, {size.width, size.height / 2}, cv::Scalar(80, 180, 80), 2);
    cv::circle(frame, {size.width / 3, size.height / 3}, 18, cv::Scalar(80, 80, 200), -1);
    cv::putText(frame, "TEST", {size.width / 4, size.height * 3 / 4}, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(200, 200, 200), 2);
    return frame;
}

std::vector<cv::Mat> makeJitteredFrames(int frame_count, cv::Size size) {
    const cv::Mat base = makeBaseFrame(size);
    std::vector<cv::Mat> frames;
    frames.reserve(frame_count);

    for (int index = 0; index < frame_count; ++index) {
        const double angle = std::sin(index * 0.17) * 1.6;
        const double dx = std::sin(index * 0.31) * 5.0;
        const double dy = std::cos(index * 0.23) * 4.0;

        cv::Mat rotation = cv::getRotationMatrix2D(cv::Point2f(size.width / 2.0F, size.height / 2.0F), angle, 1.0);
        rotation.at<double>(0, 2) += dx;
        rotation.at<double>(1, 2) += dy;

        cv::Mat frame;
        cv::warpAffine(base, frame, rotation, size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0));
        frames.push_back(frame);
    }

    return frames;
}

double averageFrameDifference(const std::vector<cv::Mat>& frames) {
    double total = 0.0;
    int count = 0;
    for (std::size_t index = 1; index < frames.size(); ++index) {
        cv::Mat diff;
        cv::absdiff(frames[index], frames[index - 1], diff);
        total += cv::mean(diff)[0] + cv::mean(diff)[1] + cv::mean(diff)[2];
        ++count;
    }
    return count == 0 ? 0.0 : total / static_cast<double>(count);
}

double averageReferenceError(const std::vector<cv::Mat>& frames, const cv::Mat& reference, const cv::Rect& roi) {
    double total = 0.0;
    for (const cv::Mat& frame : frames) {
        cv::Mat diff;
        cv::absdiff(frame(roi), reference(roi), diff);
        const cv::Scalar mean = cv::mean(diff);
        total += mean[0] + mean[1] + mean[2];
    }
    return total / static_cast<double>(frames.size());
}

void expect(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testStabilization() {
    const cv::Size size(320, 240);
    const cv::Mat reference = makeBaseFrame(size);
    const std::vector<cv::Mat> jittered = makeJitteredFrames(45, size);
    const cv::Rect roi(50, 40, size.width - 100, size.height - 80);

    image_trans::StabilizationParams params;
    params.auto_crop = false;
    image_trans::VideoStabilizer stabilizer(params);
    image_trans::MotionPlan plan;
    const std::vector<cv::Mat> stabilized = stabilizer.stabilize(jittered, &plan);

    expect(stabilized.size() == jittered.size(), "stabilization output frame count mismatch");
    expect(plan.crop_roi.width > 0 && plan.crop_roi.height > 0, "invalid crop roi");
    expect(stabilized.front().size() == size, "stabilized frame size mismatch");

    const double before = averageReferenceError(jittered, reference, roi);
    const double after = averageReferenceError(stabilized, reference, roi);
    std::ostringstream message;
    message << "stabilization did not sufficiently reduce alignment error"
            << " (before=" << before << ", after=" << after << ")";
    expect(after < before * 0.75, message.str());
}

void testAutoCrop() {
    const cv::Size size(320, 240);
    const std::vector<cv::Mat> jittered = makeJitteredFrames(30, size);

    image_trans::VideoStabilizer stabilizer;
    image_trans::MotionPlan plan;
    const std::vector<cv::Mat> stabilized = stabilizer.stabilize(jittered, &plan);

    expect(plan.crop_roi.width > 0 && plan.crop_roi.height > 0, "auto-crop roi should be valid");
    expect(plan.crop_roi.width <= size.width && plan.crop_roi.height <= size.height, "auto-crop roi exceeds frame bounds");
    expect(stabilized.front().size() == size, "auto-crop output size mismatch");
}

void testFallbackBehavior() {
    std::vector<cv::Mat> frames(5, cv::Mat(120, 160, CV_8UC3, cv::Scalar::all(0)));
    image_trans::VideoStabilizer stabilizer;
    image_trans::MotionPlan plan = stabilizer.analyze(frames);
    expect(plan.stats.fallback_frame_count >= 1, "expected fallback frames for low-texture input");
}

void testLongExposure() {
    const cv::Size size(256, 192);
    std::vector<cv::Mat> frames;
    for (int index = 0; index < 20; ++index) {
        cv::Mat frame(size, CV_8UC3, cv::Scalar(30, 30, 30));
        const int x = 20 + index * 8;
        cv::circle(frame, {x, size.height / 2}, 6, cv::Scalar(0, 255, 255), -1);
        frames.push_back(frame);
    }

    image_trans::LongExposureComposer composer;
    composer.reset(size);
    for (const cv::Mat& frame : frames) {
        composer.accumulate(frame);
    }

    const cv::Mat result = composer.finalize();
    expect(result.size() == size, "exposure output size mismatch");

    int bright_count = 0;
    for (int x = 20; x <= 20 + 19 * 8; x += 8) {
        const cv::Vec3b pixel = result.at<cv::Vec3b>(size.height / 2, x);
        if (pixel[1] > 180 && pixel[2] > 180) {
            ++bright_count;
        }
    }
    expect(bright_count >= 15, "exposure result missed too much of the bright trail");
}

void testCliRoundTrip(const std::filesystem::path& executable_dir) {
    const cv::Size size(200, 150);
    const std::vector<cv::Mat> frames = makeJitteredFrames(20, size);
    const std::filesystem::path temp_dir = std::filesystem::temp_directory_path() / "image_trans_formal_test";
    std::filesystem::create_directories(temp_dir);

    const std::filesystem::path input_path = temp_dir / "synthetic_input.avi";
    cv::VideoWriter writer(input_path.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, size, true);
    expect(writer.isOpened(), "failed to open synthetic input writer");
    for (const cv::Mat& frame : frames) {
        writer.write(frame);
    }
    writer.release();

    const std::filesystem::path output_dir = temp_dir / "outputs";
    const std::filesystem::path cli_path = executable_dir / "image_trans_cli";
    const std::string command = "\"" + cli_path.string() + "\" --input \"" + input_path.string() +
        "\" --output-dir \"" + output_dir.string() + "\" --output-prefix smoke";
    const int exit_code = std::system(command.c_str());
    expect(exit_code == 0, "cli smoke command failed");

    const std::filesystem::path output_video = output_dir / "smoke_stabilized.mp4";
    const std::filesystem::path output_image = output_dir / "smoke_exposure.png";
    expect(std::filesystem::exists(output_video), "cli output video missing");
    expect(std::filesystem::exists(output_image), "cli output image missing");

    cv::VideoCapture capture(output_video.string());
    expect(capture.isOpened(), "failed to read cli output video");

    cv::Mat exposure = cv::imread(output_image.string(), cv::IMREAD_COLOR);
    expect(!exposure.empty(), "failed to read cli output image");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        testStabilization();
        testAutoCrop();
        testFallbackBehavior();
        testLongExposure();
        const std::filesystem::path executable_dir = std::filesystem::absolute(argv[0]).parent_path();
        testCliRoundTrip(executable_dir);
        std::cout << "All synthetic tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Test failure: " << error.what() << '\n';
        return 1;
    }
}
