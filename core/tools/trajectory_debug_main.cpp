#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "capture.hpp"
#include "background_remover.hpp"
#include "tracker_processor.hpp"
#include "image_synthesis.hpp"
#include "reference_frame_selector.hpp"
#include "types.hpp"

namespace {

void PrintUsage() {
    std::cout
        << "Usage: hero_lob_trajectory_debug <input_video> <output_folder>\n"
        << "       [--max-frames N] [--gui]\n"
        << "Export debug visualizations for the trajectory pipeline.\n";
}

int CountForegroundPixels(const cv::Mat& mask) {
    return cv::countNonZero(mask);
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
    const std::string output_dir = argv[2];
    int max_frames = 0;
    bool gui_mode = false;
    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--max-frames" && i + 1 < argc) {
            max_frames = std::stoi(argv[++i]);
        } else if (arg == "--gui") {
            gui_mode = true;
        }
    }
    std::filesystem::create_directories(output_dir);
    hero_lob::PipelineConfig config;
    hero_lob::Capture capture(config);
    if (!capture.Open(input_path)) {
        std::cerr << "Failed to open video: " << input_path << '\n';
        return 1;
    }
    hero_lob::ReferenceFrameSelector ref_selector(config);
    hero_lob::BackgroundRemover bg_remover(config);
    hero_lob::TrackerProcessor tracker(config);
    hero_lob::ImageSynthesis synthesis(config);
    hero_lob::FrameData first_frame;
    if (!capture.ReadNext(first_frame)) {
        std::cerr << "Failed to read first frame\n";
        return 1;
    }
    hero_lob::TrackingResult empty_tracking;
    hero_lob::ReferenceFrameResult reference =
        ref_selector.Process(first_frame, empty_tracking);
    cv::imwrite(output_dir + "/first_frame.png", first_frame.bgr);
    std::ofstream csv(output_dir + "/summary.csv");
    csv << "frame,foreground_pixels,trajectory_pixels,accumulated_frames\n";
    std::vector<int> fg_pixel_counts;
    std::vector<int> traj_pixel_counts;
    hero_lob::TrajectoryResult last_trajectory;
    hero_lob::ForegroundMaskResult last_foreground;
    hero_lob::FrameData frame;
    int frame_count = 0;
    while (capture.ReadNext(frame)) {
        if (max_frames > 0 && frame_count >= max_frames) {
            break;
        }
        hero_lob::RegistrationResult registration;
        registration.valid = true;
        registration.frame_index = frame.frame_index;
        registration.timestamp_seconds = frame.timestamp_seconds;
        registration.registered_bgr = frame.bgr;
        registration.registered_hsv = frame.hsv;
        hero_lob::ForegroundMaskResult foreground =
            bg_remover.Process(reference, registration);
        last_foreground = foreground;
        last_trajectory = tracker.Process(foreground);
        int fg_pixels = 0;
        int traj_pixels = 0;
        if (foreground.valid && !foreground.candidate_mask.empty()) {
            fg_pixels = CountForegroundPixels(foreground.candidate_mask);
        }
        if (last_trajectory.valid && !last_trajectory.trajectory_layer.empty()) {
            cv::Mat traj_gray;
            cv::cvtColor(last_trajectory.trajectory_layer, traj_gray,
                         cv::COLOR_BGR2GRAY);
            traj_pixels = cv::countNonZero(traj_gray);
        }
        fg_pixel_counts.push_back(fg_pixels);
        traj_pixel_counts.push_back(traj_pixels);
        csv << frame_count << "," << fg_pixels << "," << traj_pixels << ","
            << last_trajectory.accumulated_frames << "\n";
        if (frame_count % 50 == 0) {
            std::cout << "\rProcessed frame " << frame_count << std::flush;
        }
        ++frame_count;
    }
    csv.close();
    std::cout << "\nProcessed " << frame_count << " frames\n";
    if (last_foreground.valid && !last_foreground.candidate_mask.empty()) {
        cv::imwrite(output_dir + "/candidate_mask_last.png",
                     last_foreground.candidate_mask);
    }
    if (last_trajectory.valid && !last_trajectory.trajectory_layer.empty()) {
        cv::Mat layer = last_trajectory.trajectory_layer;
        std::vector<float> flat;
        layer.reshape(1, 1).copyTo(flat);
        std::sort(flat.begin(), flat.end());
        int p99_idx = static_cast<int>(flat.size() * 0.99);
        if (p99_idx >= static_cast<int>(flat.size())) {
            p99_idx = static_cast<int>(flat.size()) - 1;
        }
        float max_val = flat[p99_idx];
        if (max_val < 1e-6F) {
            max_val = 1.0F;
        }
        cv::Mat normalized;
        layer.convertTo(normalized, CV_32F, 255.0 / max_val);
        cv::Mat exposure;
        normalized.convertTo(exposure, CV_8UC3);
        cv::imwrite(output_dir + "/trajectory_exposure.png", exposure);
        cv::Mat traj_gray;
        cv::cvtColor(exposure, traj_gray, cv::COLOR_BGR2GRAY);
        cv::Mat heatmap;
        cv::applyColorMap(traj_gray, heatmap, cv::COLORMAP_JET);
        cv::imwrite(output_dir + "/trajectory_heatmap.png", heatmap);
        cv::Mat overlay;
        cv::add(reference.reference_frame.bgr, exposure, overlay);
        cv::imwrite(output_dir + "/trajectory_overlay.png", overlay);
    }
    std::ofstream txt(output_dir + "/summary.txt");
    txt << "Total frames processed: " << frame_count << '\n';
    if (!fg_pixel_counts.empty()) {
        txt << "Final foreground pixels: " << fg_pixel_counts.back() << '\n';
    }
    if (!traj_pixel_counts.empty()) {
        txt << "Final trajectory pixels: " << traj_pixel_counts.back() << '\n';
    }
    txt << "Accumulated frames: " << last_trajectory.accumulated_frames
        << '\n';
    txt.close();
    std::cout << "Debug output saved to " << output_dir << '\n';
    if (gui_mode) {
        std::vector<std::string> labels = {"First Frame", "Candidate Mask",
                                           "Trajectory Exposure", "Heatmap",
                                           "Overlay"};
        std::vector<cv::Mat> images;
        images.push_back(first_frame.bgr);
        if (last_foreground.valid) {
            cv::Mat mask_vis;
            last_foreground.candidate_mask.convertTo(mask_vis, CV_8UC1);
            images.push_back(mask_vis);
        }
        if (last_trajectory.valid && !last_trajectory.trajectory_layer.empty()) {
            cv::Mat layer = last_trajectory.trajectory_layer;
            std::vector<float> flat;
            layer.reshape(1, 1).copyTo(flat);
            std::sort(flat.begin(), flat.end());
            int p99_idx = static_cast<int>(flat.size() * 0.99);
            if (p99_idx >= static_cast<int>(flat.size())) {
                p99_idx = static_cast<int>(flat.size()) - 1;
            }
            float max_val = flat[p99_idx];
            if (max_val < 1e-6F) {
                max_val = 1.0F;
            }
            cv::Mat normalized;
            layer.convertTo(normalized, CV_32F, 255.0 / max_val);
            cv::Mat exposure;
            normalized.convertTo(exposure, CV_8UC3);
            images.push_back(exposure);
            cv::Mat traj_gray;
            cv::cvtColor(exposure, traj_gray, cv::COLOR_BGR2GRAY);
            cv::Mat heatmap;
            cv::applyColorMap(traj_gray, heatmap, cv::COLORMAP_JET);
            images.push_back(heatmap);
            cv::Mat overlay;
            cv::add(reference.reference_frame.bgr, exposure, overlay);
            images.push_back(overlay);
        }
        int current = 0;
        while (true) {
            cv::Mat display;
            if (current < static_cast<int>(images.size())) {
                images[current].convertTo(display, CV_8UC3);
            }
            if (display.empty()) {
                break;
            }
            cv::imshow("Trajectory Debug", display);
            int key = cv::waitKey(0);
            if (key == 'q' || key == 27) {
                break;
            } else if (key == 'n' || key == ' ') {
                current = (current + 1) % static_cast<int>(images.size());
            } else if (key == 'p') {
                current = (current - 1 + static_cast<int>(images.size())) %
                          static_cast<int>(images.size());
            }
        }
        cv::destroyAllWindows();
    }
    return 0;
}
