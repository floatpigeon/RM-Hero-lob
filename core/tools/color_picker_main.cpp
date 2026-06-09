#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

constexpr char kWindowName[] = "Hero Lob Color Picker";
constexpr int kMaxDisplayWidth = 1440;
constexpr int kMaxDisplayHeight = 1080;

struct PickerContext {
    cv::Mat original_bgr;
    cv::Mat original_hsv;
    cv::Mat display_base;
    cv::Mat display_image;
    double display_scale = 1.0;
};

void PrintUsage() {
    std::cout << "Usage: hero_lob_color_picker <input_image>\n"
              << "Click the image to print BGR and HSV values.\n"
              << "Press q or Esc to exit.\n";
}

double ComputeDisplayScale(const cv::Size& original_size) {
    const double width_scale =
        static_cast<double>(kMaxDisplayWidth) / static_cast<double>(original_size.width);
    const double height_scale =
        static_cast<double>(kMaxDisplayHeight) / static_cast<double>(original_size.height);
    return std::min(1.0, std::min(width_scale, height_scale));
}

cv::Point MapDisplayPointToOriginal(const PickerContext& context, const cv::Point& display_point) {
    const int mapped_x =
        static_cast<int>(std::lround(static_cast<double>(display_point.x) / context.display_scale));
    const int mapped_y =
        static_cast<int>(std::lround(static_cast<double>(display_point.y) / context.display_scale));

    return {
        std::clamp(mapped_x, 0, context.original_bgr.cols - 1),
        std::clamp(mapped_y, 0, context.original_bgr.rows - 1),
    };
}

std::vector<std::string> BuildOverlayLines(
    const cv::Point& image_point, const cv::Vec3b& bgr, const cv::Vec3b& hsv) {
    std::vector<std::string> lines;

    {
        std::ostringstream stream;
        stream << "x=" << image_point.x << " y=" << image_point.y;
        lines.push_back(stream.str());
    }
    {
        std::ostringstream stream;
        stream << "BGR=(" << static_cast<int>(bgr[0]) << "," << static_cast<int>(bgr[1]) << ","
               << static_cast<int>(bgr[2]) << ")";
        lines.push_back(stream.str());
    }
    {
        std::ostringstream stream;
        stream << "HSV=(" << static_cast<int>(hsv[0]) << "," << static_cast<int>(hsv[1]) << ","
               << static_cast<int>(hsv[2]) << ")";
        lines.push_back(stream.str());
    }

    return lines;
}

void RedrawDisplay(
    PickerContext& context, const cv::Point& display_point, const cv::Point& image_point,
    const cv::Vec3b& bgr, const cv::Vec3b& hsv) {
    context.display_image = context.display_base.clone();

    const int marker_radius = std::max(
        5, static_cast<int>(std::lround(7.0 * std::max(1.0, context.display_scale * 2.0))));
    cv::circle(
        context.display_image, display_point, marker_radius, cv::Scalar(0, 255, 255), 2,
        cv::LINE_AA);

    const std::vector<std::string> lines = BuildOverlayLines(image_point, bgr, hsv);
    const int font_face = cv::FONT_HERSHEY_SIMPLEX;
    const double font_scale = 0.65;
    const int thickness = 1;
    const int padding = 10;
    const int line_gap = 6;

    int panel_width = 0;
    int panel_height = padding;
    for (const std::string& line : lines) {
        int baseline = 0;
        const cv::Size text_size =
            cv::getTextSize(line, font_face, font_scale, thickness, &baseline);
        panel_width = std::max(panel_width, text_size.width);
        panel_height += text_size.height + baseline + line_gap;
    }

    panel_width += padding * 2;
    panel_height += padding;

    const cv::Rect panel_rect(10, 10, panel_width, panel_height);
    cv::rectangle(context.display_image, panel_rect, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(context.display_image, panel_rect, cv::Scalar(255, 255, 255), 1);

    int text_y = panel_rect.y + padding + 18;
    for (const std::string& line : lines) {
        cv::putText(
            context.display_image, line, cv::Point(panel_rect.x + padding, text_y), font_face,
            font_scale, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);
        text_y += 26;
    }

    cv::imshow(kWindowName, context.display_image);
}

void OnMouse(int event, int x, int y, int /*flags*/, void* userdata) {
    if (event != cv::EVENT_LBUTTONDOWN || userdata == nullptr) {
        return;
    }

    auto* context = static_cast<PickerContext*>(userdata);
    if (x < 0 || y < 0 || x >= context->display_base.cols || y >= context->display_base.rows) {
        return;
    }

    const cv::Point display_point(x, y);
    const cv::Point image_point = MapDisplayPointToOriginal(*context, display_point);
    const cv::Vec3b bgr = context->original_bgr.at<cv::Vec3b>(image_point);
    const cv::Vec3b hsv = context->original_hsv.at<cv::Vec3b>(image_point);

    std::cout << "x=" << image_point.x << " y=" << image_point.y << " BGR=("
              << static_cast<int>(bgr[0]) << "," << static_cast<int>(bgr[1]) << ","
              << static_cast<int>(bgr[2]) << ") HSV=(" << static_cast<int>(hsv[0]) << ","
              << static_cast<int>(hsv[1]) << "," << static_cast<int>(hsv[2]) << ")\n";

    RedrawDisplay(*context, display_point, image_point, bgr, hsv);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
        PrintUsage();
        return 0;
    }

    if (argc != 2) {
        PrintUsage();
        return 1;
    }

    PickerContext context;
    context.original_bgr = cv::imread(argv[1], cv::IMREAD_COLOR);
    if (context.original_bgr.empty()) {
        std::cerr << "Failed to read input image: " << argv[1] << '\n';
        return 1;
    }

    cv::cvtColor(context.original_bgr, context.original_hsv, cv::COLOR_BGR2HSV);
    context.display_scale = ComputeDisplayScale(context.original_bgr.size());

    if (context.display_scale < 1.0) {
        const cv::Size display_size(
            std::max(
                1,
                static_cast<int>(std::lround(context.original_bgr.cols * context.display_scale))),
            std::max(
                1,
                static_cast<int>(std::lround(context.original_bgr.rows * context.display_scale))));
        cv::resize(
            context.original_bgr, context.display_base, display_size, 0.0, 0.0, cv::INTER_AREA);
    } else {
        context.display_base = context.original_bgr.clone();
    }

    context.display_image = context.display_base.clone();

    std::cout << "Loaded image: " << argv[1] << " (" << context.original_bgr.cols << "x"
              << context.original_bgr.rows << ")\n"
              << "Click the image to inspect color values. Press q or Esc to exit.\n";

    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(kWindowName, OnMouse, &context);
    cv::imshow(kWindowName, context.display_image);

    while (true) {
        const int key = cv::waitKey(30);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        const double visible = cv::getWindowProperty(kWindowName, cv::WND_PROP_VISIBLE);
        if (visible < 1.0) {
            break;
        }
    }

    cv::destroyWindow(kWindowName);
    return 0;
}
