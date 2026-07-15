#include <iostream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <image> <row> <col> [output]\n";
        std::cerr << "  Draws horizontal line at <row> and vertical line at <col>\n";
        return 1;
    }

    const std::string image_path = argv[1];
    const int row = std::stoi(argv[2]);
    const int col = std::stoi(argv[3]);
    const std::string output_path = (argc > 4) ? argv[4] : "";

    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "Failed to load image: " << image_path << "\n";
        return 1;
    }

    std::cerr << "Image size: " << img.cols << "x" << img.rows << "\n";
    std::cerr << "Drawing: horizontal line at row=" << row << ", vertical line at col=" << col << "\n";

    cv::Scalar color(0, 255, 0);
    int thickness = 2;

    if (row >= 0 && row < img.rows) {
        cv::line(img, cv::Point(0, row), cv::Point(img.cols - 1, row), color, thickness);
    } else {
        std::cerr << "Warning: row " << row << " is out of range [0, " << img.rows - 1 << "]\n";
    }

    if (col >= 0 && col < img.cols) {
        cv::line(img, cv::Point(col, 0), cv::Point(col, img.rows - 1), color, thickness);
    } else {
        std::cerr << "Warning: col " << col << " is out of range [0, " << img.cols - 1 << "]\n";
    }

    if (!output_path.empty()) {
        cv::imwrite(output_path, img);
        std::cerr << "Saved to: " << output_path << "\n";
    } else {
        cv::imshow("Line Marker", img);
        cv::waitKey(0);
    }

    return 0;
}
