#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <sstream>
#include <iomanip>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

void PrintUsage() {
    std::cout << "Usage: hero_lob_tile_split <input_image> [options]\n"
              << "\nOptions:\n"
              << "  --tile WxH       Tile size (e.g., --tile 32x32)\n"
              << "  --tile W         Square tile size (e.g., --tile 32)\n"
              << "  --output DIR     Output directory (default: tiles_output)\n"
              << "  --quality Q      JPEG quality 1-100 (default: 95)\n"
              << "\nExamples:\n"
              << "  hero_lob_tile_split input.jpg --tile 32\n"
              << "  hero_lob_tile_split input.jpg --tile 32x32 --output my_tiles\n";
}

bool IsHelpFlag(const std::string& arg) { return arg == "--help" || arg == "-h"; }

struct Options {
    std::string input_path;
    std::string output_dir = "tiles_output";
    int tile_width = 32;
    int tile_height = 32;
    int quality = 95;
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

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--tile" && i + 1 < argc) {
            ++i;
            std::string tile_str = argv[i];
            size_t x_pos = tile_str.find('x');
            if (x_pos == std::string::npos) {
                opts.tile_width = std::stoi(tile_str);
                opts.tile_height = opts.tile_width;
            } else {
                opts.tile_width = std::stoi(tile_str.substr(0, x_pos));
                opts.tile_height = std::stoi(tile_str.substr(x_pos + 1));
            }
        } else if (arg == "--output" && i + 1 < argc) {
            opts.output_dir = argv[++i];
        } else if (arg == "--quality" && i + 1 < argc) {
            opts.quality = std::stoi(argv[++i]);
        } else {
            std::cerr << "Unknown argument: " << arg << '\n';
            PrintUsage();
            return false;
        }
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

    const cv::Mat input = cv::imread(opts.input_path, cv::IMREAD_COLOR);
    if (input.empty()) {
        std::cerr << "Failed to read input image: " << opts.input_path << '\n';
        return 1;
    }

    const int img_w = input.cols;
    const int img_h = input.rows;
    const int cols = (img_w + opts.tile_width - 1) / opts.tile_width;
    const int rows = (img_h + opts.tile_height - 1) / opts.tile_height;
    const int total = cols * rows;

    std::cout << "Image: " << img_w << " x " << img_h << "\n"
              << "Tile:  " << opts.tile_width << " x " << opts.tile_height << "\n"
              << "Grid:  " << cols << " cols x " << rows << " rows = " << total << " tiles\n"
              << "----------------------------------------------\n";

    std::filesystem::create_directories(opts.output_dir);

    std::vector<std::vector<int>> tile_sizes(cols, std::vector<int>(rows, 0));
    long long total_bytes = 0;

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int x0 = c * opts.tile_width;
            int y0 = r * opts.tile_height;
            int x1 = std::min(x0 + opts.tile_width, img_w);
            int y1 = std::min(y0 + opts.tile_height, img_h);

            cv::Mat tile = input(cv::Rect(x0, y0, x1 - x0, y1 - y0)).clone();

            std::ostringstream filename;
            filename << opts.output_dir << "/" << c << "_" << r << ".jpg";

            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, opts.quality};
            cv::imwrite(filename.str(), tile, params);

            std::ifstream ifs(filename.str(), std::ios::binary | std::ios::ate);
            int size = static_cast<int>(ifs.tellg());
            tile_sizes[c][r] = size;
            total_bytes += size;

            std::cout << "  [" << std::setw(2) << c << "," << std::setw(2) << r << "]  "
                      << (x1 - x0) << "x" << (y1 - y0) << "  "
                      << std::setw(6) << size << " bytes  "
                      << c << "_" << r << ".jpg\n";
        }
    }

    std::cout << "----------------------------------------------\n"
              << "Total tiles: " << total << "\n"
              << "Total size:  " << total_bytes << " bytes (" << std::fixed << std::setprecision(1) << total_bytes / 1024.0 << " KB)\n"
              << "Average:     " << total_bytes / total << " bytes\n"
              << "Output dir:  " << std::filesystem::absolute(opts.output_dir).string() << "\n\n";

    cv::Mat preview = input.clone();
    cv::Scalar line_color(0, 0, 255);

    for (int c = 1; c < cols; ++c) {
        int x = c * opts.tile_width;
        cv::line(preview, cv::Point(x, 0), cv::Point(x, img_h), line_color, 1);
    }
    for (int r = 1; r < rows; ++r) {
        int y = r * opts.tile_height;
        cv::line(preview, cv::Point(0, y), cv::Point(img_w, y), line_color, 1);
    }

    std::string grid_path = opts.output_dir + "/_grid_preview.jpg";
    cv::imwrite(grid_path, preview, {cv::IMWRITE_JPEG_QUALITY, 95});
    std::cout << "Grid preview: " << grid_path << "\n";

    return 0;
}
