#include "pipeline.hpp"

#include <chrono>
#include <iostream>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_video> <output_image>" << std::endl;
        return 1;
    }

    hero_lob::Pipeline pipeline;
    auto start = std::chrono::steady_clock::now();
    bool ok = pipeline.Run(argv[1], argv[2]);
    auto end = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(end - start).count();
    std::cerr << "[Main] Elapsed: " << elapsed << "s\n";
    return ok ? 0 : 1;
}
