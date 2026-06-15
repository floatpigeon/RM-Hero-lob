#include "pipeline.hpp"

#include <iostream>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_video> <output_image>" << std::endl;
        return 1;
    }

    hero_lob::Pipeline pipeline;
    pipeline.Run(argv[1], argv[2]);
    return 0;
}
