#include "image_trans/simulator.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    image_trans::AppConfig config;
    bool show_help = false;
    std::string error;

    if (!image_trans::parseArguments(argc, argv, config, show_help, error)) {
        std::cerr << error << '\n';
        image_trans::printUsage(std::cerr, argv[0]);
        return 1;
    }

    if (show_help) {
        image_trans::printUsage(std::cout, argv[0]);
        return 0;
    }

    return image_trans::run(config, std::cout, std::cerr);
}
