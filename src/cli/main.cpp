#include <exception>
#include <iostream>

#include "image_trans/cli_options.hpp"
#include "image_trans/offline_processor.hpp"

int main(int argc, char** argv) {
    try {
        const image_trans::CliOptions options = image_trans::CliOptionsParser::parse(argc, argv);
        const image_trans::CompositeConfig config =
            image_trans::CliOptionsParser::make_config_from_options(options);

        image_trans::ReplayRequest request;
        request.input_path = options.input_path;
        request.trigger = options.trigger;
        request.output_dir = options.output_dir;

        image_trans::OfflineProcessor processor(config);
        const image_trans::CompositeResult result = processor.run(request);

        std::cout << "accepted_frames=" << result.accepted_frame_count
                  << " dropped_frames=" << result.dropped_frame_count << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "image_trans_cli error: " << error.what() << "\n";
        return 1;
    }
}
