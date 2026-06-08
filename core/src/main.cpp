#include <iostream>
#include <string>

#include "pipeline.hpp"

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: hero_lob <input_video> <output_image>\n";
    return 1;
  }

  hero_lob::Pipeline pipeline;
  const bool ok = pipeline.Run(argv[1], argv[2]);
  return ok ? 0 : 1;
}
