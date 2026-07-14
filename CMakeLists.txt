cmake_minimum_required(VERSION 3.20)

project(HeroLob LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)

add_library(hero_lob_core
    core/src/pipeline.cpp
    core/src/capture.cpp
    core/src/identifier.cpp
    core/src/identifier_geometry.cpp
    core/src/tracker.cpp
    core/src/reference_frame_selector.cpp
    core/src/image_registrator.cpp
    core/src/background_remover.cpp
    core/src/tracker_processor.cpp
    core/src/image_synthesis.cpp
)

target_include_directories(hero_lob_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(hero_lob_core
    PUBLIC
        ${OpenCV_LIBS}
)

add_executable(hero_lob
    core/src/main.cpp
)

target_link_libraries(hero_lob
    PRIVATE
        hero_lob_core
)

add_executable(hero_lob_color_picker
    core/tools/color_picker_main.cpp
)

target_include_directories(hero_lob_color_picker
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(hero_lob_color_picker
    PRIVATE
        ${OpenCV_LIBS}
)

add_executable(hero_lob_brighten
    core/tools/brighten_main.cpp
)

target_include_directories(hero_lob_brighten
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(hero_lob_brighten
    PRIVATE
        ${OpenCV_LIBS}
)

add_executable(hero_lob_trajectory_debug
    core/tools/trajectory_debug_main.cpp
)

target_link_libraries(hero_lob_trajectory_debug
    PRIVATE
        hero_lob_core
)

add_executable(hero_lob_frame_extractor
    core/tools/frame_extractor_main.cpp
)

target_include_directories(hero_lob_frame_extractor
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(hero_lob_frame_extractor
    PRIVATE
        ${OpenCV_LIBS}
)

add_executable(hero_lob_brightness_dist
    core/tools/brightness_dist_main.cpp
)

target_include_directories(hero_lob_brightness_dist
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(hero_lob_brightness_dist
    PRIVATE
        ${OpenCV_LIBS}
)

add_executable(hero_lob_brightness_filter
    core/tools/brightness_filter_main.cpp
)

target_include_directories(hero_lob_brightness_filter
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(hero_lob_brightness_filter
    PRIVATE
        ${OpenCV_LIBS}
)

add_executable(hero_lob_video_trimmer
    core/tools/video_trimmer_main.cpp
)

target_include_directories(hero_lob_video_trimmer
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(hero_lob_video_trimmer
    PRIVATE
        ${OpenCV_LIBS}
)

add_executable(hero_lob_compress
    core/tools/compress_main.cpp
)

target_include_directories(hero_lob_compress
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(hero_lob_compress
    PRIVATE
        ${OpenCV_LIBS}
)