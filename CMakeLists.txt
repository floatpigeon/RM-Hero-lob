cmake_minimum_required(VERSION 3.24)

project(image_trans VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)

add_library(image_trans_core
    src/replay/video_file_source.cpp
    src/windowing/window_assembler.cpp
    src/background/reference_builder.cpp
    src/registration/registration_engine.cpp
    src/motion/motion_extractor.cpp
    src/compositor/trail_compositor.cpp
    src/debug/debug_recorder.cpp
    src/pipeline/offline_processor.cpp
    src/cli/cli_options.cpp
)

target_include_directories(image_trans_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(image_trans_core
    PUBLIC
        ${OpenCV_LIBS}
)

target_compile_options(image_trans_core
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

add_executable(image_trans_cli
    src/cli/main.cpp
)

target_link_libraries(image_trans_cli
    PRIVATE
        image_trans_core
)

include(CTest)

if(BUILD_TESTING)
    add_executable(image_trans_tests
        tests/image_trans_tests.cpp
    )

    target_link_libraries(image_trans_tests
        PRIVATE
            image_trans_core
    )

    target_compile_options(image_trans_tests
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
    )

    add_test(NAME image_trans_tests COMMAND image_trans_tests)
endif()
